# Copyright (c) 2021, InOrbit, Inc.
# All rights reserved.
# Agentlet module that watches arbitrary paths for databags and sends their
# state to the cloud
# NOTE (Flor_Grosso): this module is disabled as the current agent
# implementation doesn't support ROS 2 bags.
import glob
import json
import os
import threading
import time
from collections import namedtuple
from queue import Queue

from util.artifacts import Databag
from util.concurrency import Interval
from util.overrides import overrides

from .agentlet import Agentlet
from .inorbit_pb2 import DatabagUpdateMessage

Path = namedtuple("Path", ["path", "extension", "max_disk_gb", "max_files"])

GB = 1024 * 1024 * 1024  # B -> KB -> MB -> GB

DEFAULT_SYNC_RATE_HZ = 0.1  # How often the databags list is synced with filesystem and sent
DEFAULT_UPLOAD_RATE_HZ = 1  # How frequently to wake up uploader thread
DEFAULT_DATABAGS_LIMIT = 1000  # Max number of databags to attempt syncing, by default
DEFAULT_FORCE_SYNC_THRESHOLD_SEC = 3600  # Only send full list of databags once every hour (at most)

MQTT_DATABAG_UPLOAD_TOPIC = "ros/databag/upload"
MQTT_DATABAG_UPDATE_TOPIC = "ros/databag/update"
# Flag to set on rosbags' metadata to signal these are not user-facing rosbags
BAG_PROPERTY_INTERNAL = "__inorbit_internal__"

# State fields. Reference:
# https://docs.google.com/document/d/19ufqNiuUshL7HqJkQ43auEhZgGLnt7J-EaICKI8gjns/edit#heading=h.cxxi3yh0pcir
FIELD_PATHS = "paths"  # Paths and extensions to scan for artifacts files
FIELD_SYNC_RATE_HZ = "sync_rate_hz"  # How often to scan for changes and send files updates
FIELD_UPLOAD_RATE_HZ = "upload_rate_hz"  # How often to wake up uploader thread
FIELD_DATABAGS_LIMIT = "databags_limit"  # Max number of databags to sync (per directory)
# Minimum wait (seconds) before re-sending all databags upon mqtt reconnection
FIELD_FORCE_SYNC_THRESHOLD_SEC = "force_sync_threshold_sec"
# Outgoing state fields
FIELD_AVAILABLE_DATABAGS = "available_databags"


class DatabagAgentlet(Agentlet):
    def __init__(self, uplink):
        super(DatabagAgentlet, self).__init__(uplink)
        if "INORBIT_ID" not in os.environ:
            raise RuntimeError("Missing INORBIT_ID environment variable")
        self.robot_id = os.environ["INORBIT_ID"]
        # Dictionary to store the current available databags.
        # {databag_name: Databag}
        self._databags = {}
        # Lock to prevent race conditions between threads on the databags dict
        self._databags_mutex = threading.Lock()
        # Path for bags recorded internally (from the blackbox).
        # It gets added to _databags_paths
        # TODO(lpineda.io) Port data-backfill in ROS 2 and re-enable
        # self._internal_databags_paths = (
        #     [self.uplink.blackbox.artifacts_path] if self.uplink.blackbox.artifacts_path else []
        # )
        self._internal_databags_paths = []
        # The paths to watch in the filesystem with their extensions. Note that
        # it includes agent-internal paths we wish to monitor (if any)
        self._databags_paths = self.compute_databags_paths()
        # Queue to store target bags from incoming upload requests
        self._upload_requests_queue = Queue()
        # Start assigning the worker threads variables to None for safety
        self._databag_sync_thread = None
        self._databag_upload_thread = None

        # Configuration (state) fields
        self._databags_sync_rate_hz = DEFAULT_SYNC_RATE_HZ
        self._databags_upload_rate_hz = DEFAULT_UPLOAD_RATE_HZ
        self._databags_limit = DEFAULT_DATABAGS_LIMIT
        self._force_sync_threshold_sec = DEFAULT_FORCE_SYNC_THRESHOLD_SEC
        # Timestamp to throttle full syncs. Used for reconnects, see _on_link_connected
        self._last_databags_forced_sync_at = None

    @overrides(Agentlet)
    def load(self, runlevel):
        self._states["runlevel"] = runlevel
        # Sync the 'internal databags paths' with the Blackbox in case it
        # changed while this agentlet was unloaded)
        # TODO(lpineda.io) Port data-backfill in ROS 2 and re-enable
        # self._internal_databags_paths = (
        #     [self.uplink.blackbox.artifacts_path] if self.uplink.blackbox.artifacts_path else []
        # )
        self._internal_databags_paths = []
        # Do initial blocking sync and store the state
        # NOTE: The parameter to _sync_databags should be False once the protocol supports
        # doing an initial sync from the bags in state["available_bags"]
        self._sync_databags(True)
        with self._databags_mutex:
            self._states[FIELD_AVAILABLE_DATABAGS] = list(self._databags.keys())
        # Launch the working threads if they are not already defined
        if not self._databag_sync_thread:
            self._databag_sync_thread = Interval(
                self._sync_databags, 1 / self._databags_sync_rate_hz
            ).start()
        if not self._databag_upload_thread:
            self._databag_upload_thread = Interval(
                self._upload_databags, 1 / self._databags_upload_rate_hz
            ).start()
        # Add upload topic listener
        self.uplink.add_listener(MQTT_DATABAG_UPLOAD_TOPIC, self._parse_upload_requests)
        self.uplink.add_connection_listener(
            self._on_link_connected, subscriber=self.__class__.__name__
        )

        self._states["loaded"] = True
        # Send a state update
        self.publish_state(self.uplink, self._states)
        # Record this timestamp. If MQTT (re)connects soon, do not send a full bags list update
        self._last_databags_forced_sync_at = time.time()
        return True

    @overrides(Agentlet)
    def unload(self):
        # If running, stop the threads and assign the variables to None
        if self._databag_sync_thread:
            self._databag_sync_thread.stop()
            self._databag_sync_thread = None
        if self._databag_upload_thread:
            self._databag_upload_thread.stop()
            self._databag_upload_thread = None
        self.uplink.remove_connection_listener(
            self._on_link_connected, subscriber=self.__class__.__name__
        )
        self.uplink.remove_listener(MQTT_DATABAG_UPLOAD_TOPIC)
        self._ros.remove_submodule("databag")
        self._states["loaded"] = False
        self.once_logger.reset_all()
        return True

    @overrides(Agentlet)
    def set_state(self, state):
        """
        Called whenever a set_module command is received.
        """

        # If the new state contains a list of paths, store them in the current
        # state and parse them into Path Objects. Note that we always account
        # for 'internal' paths, which get added to this state
        if FIELD_PATHS in state:
            self._states[FIELD_PATHS] = state[FIELD_PATHS]
            self._databags_paths = self.compute_databags_paths()

        if FIELD_SYNC_RATE_HZ in state:
            self._states[FIELD_SYNC_RATE_HZ] = state[FIELD_SYNC_RATE_HZ]
            self._databags_sync_rate_hz = state[FIELD_SYNC_RATE_HZ]
            # If the publisher thread is already started, change
            with self._databags_mutex:
                if self._databag_sync_thread:
                    self._databag_sync_thread.set_interval_seconds(1 / self._databags_sync_rate_hz)

        if FIELD_UPLOAD_RATE_HZ in state:
            self._states[FIELD_UPLOAD_RATE_HZ] = state[FIELD_UPLOAD_RATE_HZ]
            self._databags_upload_rate_hz = state[FIELD_UPLOAD_RATE_HZ]
            # If the uploader thread is already started, change
            with self._databags_mutex:
                if self._databag_upload_thread:
                    self._databag_upload_thread.set_interval_seconds(
                        1 / self._databags_upload_rate_hz
                    )

        if FIELD_DATABAGS_LIMIT in state:
            self._states[FIELD_DATABAGS_LIMIT] = state[FIELD_DATABAGS_LIMIT]
            self._databags_limit = state[FIELD_DATABAGS_LIMIT]

        if FIELD_FORCE_SYNC_THRESHOLD_SEC in state:
            self._states[FIELD_FORCE_SYNC_THRESHOLD_SEC] = state[FIELD_FORCE_SYNC_THRESHOLD_SEC]
            self._force_sync_threshold_sec = state[FIELD_FORCE_SYNC_THRESHOLD_SEC]

        # Do a first sync before updating state
        # NOTE: The parameter to _sync_databags should be False once the protocol supports
        # doing an initial sync from the bags in state["available_bags"]
        self._sync_databags(True)

        with self._databags_mutex:
            # DictKeys object is not JSON serializable
            self._states[FIELD_AVAILABLE_DATABAGS] = list(self._databags.keys())

        self.publish_state(self.uplink, self._states)
        # Record this timestamp. If MQTT (re)connects soon, do not send a full bags list update
        self._last_databags_forced_sync_at = time.time()

    def compute_databags_paths(self):
        """
        Returns all rosbag paths to monitor, as a list of Path elements.
        This includes those from the _state, and "internal" paths (blackbox)
        """

        state_paths = (self._states and self._states.get("paths", [])) or []
        external_paths = []
        for path in state_paths:
            # Use secure get for max disk and files attributes as they were added later to
            # the Path schema
            new_path = Path(
                path["path"], path["extension"], path.get("max_disk_gb"), path.get("max_files")
            )
            # safety checks:
            # Check that the path and extension don't have wildcards
            if "*" in new_path.path or "*" in new_path.extension:
                self.logger.warning(
                    "Wildcard '*' is not allowed. Ignoring path %s with extension %s",
                    new_path.path,
                    new_path.extension,
                )
                continue
            # Check that, if max_disk was provided, it is higher than 0
            if new_path.max_disk_gb and new_path.max_disk_gb <= 0:
                self.logger.warning(
                    "max_disk_gb needs to be greater than 0. Ignoring path %s with extension %s",
                    new_path.path,
                    new_path.extension,
                )
                continue
            # Check that, if max_files was provided, it is higher than 0
            if new_path.max_files and new_path.max_files <= 0:
                self.logger.warning(
                    "max_files needs to be greater than 0. Ignoring path %s with extension %s",
                    new_path.path,
                    new_path.extension,
                )
                continue
            external_paths.append(new_path)
        return external_paths + self._internal_databags_paths

    @overrides(Agentlet)
    def set_runlevel(self, runlevel):
        self._states["runlevel"] = runlevel

    def _send_databag_update(self, bag):
        """
        Sends a bag update to the cloud, whenever a bag is created or any of its
        fields (stored in robot, uploading status, URL) change.
        """

        bag_msg = DatabagUpdateMessage()
        bag_msg.name = bag.name
        bag_msg.stored_in_robot = bag.in_robot
        bag_msg.uploading_to_cloud = bag.uploading
        bag_msg.url = bag.url
        bag_msg.start_ts = bag.start_ts
        bag_msg.end_ts = bag.end_ts
        bag_msg.size_kb = bag.size_kb
        for k, v in bag.properties.items():
            # Serialize each value according to its type
            v = v if isinstance(v, str) else json.dumps(v)
            bag_msg.properties[k] = v
        # HACK(herchu) For initial version of agent-variant-offline: decide
        # some 'internal' flag based only on the filename
        if bag.name.startswith("blackbox"):
            bag_msg.properties[BAG_PROPERTY_INTERNAL] = "true"

        # Not using QoS=1 anymore: If a message gets lost, it will eventually be resent upon
        # agent restart
        self.uplink.publish_protobuf(MQTT_DATABAG_UPDATE_TOPIC, bag_msg)

    def _sync_databags(self, publish_updates=True):
        """
        Runs on a separate thread. Syncs the current available databags with the
        Ones in the filesystem for all the watched paths.
        It sends messages for both the new and deleted bags
        """

        # These two sets will hold list of new and deleted databags to send after releasing mutex
        databag_updates = []

        with self._databags_mutex:
            # Start with the current databags paths
            current_databags = set(self._databags.keys())
            # For each path to watch, retrieve the files in the filesystem to
            # later compare with the current available databags
            stored_databags = set()
            for path in self._databags_paths:
                # If configured, delete excess of files in the path.
                self._cleanup_space(path)
                # For the given path, find only the files with the proper
                # extension
                file_pattern = os.path.join(path.path, "*." + path.extension)
                files_list = list(filter(os.path.isfile, glob.glob(file_pattern)))
                if self._databags_limit and len(files_list) > self._databags_limit:
                    # If there are too many files to report, discard the oldest ones. This
                    # operation may be slow (in the agent) as it gets getmtime for each file, but
                    # in that same situation, it is even slower to try sending them through MQTT.
                    # Note that the sorting is based on filesystem modification time; which may
                    # not be the same as rosbags' metadata (start/end). Just a good-enough approach.
                    files_list = sorted(
                        files_list,
                        key=lambda filename: os.path.getmtime(os.path.join(path.path, filename)),
                    )[-self._databags_limit :]
                for filepath in files_list:
                    filename = os.path.basename(filepath)
                    stored_databags.add(filename)
                    if filename in current_databags:
                        # Check if properties changed and in that case add to databag_updates
                        databag = self._databags[filename]
                        changed = databag.refresh_properties()
                        if changed:
                            databag_updates.append(databag)
                    else:
                        # If the bag is not in the current ones, add it and send the update (later)
                        databag = Databag(filepath)
                        self._databags[filename] = databag
                        databag_updates.append(databag)
            # The current databags that are not in the filesystem anymore are
            # the deleted ones. Remove them and send the update
            deleted_databags = current_databags - stored_databags
            for databag_name in deleted_databags:
                databag = self._databags.pop(databag_name)
                databag.set_in_robot(False)
                databag_updates.append(databag)

        if publish_updates:
            # Mutex is released, now send all updates
            for databag in databag_updates:
                self._send_databag_update(databag)

        if databag_updates:
            # Since changes were detected from file system, if Link notifies of a MQTT reconnection
            # we need to make sure all databags are propagated (there is no way to know if
            # each _send_databag_update gets through or if the link is actually down).
            # See _on_link_connected for details on _last_databags_forced_sync_at
            self._last_databags_forced_sync_at = None

    def _upload_databags(self):
        """
        Runs on a separate thread. Checks upload requests buffer and makes a
        bag upload if possible.
        """

        while self._upload_requests_queue.qsize():
            databag = self._upload_requests_queue.get()
            self.logger.info("Uploading %s to cloud.", databag.name)
            # TODO (Flor_Grosso): check for robot task status and network
            # connection before uploading. Schedule for upload when
            # conditions are appropriate.
            # Upload databag to cloud. If it fails, clear the uploading
            # status.
            # TODO (Flor_Grosso): clear local databag references.
            try:
                # Callback argument to receive transferred bytes, directly add them as tx_bytes
                databag.upload(subpath=self.robot_id, callback=self.add_tx_bytes)
            except Exception as e:
                self.logger.exception("Could not upload %s to cloud.", databag.name)
                databag.set_uploading(False)
            # Send rosbag updates to cloud
            self._send_databag_update(databag)

    def _parse_upload_requests(self, payload):
        """
        Called from the server.
        Receives and parses rosbag upload commands.
        """

        # Parse payload
        [seq, ts, databag_name] = payload.decode("utf-8").split("|")
        # Make sure that the syncing thread is not taking place
        with self._databags_mutex:
            databag = self._databags.get(databag_name)
        if databag is None:
            self.logger.warning(
                "Failed to upload %s to cloud. Can't find file in robot.", databag_name
            )
        else:
            # Notify the client that we are beginning the upload process
            databag.set_uploading(True)
            self._send_databag_update(databag)
            # Add request to uploads queue
            self._upload_requests_queue.put(databag)

    def _cleanup_space(self, path):
        """
        Called in databags sync cycle.
        If configured, checks if the path is beyond its permitted disk space or maximum number
        of files and removes old files until it is not.
        """

        # NOTE(diegobatt): As this is called inside _sync_bags, lock is already acquired
        # If no max disk configured, return
        # If neither max disk nor max files configured, return
        if path.max_disk_gb is None and path.max_files is None:
            return
        # If not provided, assume infinite space
        max_disk_gb = path.max_disk_gb or float("inf")
        max_files = path.max_files or float("inf")
        try:
            file_pattern = os.path.join(path.path, "*." + path.extension)
            files = glob.glob(file_pattern)
            # Sort by last modification date ascending, in place
            # If maximum number of files exceeded, delete the oldests
            while files and len(files) > max_files:
                oldest_file = files.pop()
                os.remove(oldest_file)
            # Keep track of each file's size
            file_sizes = list(map(os.path.getsize, files))
            path_size_gb = sum(file_sizes) / GB
            # If no disk space available, remove as many files as needed to get space
            while files and path_size_gb > max_disk_gb:
                oldest_file = files.pop()
                oldest_file_size = file_sizes.pop()
                path_size_gb -= oldest_file_size / GB
                os.remove(oldest_file)
        except Exception as e:
            self.once_logger.exception(
                "cleanup_space", "Failed to clean-up space in path %s: %s" % (path, str(e))
            )

    def _on_link_connected(self):
        """
        Called when the Link has (re)connected. Since we cannot determine if this
        is a first time or a reconnection, or which bags have been sent already,
        proactively re-send the complete list of available artifacts.
        This is not the same as _sync_databags, which only sends messages for
        changes detected (when file system vs. object state differ).
        """

        if (
            self._last_databags_forced_sync_at is not None
            and self._force_sync_threshold_sec > 0
            and time.time() - self._last_databags_forced_sync_at < self._force_sync_threshold_sec
        ):
            # There was already a forced full sync recently sent by this method. Ignore this call
            self.logger.info("Skipping full sync in on_link_connected call; recently processed")
            return

        with self._databags_mutex:
            for databag in iter(self._databags.values()):
                self._send_databag_update(databag)
        # Record this timestamp to throttle calls to this method if another reconnect happens soon
        self._last_databags_forced_sync_at = time.time()
