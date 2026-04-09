# Copyright (c) 2018, InOrbit, Inc.
# All rights reserved.
# Agentlet module that records rosbags
# Depends on the ROS and system modules.
import json
import os
import subprocess
import threading
import time
from queue import Queue
import psutil

from inorbit import INORBIT_AGENT_ROSBAGS_PATH
from .inorbit_pb2 import RosbagUpdateMessage
from util.artifacts import Rosbag
from util.concurrency import Interval
from util.overrides import overrides

from .agentlet import Agentlet


# Duration of record commands, initially 5 minutes
BAG_DURATION_SECONDS = 300

DEFAULT_TOPICS = ["cmd_vel", "diagnostics_agg", "map", "rosout", "scan", "tf"]

MQTT_UPLOAD_CMD_TOPIC = "ros/rosbag/upload"
# Same topic as in ingest/databags.js
MQTT_ROSBAG_UPDATE_TOPIC = "ros/rosbag/update"

RESOURCES_QUERY_RATE_HZ = 1.0

UPLOAD_ROSBAG_RATE_HZ = 1.0

# Time to wait before retrying to record a bag if last attempt wasn't
# successful
RETRY_RECORD_WAIT_TIME_SECONDS = 10

# Max disk space allowed for storing rosbags (GB) - Default.
MAX_HDD_ALLOWED_DEFAULT = 0.5

DEFAULT_DATE_IN_NAME_FORMAT = "%Y-%m-%dT%H:%M:%S"


class RosbagAgentlet(Agentlet):
    def __init__(self, uplink, ros):
        super(RosbagAgentlet, self).__init__(uplink)
        if "INORBIT_ID" not in os.environ:
            raise RuntimeError("Missing INORBIT_ID environment variable")
        self.robot_id = os.environ["INORBIT_ID"]

        self._ros = ros

        # Max disk space allowed for storing rosbags
        self._states["max_hdd_usage_gb"] = MAX_HDD_ALLOWED_DEFAULT
        self._states["topics_to_record"] = DEFAULT_TOPICS

        self._current_bag = None

        # Queue to store target bags from incoming upload requests
        self._upload_requests_queue = Queue()

        # Current hdd space used by /rosbag directory
        self._states["current_hdd_usage_gb"] = 0

        # List of available inorbit rosbags in the robot.
        self._available_inorbit_rosbags = []
        # Different threads are concurrently modifying and reading in a
        # non-atomic way the available rosbags list, we need a lock
        # to preserve it while doing so
        self._available_inorbit_rosbags_mutex = threading.Lock()

        # Resources usage state
        self._resources_over_limits = False

        # For safety, initialize worker threads as None
        self._resources_checker_thread = None
        self._bag_recorder_thread = None
        self._bag_uploader_thread = None

        self._states["rosbags_path"] = self._states.get("rosbags_path", INORBIT_AGENT_ROSBAGS_PATH)

    @overrides(Agentlet)
    def load(self, runlevel):
        self._states["runlevel"] = runlevel

        try:
            # Create directory to store rosbags if it doesn't exist
            if not os.path.exists(self._states["rosbags_path"]):
                os.makedirs(self._states["rosbags_path"])

            # Start with the rosbags available in the system
            self._populate_rosbags_from_filesystem()

            # Initial resources usage state
            self._resources_over_limits = self._is_resource_utilization_over_limit()

            # Load currently used HDD space
            self._states["current_hdd_usage_gb"] = self._compute_rosbags_hdd_usage()

        except Exception as e:
            self.logger.exception("Exception reading rosbags from filesystem.")
            return False

        self._ros.add_submodule("rosbag")

        self._states["available_rosbags"] = self._list_rosbag_filenames_in_robot()

        # Register for upstream incoming upload commands
        self.uplink.add_listener(MQTT_UPLOAD_CMD_TOPIC, self._parse_upload_requests)

        # Start the worker threads. Check that they are not already working
        if not self._bag_recorder_thread:
            self._bag_recorder_thread = Interval(self._record_rosbag, 0).start()
        if not self._resources_checker_thread:
            self._resources_checker_thread = Interval(
                self._check_resources, 1.0 / RESOURCES_QUERY_RATE_HZ
            ).start()
        if not self._bag_uploader_thread:
            self._bag_uploader_thread = Interval(
                self._upload_rosbags, 1.0 / UPLOAD_ROSBAG_RATE_HZ
            ).start()

        self._states["loaded"] = True

        # Send a state update
        self.publish_state(self.uplink, self._states)

        return True

    @overrides(Agentlet)
    def unload(self):
        # Shutdown threads
        if self._bag_recorder_thread:
            self._bag_recorder_thread.stop()
            self._bag_recorder_thread = None
        if self._resources_checker_thread:
            self._resources_checker_thread.stop()
            self._resources_checker_thread = None
        if self._bag_uploader_thread:
            self._bag_uploader_thread.stop()
            self._bag_uploader_thread = None
        if self._current_bag and self._current_bag.recording:
            self._current_bag.halt_recording()
        # Remove upload topic listener
        self.uplink.remove_listener(MQTT_UPLOAD_CMD_TOPIC)
        # Remove submodule
        self._ros.remove_submodule("rosbag")
        self._states["loaded"] = False
        # Re-initialize exception reporting
        self.once_logger.reset_all()
        return True

    @overrides(Agentlet)
    def set_runlevel(self, runlevel):
        self._states["runlevel"] = runlevel

    def set_state(self, state):
        """
        Called whenever a set_module command is received.

        TODO (Flor_Grosso): Rework this method using protobuf.
        """

        if "max_hdd_usage_gb" in state:
            self._states["max_hdd_usage_gb"] = state["max_hdd_usage_gb"]

        if "topics_to_record" in state:
            self._states["topics_to_record"] = state["topics_to_record"]
            # Re-initialize exception reporting
            self.once_logger.reset_all()

        if "rosbags_path" in state:
            self._states["rosbags_path"] = state["rosbags_path"]

        # Update available rosbags list
        self._states["available_rosbags"] = self._list_rosbag_filenames_in_robot()
        # Send a state update
        self.publish_state(self.uplink, self._states)

    def _kill_recording_process_if_running(self):
        """
        Search for 'ros2 bag record` processes currently running and terminate them.

        Invoked before starting a new recording to clean orphan processes.
        """
        for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
            pid = proc.info.get('pid')
            proc_name = proc.info.get('name')
            cmdline = proc.info.get('cmdline')
            if (
                proc_name == "ros2" and 
                "bag" in cmdline and
                "record" in cmdline and 
                any([param.startswith(f"--output={self._states['rosbags_path']}") for param in cmdline])
            ):
                self.logger.info(f"Terminating orphan 'ros2 bag record' process. PID: {pid}")
                psutil.Process(pid).terminate()

    def _record_rosbag(self):
        """
        Runs on a separate thread. Handles rosbag recordings at a fixed rate,
        determined by BAG_DURATION_SECONDS.
        """

        # Check for any rosbag file deleted manually and do the
        # necessary updates
        self._check_rosbag_manual_deletion()

        if self._resources_over_limits:
            time.sleep(RETRY_RECORD_WAIT_TIME_SECONDS)
        else:
            try:
                # Stop current bag recording if there's an existing one
                self._kill_recording_process_if_running()
                rosbag_name = time.strftime(DEFAULT_DATE_IN_NAME_FORMAT, time.localtime())
                # NOTE: 'ros2 bag record' can generate multiple files under the output
                # directory if certain parameters are used. This implementation assumes a
                # single '.db3' file per subdirectory under ~/.inorbit/local/rosbags folder
                # .e.g:
                #   ~/.inorbit/local/rosbags/<ROSBAG_A>/<ROSBAG_A>_0.db3
                #   ~/.inorbit/local/rosbags/<ROSBAG_B>/<ROSBAG_B>_0.db3
                #   ~/.inorbit/local/rosbags/<ROSBAG_C>/<ROSBAG_C>_0.db3
                # 'ros2 bag record' adds a '_0' suffix to the first file it creates when
                # recording. As we expect a single '.db3' per subdirectory the robot_path
                # can be obtained with: {rosbags_path}/{rosbag_name}/{rosbag_name}_0.db3
                # See details on 'Rosbag.record' method.
                rosbag_path = os.path.join(
                    self._states["rosbags_path"], rosbag_name, f"{rosbag_name}_0.db3"
                )
                self._current_bag = Rosbag(rosbag_path)
                self._current_bag.record(
                    self._states["topics_to_record"], BAG_DURATION_SECONDS
                )

                # TODO (Flor_Grosso): add a safety check to avoid uploading
                # data when a rosbag wasn't recorded successfully

                self.logger.info("Recorded %s.", self._current_bag.name)

                # Update available inorbit rosbags
                with self._available_inorbit_rosbags_mutex:
                    self._available_inorbit_rosbags.append(self._current_bag)

                # Send bag update to cloud
                self._send_bag_update(self._current_bag)

                # Update current HDD usage
                self._send_hdd_usage_update()

            except Exception as recording_error:
                # Wait a certain time before attempting to record another
                # rosbag
                self.once_logger.exception(
                    "rosbag_recording",
                    f"Exception recording rosbag. {recording_error}",
                )
                time.sleep(RETRY_RECORD_WAIT_TIME_SECONDS)

    def _check_resources(self):
        """
        Runs on a separate thread. Checks resource utilization and performs old
        bags clean-up or halts the current recording if necessary.
        """

        if self._is_resource_utilization_over_limit():
            self._resources_over_limits = True

            # Clean oldest rosbags if possible and continue
            if self._clean_up_rosbags():
                return

            # If there are no bags available yet and current one is
            # recording, halt it
            if self._current_bag is not None and self._current_bag.recording:
                self._current_bag.halt_recording()
                self.logger.warning("Halting current recording due to" " lack of space.")
        else:
            self._resources_over_limits = False

    def _upload_rosbags(self):
        """
        Runs on a separate thread. Checks uploads requests buffer and makes a
        bag upload if possible.
        """

        while self._upload_requests_queue.qsize():
            # Get inorbit bag to upload from request's buffer
            bag_to_upload = self._upload_requests_queue.get()

            self.logger.info(f"Uploading {bag_to_upload.name} to cloud.")

            # TODO (Flor_Grosso): check for robot task status and network
            # connection before uploading. Schedule for upload when
            # conditions are appropriate.

            # Upload rosbag to cloud. If it fails, clear the uploading
            # status.
            # TODO (Flor_Grosso): clear local bag references.
            try:
                # Callback argument to receive transferred bytes, directly add them as tx_bytes
                bag_to_upload.upload(subpath=self.robot_id, callback=self.add_tx_bytes)
            except Exception as e:
                self.logger.exception(f"Could not upload {bag_to_upload.name} to cloud.")
                bag_to_upload.set_uploading(False)

            # Send rosbag updates to cloud
            self._send_bag_update(bag_to_upload)

    def _is_resource_utilization_over_limit(self):
        """
        Checks resource utilization and returns True if values are above critical
        levels.
        TODO (Flor_Grosso): consider extending resource monitoring to memory and
        cpu usage, as well as network stats ans task/mission status.
        """

        # Compute the space occupied by inorbit rosbags.
        self._states["current_hdd_usage_gb"] = self._compute_rosbags_hdd_usage()

        return self._states["current_hdd_usage_gb"] > self._states["max_hdd_usage_gb"]

    def _compute_rosbags_hdd_usage(self):
        """
        Computes hdd space occupied by /rosbags dir in GB.
        """

        # Get the storage used by rosbags (in megabytes)
        rosbags_hdd_usage_mb = float(
            subprocess.check_output(["du", "-sm", self._states["rosbags_path"]]).split()[0]
        )

        return rosbags_hdd_usage_mb / 1024

    def _list_rosbag_filenames_in_robot(self):
        """
        Returns a list with the rosbag file names stored in the robot under the
        INORBIT_AGENT_ROSBAGS_PATH.
        """

        bag_files = []
        if not os.path.exists(self._states["rosbags_path"]):
            return bag_files

        for bag_dir in os.listdir(self._states["rosbags_path"]):
            bag_dir_path = os.path.join(self._states["rosbags_path"], bag_dir)
            if os.path.isdir(bag_dir_path):
                for bag_file in os.listdir(bag_dir_path):
                    if bag_file.endswith("db3"):
                        bag_files.append(f"{bag_file}")
        
        return bag_files

    def _populate_rosbags_from_filesystem(self):
        """
        Populates rosbag list from existent files, if any. Return list sorted by
        date.
        """

        current_rosbags_in_robot = set(self._list_rosbag_filenames_in_robot())
        with self._available_inorbit_rosbags_mutex:
            available_inorbit_rosbags = set(bag.name for bag in self._available_inorbit_rosbags)
            new_bags = current_rosbags_in_robot - available_inorbit_rosbags
            # TODO (Flor_Grosso): consider saving URL of previous files to pass
            # it here too.
            for new_bag_name in new_bags:
                # HACK(@lpineda.io): add ROS2 rosbag subdirectory from bag_name.
                # On ROS2 rosbags are stored one level deeper under `self._states["rosbags_path"]`,
                # e.g. ~/.inorbit/local/rosbags/<ROSBAG_A>/<ROSBAG_A>_0.db3
                # NOTE: splitting by '_' is safe because the bag name is created with a timestamp
                # formatted with 'DEFAULT_DATE_IN_NAME_FORMAT' template.
                bag_name = new_bag_name.split('_')[0] # Remove '_0.db3'
                bag_path = os.path.join(self._states["rosbags_path"], bag_name, new_bag_name)
                new_bag = Rosbag(bag_path)
                self._available_inorbit_rosbags.append(new_bag)
                self._send_bag_update(new_bag)

            # If there are new bags, sort by start_ts property
            if new_bags:
                self._available_inorbit_rosbags.sort(key=lambda bag: bag.start_ts)

    def _parse_upload_requests(self, payload):
        """
        Called from the server.
        Receives and parses rosbag upload commands.
        """

        # Parse payload
        [seq, ts, rosbag_file] = payload.decode("utf-8").split("|")
        with self._available_inorbit_rosbags_mutex:
            bag_to_upload = next(
                (bag for bag in self._available_inorbit_rosbags if bag.name == rosbag_file), None
            )

        if bag_to_upload is None:
            self.logger.warning(
                f"Failed to upload {rosbag_file} to cloud. Can't find file in robot."
            )
        else:
            # Notify the client that we are beginning the upload process
            bag_to_upload.set_uploading(True)
            self._send_bag_update(bag_to_upload)

            # Add request to uploads queue
            self._upload_requests_queue.put(bag_to_upload)

    def _clean_up_rosbags(self):
        """
        Removes oldest bag stored in robot.
        """

        if not self._available_inorbit_rosbags:
            return False

        # Get oldest rosbag. This assumes that
        # self._available_inorbit_rosbags is sorted.
        with self._available_inorbit_rosbags_mutex:
            oldest_bag = self._available_inorbit_rosbags.pop(0)
            # If there is only one available rosbag in robot and it is not
            # uploaded, don't remove it.
            # TODO (Flor_Grosso): this avoids constantly recording, halting and
            # immediately deleting a single bag due to lack of space. Consider
            # the case when this single bag gets old in the system without
            # being uploaded.
            if not self._available_inorbit_rosbags and oldest_bag.url == "":
                self._available_inorbit_rosbags.append(oldest_bag)
                return False

        # Remove oldest rosbag from robot
        try:
            oldest_bag.delete_from_robot()
            self.logger.info(f"{oldest_bag.name} removed from robot.")
        except Exception as e:
            return False

        # Send update to cloud
        self._send_bag_update(oldest_bag)
        self._send_hdd_usage_update()

        return True

    def _send_bag_update(self, bag):
        """
        Sends a bag update to the cloud, whenever a bag is created or any of its
        fields (stored in robot, uploading status, URL) change.
        """

        rosbag_update = RosbagUpdateMessage()
        rosbag_update.name = bag.name
        rosbag_update.stored_in_robot = bag.in_robot
        rosbag_update.uploading_to_cloud = bag.uploading
        rosbag_update.url = bag.url
        rosbag_update.start_ts = bag.start_ts
        # replaced with start_ts, kept for backwards compatibility
        rosbag_update.ts = bag.start_ts
        rosbag_update.end_ts = bag.end_ts
        rosbag_update.size_kb = bag.size_kb
        rosbag_update.topics[:] = bag.topics
        for k, v in bag.properties.items():
            # Serialize each value according to its type
            v = v if isinstance(v, str) else json.dumps(v)
            rosbag_update.properties[k] = v

        self.uplink.publish_protobuf(MQTT_ROSBAG_UPDATE_TOPIC, rosbag_update, qos=1)

    def _send_hdd_usage_update(self):
        """
        Sends an update with the current hdd usage to the cloud.
        """

        required_fields = ["module_name", "current_hdd_usage_gb"]
        reduced_state = {
            key: value for key, value in self._states.items() if key in required_fields
        }

        self.publish_state(self.uplink, reduced_state)

    def _check_rosbag_manual_deletion(self):
        """
        Checks for files which have been deleted manually, sends an update to the
        cloud and removes the inorbit bag from the list of available ones.
        """

        current_rosbags_in_robot = set(self._list_rosbag_filenames_in_robot())
        with self._available_inorbit_rosbags_mutex:
            deleted_bags = [
                bag
                for bag in self._available_inorbit_rosbags
                if bag.name not in current_rosbags_in_robot
            ]

            for rosbag in deleted_bags:
                bag_index = self._available_inorbit_rosbags.index(rosbag)
                deleted_bag = self._available_inorbit_rosbags.pop(bag_index)
                deleted_bag.set_in_robot(False)
                self._send_bag_update(deleted_bag)
