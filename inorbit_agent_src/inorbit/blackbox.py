#
# Agent "blackbox" implementation: continuous data recording
#
# Copyright (c) 2018, InOrbit, Inc.
# All rights reserved.
#
# NOTE (Flor_Grosso): this module is disabled as the current agent
# implementation doesn't support ROS 2 bags.
"""
TODOs:
 - share code with Rosbag agentlet
 - compress data
"""
import math
import os
import threading
import time

import inorbit.logger
from inorbit import INORBIT_HOME
from inorbit.agentlets.databag import Path
from inorbit.agentlets.rosbag import DEFAULT_DATE_IN_NAME_FORMAT
from inorbit.msg._InOrbitOut import InOrbitOut  # message compiled with catkin
from util.artifacts import Rosbag
from util.concurrency import Interval
from util.once_logger import OnceLogger

# Python 2/3 compatibility: using  'basestring'
# https://portingguide.readthedocs.io/en/latest/strings.html#strings


# Internal topic to write our MQTT communications
INORBIT_MQTT_OUT_ROS_TOPIC = "/__inorbit_internal__/mqtt_blackbox"
BAG_EXTENSION = "bag"
ACTIVE_SUFFIX = ".active"
CLEANUP_ACTIVE_BAG_SECONDS_OLD = 60 * 60 * 24 * 2  # 2 days

# Environment variables
ENV_ENABLE_BLACKBOX = "INORBIT_ENABLE_BLACKBOX"
ENV_BLACKBOX_PATH = "INORBIT_BLACKBOX_PATH"
ENV_BLACKBOX_DURATION_MINS = "INORBIT_BLACKBOX_DURATION_MINS"
ENV_BLACKBOX_MAX_SPACE_GB = "INORBIT_BLACKBOX_MAX_SPACE_GB"
ENV_BLACKBOX_MAX_TIME_MINS = "INORBIT_BLACKBOX_MAX_TIME_MINS"
# Default values
DEFAULT_BLACKBOX_PATH = os.path.join(INORBIT_HOME, "local/blackbox")
DEFAULT_MAX_DISK_GB = 1
DEFAULT_BAG_DURATION_MINUTES = 1
DEFAULT_MAX_TIME_MINS = 2 * 60  # 2 hours


class Recorder:
    """
    This class represents the agent's Blackbox: It records all types of messages
    and events for later inspection and to recover in case of problems: most
    importantly, when an agent goes offline - so we still record data for later
    transmission.

    It currently supports recording only outgoing MQTT messages.
    """

    def __init__(self):
        self.logger = inorbit.logger.getLog(__name__)
        self._enabled = False
        # Parse configuration from environment variables
        if ENV_ENABLE_BLACKBOX in os.environ:
            self._enabled = os.environ[ENV_ENABLE_BLACKBOX] == "yes"
        message = "Blackbox is " + ("enabled" if self.enabled else "disabled")
        if self._enabled:  # Log env var values if they differ from defaults
            if self.rosbags_path != DEFAULT_BLACKBOX_PATH:
                message += ". Path=" + str(self.rosbags_path)
            if self.max_space_gb != DEFAULT_MAX_DISK_GB:
                message += ". Space=" + str(self.max_space_gb) + "Gb"
            if self.bag_duration_mins != DEFAULT_BAG_DURATION_MINUTES:
                message += ". Rosbags=" + str(self.bag_duration_mins) + "mins"
        self.logger.info(message)

        # Current Rosbag object being recorder. When not `None`, it means it is
        # currently recording
        self._rosbag = None
        # Thread used as interval to close/start new rosbags
        self._ticker_thread = None
        self.once_logger = OnceLogger(self.logger)
        self._mutex = threading.Lock()
        if self.enabled:
            self.start()
            # Clean old enough rosbags that might have been left active
            self._cleanup_old()

    def _start_rosbag(self):
        """
        Opens a new rosbag with the current clock time as timestamp, and returns
        this Rosbag object. It does not start any thread, timers or messages.

        @return The rosbag object. Note that it is the caller's responsibility to
        # save a pointer to it
        """

        try:
            os.makedirs(self.rosbags_path, exist_ok=True)
            rosbag_name = (
                "blackbox-"
                + time.strftime(DEFAULT_DATE_IN_NAME_FORMAT, time.localtime())
                + "."
                + BAG_EXTENSION
                + ACTIVE_SUFFIX
            )
            rosbag_path = os.path.join(self.rosbags_path, rosbag_name)
            rosbag = Rosbag(rosbag_path)
            if not rosbag.start_writer():
                self.once_logger.exception("start_writer", "Unable to start blackbox rosbag")
                return None
            self.logger.debug(f"Starting blackbox rosbag {rosbag.name}")
            return rosbag
        except Exception as recording_error:
            self.once_logger.exception(
                "blackbox_recording",
                f"Exception recording blackbox rosbag. {recording_error}",
            )

    def _stop_rosbag(self, rosbag):
        """
        Stops (closes) writing a rosbag object. It does not set instance variables.
        """

        self.logger.debug(f"Closing blackbox rosbag {rosbag.name}")
        # It assumes it is called with mutex LOCKED and non-null self._rosbag
        try:
            rosbag.stop_writer()
            filename = rosbag.path
            if filename.endswith(ACTIVE_SUFFIX):
                os.rename(filename, filename[0 : len(filename) - len(ACTIVE_SUFFIX)])

            # TODO(herchu): Write metadata (yaml) with:
            # start_ts, end_ts, topics and [BAG_PROPERTY_INTERNAL]=true
        except Exception:
            self.once_logger.exception("stop_rosbag", "Error stopping or renaming rosbag")

    @property
    def enabled(self):
        return self._enabled

    @property
    def rosbags_path(self):
        """
        Returns the path where rosbags are recorded.
        """

        return os.environ.get(ENV_BLACKBOX_PATH) or DEFAULT_BLACKBOX_PATH

    @property
    def max_space_gb(self):
        """
        Returns the maximum disk space (in Gb) to use in blackbox rosbags.
        """

        if ENV_BLACKBOX_MAX_SPACE_GB in os.environ:
            try:
                return float(os.environ[ENV_BLACKBOX_MAX_SPACE_GB])
            except Exception:
                self.once_logger.exception(
                    "bad_env_max_space", ENV_BLACKBOX_MAX_SPACE_GB + "'s value must be a number"
                )
        return DEFAULT_MAX_DISK_GB

    @property
    def bag_duration_mins(self):
        """
        Returns the rosbag duration, in minutes.
        """

        if ENV_BLACKBOX_DURATION_MINS in os.environ:
            try:
                return float(os.environ[ENV_BLACKBOX_DURATION_MINS])
            except Exception:
                self.once_logger.exception(
                    "bad_env_bag_duration", ENV_BLACKBOX_DURATION_MINS + "'s value must be a number"
                )
        return DEFAULT_BAG_DURATION_MINUTES

    @property
    def max_time_mins(self):
        """
        Returns the maximum amount of minutes to be stored in blackboxes.
        """

        if ENV_BLACKBOX_MAX_TIME_MINS in os.environ:
            try:
                return float(os.environ[ENV_BLACKBOX_MAX_TIME_MINS])
            except Exception:
                self.once_logger.exception(
                    "bad_env_max_time_mins",
                    ENV_BLACKBOX_MAX_TIME_MINS + "'s value must be a number",
                )
        return DEFAULT_MAX_TIME_MINS

    @property
    def artifacts_path(self):
        """
        Returns the Path where artifacts are stored.
        Interface added to exchange this information with DatabagsAgentlet for
        monitoring the folder.

        @return a Path object or None if the blackbox is not enabled.
        """

        if not self.enabled:
            return None
        # Ensure we will store at most max_time_mins by making the databags agentlet keep
        # the corresponding amount of files, given the duration of each file
        # NOTE(diegobatt) This assumes all the databags were recorded with the same configuration
        max_files = int(math.ceil(self.max_time_mins / self.bag_duration_mins))

        return Path(
            path=self.rosbags_path,
            extension=BAG_EXTENSION,
            max_disk_gb=self.max_space_gb,
            max_files=max_files,
        )

    def cycle_rosbag(self, soft=False):
        """
        Call to stop current rosbag and restart a new one, if recording is ON.
        This is called from an interval thread.

        If param `soft` is used, this is just a _hint_ that it is a good time to
        cycle to next rosbag, for example when the link has just reconnected and
        it would be useful to upload batched data. (This argument is ignored for
        now).
        """

        if not self.enabled:  # Ignore when disabled; do not create rosbags
            return
        with self._mutex:
            if self._rosbag:
                self._stop_rosbag(self._rosbag)
            # Give the chance to print errors at least once per rosbag
            self.once_logger.reset_one("write")
            self.once_logger.reset_one("stop_rosbag")
            # If previous rosbag was not opened correctly or with an exception,
            # a new one starts anyway
            self._rosbag = self._start_rosbag()  # It can returns None

    def _cleanup_old(self):
        """
        Clean old rosbags that, for some reason (i.e. forced OS restart or process
        killed), were left open.
        # TODO(diegobatt): Rather than removing them, do a re-index and keep whatever
        # data they have.
        """

        with self._mutex:
            try:
                for bag in os.listdir(self.rosbags_path):
                    # If bag is not active, no need to consider it for cleanup
                    if not bag.endswith(ACTIVE_SUFFIX):
                        continue
                    bag_path = os.path.join(self.rosbags_path, bag)
                    last_modified = os.path.getmtime(bag_path)
                    # If the last modification timestamp is recent enough,
                    # don't consider it either
                    if last_modified > time.time() - CLEANUP_ACTIVE_BAG_SECONDS_OLD:
                        continue
                    # If it is an old enough active rosbag, remove it
                    os.remove(bag_path)
            except Exception as e:
                self.once_logger.exception(
                    "cleanup_old", "Failed to clean old active rosbags %s" % str(e)
                )

    def start(self):
        """
        Starts the blackbox; it will start continuously recording messages.
        Ignored if it was already recording.
        """

        if not self.enabled:  # Ignore when disabled; do not create rosbags
            return
        with self._mutex:
            if not self._ticker_thread:
                self._ticker_thread = Interval(
                    self.cycle_rosbag, self.bag_duration_mins * 60
                ).start()
            # ticker thread does an immediate call to cycle()

    def stop(self):

        if not self.enabled:  # Ignore (no expensive check for mutex below)
            return
        with self._mutex:
            if self._rosbag:
                self._stop_rosbag(self._rosbag)
                self._rosbag = None
            if self._ticker_thread:
                self._ticker_thread.stop()
                self._ticker_thread = None

    def mqtt_out_write(self, topic, mqtt_msg, sent=True):
        """
        Records a single outgoing MQTT message.

        TODO(herchu) / TBD:  If mqtt_msg is string, binary, protobuf or....
        """

        if not self.enabled:  # Ignore all writes
            return
        # First check if we are recording, to avoid serializing the message for
        # no reason.
        # Threading note: This is NOT a guarantee that self._rosbag will not be
        # None a few lines later when we actually acquire the lock, but it is
        # done earlier for performance reasons (no unnecessary locking when
        # blackbox is disabled)
        if self._rosbag:
            # Link.publish() receives both strings and bytearrays, any of them
            # can be called here
            data = mqtt_msg.encode() if isinstance(mqtt_msg, str) else mqtt_msg
            msg = InOrbitOut(topic=topic, data=data, sent=sent)
            with self._mutex:
                if self._rosbag:
                    try:
                        self._rosbag.write_message(INORBIT_MQTT_OUT_ROS_TOPIC, msg)
                    except Exception:
                        self.once_logger.exception("write", "Exception recording blackbox data")
