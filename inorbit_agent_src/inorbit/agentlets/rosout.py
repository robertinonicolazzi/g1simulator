# Copyright (c) 2018, InOrbit, Inc.
# All rights reserved.
# Agentlet module that publishes /rosout
import threading
import zlib

import agentlet
from inorbit_pb2 import RosOutMessage
from util.overrides import overrides

MAX_BYTES_PER_MSG = 2000

PUBLISHER_PERIOD_DEFAULT_RUNLEVEL = 1
PUBLISHER_PERIOD_MINIMAL_RUNLEVEL = 10

# V2, for rosout in protobuf format
MQTT_ROSOUT_TOPIC = "ros/rosout2"


class RosoutAgentlet(agentlet.Agentlet):
    def __init__(self, uplink, ros):
        super(RosoutAgentlet, self).__init__(uplink)
        self.ros = ros
        self._publisher_running = False

        # Flag to indicate that the published rosout mssgs has skipped
        # messages due to lack of space.
        self._has_skipped_msgs = False

        # Array of collected -and processed- rosout message within publishing
        # periods.
        self._last_rosout_msgs = []
        # String composed by concatenating items in self._last_rosout_msgs.
        self._last_msgs_concat = None
        # Last (individual) ros msg received, raw.
        self._last_msg = None
        # Counts the number of consecutive occurrences of a same message.
        self._last_rosout_msg_times = 0
        # InOrbit log verbosity levels to ROS verbosity levels.
        self._ros_verbosity_levels = {
            "DEBUG": 1,
            "INFO": 2,
            "WARN": 4,
            "ERROR": 8,
            "FATAL": 16,
        }
        # Set verbosity level to DEBUG by default.
        self._ros_verbosity_level = self._ros_verbosity_levels["DEBUG"]

        # Condition to handle publisher's sleep/wake up timing.
        self._condition = threading.Condition()

    @overrides(agentlet.Agentlet)
    def load(self, runlevel):
        self._states["runlevel"] = runlevel
        try:
            import rosgraph_msgs

            global rosgraph_msgs
        except Exception as e:
            self.once_logger.exception("rosgraph_msgs_load", "Exception loading rosgraph msgs.")
            return False

        self.ros.add_submodule(
            "rosout", subs=(("rosout", rosgraph_msgs.msg.Log, self._ros_on_rosout),)
        )

        # Start uplink publishing thread
        threading.Thread(target=self._publish_loop).start()

        self._states["loaded"] = True
        return True

    @overrides(agentlet.Agentlet)
    def unload(self):
        self.ros.remove_submodule("rosout")
        self._publisher_running = False
        self._states["loaded"] = False
        return True

    @overrides(agentlet.Agentlet)
    def set_runlevel(self, runlevel):
        self._states["runlevel"] = runlevel
        self.wake_up_publisher(self._condition)

    def set_state(self, state):
        """
        Called whenever a set_module command is received.
        """

        if "rosout_verbosity_level" in state:
            self._states["rosout_verbosity_level"] = state["rosout_verbosity_level"]
            if state["rosout_verbosity_level"] in self._ros_verbosity_levels:
                self._ros_verbosity_level = self._ros_verbosity_levels[
                    state["rosout_verbosity_level"]
                ]
            else:
                self._ros_verbosity_level = self._ros_verbosity_levels["DEBUG"]
                self.logger.exception(
                    "Unknown verbosity level "
                    + state["rosout_verbosity_level"]
                    + ". Setting it to DEBUG by default."
                )

        # Send a state update
        self.publish_state(self.uplink, self._states)

    def _ros_on_rosout(self, data):
        """
        Callback for rosout messages.
        """

        # TODO (Flor_Grosso) cache locally the dropped information.
        # If the user wants the missing information, go back Z time and
        # start sending all the information starting then.
        try:
            if self._should_append_message(data):
                self._append_message(data.msg)
        except Exception as e:
            self.logger.exception("Exception reading logs")

    def _should_append_message(self, data):
        """
        Returns True if message verbosity level is equal or higher than the
        configured verbosity level.
        """

        should_append = data.level >= self._ros_verbosity_level
        return should_append

    def _append_message(self, msg):
        """
        Appends a new message to the list of last messages. Checks the number of
        occurrences and increments the counter if it's repeated.
        """

        # If current msg is equal to the last one received, increment counter
        if self._last_rosout_msgs and msg == self._last_msg:
            self._last_rosout_msg_times += 1
            # Update last element with the latest count flag.
            self._last_rosout_msgs.pop()
            msg_to_append = msg + f" [{self._last_rosout_msg_times} times]"
        else:
            # Reset counter for different messages
            self._last_rosout_msg_times = 1
            msg_to_append = msg

        self._last_rosout_msgs.append(msg_to_append)
        self._last_msg = msg

    def _publish_loop(self):
        """
        Publishes rosout messages at a rate depending on the agentlet's runlevel.
        """

        self._publisher_running = True
        while self._publisher_running is True:
            try:
                self._publish_msgs_if_available()
            except Exception as e:
                self.logger.exception("Exception publishing data.")

            self._condition.acquire()
            # Throttle differently depending on the module runlevel
            if self.get_runlevel() == agentlet.RUNLEVEL_DEFAULT:
                self._condition.wait(PUBLISHER_PERIOD_DEFAULT_RUNLEVEL)
            # If the runlevel is not recognized, publish at minimum
            else:
                self._condition.wait(PUBLISHER_PERIOD_MINIMAL_RUNLEVEL)
            self._condition.release()

        self.logger.info("Publisher thread shutting down.")

    def _publish_msgs_if_available(self):
        """
        Publishes the latest rosout messages if possible and appropriate.
        """

        if self._last_rosout_msgs:
            [msgs, has_skipped_msgs] = self._make_compressed_msg()

            # If there are no msgs to publish, abort
            if msgs:
                # Build the protobuf message object and publish it
                data = RosOutMessage()
                data.ts = self.get_ts()
                data.log = msgs
                data.has_skipped_msgs = has_skipped_msgs
                self.uplink.publish_protobuf(MQTT_ROSOUT_TOPIC, data)

            # Clean up vars for next publish task
            self._last_rosout_msgs = []
            self._last_msg = None
            self._last_rosout_msg_times = 1

    def _make_compressed_msg(self):
        """
        Returns the collected rosout messages, concatenated and compressed.
        Checks message length and returns a maximum message size defined by
        MAX_BYTES_PER_MSG.
        """

        msgs = "\n".join(self._last_rosout_msgs)
        msgs_len = len(msgs)

        if msgs_len > MAX_BYTES_PER_MSG:
            return [zlib.compress(msgs[:MAX_BYTES_PER_MSG]), True]

        return [zlib.compress(msgs), False]
