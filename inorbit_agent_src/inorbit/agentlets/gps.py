# Copyright (c) 2023, InOrbit, Inc.
# All rights reserved.
#
# GPS agentlet. Handles GPS data for outdoor navigation.
#

from .agentlet import Agentlet
from .agentlet import RUNLEVEL_DEFAULT
from .agentlet import RUNLEVEL_FULL
from .agentlet import RUNLEVEL_SILENT
from .inorbit_pb2 import GpsFixMessage
from math import sqrt

from util.overrides import overrides
import threading
# TODO(elvio.aruta98): get_message should be imported in the RosAgentlet, and it should be the one
# that provides to the other agentlets the capability of getting the python class
# associated to a message type (String -> Python Class)
from rosidl_runtime_py.utilities import get_message

# ROS topics
GPS_DEFAULT_TOPIC = "gps/fix"
# GPS Fix message type
NAVSATFIX_MSG_TYPE = "sensor_msgs/msg/NavSatFix"
GPSFIX_MSG_TYPE = "gps_msgs/msg/GPSFix"

# MQTT topics
GPS_FIX_MQTT_TOPIC = "ros/loc/gps/fix"

# Covariance type
# COVARIANCE_TYPE_UNKNOWN: GPS does not provide any quality estimation and it's ok to
# assume large possible errors
COVARIANCE_TYPE_UNKNOWN = 0
# COVARIANCE_TYPE_APPROXIMATED:
# TODO(elvio.aruta98): this case is not very well documented, for now is OK to just assume
# there are values in the diagonal and treat it like diagonal_known or type_known
COVARIANCE_TYPE_APPROXIMATED = 1
# COVARIANCE_TYPE_DIAGONAL_KNOWN: Diagonal known allows having variance and standard deviation.
# Likely the most useful case, since we only need the values of the diagonal [0,4] related to
# longitude and latitude
COVARIANCE_TYPE_DIAGONAL_KNOWN = 2
# COVARIANCE_TYPE_KNOWN: Represents what should be reported if the GPS receiver actually
# outputs the 3x3 covariance matrix as a set of 9 numbers. (High Quality GPS)
COVARIANCE_TYPE_KNOWN = 3

DEFAULT_PUBLISHING_RATE = 1.0


class GPSAgentlet(Agentlet):
    def __init__(self, uplink, ros):
        super(GPSAgentlet, self).__init__(uplink)
        self._ros = ros

        # GPS Fix data buffer
        self._last_fix = None

        # Rate used for publisher thread.
        self._publishing_rate = DEFAULT_PUBLISHING_RATE

        # Available NavSatFix topics for the agent to subscribe to.
        self._states["available_navsatfix_topics"] = []

        # Available GPSFix topics for the agent to subscribe to.
        self._states["available_gpsfix_topics"] = []

        # NOTE (elvio.aruta98):
        # Agent 4.7.* was using another module states, before 4.8.* version the module_state for
        # the GPS topic had a different shape since only NavSatFix messages were supported.
        # instead of being self._states["gps_topic"] = {}, the format was:
        # self._states["navsatfix_topic"] = "topic" (String)
        # -----------------------------------------------------------------------------------------
        # GPS topic that the agent should subscribe to.
        # It is an object with the shape:
        # {
        #   topic: String,
        #   msg_type: String <- NAVSATFIX_MSG_TYPE or GPSFIX_MSG_TYPE
        # }
        self._states["gps_topic"] = {}

        # Flag variable for GPSFix msg availability
        self._gpsfix_msg_available = False

        # Flag variable for NavSatFix msg availability
        self._navsatfix_msg_available = False

        # Publisher thread running state
        self._running = False

        # Condition to handle publisher's sleep/wake up timing
        self._condition = threading.Condition()

    @overrides(Agentlet)
    def load(self, runlevel):
        self._states["runlevel"] = runlevel
        try:
            import gps_msgs
            import gps_msgs.msg
            global gps_msgs
            self._gpsfix_msg_available = True
        except Exception as e:
            self.once_logger.exception("gps_msgs", "Exception loading gps_msgs.")
            # It doesn't return False, since if these messages are not imported
            # the agentlet still works with NavSatFix messages (it will report less data)

        try:
            import sensor_msgs
            import sensor_msgs.msg
            global sensor_msgs
            self._navsatfix_msg_available = True
        except Exception as e:
            self.once_logger.exception("sensor_msgs", "Exception loading sensor_msgs.")
            # if GPSFix messages are not present and NavSatFix also could not be imported
            # it fails loading the agentlet
            if not self._gpsfix_msg_available:
                return False

        try:
            from rclpy import qos
            global qos
            # TODO: (elvio.aruta98) Review the QOS Profile
            global GPS_QOS_PROFILE_DEFAULT
            GPS_QOS_PROFILE_DEFAULT = qos.QoSProfile(
                history=qos.QoSHistoryPolicy.KEEP_LAST,
                depth=1,
                durability=qos.QoSDurabilityPolicy.VOLATILE,
                reliability=qos.QoSReliabilityPolicy.RELIABLE,
            )
        except Exception as e:
            return False

        self._ros.add_submodule("gps", subs=self._get_gps_subs(), pubs=[])

        # Start uplink publishing thread
        threading.Thread(target=self._publish_loop).start()

        self._states["loaded"] = True
        # Send a state update to the cloud
        self.publish_state(self.uplink, self._states)
        return True

    @overrides(Agentlet)
    def unload(self):
        # Shutdown publisher thread
        self._running = False
        # Remove ROS subscribers
        self._ros.remove_submodule("gps")
        self._states["loaded"] = False
        # Re-initialize exception reporting
        self.once_logger.reset_all()
        return True

    @overrides(Agentlet)
    def set_runlevel(self, runlevel):
        self._states["runlevel"] = runlevel
        self.wake_up_publisher(self._condition)

    def set_state(self, state):
        """
        Called whenever a set_module command is received.
        """
        if "gps_topic" in state:
            # If the GPS topic is changed while the module is loaded, then we need
            # to switch the currently subscribed topic and reset the agentlet state
            # NOTE(elvio.aruta98): .get() is needed in "old_" variables to avoid key error
            # since self._states["gps_topic"] could be an empty object {}
            old_topic = self._states["gps_topic"].get("topic")
            new_topic = state["gps_topic"]["topic"]
            new_msg_type = state["gps_topic"]["msg_type"]
            old_msg_type = self._states["gps_topic"].get("msg_type")
            # TODO(elvio.aruta98): handle the case when the topic is the same
            # but the msg_type has changed
            if self._states["loaded"] and new_topic != old_topic:
                msg_type = new_msg_type if old_msg_type != new_msg_type else old_msg_type
                msg_class = self._get_msg_class_by_msg_type(msg_type)
                if msg_class:
                    sub = (
                        new_topic,
                        msg_class,
                        self._ros_on_fix,
                        GPS_QOS_PROFILE_DEFAULT
                    )
                    self._ros.update_subscriber_topic("gps", old_topic, sub)
                else:
                    self.logger.warning(
                        f"""No ROS '{msg_type}' msg type available.
                            ROS suscribers will not be updated."""
                    )
            self._states["gps_topic"] = state["gps_topic"]
        # Send a state update
        self.publish_state(self.uplink, self._states)

    def _ros_on_fix(self, msg):
        """
        Called whenever a GPS message is received.
        """
        self._last_fix = msg

    def _get_gps_subs(self):
        self._set_initial_topics()
        topic_msg_type = self._states["gps_topic"]["msg_type"]
        topic_msg_class = self._get_msg_class_by_msg_type(topic_msg_type)
        subs = []
        if topic_msg_class:
            subs = [
                (
                    self._states["gps_topic"]["topic"],
                    topic_msg_class,
                    self._ros_on_fix,
                    GPS_QOS_PROFILE_DEFAULT
                )
            ]
        return subs

    def _get_available_topics(self, msg_type):
        topics = self._ros.get_topics_publishing(msg_type)
        return topics

    def _set_initial_topics(self):
        if self._navsatfix_msg_available:
            self._states["available_navsatfix_topics"] = self._get_available_topics(
                NAVSATFIX_MSG_TYPE)
        if self._gpsfix_msg_available:
            self._states["available_gpsfix_topics"] = self._get_available_topics(GPSFIX_MSG_TYPE)
        # If there is no configuration provided, create sensible defaults
        if self._states.get("gps_topic"):
            # If the topic is already set, use the same topic
            topic = self._states["gps_topic"]["topic"]
            msg_type = self._states["gps_topic"]["msg_type"]
        else:
            # trying to fetch topics that publish gps_msgs/msg/GPSFix messages
            # GPSFix messages are preferred over NavSatFix messages
            if self._states["available_gpsfix_topics"]:
                topic = self._states["available_gpsfix_topics"][0]
                msg_type = GPSFIX_MSG_TYPE
            # If no GPSFix message topics exist, use NavSatFix
            elif self._states["available_navsatfix_topics"]:
                topic = self._states["available_navsatfix_topics"][0]
                msg_type = NAVSATFIX_MSG_TYPE
            else:
                # If GPSFix and NavSatFix messages are not found, set a default topic
                self.logger.warning(
                    f"""No ROS '{GPSFIX_MSG_TYPE} or {NAVSATFIX_MSG_TYPE}' topics available.
                      Setting '{GPS_DEFAULT_TOPIC}' as default."""
                )
                topic = GPS_DEFAULT_TOPIC
                # Using NavSatFix msg as default
                msg_type = NAVSATFIX_MSG_TYPE

        # set-up this topic as default
        self._states["gps_topic"]["topic"] = topic
        # set-up the msg_type
        self._states["gps_topic"]["msg_type"] = msg_type

        # Trigger a state update
        self.publish_state(self.uplink, self._states)

    def _publish_gps_fix_if_available(self):
        """
        Publishes the latest gps fix update.
        """
        # If there are no coordinates at all, quit.
        if not self._last_fix:
            return

        fix = self._last_fix

        data = GpsFixMessage()
        data.ts = self.get_ts()
        data.latitude = fix.latitude
        data.longitude = fix.longitude
        data.altitude = fix.altitude

        # Calculates the accuracy of the fix
        if fix.position_covariance_type == COVARIANCE_TYPE_UNKNOWN:
            # Handled server side, this GPS fix could be totally wrong
            data.has_accuracy = False
            data.accuracy_meters = 0.0
        else:
            # TODO(elvio.aruta98): handle each case of covariance in a particular way
            # and not generically as is being done here.
            # COVARIANCE_TYPE_APPROXIMATED, COVARIANCE_TYPE_DIAGONAL_KNOWN ..
            data.has_accuracy = True
            # Takes the covariance from the diagonal, since this is a 3x3 matrix and it's reported
            # in row-major order longitude and latitude should be positions 0 and 4 (1,1 and 2,2)
            long_covariance = fix.position_covariance[0]
            lat_covariance = fix.position_covariance[4]
            # Covariance is in [m^2], transforms it to standard deviation by calculating
            # the square root
            long_standard_deviation = sqrt(long_covariance)
            lat_standard_deviation = sqrt(lat_covariance)
            # Average of standard deviations (lat, long deviations)
            accuracy_avg_standard_deviation = (lat_standard_deviation + long_standard_deviation) / 2
            data.accuracy_meters = accuracy_avg_standard_deviation

        # If the topic is GPSFix type, publish more data like bearing
        if self._states["gps_topic"]["msg_type"] == GPSFIX_MSG_TYPE:
            data.has_bearing = True
            data.bearing = fix.track

        self.uplink.publish_protobuf(GPS_FIX_MQTT_TOPIC, data)
        self._last_fix = None

    def _publish_loop(self):
        """
        Runs on a separate thread. Publishes GPS fixes at a rate dependent
        on the runlevel.
        """
        # TODO(elvio.aruta98): The agenlet as it is now doesn't have different runlevels, it will
        # run always at the same rate, consider to add run levels handling here
        self._running = True
        while self._running is True:
            try:
                self._publish_gps_fix_if_available()
            except Exception as e:
                self.once_logger.exception(
                    "_publish_gps_fix_if_available", "Exception publishing data."
                )

            # Throttle differently depending on the module runlevel
            self._condition.acquire()
            self._condition.wait(1.0 / self._publishing_rate)
            self._condition.release()

        self.logger.info("Publisher thread shutting down.")

    def _get_msg_class_by_msg_type(self, msg_type):
        """
        Returns the python class associated to the type of message
        """
        return get_message(msg_type)
