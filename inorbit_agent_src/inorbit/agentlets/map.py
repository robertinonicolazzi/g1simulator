# Copyright (c) 2021, InOrbit, Inc.
# All rights reserved.
#
# Maps agentlet.
#
import hashlib
import io
import threading
import time
from builtins import int

import png
from util.concurrency import Interval
from util.overrides import overrides
from util.rate_limiter import RateLimiter

from .agentlet import Agentlet
from .inorbit_pb2 import MapMessage
from .inorbit_pb2 import MapRequest


ROBOT_FRAME = "base_link"
DEFAULT_ROS_MAP_TOPIC = "map"
ROS_MAP_MSG_TYPE = "nav_msgs/OccupancyGrid"
# Maximum size in pixels to choose to send a map each time it is received without
# waiting for the server to request it
MAX_INITIAL_MAP_SEND_SIZE = 1000 * 1000
# Map updates closer than every 1 minute are discarded
DEFAULT_MAP_MAX_RATE = 0.016
# Topic to listen for server requests to send full maps
MQTT_MAP_REQ = "ros/loc/mapreq"
# v2, for map in protobuf format
MQTT_MAP_TOPIC = "ros/loc/map2"
# How many seconds to keep maps in memory before truncating them to free memory
DEFAULT_ROS_MAP_TRUNCATE_TIME = 60
# Frame/Map id related constants
DEFAULT_MAP_FRAME_ID = DEFAULT_ROS_MAP_TOPIC
DEFAULT_CUSTOM_TOPIC = "inorbit/custom_data/0"
FRAME_ID_TOPIC_SUBSCRIBER_ID = "MapAgentlet frame_id"
MAP_ID_TOPIC_SUBSCRIBER_ID = "MapAgentlet map_id"
DEFAULT_SEND_MAP_DELAY = 0.5
# Module states constants for frame_id/map_id configuration
# map_config dict keys
CONFIG_FRAME_ID_SOURCE = "frame_id_source"
CONFIG_FRAME_ID_TOPIC = "frame_id_topic"
CONFIG_FRAME_ID_KEY = "frame_id_key"
CONFIG_MAP_ID_SOURCE = "map_id_source"
CONFIG_MAP_ID_TOPIC = "map_id_topic"
CONFIG_MAP_ID_KEY = "map_id_key"
CONFIG_SEND_MAP_DELAY = "map_delay"
# Frame id source kinds
CONFIG_FRAME_ID_SOURCE_MAP_MSG = "map_msg"
CONFIG_FRAME_ID_SOURCE_TOPIC = "topic"
# Map id source kinds
CONFIG_MAP_ID_SOURCE_MAP_TOPIC = "map_topic"
CONFIG_MAP_ID_SOURCE_TOPIC = "topic"


class RosMapAgentlet(Agentlet):
    def __init__(self, uplink, ros, custom_data_agentlet):
        super(RosMapAgentlet, self).__init__(uplink)
        self._ros = ros
        # CustomDataAgentlet instance to get frame changes trough the configured topic
        self._custom_data_agentlet = custom_data_agentlet
        # Current map frame ID, it changes when a map msg is received
        self._map_frame_id = DEFAULT_MAP_FRAME_ID
        # Current inorbit_frame_id. Requested by other modules through inorbit_frame_id
        # property
        # NOTE(MarianoCereda): The namespacing convention in ROS seems to add namespaces
        # to all frames in the tree starting from odom and going down, leaving map clear.
        # For this reason we are not adding the namespace in this case.
        self._inorbit_frame_id = DEFAULT_MAP_FRAME_ID
        # Current map_id, defaults to map topic
        self._map_id = DEFAULT_ROS_MAP_TOPIC
        # Whether the map has been already sent
        self._map_published = False
        # Lock to truncate map data after a short time to free memory.
        # Protects _current_map_msg and _current_map_truncate_at.
        self._clear_map_mutex = threading.Lock()
        # Stores a temporary copy of the last map message to allow the server
        # to request it if it needs it
        self._current_map_msg = None
        self._current_map_msg_topic = None
        # Timestamp in seconds when this map should be deleted to free memory
        self._current_map_truncate_at = None
        # Map checksum to be used by other modules
        self._map_checksum = hashlib.md5()
        # Frame id state variables
        self._frame_id_source = CONFIG_FRAME_ID_SOURCE_MAP_MSG
        self._frame_id_topic = DEFAULT_CUSTOM_TOPIC
        self._frame_id_key = None
        # map id state variables
        self._map_id_source = CONFIG_MAP_ID_SOURCE_MAP_TOPIC
        self._map_id_topic = DEFAULT_CUSTOM_TOPIC
        self._map_id_key = None
        # Variable to hold the timer thread that sends the map and its mutex
        self._timer_send_map = None
        self._timer_send_map_mutex = threading.Lock()
        # Defaut rate limiter for sending maps
        self._map_rate_limiter = RateLimiter(DEFAULT_MAP_MAX_RATE)
        # Initial state
        self._states["available_map_topics"] = []
        self._states["map_topic"] = None
        self._states["map_truncate_time"] = DEFAULT_ROS_MAP_TRUNCATE_TIME
        # Flag to prevent map uploads when a different map_topic is configured just before
        # the map image is uploaded. By default, don't upload an outdated map.
        self._states["map_prevent_outdated_upload"] = True
        # For safety, initialize the map clearing thread as None
        self._map_clearing_thread = None

    @overrides(Agentlet)
    def load(self, runlevel):
        self._states["runlevel"] = runlevel

        try:
            import nav_msgs
            import nav_msgs.msg

            global nav_msgs
        except Exception as e:
            self.once_logger.exception("nav_msgs_load", "Exception loading nav_msgs.")
            return False
        try:
            from rclpy import qos

            global qos

            global ROS_MAP_QOS_PROFILE_DEFAULT
            ROS_MAP_QOS_PROFILE_DEFAULT = qos.QoSProfile(
                history=qos.QoSHistoryPolicy.KEEP_LAST,
                depth=1,
                durability=qos.QoSDurabilityPolicy.TRANSIENT_LOCAL,
                reliability=qos.QoSReliabilityPolicy.RELIABLE,
            )
        except Exception as e:
            return False

        self._set_topics()

        sub = (
            self._states["map_topic"],
            nav_msgs.msg.OccupancyGrid,
            lambda msg, map_topic=self._states["map_topic"]: self._ros_on_map(msg, map_topic),
            ROS_MAP_QOS_PROFILE_DEFAULT,
        )
        self._ros.add_submodule("map", subs=[sub], pubs=[])
        # Register for upstream incoming map requests
        self.uplink.add_listener(MQTT_MAP_REQ, self._process_mapreq, 2)
        # Start map cleaning thread
        if not self._map_clearing_thread:
            self._map_clearing_thread = Interval(
                self._clear_map_memory, DEFAULT_ROS_MAP_TRUNCATE_TIME
            ).start()
        # If the frame_id source is topic, add a listener to custom_data
        if self._frame_id_source == CONFIG_FRAME_ID_SOURCE_TOPIC:
            self._custom_data_agentlet.add_listener(
                self._set_inorbit_frame_id,
                self._frame_id_topic,
                self._frame_id_key,
                subscriber=FRAME_ID_TOPIC_SUBSCRIBER_ID,
            )
        # Same for the map_id
        if self._map_id_source == CONFIG_MAP_ID_SOURCE_TOPIC:
            self._custom_data_agentlet.add_listener(
                self._set_map_id,
                self._map_id_topic,
                self._map_id_key,
                subscriber=MAP_ID_TOPIC_SUBSCRIBER_ID,
            )
        self._states["loaded"] = True
        self.publish_state(self.uplink, self._states)
        return True

    @overrides(Agentlet)
    def unload(self):
        self._ros.remove_submodule("map")
        self.uplink.remove_listener(MQTT_MAP_REQ)
        # Stop map clearing thread
        if self._map_clearing_thread:
            self._map_clearing_thread.stop()
            self._map_clearing_thread = None
        # If the frame_id was coming from a topic, remove the listener
        if self._frame_id_source == CONFIG_FRAME_ID_SOURCE_TOPIC:
            self._custom_data_agentlet.remove_listener(
                self._frame_id_topic, self._frame_id_key, subscriber=FRAME_ID_TOPIC_SUBSCRIBER_ID
            )
        # Same for the map_id
        if self._map_id_source == CONFIG_MAP_ID_SOURCE_TOPIC:
            self._custom_data_agentlet.remove_listener(
                self._map_id_topic, self._map_id_key, subscriber=MAP_ID_TOPIC_SUBSCRIBER_ID
            )
        self.uplink.publish(MQTT_MAP_TOPIC, None, qos=1, retain=True)
        self.once_logger.reset_all()
        self._states["loaded"] = False
        self.publish_state(self.uplink, self._states)
        return True

    @overrides(Agentlet)
    def set_runlevel(self, runlevel):
        self._states["runlevel"] = runlevel
        self.publish_state(self.uplink, self._states)

    """
    Called whenever a set_module command is received.
    """

    def set_state(self, state):
        if "map_topic" in state:
            # If map is changed while the module is loaded, then we need
            # to switch the currently subscribed topic and reset the
            # agentlet state so that it publishes the new map.
            old_topic = self._states["map_topic"]
            new_topic = state["map_topic"]
            if self._states["loaded"] and new_topic != old_topic:
                self._map_published = False
                self._map_rate_limiter.reset()
                sub = (
                    new_topic,
                    nav_msgs.msg.OccupancyGrid,
                    lambda msg, map_topic=new_topic: self._ros_on_map(msg, map_topic),
                )
                self._ros.update_subscriber_topic("map", old_topic, sub)
            self._states["map_topic"] = state["map_topic"]

        if "map_truncate_time" in state:
            self._states["map_truncate_time"] = state["map_truncate_time"]
        if "map_config" in state:
            self._states["map_config"] = state["map_config"]
            self._set_state_frame_id()
            self._set_state_map_id()
            # If map config changes, reset the rate limiter to accept a new max_rate
            # (This may publish an extra map even if max_rate did not change, but only once)
            self._map_rate_limiter = RateLimiter(
                state["map_config"].get("max_rate", DEFAULT_MAP_MAX_RATE)
            )
        if "map_prevent_outdated_upload" in state:
            self._states["map_prevent_outdated_upload"] = state["map_prevent_outdated_upload"]
        # Send a state update
        self.publish_state(self.uplink, self._states)
        # Reset logger to get new exception after changed state
        self.once_logger.reset_all()

    def _set_state_frame_id(self):
        """
        Abstracts the frame_id logic from set_state.
        """

        new_frame_id_source = self._states["map_config"].get(
            CONFIG_FRAME_ID_SOURCE, CONFIG_FRAME_ID_SOURCE_MAP_MSG
        )
        # Check that the frame_id_source is in the two only possible values
        if new_frame_id_source not in [
            CONFIG_FRAME_ID_SOURCE_MAP_MSG,
            CONFIG_FRAME_ID_SOURCE_TOPIC,
        ]:
            self.logger.error(
                "Invalid frame_id_source config: '%s', defauting to map msg", new_frame_id_source
            )
            new_frame_id_source = CONFIG_FRAME_ID_SOURCE_MAP_MSG
        new_frame_id_topic = self._states["map_config"].get(
            CONFIG_FRAME_ID_TOPIC, DEFAULT_CUSTOM_TOPIC
        )
        new_frame_id_key = self._states["map_config"].get(CONFIG_FRAME_ID_KEY)
        # If the new configuration is for map_msg, or if the topics change. We need
        # to unsubscribe the listener for the customDataAgentlet
        # NOTE: There is no harm in unsubscribing to a key that was never
        # subscribed in the first place, and it simplifies a lot the flow control
        # in this part
        if (
            new_frame_id_source == CONFIG_FRAME_ID_SOURCE_MAP_MSG
            or self._frame_id_topic != new_frame_id_topic
            or self._frame_id_key != new_frame_id_key
        ):
            self._custom_data_agentlet.remove_listener(
                self._frame_id_topic, self._frame_id_key, subscriber=FRAME_ID_TOPIC_SUBSCRIBER_ID
            )
        # If the new source is a topic, we need to subscribe the listener to the
        # CustomDataAgentlet
        # NOTE: There is no harm in subscribing again to the same key-value pair
        # and it simplifies a lot the flow control in this part
        if new_frame_id_source == CONFIG_FRAME_ID_SOURCE_TOPIC:
            self._custom_data_agentlet.add_listener(
                self._set_inorbit_frame_id,
                new_frame_id_topic,
                new_frame_id_key,
                subscriber=FRAME_ID_TOPIC_SUBSCRIBER_ID,
            )
        self._frame_id_source = new_frame_id_source
        self._frame_id_topic = new_frame_id_topic
        self._frame_id_key = new_frame_id_key

    def _set_state_map_id(self):
        """
        Abstracts the map_id logic from set_state.
        TODO(diegobatt): Extremely similar to _set_state_frame_id but with other variables.
        Think about how to re-use code.
        """

        new_map_id_source = self._states["map_config"].get(
            CONFIG_MAP_ID_SOURCE, CONFIG_MAP_ID_SOURCE_MAP_TOPIC
        )
        # Check that the map_id_source is in the two only possible values
        if new_map_id_source not in [CONFIG_MAP_ID_SOURCE_MAP_TOPIC, CONFIG_MAP_ID_SOURCE_TOPIC]:
            self.logger.error(
                "Invalid map_id_source config: '%s', defauting to map_topic", new_map_id_source
            )
            new_map_id_source = CONFIG_MAP_ID_SOURCE_MAP_TOPIC
        new_map_id_topic = self._states["map_config"].get(CONFIG_MAP_ID_TOPIC, DEFAULT_CUSTOM_TOPIC)
        new_map_id_key = self._states["map_config"].get(CONFIG_MAP_ID_KEY)
        # If the new configuration is for map_topic, or if the topics change. We need
        # to unsubscribe the listener for the customDataAgentlet
        if (
            new_map_id_source == CONFIG_MAP_ID_SOURCE_MAP_TOPIC
            or self._map_id_topic != new_map_id_topic
            or self._map_id_key != new_map_id_key
        ):
            self._custom_data_agentlet.remove_listener(
                self._map_id_topic, self._map_id_key, subscriber=MAP_ID_TOPIC_SUBSCRIBER_ID
            )
        # If the new source is a topic, we need to subscribe the listener to the
        # CustomDataAgentlet
        if new_map_id_source == CONFIG_MAP_ID_SOURCE_TOPIC:
            self._custom_data_agentlet.add_listener(
                self._set_map_id,
                new_map_id_topic,
                new_map_id_key,
                subscriber=MAP_ID_TOPIC_SUBSCRIBER_ID,
            )
        self._map_id_source = new_map_id_source
        self._map_id_topic = new_map_id_topic
        self._map_id_key = new_map_id_key

    def _set_topics(self):
        """
        Sets the map topic.
        If not set already, it will choose either a default or the first available
        """

        map_topics = self._ros.get_topics_publishing(ROS_MAP_MSG_TYPE) or []
        map_topics = list(filter(lambda x: "costmap" not in x, map_topics))
        map_topics.sort()
        self._states["available_map_topics"] = map_topics

        if not self._states["map_topic"]:
            if not self._states["available_map_topics"]:
                self.logger.warning(
                    "No ROS map_topic available. Setting '%s' as default.",
                    DEFAULT_ROS_MAP_TOPIC,
                )
                self._states["map_topic"] = DEFAULT_ROS_MAP_TOPIC
            else:
                # If there are available topics, set the first on the list
                # as the current one.
                self._states["map_topic"] = self._states["available_map_topics"][0]

    def _ros_on_map(self, msg, map_topic):
        """
        Callback for map messages.
        """

        # Record the map frame ID
        # Note(MarianoCereda): remove the initial '/' in case it's present.
        if msg.header.frame_id != "":
            if msg.header.frame_id[0] == "/":
                self._map_frame_id = msg.header.frame_id[1:]
            else:
                self._map_frame_id = msg.header.frame_id

        # If there is a throttling limit for map processing, this map message
        # may be ignored
        if not self._map_rate_limiter.accepts():
            return

        # Send the map_contents directly if the map is small enough
        # NOTE(diegobatt): If the message is too big, we start a negotiation with the
        # server to see if sending the msg data is worth the use of the network.
        # If the robot is offline, this negotiation won't take place and we just lose
        # the map data, and since if the robot is offline there is no network to take
        # care of, we "send" it anyways so it gets to the blackbox rosbag
        # TODO(diegobatt): Consider adding a cache to avoid "sending" the same map
        # (hash-wise) several times
        if (
            msg.info.width * msg.info.height <= MAX_INITIAL_MAP_SEND_SIZE
            or not self.uplink.connected
        ):
            # Send the whole map
            self._send_map(msg, msg.data, msg_topic=map_topic)
        else:
            # Send the map metadata and hash
            self._send_map(msg, msg_topic=map_topic)
        # Keep the map message around in case it is requested
        with self._clear_map_mutex:
            self._current_map_msg = msg
            self._current_map_msg_topic = map_topic
            # Set a timer to remove the map from memory unless it's set to 0
            if self._states["map_truncate_time"] > 0:
                self._current_map_truncate_at = time.time() + self._states["map_truncate_time"]
            else:
                self._current_map_truncate_at = None

    def _clear_map_memory(self):
        """
        HACK(adamantivm) Truncates the data element of a map message to force garbage collection
        This is a trick to avoid the last published message to remain in memory, which for some
        maps it could mean a lot of memory.
        @see https://inorbit.atlassian.net/browse/IO-1010

        Runs on a separate thread.
        """

        if not self._current_map_truncate_at or time.time() < self._current_map_truncate_at:
            return
        self.logger.info("Truncating map message to free memory")
        with self._clear_map_mutex:
            if self._current_map_msg is not None:
                self._current_map_msg.data = ()
                self._current_map_msg = None
                self._current_map_truncate_at = None

    def _process_mapreq(self, payload):
        """
        Callback for ingest requests to send the map data.
        """

        try:
            message = MapRequest()
            message.ParseFromString(payload)
            requested_label = message.label
            requested_hash = int(message.data_hash)
        except Exception as e:
            self.logger.warning("Failed to parse MapRequest")
            return

        self.logger.info("map upload requested: %s %s", requested_label, requested_hash)

        # TODO(adamantivm) Instead of having a saved map in memory, load the map on request
        # using the provided label as topic - https://inorbit.atlassian.net/browse/IO-1010

        # Check if we need to save a pointer to the data
        # NOTE(adamantivm) This is done in a separate step in order to make the critical
        # section very small - instead of locking during the whole map sending function.
        msgdata = None
        msg = None
        with self._clear_map_mutex:
            if (
                self._current_map_msg is not None
                and self.map_id == requested_label
                and hash(tuple(self._current_map_msg.data)) == requested_hash
            ):
                msg = self._current_map_msg
                # Make an explicit copy of the reference to the data, as the data may be replaced
                # inside the msg object by truncation - see _clear_map_memory
                msgdata = msg.data
                map_topic = self._current_map_msg_topic

        if msgdata is not None:
            self._send_map_async(msg, msgdata)
        else:
            self.logger.warning("Requested map doesn't match the last received map")

    def _send_map(self, msg, msgdata=None, is_update=False, msg_topic=None):
        """
        Sends the information about a map and optionally its contents.
        The map message data is passed as a separate variable to indicate
        that the map data needs to be actually sent.
        An is_update flag can be provided to inform in the message that this is an update
        message, aimed to correct a previous one, currently used for frame_id/map_id updates
        """

        if msgdata is not None:
            # Go through the occupancy grid and convert to a three-valued grayscale
            # 2D pixel array. (Note color values are 0, 1, 3 - reduced bitdepth)
            fn = lambda data: 0 if data == 100 else (3 if data == 0 else 1)
            pixels = self._map_to_pixels(msg, msgdata, fn)

            # Encode into PNG format
            f = io.BytesIO()
            w = png.Writer(msg.info.width, msg.info.height, greyscale=True, bitdepth=2)
            w.write(f, pixels)
            f.flush()
            # Compute checksum of the map data
            self._map_checksum.update(f.getvalue())

        # Build the protobuf message object and publish it
        data = MapMessage()
        data.width = msg.info.width
        data.height = msg.info.height
        data.data_hash = hash(tuple(msgdata) if msgdata is not None else tuple(msg.data))
        # NOTE(diegobatt): Label is deprecated in favor of map_id, kept for backwards
        # compatibility
        data.label = self.map_id
        data.map_id = self.map_id
        data.frame_id = self.inorbit_frame_id
        if msgdata is not None:
            data.pixels = f.getvalue()
        data.x = msg.info.origin.position.x
        data.y = msg.info.origin.position.y
        data.resolution = msg.info.resolution
        data.ts = self.get_ts()
        data.is_update = is_update

        # If the map_topic configuration changed and is different from the one just processed,
        # avoid sending the map. A map upload in this condition could lead to inconsistencies,
        # as the map data doesn't match the label or map_id fields indicated (these reflect the
        # current state).
        # NOTE(FlorGrosso): this may happen in any agentlet where the module state changed
        # while processing data for the previous configuration. Agents loaded by default
        # are more prone to this inconsistency as they start right away, before receiving
        # the module states from the server.
        # Consider designing a more generic approach that deals with this data / state
        # mismatch.
        if (
            self._states["map_prevent_outdated_upload"]
            and msg_topic is not None
            and msg_topic != self._states["map_topic"]
        ):
            self.logger.info(
                f"Processed map '{msg_topic}' is different from the current map configured "
                f"'{self._states['map_topic']}'"
            )
            return

        self.uplink.publish_protobuf(MQTT_MAP_TOPIC, data, qos=1, retain=True)
        if msgdata is not None:
            self.logger.info("map upload finished: %s %s", data.label, data.data_hash)
        self._map_published = True

    def _send_map_async(self, msg, msgdata=None, is_update=False, msg_topic=None):
        """
        Sends the map in a separate thread to avoid blocking the calling thread.
        """

        threading.Thread(target=self._send_map, args=(msg, msgdata, is_update, msg_topic)).start()

    def _send_map_with_delay(self, msg, msgdata=None, is_update=False, msg_topic=None):
        """
        Sends the map in a separate thread after a configured amount of seconds.
        """

        with self._timer_send_map_mutex:
            # If there is a timer waiting for execute, cancel it
            self._timer_send_map and self._timer_send_map.cancel()
            delay = self._states["map_config"].get(CONFIG_SEND_MAP_DELAY, DEFAULT_SEND_MAP_DELAY)
            self._timer_send_map = threading.Timer(
                delay, self._send_map, args=(msg, msgdata, is_update, msg_topic)
            )
            self._timer_send_map.start()

    def _map_to_pixels(self, msg, msgdata, fn):
        """
        Converts an OccupancyGrid map
        http://docs.ros.org/api/nav_msgs/html/msg/OccupancyGrid.html
        into a matrix of pixels.
        This method just creates the matrix and iterates it; the lambda function in
        the fn argument must care of creating each actual pixel (grayscal, rgb, etc)
        NOTE(adamantivm) The map data itself is passed as a separate argument in case
        the publisher thread needs to truncate the map while we're publishing.
        NOTE(diegobatt) This should be the same as the _map_to_pixels in the
        LocalizationAgentlet
        """

        # Go through the occupancy grid and convert to a three-valued grayscale
        # 2D pixel array.
        W = msg.info.width
        H = msg.info.height
        pixels = [None] * H
        for row in range(H):
            pixels[row] = [None] * W
            # Determine index into the map data
            mapI = row * W
            for col in range(W):
                # Determine the value
                pixels[row][col] = fn(msgdata[mapI])
                # Move to next pixel ix
                mapI += 1
        return pixels

    def _set_inorbit_frame_id(self, value):
        """
        Updates the _inorbit_frame_id value and trigger a map message if needed.
        """

        old_frame_id = self._inorbit_frame_id
        self._inorbit_frame_id = value
        # TODO(diegobatt): If the frame_id changed, this calls the unthrottled _send_map
        # directly, so if the the frame_id is changing erratically this might need its
        # own rate limiter.
        # HACK(diegobatt): The map is sent in a separated thread after a timer goes off,
        # This is because we want to separate the correction message from the original
        # message as to avoid race conditions in the server and to give a slack to the
        # map_id so as to pack the update messages if they are updated together within
        # a few milliseconds
        if old_frame_id != self._inorbit_frame_id and self._current_map_msg:
            msg = self._current_map_msg
            msg_topic = self._current_map_msg_topic
            self._send_map_with_delay(msg, is_update=True, msg_topic=msg_topic)

    def _set_map_id(self, value):
        """
        Updates the _map_id value and trigger a map message if needed.
        """

        old_map_id = self._map_id
        self._map_id = value
        # TODO(diegobatt): Same comment as in _set_inorbit_frame_id
        # HACK(diegobatt): Same comment as in _set_inorbit_frame_id
        if old_map_id != self._map_id and self._current_map_msg:
            msg = self._current_map_msg
            msg_topic = self._current_map_msg_topic
            self._send_map_with_delay(msg, is_update=True, msg_topic=msg_topic)

    @property
    def map_frame_id(self):
        """
        Returns the current map frame id value, derived from ROS maps information.
        This is called from other agentlets (such as Pose) to be used in the coordinates
        transformation from base_link to map
        """

        return self._map_frame_id

    @property
    def inorbit_frame_id(self):
        """
        Returns the current inorbit frame id value.
        Depending on the frame_id_source configuration, it can be either the map_frame_id
        or an arbitrary value from the topic
        """

        if self._frame_id_source == CONFIG_FRAME_ID_SOURCE_MAP_MSG:
            return self.map_frame_id
        else:  # frame_id_source == CONFIG_FRAME_ID_SOURCE_TOPIC
            return self._inorbit_frame_id

    @property
    def map_id(self):
        """
        Returns the current map_id value.
        Depending on the map_id_source configuration, it can be either the map topic
        or an arbitrary value from another topic.
        """

        if self._map_id_source == CONFIG_MAP_ID_SOURCE_MAP_TOPIC:
            return self._states["map_topic"] or DEFAULT_ROS_MAP_TOPIC
        else:  # map_id_source == CONFIG_MAP_ID_SOURCE_TOPIC
            return self._map_id

    @property
    def map_published(self):
        """
        Returns the publish state of the map.
        """

        return self._map_published
