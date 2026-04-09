# Copyright (c) 2018, InOrbit, Inc.
# All rights reserved.
# Agentlet module that publishes camera images
# TODO (Flor_Grosso): generalize ros subscriptions/unsubscriptions properly on
# a parent class. Share this logic with localization agentlet for double laser
# support.
import threading

from util.overrides import overrides

from .agentlet import Agentlet
from .inorbit_pb2 import CameraMessage

MQTT_CAMERA_TOPIC_V2 = "ros/camera2"
ROS_CAMERA_TOPIC_DEFAULT = "camera/image_raw"
ROS_CAMERA_MSG_TYPE = "sensor_msgs/msg/Image"
ROS_CAMERA_COMPRESSED_MSG_TYPE = "sensor_msgs/msg/CompressedImage"

# Width and height of the captured image
DEFAULT_IMG_WIDTH = 320
DEFAULT_IMG_HEIGHT = 240

# Quality of compression of the jpeg image (0 = lowest, ..., 100 = highest)
JPG_QUALITY_DEFAULT = 5
# Default encoding for the processed images
OUT_ENCODING_DEFAULT = "mono8"

# Image publisher's rate [Hz]
MIN_PUBLISHING_RATE = 0.1
MAX_PUBLISHING_RATE = 2.0
DEFAULT_PUBLISHING_RATE = 1.0


class RosImageAgentlet(Agentlet):
    def __init__(self, uplink, ros):
        super(RosImageAgentlet, self).__init__(uplink)
        self._ros = ros
        # Dictionary of camera ids to configs (topic, rate, encoding,
        # quality, is_on).
        self._states["cameras_config"] = {}
        # Dictionary of camera topics to configs (rate, encoding,
        # quality, is_on) made from the state received. Used
        # internally.
        self._topics_config = {}
        # Dictionary of camera topic names to message types.
        self._msg_type = {}
        self._states["available_camera_topics"] = []
        # Rate used for publisher thread.
        self._publishing_rate = DEFAULT_PUBLISHING_RATE

        # Keeps the count of publish cycles.
        self._n_publish_cycles = 0

        # Publisher thread running state
        self._running = False
        # Trivial internal buffer for camera data. This dictionary is
        # indexed by camera topic
        self._last_image = {}

        # Condition to handle publisher's sleep/wake up timing
        self._condition = threading.Condition()

    @overrides(Agentlet)
    def load(self, runlevel):
        self._states["runlevel"] = runlevel

        try:
            from util.Image import Image

            global Image
        except Exception as e:
            self.once_logger.exception(
                "image_available", "Exception when loading Image package:" " " + str(e)
            )
            return False
        else:
            self.logger.info("Using %s as image processing package." % Image._PACKAGE)

        try:
            import sensor_msgs
            import sensor_msgs.msg

            global sensor_msgs
        except Exception as e:
            self.once_logger.exception("sensor_msgs_load", "Exception loading sensor_msgs.")
            return False

        # Dictionary of camera message types to camera message class.
        self._msg_type_to_class = {
            ROS_CAMERA_MSG_TYPE: sensor_msgs.msg.Image,
            ROS_CAMERA_COMPRESSED_MSG_TYPE: sensor_msgs.msg.CompressedImage,
        }

        self._ros.add_submodule("camera", subs=(self._create_ros_subs()))

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
        self._ros.remove_submodule("camera")
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

        if "cameras_config" in state.keys():
            config = state["cameras_config"]
            # Use the state received to make a dictionary of camera
            # configs indexed by topic name.
            topics_config = {
                config[id]["topic"]: config[id]
                for id in config.keys()
                if (config[id].get("topic") and config[id].get("is_on", True))
            }

            # Calculate publishing rate as the max value received,
            # with a ceiling of MAX_PUBLISHING_RATE to avoid higher looping
            # speeds.
            self._publishing_rate = self._calculate_publishing_rate(config)

            if self._states["loaded"]:
                self._update_ros_subs(topics_config)

            # Update state
            self._topics_config = topics_config
            self._states["cameras_config"] = state["cameras_config"]

        # Send a state update to the cloud
        self.publish_state(self.uplink, self._states)

    def _calculate_publishing_rate(self, cameras_config):
        """
        Calculates publishing rate as the max value received, within:

        MIN_PUBLISHING_RATE <= rate <= MAX_PUBLISHING_RATE

        If none of the values satisfy the condition, then the closest
        boundary is returned.
        """

        # If there is no config, return the default publishing rate.
        if not cameras_config:
            return DEFAULT_PUBLISHING_RATE

        # Get the maximum 'requested' rate
        max_req_rate = max(
            [cameras_config[id].get("rate", MIN_PUBLISHING_RATE) for id in cameras_config.keys()]
        )

        # Never allow more than MAX_... , or less than MIN)
        return min(MAX_PUBLISHING_RATE, max(MIN_PUBLISHING_RATE, max_req_rate))

    def _publish_loop(self):
        """
        Runs on a separate thread. Publishes images at a rate dependent
        on the runlevel.
        """

        self._running = True
        while self._running is True:
            try:
                self._publish_images_if_available()
            except Exception as e:
                self.once_logger.exception(
                    "camera_publish_if_available", "Exception publishing data."
                )

            # Throttle differently depending on the module runlevel
            self._condition.acquire()
            self._condition.wait(1.0 / self._publishing_rate)
            self._condition.release()

        self.logger.info("Publisher thread shutting down.")

    def _publish_images_if_available(self):
        """
        Publishes the latest recorded images if possible and appropriate.
        """

        # If there are no images at all, quit.
        if not self._last_image:
            return

        # Increment the counter with this new cycle
        self._n_publish_cycles += 1

        for camera_topic in self._last_image:
            image = self._last_image.get(camera_topic)
            img_rate = self._topics_config.get(camera_topic, {}).get(
                "rate", DEFAULT_PUBLISHING_RATE
            )

            # Throttle images that have a slower publishing rate than
            # the current publishing one.
            # NOTE (Flor_Grosso): it is assumed here that higher rates
            # are a common multiple of the slower ones.
            if img_rate and self._n_publish_cycles % (self._publishing_rate / img_rate) != 0:
                continue

            if image:
                [image_buffer, img_width, img_height] = self._ros_image_to_byte_array(
                    image, camera_topic
                )

                # If there is no image to publish, abort
                if not image_buffer:
                    continue

                # Build the protobuf message object and publish it
                data = CameraMessage()
                data.camera_id = camera_topic
                data.width = img_width
                data.height = img_height
                data.ts = self.get_ts()
                data.image = image_buffer
                self.uplink.publish_protobuf(MQTT_CAMERA_TOPIC_V2, data)

                self._last_image[camera_topic] = None

    def _ros_on_image(self, msg, camera_topic):
        """
        Callback for camera messages.
        """

        try:
            self._last_image[camera_topic] = msg
        except Exception as e:
            self.once_logger.exception(
                f"ros_callback for {camera_topic}",
                f"Exception receiving image data from: '{camera_topic}'.",
            )

    def _ros_image_to_byte_array(self, img_msg, camera_topic):
        """
        Converts ROS image to jpg format. Returns it as a byte array.
        """

        output_encoding = self._topics_config.get(camera_topic, {}).get(
            "output_encoding", OUT_ENCODING_DEFAULT
        )
        quality = self._topics_config.get(camera_topic, {}).get("quality", JPG_QUALITY_DEFAULT)
        width = self._topics_config.get(camera_topic, {}).get("img_width", DEFAULT_IMG_WIDTH)
        height = self._topics_config.get(camera_topic, {}).get("img_height", DEFAULT_IMG_HEIGHT)
        msg_type = self._msg_type.get(camera_topic, ROS_CAMERA_MSG_TYPE)
        brightness = self._topics_config.get(camera_topic, {}).get("brightness")
        contrast = self._topics_config.get(camera_topic, {}).get("contrast")

        try:
            im = Image()
            # Process images differently depending on the message type.
            # TODO (Flor_Grosso): add a 'no processing' flag to send
            # the image as is (output_encoding = 'passthrough',
            # no image resizing nor additional encoding).
            if msg_type == ROS_CAMERA_MSG_TYPE:
                im.fromImgMsg(img_msg, output_encoding)
            elif msg_type == ROS_CAMERA_COMPRESSED_MSG_TYPE:
                im.fromCompressedImgMsg(img_msg, output_encoding)

            im.resize((width, height))

            # Enhance image if brightness or contrast were set
            if brightness is not None or contrast is not None:
                brightness = brightness if brightness is not None else im.DEFAULT_BRIGHTNESS
                contrast = contrast if contrast is not None else im.DEFAULT_CONTRAST
                im.enhance(brightness, contrast)

            # Convert image to jpeg
            im.encode(False, quality)
            return [im.toBytes(), width, height]
        except Exception as e:
            self.once_logger.exception(
                f"cv_bridge for {camera_topic}",
                f"Exception processing image from: '{camera_topic}'.",
            )
            return [None, None, None]

    def _get_available_camera_topics(self):
        """
        Finds topics publishing ROS_CAMERA_MSG_TYPE and ROS_CAMERA_COMPRESSED_MSG_TYPE
        and returns a list of them.
        """

        self._decompressed_camera_topics = self._ros.get_topics_publishing(ROS_CAMERA_MSG_TYPE)
        self._compressed_camera_topics = self._ros.get_topics_publishing(
            ROS_CAMERA_COMPRESSED_MSG_TYPE
        )

        return self._compressed_camera_topics + self._decompressed_camera_topics

    def _set_initial_camera_topics(self):
        """
        Sets the initial camera topics for when the agentlet is first loaded
        """

        self._states["available_camera_topics"] = self._get_available_camera_topics()

        # Check if the state received is a default state, with camera
        # settings only and no topic configured.
        is_new_user = (
            len(self._states.get("cameras_config").keys()) == 1
            and "0" in self._states.get("cameras_config").keys()
            and not self._states.get("cameras_config").get("0", {}).get("topic")
        )

        # If there is no configuration provided or settings have no
        # topic defined, create sensible defaults trying to fetch
        # one camera with the first available image topic.
        if not self._states.get("cameras_config") or is_new_user:
            # One camera
            self._states["cameras_config"]["0"] = {}

            # Try to detect a topic, otherwise use a hardcoded value
            if not self._states["available_camera_topics"]:
                self.logger.warning(
                    f"No ROS camera topics available. Setting '{ROS_CAMERA_TOPIC_DEFAULT}' as "
                    "default."
                )
                topic = ROS_CAMERA_TOPIC_DEFAULT
            else:
                # If there are camera topics, set the first on the list as
                # the current one.
                topic = self._states["available_camera_topics"][0]

            # set-up this topic as default
            self._states["cameras_config"]["0"]["topic"] = topic
            self._topics_config[topic] = {}

            # Trigger a state update
            self.publish_state(self.uplink, self._states)

    def _create_ros_subs(self):
        """
        Creates a list of subscribers to ROS topics.
        """

        # First, set initial camera topics
        self._set_initial_camera_topics()

        # Make ROS subs list
        current_camera_topics = set(self._topics_config.keys())

        subs = []
        for topic in current_camera_topics:
            sub = self._create_camera_sub(topic)

            if sub:
                subs.append(sub)
        return subs

    def _create_camera_sub(self, camera_topic):
        """
        Creates a subscriber to a ROS topic where camera messages are
        published to.
        """

        self._last_image[camera_topic] = {}

        # Check if the compressed key was set or not
        if "compressed" not in self._topics_config[camera_topic].keys():
            msg_type = self._ros.get_msg_type_for(camera_topic)
            # TODO(MarianoCereda): If a message type is detected, use it to set the "compressed" flag.
        elif self._topics_config[camera_topic]["compressed"]:
            msg_type = ROS_CAMERA_COMPRESSED_MSG_TYPE
        else:
            msg_type = ROS_CAMERA_MSG_TYPE

        # If the topic type is not available use a default
        if msg_type is None:
            self.logger.warning(
                "Unable to find message type for "
                f"{camera_topic}. Using {ROS_CAMERA_MSG_TYPE} as default"
            )
            msg_type = ROS_CAMERA_MSG_TYPE

        # The type is available but it's not within the supported image
        # message types. Abort the subscription.
        if msg_type not in self._msg_type_to_class:
            self.logger.warning(f"Unsupported camera msg type {msg_type} for topic {camera_topic}")
            return None
        else:
            self._msg_type[camera_topic] = msg_type
            msg_class = self._msg_type_to_class.get(msg_type)
            
            return (
                camera_topic,
                msg_class,
                lambda msg, camera_topic=camera_topic: self._ros_on_image(msg, camera_topic),
            )

    def _update_ros_subs(self, topics_config):
        """
        Updates ros subscribers after a state update.
        """

        current_camera_topics = set(topics_config.keys())
        previous_camera_topics = set(self._topics_config.keys())

        deleted_topics = previous_camera_topics - current_camera_topics
        new_topics = current_camera_topics - previous_camera_topics

        # Remove listeners for deleted topics
        for topic in deleted_topics:
            self._ros.unregister_subscriber_to(topic, "camera")
            # Remove key from raw images store.
            self._last_image.pop(topic, None)
            self._msg_type.pop(topic, None)

        # Add new listeners
        for topic in new_topics:
            sub = self._create_camera_sub(topic)

            # Sanity check in case the submodule couldn't be created.
            if not sub:
                continue

            self._ros.add_submodule_listeners("camera", subs=[sub], pubs=[])
            self.once_logger.reset_all()
