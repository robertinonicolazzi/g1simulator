# Copyright (c) 2018, 2020, InOrbit, Inc.
# All rights reserved.
# CustomData agentlet.
# Depends on the ROS module.
#
# NOTE(herchu) Some of this agentlet's functionality is now emulated
# in a new (experimental) job, job-rosbag-importer (under /ingest):
# Key-values string parsing, possible rate limiting and reporting.
import os
import threading
import time
import zlib

from util.overrides import overrides

from .agentlet import Agentlet
from .agentlet import RUNLEVEL_SILENT
from .inorbit_pb2 import CustomDataMessage
from .inorbit_pb2 import KeyValueCustomElement

# Custom data publisher's period for different runlevels
# (seconds)
PUBLISHER_PERIOD_DEFAULT_RUNLEVEL = 10

MQTT_CUSTOM_DATA_TOPIC = "custom"

# Element types
TYPE_KEY_VALUE = "key_value"
TYPE_TEXT_FILE = "text_file"
TYPE_IMAGE = "image"
TYPE_DIAGNOSTICS = "diagnostics"

"""
KEY VALUES CONFIG
"""
DEFAULT_KV_ROS_TOPIC = "inorbit/custom_data"
DEFAULT_KV_CUSTOM_FIELD = "0"

# Default data key to use when key-value pairs come without one
DEFAULT_DATA_KEY = "__default__"

# Data sampling modes
# regular: always sent every 10 seconds, used for telemetry
SAMPLING_MODE_REGULAR = "regular"
# diff: sent every 10 seconds only if the value changed or after
# MAX_INTERVAL since the last publication.
SAMPLING_MODE_DIFF = "diff"
# event: sent new value as soon as it is received
# It uses RobotEvents agentlet (events.py)
SAMPLING_MODE_EVENT = "event"

# By default use sampling mode 'regular'
DEFAULT_SAMPLING_MODE = SAMPLING_MODE_REGULAR

# Diff mode: send updated keys at least every these number of seconds.
# Special value -1 = never.
DEFAULT_MAX_INTERVAL_DIFF = 3600
# Event mode: if event keeps repeating with the same value, send as a
# new event after this number of seconds. Special value -1 = never.
DEFAULT_MAX_INTERVAL_EVENT = -1

# Maximum number of bytes allowed per custom data message by default
MAX_BYTES_PER_KEY_VAL_MSG = 5000

"""
TEXT FILES CONFIG
"""
# Maximum number of bytes allowed per text file blob.
MAX_BYTES_PER_BLOB = 20000
DEFAULT_BYTES_PER_BLOB = MAX_BYTES_PER_KEY_VAL_MSG

# Dictionary of text file settings, each of them containing params used
# for validating args received from the cloud (bytes_per_msg)
VALID_TEXT_FILE_ARGS = {
    "bytes_per_msg": {
        "type": int,
        "min": 1,
        "max": MAX_BYTES_PER_BLOB,
    }
}

# Read order for text files
TAIL = "tail"
HEAD = "head"
DEFAULT_READ_ORDER = TAIL

"""
IMAGES CONFIG
"""
# Width and height of the captured image
DEFAULT_IMG_WIDTH = 320
DEFAULT_IMG_HEIGHT = 240

# Quality of compression of the jpeg image (0 = lowest, ..., 100 = highest)
DEFAULT_JPG_QUALITY = 30

GENERAL_IMG_CONFIG_KEY = "custom_image_config"
BY_FIELD_IMG_CONFIG_KEY = "custom_image_config_by_custom_field"

# Dictionary of image settings, each of them containing params used for
# validating args received from the cloud (type, range for numbers)
VALID_IMG_ARGS = {
    "width": {
        "type": int,
        "min": 1,
    },
    "height": {
        "type": int,
        "min": 1,
    },
    "jpg_quality": {
        "type": int,
        "min": 0,
    },
    "skip_conversion": {
        "type": bool,
    },
}


class CustomDataAgentlet(Agentlet):
    def __init__(self, uplink, ros, diagnostics, events):
        super(CustomDataAgentlet, self).__init__(uplink)
        self._ros = ros
        self._diagnostics = diagnostics
        self._events = events

        # Dictionary of custom data sources objects, indexed by id
        self._custom_data_sources = {}

        # Array of custom data sources objects, containing element id,
        # name, type and path
        self._states["custom_data_sources"] = []

        # Custom data publisher thread running state
        self._custom_data_publisher_running = False

        # Condition to handle publisher's sleep/wake up timing
        self._condition = threading.Condition()

        # ----------------------------------- KEY VALUE PAIRS ------------------------------------

        # Flag to indicate if ROS key-value pairs should be supported or not.
        self._ros_key_values_enabled = True

        # Store latest custom data elements in a dictionary with:
        # keys= customfield, value= custom data for this element field
        # depending on the type:
        self._last_kv = {}

        # List of published key value data, indexed by key. Each key
        # contains two fields:
        # key: {
        #   value: <String>,
        #   ts: <ts when last published>
        # }
        # NOTE: the list of keys is preserved throughout the agentlet's
        # life.
        self._published_kv = {}

        # Dictionary of callbacks subscribed to the corresponding topics and keys
        # <topic, <key, <subscriber(default: None), callback>>>
        self._callbacks = {}

        # Mutex used to access _last_kv
        self._mutex = threading.Lock()

        # --------------------------------------- IMAGES ----------------------------------------
        # Dictionary of configuration details for image data. Includes output
        # width, height and jpg quality.
        self._states[GENERAL_IMG_CONFIG_KEY] = {
            "width": DEFAULT_IMG_WIDTH,
            "height": DEFAULT_IMG_HEIGHT,
            "jpg_quality": DEFAULT_JPG_QUALITY,
            "skip_conversion": False,
        }

        # Dictionary of configuration details for image data per data
        # source id.
        self._states[BY_FIELD_IMG_CONFIG_KEY] = {}

        # Dictionary of configuration details for image data per data
        # source id, built from merging self._states[GENERAL_IMG_CONFIG_KEY]
        # and self._states[BY_FIELD_IMG_CONFIG_KEY].
        self._images_config = {}

        # Flag to indicate if custom images should be processed or not.
        self._custom_images_enabled = True

        # -------------------------------------- TEXT FILES --------------------------------------
        # Dictionary of configuration details for image data. Includes output
        # width, height and jpg quality.
        self._states["custom_text_file_config"] = {}

        # Dictionary containing the time of latest publish [milliseconds]
        # of files, indexed per id
        self._file_last_publish_time = {}

    @overrides(Agentlet)
    def load(self, runlevel):
        self._states["runlevel"] = runlevel

        try:
            from std_msgs import msg as std_msgs

            global std_msgs
        except Exception as e:
            self.once_logger.warn(
                "std_msgs_load", "Can't support ROS key value pairs. " "Exception loading std_msgs."
            )
            self._ros_key_values_enabled = False

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

        # Only add ros submodule if necessary.
        if self._ros_key_values_enabled:
            self._ros.add_submodule("custom_data", subs=self._create_initial_ros_subs())

        # Start uplink publishing thread if runlevel is other than silent
        if self.get_runlevel() != RUNLEVEL_SILENT:
            self._launch_publisher_thread()

        self._states["loaded"] = True

        # Send a state update to the cloud
        self.publish_state(self.uplink, self._states)
        return True

    @overrides(Agentlet)
    def unload(self):
        # Shutdown custom data publisher thread
        self._custom_data_publisher_running = False
        if self._ros_key_values_enabled:
            # Remove ROS subscribers
            self._ros.remove_submodule("custom_data")
        self._states["loaded"] = False
        self.once_logger.reset_all()
        return True

    # TODO (FlorGrosso): Generalize this for all agentlets which allow setting
    # runlevel 0.
    @overrides(Agentlet)
    def set_runlevel(self, runlevel):
        # Do not allow setting runlevel if agentlet is not loaded
        if not self._states["loaded"]:
            # Save runlevel update and don't do anything else
            self._states["runlevel"] = runlevel
            return

        # Store previous runlevel
        last_runlevel = self.get_runlevel()

        # If old and new runlevels match, do nothing
        if runlevel == last_runlevel:
            return

        # Update runlevel
        self._states["runlevel"] = runlevel

        # If last runlevel is empty, do nothing
        if last_runlevel is None:
            return

        # If desired runlevel is silent, shutdown publisher thread
        if runlevel == RUNLEVEL_SILENT:
            self._shutdown_publisher_thread()

        # If desired run level is non silent and last runlevel was silent,
        # spawn publisher thread
        elif last_runlevel == RUNLEVEL_SILENT:
            # TODO(Flor_Grosso): note that if a previous publisher thread
            # hasn't shutdown yet there could be more than one publisher
            # running for a moment
            self._launch_publisher_thread()

        # The agentlet will be switching between publishing runlevels. Wake up
        # publisher thread to change publishing rate immediately.
        else:
            self.wake_up_publisher(self._condition)

    def set_state(self, state):
        """
        Called whenever a set_module command is received.
        """

        update_imgs_config = False
        republish_imgs = False

        if "custom_data_sources" in state.keys():
            # Create a dictionary of data sources, indexed per id
            updated_custom_data_sources = {
                d.pop("id"): d for d in map(dict, state["custom_data_sources"])
            }

            # Apply default configs in case data sources have a missing
            # field (e.g. topic name)
            updated_custom_data_sources = self._apply_defaults(updated_custom_data_sources)
            # Update ROS subscribers if needed.
            if self._states["loaded"]:
                self._update_sources(updated_custom_data_sources)

            self._states["custom_data_sources"] = state["custom_data_sources"]

            # Update internal dictionary
            self._custom_data_sources = updated_custom_data_sources
            # Check if an image data source was added and update config
            # for it.
            update_imgs_config = True

        if GENERAL_IMG_CONFIG_KEY in state.keys():
            self._states[GENERAL_IMG_CONFIG_KEY].update(state[GENERAL_IMG_CONFIG_KEY])
            # Update image data source config based on updates for
            # default config. Trigger an image update afterwards.
            update_imgs_config = True
            republish_imgs = True

        if BY_FIELD_IMG_CONFIG_KEY in state.keys():
            self._states[BY_FIELD_IMG_CONFIG_KEY] = state[BY_FIELD_IMG_CONFIG_KEY]
            # Update image data source config based on updates. Trigger
            # an image update afterwards.
            update_imgs_config = True
            republish_imgs = True

        if update_imgs_config:
            self._build_images_config()
        if republish_imgs:
            self._reset_last_publish_time_for(TYPE_IMAGE)

        if "custom_text_file_config" in state.keys():
            self._reset_once_logger_count_for("txt_file_args_custom_field", TYPE_TEXT_FILE)

            self._states["custom_text_file_config"] = state["custom_text_file_config"]
            # Clean up last publish time for images
            self._reset_last_publish_time_for(TYPE_TEXT_FILE)

        # Send a state update to the cloud
        self.publish_state(self.uplink, self._states)

    def _on_custom_data(self, data, topic):
        """
        Callback for custom data messages.
        """

        # TODO (Flor_Grosso): support more element types other than
        # key-value pairs.
        self._process_key_value_pair_message(data, topic)

    def _publish_loop(self):
        """
        Runs on a separate thread. Publishes custom data at a rate dependent
        on the runlevel.
        """

        self._custom_data_publisher_running = True
        while self._custom_data_publisher_running:
            try:
                self._publish_custom_data_if_available()
            except Exception as e:
                self.once_logger.exception("custom_data_publish", "Exception publishing data")

            self._condition.acquire()
            # Throttle differently depending on the module runlevel
            self._condition.wait(PUBLISHER_PERIOD_DEFAULT_RUNLEVEL)
            self._condition.release()

        self.logger.info("Publisher thread shutting down")

    def _publish_custom_data_if_available(self):
        """
        Publishes the latest custom data values if possible and appropriate.
        """

        for custom_field, source in self._custom_data_sources.items():
            # Build the protobuf message object and publish it
            data = CustomDataMessage()
            data.custom_field = custom_field
            data.ts = self.get_ts()

            if source["type"] == TYPE_KEY_VALUE:
                # ROS - dependent message
                payload = self._create_key_value_msg(custom_field)

                # If payload could not be created, skip publishing it
                if not payload:
                    continue

                data.key_value_payload.pairs.extend(payload)

            elif source["type"] == TYPE_TEXT_FILE:
                path = source.get("path")

                # If the file hasn't changed since the last publishing,
                # avoid resending it.
                if not self._was_file_updated(path, custom_field):
                    continue

                file_data = self._load_file_data(path, custom_field)

                # If file could not be read or data size is greater than
                # MAX_BYTES_PER_BLOB, skip publishing the message
                if not file_data["data"] or file_data["blob_size"] > MAX_BYTES_PER_BLOB:
                    continue

                data.text_file_payload_2.data = file_data["data"]
                data.text_file_payload_2.total_file_size = file_data["total_file_size"]
                data.text_file_payload_2.blob_size = file_data["blob_size"]
                data.text_file_payload_2.blob_offset = file_data["blob_offset"]

                self._file_last_publish_time[custom_field] = time.time()

            elif source["type"] == TYPE_IMAGE:
                # Process custom images if enabled only
                if not self._custom_images_enabled:
                    continue
                path = source.get("path")

                if not self._was_file_updated(path, custom_field):
                    continue

                image_data = self._load_image_data(path, custom_field)
                if not image_data:
                    continue

                data.image_payload = image_data
                self._file_last_publish_time[custom_field] = time.time()

            # TODO(Flor_Grosso): Add support for key value pairs from ROS
            # diagnostics, once the Diagnostics module is enabled for
            # ROS 2.
            # elif source['type'] == TYPE_DIAGNOSTICS:
            #     # Key to look for in ROS DiagnosticStatus/KeyValue message
            #     key = source.get('diagnostics_key')

            #     # Filter diagnostics message to get the key-value pair
            #     # as a dictionary indexed by the given key.
            #     diagnostics = self._diagnostics.get_diagnostics_value(
            #         source.get('diagnostics_name'), key)

            #     if not diagnostics:
            #         continue

            #     diagnostics_value = diagnostics.get(key)
            #     # Avoid publishing pairs with no value.
            #     if not diagnostics_value:
            #         continue

            #     # Send the source name as a more representative label other
            #     # than the raw diagnostics key
            #     data.diagnostics_payload.label = source.get('name')
            #     data.diagnostics_payload.key = key
            #     data.diagnostics_payload.value = diagnostics_value
            else:
                continue

            self.uplink.publish_protobuf(MQTT_CUSTOM_DATA_TOPIC, data)

    def _process_key_value_pair_message(self, data_element, topic):
        """
        Processes a key value pair data message received.
        """

        msg = data_element.data
        # If message is empty, do nothing
        if not msg:
            return

        # Get the data which data sources point to this topic and store the
        # data indexed by custom_field
        custom_field = self._key_value_topic_to_custom_field(topic)

        if custom_field is None:
            return

        # Check if a key should be forced for this custom_field. This is handy for those cases
        # where the custom data message published by the robot is missing a key.
        if not self._custom_data_sources[custom_field].get("force_key"):
            key, value = self._extract_key_value(msg)
        else:
            # TODO (Flor_Grosso): all msgs with no key will end up going to
            # a same __default__ value. Consider adding a different ID for
            # them (making sure the same data does not get published under
            # different ids)
            key = self._custom_data_sources[custom_field]["force_key"]
            value = msg

        # Events handling.
        send_event = False

        with self._mutex:
            if self._last_kv.get(custom_field) is None:
                self._last_kv[custom_field] = {}

            if self._published_kv.get(custom_field) is None:
                self._published_kv[custom_field] = {}

            # Calculate whether the k/v should be sent out now as an event
            send_event = self._should_publish_kv(custom_field, key, value, True)
            # If a callback is registered for that key, call it with its value
            callbacks = self._callbacks.get(topic, {}).get(key, {})
            for subscriber, callback in iter(callbacks):
                try:
                    callback and callback(value)
                except Exception as e:
                    self.once_logger.exception(
                        (
                            f"Callback failed for topic {topic}, key {key}"
                            "and subscriber {subscriber}: {e}"
                        )
                    )
            self._last_kv.get(custom_field)[key] = value

            # Keep historic data of sent events
            if send_event:
                self._published_kv[custom_field][key] = {
                    "value": value,
                    "ts": self.get_ts(),
                }

        # If necessary, send the event using the RobotEvents agentlet
        if send_event:
            self._events.put_key_value(key, value, custom_field)

    def process_key_value_pair_from_api(self, key, value):
        """
        Processes a key value pair data message received through the agent api
        TODO(adamantivm) Change this to use _process_key_value_pair_message
        so that it will support sampling modes, etc.
        """

        # If message is empty, skip it
        if not value:
            return

        # Use a default key if none is provided
        if not key:
            key = DEFAULT_DATA_KEY

        with self._mutex:
            self._last_kv[DEFAULT_KV_CUSTOM_FIELD][key] = value

    def _create_key_value_msg(self, custom_field):
        """
        Creates a key value pair protobuf message.
        """

        # Array of key value pairs to return
        key_value_array = []
        accumulated_size = 0

        max_bytes_per_key_val_msg = self._custom_data_sources.get(
            custom_field, {}).get("max_bytes_per_key_val_msg", MAX_BYTES_PER_KEY_VAL_MSG)

        with self._mutex:
            custom_keys = self._last_kv.get(custom_field, {}).keys()

            for key in custom_keys:
                value = self._last_kv[custom_field][key]

                # Check whether this key/value pair needs to be published
                # according to the sampling options.
                if not self._should_publish_kv(custom_field, key, value, False):
                    continue

                # If current value is None, don't include it on the message
                if value is None:
                    continue

                bytes_in_string = self._bytes_in_string(key + value)

                if bytes_in_string is None:
                    self.once_logger.warn(
                        "computing_bytes_length_" + key,
                        f"Could not compute bytes length for '{key}'. " "Skipping publication.",
                    )
                    continue

                accumulated_size += bytes_in_string

                if accumulated_size > max_bytes_per_key_val_msg:
                    self.once_logger.warn(
                        "MAX_BYTES_PER_KEY_VAL_MSG", "MAX_BYTES_PER_KEY_VAL_MSG exceeded."
                    )
                    break

                item = KeyValueCustomElement()
                item.key = key
                item.value = value
                key_value_array.append(item)

                # Keep historic data of a published key, including its
                # value and ts when published. DON'T DELETE KEYS after
                # publishing them
                self._published_kv.get(custom_field)[key] = {"value": value, "ts": self.get_ts()}

            # Clean up the last key values to start with fresh values on
            # the next loop
            self._last_kv[custom_field] = {}

        return key_value_array

    def _load_file_data(self, file_path, custom_field):
        """
        Reads file data from the given path. Returns an object with the blob
        content and indicators such as file size, blob size and blob offset.
        """

        read_order = (
            self._states["custom_text_file_config"]
            .get(custom_field, {})
            .get("read_order", DEFAULT_READ_ORDER)
        )
        bytes_per_msg = self._calculate_bytes_per_msg_from(
            self._states["custom_text_file_config"].get(custom_field, {}).get("bytes_per_msg"),
            file_path,
            custom_field,
        )

        # Object to return with default settings
        file_data = {"data": None, "total_file_size": 0, "blob_size": 0, "blob_offset": 0}
        try:
            with open(os.path.expanduser(file_path), "rb") as in_file:
                file_data["total_file_size"] = os.fstat(in_file.fileno()).st_size
                # Read the last N bytes if the read order is in tail
                # mode and the bytes to read are less than the total
                # file size. If not, read in head mode.
                if read_order == TAIL and file_data["total_file_size"] > bytes_per_msg:
                    in_file.seek(-bytes_per_msg, os.SEEK_END)
                    data = in_file.read()
                    file_data["blob_offset"] = file_data["total_file_size"] - bytes_per_msg
                else:
                    data = in_file.read(bytes_per_msg)
            file_data["blob_size"] = len(data)

        except Exception as e:
            self.once_logger.exception(
                custom_field,
                "Can't read '{:s}' data at '{:s}'.".format(
                    self._custom_data_sources[custom_field]["name"], file_path if file_path else ""
                ),
            )
            return file_data

        # Compress data with a default level (6).
        # This goes from 0 to 10 (lowest -> highest)
        file_data["data"] = zlib.compress(data)
        return file_data

    def _load_image_data(self, image_path, custom_field):
        """
        Reads file data from the given path.
        """

        try:
            # Validate configuration params first
            if not self._is_img_config_valid(custom_field):
                return None

            width = self._images_config[custom_field]["width"]
            height = self._images_config[custom_field]["height"]
            quality = self._images_config[custom_field]["jpg_quality"]
            skip_conversion = self._images_config[custom_field]["skip_conversion"]

            im = Image()
            source_fmt = im.open(image_path)

            # If the user has decided to keep an encoding which is not supported,
            # abort image processing and update last publish time to avoid
            # attempting to read the file once again.
            if skip_conversion and not im.isFormatSupported(source_fmt):
                self.logger.warning(
                    f"Exception processing {image_path}: output format "
                    f"'{source_fmt}' not supported."
                )
                self._file_last_publish_time[custom_field] = time.time()
                return None

            # Resize image to IMG_WIDTHxHEIGHT
            im.resize((width, height))
            # encode image to the desired format with a given quality
            im.encode(skip_conversion, quality)

            return im.toBytes()
        except Exception as e:
            self.once_logger.exception(
                custom_field,
                "Exception processing '{:s}' image at '{:s}'.".format(
                    self._custom_data_sources[custom_field]["name"],
                    image_path if image_path is not None else "",
                ),
            )
            return None

    def _bytes_in_string(self, input_string, encoding="utf-8"):
        """
        Calculates the number of bytes of a given string, with unicode
        encoding as default.
        """

        try:
            return len(input_string.encode(encoding))
        except UnicodeDecodeError:
            try:
                # The encoding was not "utf-8", decode it first.
                return len(input_string.decode(encoding).encode(encoding))
            except Exception as e:
                # Use part of the string as the key for the once_logger.
                logger_id = input_string[:10] if len(input_string) > 9 else input_string
                self.once_logger.warn(
                    logger_id, "Unable to calculate the length of the string: " + str(e)
                )
                return None

    def _was_file_updated(self, path, custom_field):
        """
        Checks if a given file has been updated after the last publishing
        time. Returns True if so.
        """

        try:
            return os.path.getmtime(os.path.expanduser(path)) > self._file_last_publish_time.get(
                custom_field, 0
            )
        except Exception as e:
            self.once_logger.exception(
                custom_field,
                "Can't load data source '{:s}' at '{:s}'.".format(
                    self._custom_data_sources[custom_field]["name"],
                    path if path is not None else "",
                ),
            )
            return False

    def _launch_publisher_thread(self):
        """
        Starts uplink publishing thread.
        """

        threading.Thread(target=self._publish_loop).start()

    def _shutdown_publisher_thread(self):
        """
        Shutdowns publisher thread by setting its state to not running.

        TODO(Flor_Grosso): Consider implementing a way to make sure that thread
        is killed properly
        """

        self._custom_data_publisher_running = False

    def _create_initial_ros_subs(self):
        """
        Returns an array of subscribers to configured ROS topics where key
        values are published when the agentlet is first loaded.
        """

        subs = []
        for custom_field in self._custom_data_sources:
            # Look for custom data of type key value
            if self._custom_data_sources.get(custom_field, {}).get("type") == TYPE_KEY_VALUE:
                topic = self._custom_data_sources[custom_field].get("topic")

                if topic is not None:
                    subs.append(self._create_key_value_sub(topic))

        return subs

    def _create_key_value_sub(self, topic):
        """
        Returns a subscriber element to a ROS topic where a key value topic
        is published.
        """

        new_sub = (
            topic,
            std_msgs.String,
            lambda data, topic=topic: self._on_custom_data(data, topic),
        )

        return new_sub

    def _update_sources(self, updated_sources):
        """
        Updates file paths if necessary..
        TODO (Flor_Grosso): consider moving each element to a separate class to
        avoid performing clean ups manually on each case.
        """

        new_custom_fields = set(updated_sources.keys())
        old_custom_fields = set(self._custom_data_sources.keys())

        # Check for elements which might have been updated (intersection
        # between sets)
        persistent_elements = new_custom_fields & old_custom_fields
        for custom_field in persistent_elements:
            # Ignore if has not path field
            if "path" not in updated_sources[custom_field]:
                continue
            # Ignore those elements which remain unchanged.
            if (
                "path" in self._custom_data_sources[custom_field]
                and self._custom_data_sources[custom_field]["path"]
                == updated_sources[custom_field]["path"]
            ):
                continue
            else:
                # If file path was updated, reset last publish time and
                # reporting of exceptions
                self._file_last_publish_time[custom_field] = 0
                self.once_logger.reset_one(custom_field)

        # Now get a set of the topics, both on the latest state and the
        # one already stored
        new_topics = set(
            updated_sources[custom_field]["topic"]
            for custom_field in updated_sources.keys()
            if "topic" in updated_sources[custom_field]
        )
        old_topics = set(
            self._custom_data_sources[custom_field]["topic"]
            for custom_field in self._custom_data_sources.keys()
            if "topic" in self._custom_data_sources[custom_field]
        )

        # Identify if there are topics which no longer need a subscription
        # or if a subscription for a new topic is required
        deleted_topics = old_topics - new_topics
        added_topics = new_topics - old_topics

        for topic in added_topics:
            if topic is None:
                continue

            self._ros.add_submodule_listeners(
                "custom_data", subs=[self._create_key_value_sub(topic)], pubs=[]
            )

        # Remove listeners for deleted topics
        for topic in deleted_topics:
            self._ros.unregister_subscriber_to(topic, "custom_data")

    def _apply_defaults(self, updated_states):
        """
        Sets default values to missing fields on the updated states.
        """

        for custom_field in updated_states:
            # Key_value type
            # If no topic name is specified, then set it to
            # /inorbit/custom_data/<custom_field>
            if updated_states[custom_field].get("type") == TYPE_KEY_VALUE and not updated_states[
                custom_field
            ].get("topic"):
                # Ros2 does not allow topic name tokens to begin with a number
                if custom_field[0].isnumeric():
                    topic_name = DEFAULT_KV_ROS_TOPIC
                else:
                    topic_name = DEFAULT_KV_ROS_TOPIC + "/" + custom_field
                updated_states[custom_field]["topic"] = topic_name
        return updated_states

    def _reset_last_publish_time_for(self, element_type):
        """
        Sets last publish time to 0 for the specified element types under data
        sources
        """

        for custom_field, source in self._custom_data_sources.items():
            if source["type"] == element_type:
                self._file_last_publish_time[custom_field] = 0

    def _build_images_config(self):
        """
        Builds images config object merging general and per field settings.
        """

        # Get list of data source ids which are images
        custom_fields = [
            custom_field
            for custom_field in self._custom_data_sources
            if self._custom_data_sources[custom_field].get("type") == TYPE_IMAGE
        ]

        # There are no data sources of type images, no need to build
        # the config.
        if not custom_fields:
            return

        field_config = self._states[BY_FIELD_IMG_CONFIG_KEY]
        general_config = self._states[GENERAL_IMG_CONFIG_KEY]
        self._images_config = self.merge_states(custom_fields, field_config, general_config)

        # Clean up processing exceptions for images
        self.once_logger.reset_set(custom_fields)

    def _is_img_config_valid(self, custom_field):
        """
        Checks if config values for an image are valid (expected type and
        if number within a certain range).
        """

        img_config = self._images_config.get(custom_field)
        for config_name in img_config:
            config_value = img_config.get(config_name)
            # Check for all argument types and in the case of numeric
            # values check that they within the expected range.
            if config_name not in VALID_IMG_ARGS or not self.is_arg_valid(
                VALID_IMG_ARGS, config_name, config_value
            ):
                self.once_logger.warn(
                    custom_field,
                    f"Invalid config {config_name}:{config_value} for data source {custom_field}",
                )
                return False
        return True

    def _reset_once_logger_count_for(self, key_prefix, data_source_type):
        """
        Resets once logger count for a given data source type, building
        the keys as 'key_prefix' + 'custom_field'
        TODO (Flor_Grosso): consider making this a proper once logger method
        """

        # Get list of data source ids which are of the given type
        for custom_field in self._custom_data_sources:
            if self._custom_data_sources[custom_field].get("type") == data_source_type:
                self.once_logger.reset_one(key_prefix + custom_field)

    def _calculate_bytes_per_msg_from(self, bytes_per_msg, file_path, custom_field):
        """
        Computes bytes per msg for custom text files from a given arg value,
        checking whether it's of the right type (int) and within the
        expected range. In case any of these fails, a valid value is
        returned (it could be either the default setting or the closest
        valid value).
        """

        # If argument provided for blob size is not an integer, use
        # default settings.
        if not self._is_arg_type_valid(VALID_TEXT_FILE_ARGS, "bytes_per_msg", bytes_per_msg):
            self.once_logger.warn(
                "txt_file_args_field" + custom_field,
                f"Invalid blob size '{bytes_per_msg}' [Bytes] provided for {file_path}. "
                f"Reading {DEFAULT_BYTES_PER_BLOB} Bytes as default.",
            )
            return DEFAULT_BYTES_PER_BLOB

        # If argument provided for blob size is not within the expected
        # range, return the closest valid value.
        elif not self._is_arg_within_range(VALID_TEXT_FILE_ARGS, "bytes_per_msg", bytes_per_msg):
            valid_bytes_per_msg = self._get_valid_number_from(
                bytes_per_msg,
                VALID_TEXT_FILE_ARGS.get("bytes_per_msg", {}).get("min"),
                VALID_TEXT_FILE_ARGS.get("bytes_per_msg", {}).get("max"),
            )
            self.once_logger.warn(
                "txt_file_args_field" + custom_field,
                f"Invalid blob size '{bytes_per_msg}' [Bytes] provided for {file_path}. "
                f"Reading {valid_bytes_per_msg} Bytes.",
            )
            return valid_bytes_per_msg

        # Arg value passes all checks
        return bytes_per_msg

    def is_arg_valid(self, args_config, arg_name, arg_value):
        """
        Checks if a given argument is valid by confirming the type is as
        expected and, in case of numbers, it is within the expected range.
        TODO (Flor_Grosso): consider moving this method to a helper module,
        so that it can be used by other agentlets.
        """

        return self._is_arg_type_valid(
            args_config, arg_name, arg_value
        ) and self._is_arg_within_range(args_config, arg_name, arg_value)

    def _is_arg_type_valid(self, args_config, arg_name, arg_value):
        """
        Checks if a given argument type is the expected one.
        TODO (Flor_Grosso): consider moving this method to a helper module,
        so that it can be used by other agentlets.
        """

        return isinstance(arg_value, args_config[arg_name].get("type"))

    def _is_arg_within_range(self, args_config, arg_name, arg_value):
        """
        For a given argument, if it is of type number, it checks that its
        value is within the expected range.
        TODO (Flor_Grosso): consider moving this method to a helper module,
        so that it can be used by other agentlets.
        """

        if isinstance(arg_value, (int, float)) and not isinstance(arg_value, (bool)):
            min_value = args_config[arg_name].get("min")
            max_value = args_config[arg_name].get("max")

            if min_value is not None and max_value is not None:
                return arg_value >= min_value and arg_value <= max_value

            elif min_value is not None:
                return arg_value >= min_value

            elif max_value is not None:
                return arg_value <= max_value

        # TODO (Flor_Grosso): this is returning True for non numeric
        # values. Consider using this method just for numbers and
        # fail if the given arg_value is one.
        return True

    def _get_valid_number_from(self, input_number, min_value=None, max_value=None):
        """
        Given a number and a range defined by [min_value, max_value],
        it returns the closest value to the input number within that range.
        TODO (Flor_Grosso): consider moving this method to a helper module,
        so that it can be used by other agentlets.
        """

        if min_value is not None and max_value is not None:
            return min(max(min_value, input_number), max_value)

        elif min_value is not None:
            return max(min_value, input_number)

        elif max_value is not None:
            return min(input_number, max_value)

        return input_number

    def _should_publish_kv(self, custom_field, key, value, as_event=False):
        """
        Returns True if a key value pair needs to be published on the current
        iteration. The flag is computed based on the sampling mode and options
        configured for the custom_field.
        If as_event is True, this methods returns if the message should be
        published as event as soon as it is received. Otherwise, it returns
        if the message should be published during the regular publish loop.
        Requisite: mutex is locked
        """

        # Get sampling options for this custom_field
        sampling_mode = self._custom_data_sources.get(custom_field, {}).get(
            "sampling_mode", DEFAULT_SAMPLING_MODE
        )
        include_keys = self._custom_data_sources.get(custom_field, {}).get("include_keys", [])
        exclude_keys = self._custom_data_sources.get(custom_field, {}).get("exclude_keys", [])
        default_max_interval = (
            DEFAULT_MAX_INTERVAL_DIFF
            if sampling_mode == SAMPLING_MODE_DIFF
            else DEFAULT_MAX_INTERVAL_EVENT
        )
        max_interval = self._custom_data_sources.get(custom_field, {}).get(
            "max_interval", default_max_interval
        )

        # Current ts, in milliseconds
        ts_now = self.get_ts()

        # Common include / exclude processing for DIFF and EVENT modes
        if sampling_mode == SAMPLING_MODE_EVENT or sampling_mode == SAMPLING_MODE_DIFF:
            # If the key is not included in the sampling mode then it's regular
            # mode -> publish as regular (not event)
            if not self._is_key_included(key, include_keys, exclude_keys):
                return not as_event

            # Check the value's diff for any updates
            last_published_value = self._published_kv[custom_field].get(key, {}).get("value")
            was_kv_updated = value != last_published_value

            # Special value: if max_interval is -1 then only publish when changed regardless
            # of the time elapsed
            if not was_kv_updated and max_interval == -1:
                return False

            # Compute the elapsed time in ms since the last publication
            # of this value
            last_published_ts = self._published_kv[custom_field].get(key, {}).get("ts", ts_now)
            last_publish_interval = ts_now - last_published_ts

            if was_kv_updated or last_publish_interval > (max_interval * 1000):
                if sampling_mode == SAMPLING_MODE_EVENT:
                    return as_event
                else:
                    return not as_event
            else:
                # Default: don't send value if it wasn't updated and
                # there was no timeout
                return False

        # Default mode: regular -> publish in regular loop, not as event
        return not as_event

    def _is_key_included(self, key, include_keys, exclude_keys):
        """
        Helper function to determine if a key is included or not in the specified sampling mode
        based on the provided include and exclude lists.
        Returns True if the key is included and False if excluded
        """

        # There is an 'include_keys' array explicitly defined
        if len(include_keys) > 0:
            # The key isn't included on the sampling mode
            if key not in include_keys:
                return False

        # There is an 'exclude_keys' array explicitly defined
        # NOTE: if a key is in both 'include' and 'exclude' arrays,
        # then it will be taken as not included in the sampling mode
        if len(exclude_keys) > 0:
            # The key is excluded from the custom mode
            if key in exclude_keys:
                return False

        return True

    def _extract_key_value(self, string):
        """
        Returns a tuple with (key, value) from a string. By default it will search for an '='
        separating key and value. If it doesn't find it, it will use a DEFAULT_DATA_KEY and
        consider the string as the value.
        """

        if "=" in string:
            key, value = string.split("=", 1)
        else:
            # TODO (Flor_Grosso): all msgs with no key will end up going to
            # a same __default__ value. Consider adding a different ID for
            # them (making sure the same data does not get published under
            # different ids)
            key = DEFAULT_DATA_KEY
            value = string

        return (key, value)

    def _key_value_topic_to_custom_field(self, topic):
        """
        Returns the custom_field ('0', '1', ...) given a key/value topic.
        Note: this assumes that a topic is configured for a single custom_field.
        """

        for custom_field in self._custom_data_sources:
            # Check which data sources point to this topic and store the
            # data indexed by custom_field
            if (
                self._custom_data_sources[custom_field].get("type") == TYPE_KEY_VALUE
                and self._custom_data_sources[custom_field].get("topic") == topic
            ):
                return custom_field
        return None
