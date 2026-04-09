# Directory Structure
```
inorbit/
  agentlets/
    __init__.py
    agent_api.py
    agentlet.py
    alert_manager.py
    camera.py
    custom_commands.py
    custom_data.py
    databag.py
    diagnostics.py
    events.py
    gps.py
    inorbit_pb2.py
    localization.py
    map.py
    meters.py
    odometry.py
    pose.py
    ros_monitoring.py
    ros.py
    rosbag.py
    rosout.py
    spatial_annotations.py
    state_manager.py
    system.py
    teleop.py
  msg/
    __init__.py
    _InOrbitOut.py
  rossetup/
    __init__.py
    ros.py
  __init__.py
  blackbox.py
  link.py
  log_manager.py
  logger.py
ros_monitor/
  __init__.py
  nodes_monitor.py
  params_monitor.py
  topics_monitor.py
scripts/
  agent_watchdog.sh
  start.sh
  uninstall.sh
util/
  __init__.py
  array_util.py
  artifacts.py
  concurrency.py
  CvBridgeCustom.py
  Image.py
  math_util.py
  networking_mixin.py
  once_logger.py
  overrides.py
  rate_limiter.py
  robot_script_action.py
  s3_upload_helper.py
  simplify.py
  suppress_stderr.py
  tf_util.py
  topic_info.py
common_requirements.txt
inorbit.py
python3_requirements.txt
RELEASE.txt
```

# Files

## File: inorbit/agentlets/__init__.py
````python
# Copyright (c) 2018, InOrbit, Inc.
# All rights reserved.
````

## File: inorbit/agentlets/agent_api.py
````python
# Copyright (c) 2018, InOrbit, Inc.
# All rights reserved.
# Agentlet module that serves as an Agent API to receive data from
# the robot using local HTTP.
import threading
import time
from util.overrides import overrides
from .agentlet import Agentlet
PLAIN_TEXT_KEY_VALUES_URL = "/api/v1/data/<key>"
JSON_DATA_URL = "/api/v1/data"
# HTTP request status codes extracted from
# https://www.flaskapi.org/api-guide/status-codes/
# Note: these are defined here since the status module is not
# part of the flask core package.
STATUS_HTTP_200_OK = "200"
STATUS_HTTP_409_CONFLICT = "409"
STATUS_HTTP_415_UNSUPPORTED_MEDIA_TYPE = "415"
STATUS_HTTP_501_NOT_IMPLEMENTED = "501"
DEFAULT_WEBSERVER_PORT = 5000
# Timeout to retry loading the server [seconds]
RETRY_TIMEOUT = 30.0
# Max number of attempts to load server
MAX_LOAD_ATTEMPTS = 4
class ServerThread(threading.Thread):
    """
    Server class. Starts a WSGI application on localhost, on the given
    port. Runs on its own thread.
    """
    def __init__(self, port, app, logger):
        threading.Thread.__init__(self)
        self.srv = make_server("localhost", port, app)
        self.port = port
        self.logger = logger
        self.ctx = app.app_context()
        self.ctx.push()
    def run(self):
        self.logger.info(f"Starting server at Port {self.port}")
        self.srv.serve_forever()
    def shutdown(self):
        self.logger.info(f"Shutting down server at Port {self.port}")
        self.srv.shutdown()
class APIAgentlet(Agentlet):
    """
    Agent API class. Gets post requests from the robot, processes the received data
    and redirects it to the corresponding agentlets.
    """
    def __init__(self, uplink, custom_data_agentlet):
        super(APIAgentlet, self).__init__(uplink)
        # NOTE(adamantivm) Flask app is created during load
        self._app = None
        self._custom_data_agentlet = custom_data_agentlet
        self._server = None
        self._states["api_webserver_port"] = DEFAULT_WEBSERVER_PORT
        # Counter for attempts to load server
        self._server_load_attempt = 0
    @overrides(Agentlet)
    def load(self, runlevel):
        # Dynamically load Flask dependencies, to be more protected from possible import errors
        # TODO(adamantivm) Generalize this as a 'try to load dependency' kind of thing
        try:
            from flask import Flask
            from flask import request
            from flask import Response
            from werkzeug.serving import make_server  # this is installed as a flask dependency
            global Flask
            global request
            global Response
            global make_server
        except Exception as e:
            self.once_logger.exception("Flask load", "Exception importing Flask.")
            return False
        if self._app is None:
            self._app = Flask(__name__)
        self._states["runlevel"] = runlevel
        # Add an URL rule for each of the URLS handled by the API.
        self._app.add_url_rule(
            PLAIN_TEXT_KEY_VALUES_URL, "key_values", self._on_key_values, methods=["POST"]
        )
        self._app.add_url_rule(JSON_DATA_URL, "json_data", self._on_json_data, methods=["POST"])
        # Launch server starter thread to make the webserver run on the
        # configured port. If there is no configuration available, use the
        # default port number.
        threading.Thread(target=self._start_server_loop).start()
        self._states["loaded"] = True
        # Send a state update to the cloud
        self.publish_state(self.uplink, self._states)
        return True
    @overrides(Agentlet)
    def unload(self):
        # Shutdown server and kill thread
        try:
            self._shutdown_server()
        except Exception as e:
            self.logger.exception("Unable to shutdown webserver")
        self._server_loader_running = False
        self._states["loaded"] = False
        return True
    @overrides(Agentlet)
    def set_runlevel(self, runlevel):
        self._states["runlevel"] = runlevel
    @overrides(Agentlet)
    def set_state(self, state):
        if "api_webserver_port" in state.keys():
            if self._states["loaded"]:
                # Only restart the server if port number is updated
                if state["api_webserver_port"] != self._states["api_webserver_port"]:
                    try:
                        # Restart server with the new config
                        self._shutdown_server()
                        self._start_server(state["api_webserver_port"])
                    except Exception as e:
                        self.logger.exception(
                            f"Unable to restart agent API on port {state['api_webserver_port']}"
                        )
            self._states["api_webserver_port"] = state["api_webserver_port"]
        # Send a state update
        self.publish_state(self.uplink, self._states)
    def _start_server_loop(self):
        """
        Server starter loop. It runs on a separate thread and tries to start the
        server thread when the agentlet is loaded. In case of error, it will try
        again after RETRY_TIMEOUT seconds, until MAX_LOAD_ATTEMPTS is reached.
        """
        self._server_loader_running = True
        while self._server_loader_running and self._server_load_attempt < MAX_LOAD_ATTEMPTS:
            try:
                self._start_server(self._states["api_webserver_port"])
                self._server_loader_running = False
            except Exception as e:
                self._server_load_attempt += 1
                self.logger.warning(f"Unable to load server. Attempt {self._server_load_attempt}.")
                time.sleep(RETRY_TIMEOUT)
        self.logger.info("Server starter thread shutting down.")
    def _start_server(self, port):
        self._server = ServerThread(port, self._app, self.logger)
        self._server.daemon = True
        self._server.start()
    def _shutdown_server(self):
        self._server.shutdown()
    def _on_key_values(self, key):
        """
        Callback for key-values POST requests made on PLAIN_TEXT_KEY_VALUES_URL.
        Takes the key and values received and redirects them to the
        CustomDataAgentlet.
        """
        # Sanity check. Only accept data of text/plain type.
        if request.headers["Content-Type"] != "text/plain":
            return Response(
                "Expected content-type: text/plain", STATUS_HTTP_415_UNSUPPORTED_MEDIA_TYPE
            )
        try:
            data = request.get_data()
            self._custom_data_agentlet.process_key_value_pair_from_api(key, data)
            return Response("OK", STATUS_HTTP_200_OK)
        except Exception as e:
            return Response("Error processing request", STATUS_HTTP_409_CONFLICT)
    def _on_json_data():
        """
        Callback for POST requests made on JSON_DATA_URL.
        Note (Flor_Grosso): This URL is not yet supported. The method is left here
        to be completed when needed.
        Consider enabling this as a way to receive multiple key value pairs per
        request or other types of data.
        """
        if request.headers["Content-Type"] != "application/json":
            Response(
                "Expected content-type: application/json", STATUS_HTTP_415_UNSUPPORTED_MEDIA_TYPE
            )
        try:
            data = request.get_data()
            # TODO (Flor_Grosso): Implement data processing.
            return Response("Not yet implemented", STATUS_HTTP_501_NOT_IMPLEMENTED)
        except Exception as e:
            return Response("Error processing request", STATUS_HTTP_409_CONFLICT)
````

## File: inorbit/agentlets/agentlet.py
````python
# Copyright (c) 2018, InOrbit, Inc.
# All rights reserved.
# Agent interface / abstract class
# Contains the skeleton and lays out the lifecycle that all agentlets should follow
import json
import os
import time
from copy import deepcopy
import inorbit.logger
from util.networking_mixin import NetworkingMixin
from util.once_logger import OnceLogger
RUNLEVEL_SILENT = 0
RUNLEVEL_MINIMAL = 1
RUNLEVEL_PARTIAL = 2  # Enables publishing some additional data, but not all.
RUNLEVEL_DEFAULT = 5
RUNLEVEL_FULL = 10
MODULES_TOPIC = "modules/states"
ROS_MSG_ANY = "ROSPY_MSG_ANY"
class Agentlet(NetworkingMixin):
    def __init__(self, uplink):
        NetworkingMixin.__init__(self)
        self.robot_namespace = os.environ.get("ROBOT_NAMESPACE", "")
        if self.robot_namespace:
            self.robot_namespace += "/"
        self.uplink = uplink
        self._states = {}
        self._states["loaded"] = False
        self._states["runlevel"] = RUNLEVEL_DEFAULT
        # Set self's module name, by default the agentlet class name
        self._states["module_name"] = self.__class__.__name__
        # Instead of a module name, agentlets will log from
        # their classname (setup is free this way)
        self.logger = inorbit.logger.getLog(self.__class__.__name__)
        # Logger for exceptions that might occur at a high rate and
        # need to be reported on first appearance only
        self.once_logger = OnceLogger(self.logger)
    def load(self, runlevel=RUNLEVEL_DEFAULT):
        raise NotImplementedError()
    def unload(self):
        raise NotImplementedError()
    def set_runlevel(self, runlevel):
        raise NotImplementedError()
    def set_state(self, state):
        raise NotImplementedError()
    def get_state(self):
        return deepcopy(self._states)
    def get_state_options(self, state_name):
        return ()
    def get_runlevel(self):
        return self._states["runlevel"]
    def publish_state(self, link, state):
        """
        Sends agent state updates to the cloud.
        """
        if self._states["module_name"] is not None:
            state = json.dumps(state)
            link.publish(MODULES_TOPIC, state, qos=1)
    def get_ts(self):
        """
        Returns current timestamp, in milliseconds.
        """
        return int(time.time() * 1000)
    def is_time_expired(self, ts_hint=None, max_delay=None):
        """
        Returns whether the given command timestamp is older
        than the given maximum threshold.
        Parameters are always in milliseconds.
        """
        return ts_hint is None or max_delay is None or (self.get_ts() - float(ts_hint)) > max_delay
    def wake_up_publisher(self, condition):
        """
        Wakes up a publisher thread waiting for the condition variable.
        """
        condition.acquire()
        condition.notify()
        condition.release()
    @staticmethod
    def merge_states(ids, per_id_state, defaults):
        """
        Returns a dictionary of settings indexed by id, built from merging
        general (defaults) with per id (per_id_state) configs. For example,
        ids = ['0', '1', '2', '99']
        per_id_state = { '0': {'opt1': 'a', 'opt2': 'b'},
                        '1': {'opt1': 'a', 'opt2': 'd', 'opt3': 'e'},
                        '99': {'opt1': 'a', 'opt3': 'e'}}
        defaults = {'opt1': 'a', 'opt2': 'f', 'opt3': 'c', 'opt4': 'g'}
        Returns:
        { '0': {'opt1': 'a', 'opt2': 'b', 'opt3': 'c', 'opt4': 'g'},
        '1': {'opt1': 'a', 'opt2': 'd', 'opt3': 'e', 'opt4': 'g'},
        '2': {'opt1': 'a', 'opt2': 'f', 'opt3': 'c', 'opt4': 'g'},
        '99': {'opt1': 'a', 'opt2': 'f', 'opt3': 'e', 'opt4': 'g'}}
        """
        merged_states = {}
        for id in ids:
            if defaults is not None:
                merged_states[id] = defaults.copy()
            else:
                merged_states[id] = {}
            if id in per_id_state and per_id_state[id] is not None:
                merged_states[id].update(per_id_state[id])
        return merged_states
````

## File: inorbit/agentlets/alert_manager.py
````python
# Copyright (c) 2018, InOrbit, Inc.
# All rights reserved.
# Alert Manager agentlet.
#
import threading
import time
from builtins import int
import agentlet
from inorbit_pb2 import AlertMessage
from util.overrides import overrides
ALERT_PUBLISH_RATE_HZ = 1
DIAGNOSTIC_LEVEL_OK = 0
DIAGNOSTIC_LEVEL_WARN = 1
DIAGNOSTIC_LEVEL_ERROR = 2
DIAGNOSTIC_LEVEL_STALE = 3
MQTT_ALERT_TOPIC = "alerts/out"
class AlertManagerAgentlet(agentlet.Agentlet):
    def __init__(self, uplink, diagnostics):
        super(AlertManagerAgentlet, self).__init__(uplink)
        self._diagnostics = diagnostics
        # Dictionary of diagnostics component ids ("RosDiag:" + alert.name)
        # to level
        self._alerts_history = {}
        self._diagnostics_msgs = []
        self._diagnostics_ts = int(0)
        # Mutex used to access _alerts_history.
        self._mutex = threading.Lock()
        # Publisher thread running state
        self._alert_publisher_running = False
    @overrides(agentlet.Agentlet)
    def load(self, runlevel):
        self._states["runlevel"] = runlevel
        # Start publishing thread
        threading.Thread(target=self._publish_loop).start()
        self._states["loaded"] = True
        return True
    @overrides(agentlet.Agentlet)
    def unload(self):
        self._states["loaded"] = False
        self._alert_publisher_running = False
        self.once_logger.reset_all()
        return True
    @overrides(agentlet.Agentlet)
    def set_runlevel(self, runlevel):
        self._states["runlevel"] = runlevel
    def _build_alert(self, diagnostic_name, ts):
        alert = AlertMessage()
        alert.ts = ts
        alert.name = diagnostic_name
        alert.level = str(self._diagnostics_msgs[diagnostic_name].get("level"))
        alert.message = self._diagnostics_msgs[diagnostic_name].get("msg")
        alert.component_id = "RosDiag:" + alert.name.strip()
        return alert
    def _is_not_alert(self, level, component_id):
        """
        Returns True if the alert is either a known issue (present in alerts
        history, with the same level) or if it's stale.
        """
        message_is_known = (
            component_id in self._alerts_history and self._alerts_history[component_id] == level
        )
        message_is_stale = int(level) == DIAGNOSTIC_LEVEL_STALE
        return message_is_known or message_is_stale
    def _is_new_alert(self, level, component_id):
        """
        Returns True if the level is other than OK and the component id
        wasn't reported/ was OK before.
        """
        return int(level) > DIAGNOSTIC_LEVEL_OK and self._alerts_history.get(component_id) != level
    def _is_alert_cleared(self, level, component_id):
        """
        Returns True if a component id previously reported as error, has
        now changed to level OK.
        """
        return int(level) == DIAGNOSTIC_LEVEL_OK and self._alerts_history.get(component_id) != level
    def _parse_diagnostic_msg(self):
        diagnostics_data = self._diagnostics.get_diagnostics()
        if diagnostics_data:
            self._diagnostics_msgs = diagnostics_data.get("data", {})
            self._diagnostics_ts = diagnostics_data.get("ts", int(0))
    def _publish_loop(self):
        self._alert_publisher_running = True
        while self._alert_publisher_running:
            try:
                self._publish_diagnostics_if_available()
            except Exception as e:
                self.once_logger.exception("alerts_publish", "Exception publishing data.")
            time.sleep(1.0 / ALERT_PUBLISH_RATE_HZ)
        self.logger.info("Publisher thread shutting down.")
    def _publish_diagnostics_if_available(self):
        self._parse_diagnostic_msg()
        if len(self._diagnostics_msgs) > 0:
            with self._mutex:
                for component in self._diagnostics_msgs:
                    # Ignore empty diagnostics data
                    if not self._diagnostics_msgs.get(component):
                        continue
                    alert_data = self._build_alert(component, self._diagnostics_ts)
                    level = alert_data.level
                    component_id = alert_data.component_id
                    if self._is_not_alert(level, component_id):
                        continue
                    if self._is_new_alert(level, component_id):
                        alert_data.status = "new"
                    elif self._is_alert_cleared(level, component_id):
                        alert_data.status = "cleared"
                    # Update alerts history
                    self._alerts_history[component_id] = level
                    self.uplink.publish_protobuf(MQTT_ALERT_TOPIC, alert_data, qos=1)
    def _trigger_alerts_resend(self):
        """
        Forces the module to resend alerts by cleaning up alerts history.
        """
        with self._mutex:
            self._alerts_history = {}
        self.logger.info("Triggering a diagnostics alerts resend")
````

## File: inorbit/agentlets/camera.py
````python
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
````

## File: inorbit/agentlets/custom_commands.py
````python
# Copyright (c) 2018, InOrbit, Inc.
# All rights reserved.
# Custom Commands agentlet. Receives command requests from the cloud, processes
# and redirects them to the robot.
import os
import threading
from inorbit import INORBIT_ACTIONS_PATH_DEFAULT
from util.overrides import overrides
from util.robot_script_action import RobotScriptAction
from util.robot_script_action import STATUS_ABORTED
from util.robot_script_action import STATUS_FINISHED
from util.robot_script_action import STATUS_INSTALLED
from util.robot_script_action import STATUS_NOT_INSTALLED
from .agentlet import Agentlet
from .inorbit_pb2 import CustomScriptCommandMessage
from .inorbit_pb2 import CustomScriptStatusMessage
MQTT_CUSTOM_SCRIPTS_TOPIC = "custom_command/script/command"
MQTT_SCRIPT_OUTPUT_TOPIC = "custom_command/script/status"
# Timeout for script execution
SCRIPT_TIMEOUT_SEC = 30
# Maximum character len of stdout and stderr in CustomScriptStatusMessage. If
# they are longer, they are truncated to the last SCRIPT_OUTPUT_LEN characters.
SCRIPT_OUTPUT_LEN = 2048
DEFAULT_MAX_PARALLEL_EXECS = 3
# Possible values for concurrent execution (always enabled, disabled,
# disables for the same combination of filename + args)
CONCURRENT_EXEC_ENABLED = "enabled"
CONCURRENT_EXEC_DISABLED = "disabled"
CONCURRENT_EXEC_DISABLED_FOR_SAME_ARGS = "disabled_for_same_args"
# Disable running the same script more than once at the same time.
DEFAULT_CONCURRENT_EXEC = CONCURRENT_EXEC_DISABLED
# Supported options to configure concurrent executions state.
CONCURRENT_EXEC_OPTIONS = [
    CONCURRENT_EXEC_ENABLED,
    CONCURRENT_EXEC_DISABLED,
    CONCURRENT_EXEC_DISABLED_FOR_SAME_ARGS,
]
# ATTENTION!! AGENT VERSION 3.26.
# Flag to indicate whether to run script actions on a clean environment or not.
# Agent versions prior to 3.26 had this set to False by default. However, if
# the environment is not cleaned, the script will be run with the environment
# that the agent is using. This includes the agent's specific python virtualenv
# and other environment variable settings which can interfere with the user's
# expected environment to run scripts (which is most likely the robot's env).
# For this reason, starting on agent version 3.26 we will clean the environment
# before running a user script, unless this is specifically disabled.
CLEAN_ENV = True
class CustomCommandsAgentlet(Agentlet):
    def __init__(self, uplink):
        super(CustomCommandsAgentlet, self).__init__(uplink)
        self._available_custom_scripts = []
        # Set default values for states
        self._states["script_execution_timeout"] = SCRIPT_TIMEOUT_SEC
        self._states["script_max_parallel_execs"] = DEFAULT_MAX_PARALLEL_EXECS
        self._states["script_concurrent_execs"] = DEFAULT_CONCURRENT_EXEC
        self._states["clean_env"] = CLEAN_ENV
        # Dictionary execution ids to file info (names and args) with
        # status "running"
        self._current_running_scripts = {}
        # Mutex used to protect shared resources between robot script
        # actions thread (both uplink and _current_running_scripts) and
        # prevent race conditions.
        self._mutex = threading.Lock()
        # Define the path where the custom scripts are located
        if "INORBIT_ACTIONS_PATH" in os.environ:
            user_defined_path = os.environ["INORBIT_ACTIONS_PATH"]
            # The user has configured a path for actions. If it's
            # a valid directory, use this instead of the default.
            self.logger.info(
                f"Found INORBIT_ACTIONS_PATH environment configuration = {user_defined_path}."
            )
            if os.path.isdir(user_defined_path):
                self._configure_actions_path(user_defined_path)
            else:
                self.logger.warning(
                    f"The actions path configured {user_defined_path} does not exist."
                )
                self._configure_actions_path(INORBIT_ACTIONS_PATH_DEFAULT)
        else:
            # Set default path for actions script
            self._configure_actions_path(INORBIT_ACTIONS_PATH_DEFAULT)
    @overrides(Agentlet)
    def load(self, runlevel):
        self._states["runlevel"] = runlevel
        # Register for upstream incoming commands
        # Note that incoming commands should be guaranteed to arrive (and exactly once, should)
        # not repeat! QoS=2
        self.uplink.add_listener(
            MQTT_CUSTOM_SCRIPTS_TOPIC, self._process_custom_scripts_command_message, 2
        )
        try:
            if self._files_path == INORBIT_ACTIONS_PATH_DEFAULT and not os.path.exists(
                self._files_path
            ):
                os.makedirs(self._files_path)
        except Exception as e:
            self.logger.exception("Exception reading scripts from filesystem.")
        self._states["available_custom_scripts"] = self._list_custom_scripts_in_robot()
        self._states["loaded"] = True
        # Send a state update to the clouds
        self.publish_state(self.uplink, self._states)
        return True
    @overrides(Agentlet)
    def unload(self):
        self._states["loaded"] = False
        return True
    @overrides(Agentlet)
    def set_runlevel(self, runlevel):
        self._states["runlevel"] = runlevel
    def set_state(self, state):
        """
        Called whenever a set_module command is received.
        """
        if "script_execution_timeout" in state.keys():
            self._states["script_execution_timeout"] = state["script_execution_timeout"]
        if "script_max_parallel_execs" in state.keys():
            self._states["script_max_parallel_execs"] = state["script_max_parallel_execs"]
        if "script_concurrent_execs" in state.keys():
            self._store_concurrent_execs_state(state["script_concurrent_execs"])
        if "clean_env" in state.keys():
            self._states["clean_env"] = state["clean_env"]
        # Send a state update to the cloud
        self.publish_state(self.uplink, self._states)
    def _list_custom_scripts_in_robot(self):
        """
        Returns the list of files in directory self._files_path.
        NOTE (Flor_Grosso): This could be updated in order to only list
        files with execution permissions.
        """
        return [
            file
            for file in os.listdir(self._files_path)
            if os.path.isfile(os.path.join(self._files_path, file))
        ]
    def _send_status_base(
        self, file_name, execution_id, status, send_output, output, status_details=None
    ):
        """
        Publishes a CustomScriptStatusMessage in MQTT_SCRIPT_OUTPUT_TOPIC.
        This is the base method used to send status updates for different tasks
        (script installation, execution status, status_details, script output).
        """
        message = CustomScriptStatusMessage()
        message.ts = self.get_ts()
        message.file_name = file_name
        message.execution_id = execution_id
        message.execution_status = status
        # If execution details are available (eg. errors when trying
        # to execute a script), send them.
        if status_details:
            message.execution_status_details = status_details
        if send_output and output:
            message.return_code = output.get("return_code")
            message.stdout = output.get("stdout")[-SCRIPT_OUTPUT_LEN:]
            message.stderr = output.get("stderr")[-SCRIPT_OUTPUT_LEN:]
        self.uplink.publish_protobuf(MQTT_SCRIPT_OUTPUT_TOPIC, message, qos=1)
    def _send_script_execution_status(
        self, file_name, execution_id, status, send_output=False, output={}, status_details=None
    ):
        """
        Publishes execution status of a script.
        """
        # If script execution is over (either aborted or finished
        # correctly), remove it from the dictionary tracking current
        # runs.
        with self._mutex:
            if execution_id in self._current_running_scripts and (
                status == STATUS_FINISHED or status == STATUS_ABORTED
            ):
                self._current_running_scripts[execution_id] = None
                self._current_running_scripts.pop(execution_id)
            self._send_status_base(
                file_name, execution_id, status, send_output, output, status_details
            )
    def _send_script_installation_status(self, file_name, execution_id, status):
        """
        Publishes the installation status of a script.
        Here we aren't using the _last_script and _script_execution_status
        members, in order to be able to install a file while another script
        is running.
        """
        with self._mutex:
            self._send_status_base(file_name, execution_id, status, None, False, {})
    def _process_custom_scripts_command_message(self, in_msg):
        """
        Called from the server. Processes a custom script command message and
        creates a new script action from it. It triggers the creation of a
        new file and its execution, if requested.
        """
        message = CustomScriptCommandMessage()
        message.ParseFromString(in_msg)
        file_name = message.file_name
        exec_id = message.execution_id
        # Convert the protobuf message to a python list
        args = [arg for arg in message.arg_options] if message.arg_options else []
        # Check if message is well formed
        if message.file_name == "":
            # NOTE(Flor_Grosso): It is very unlikely that the UI would
            #                    send a status with no name field.
            self.logger.warning("Received a script run requirement without" " file_name field")
            return
        # Create a new script action
        script = RobotScriptAction(
            file_name, self._files_path, exec_id, args, self._states["clean_env"]
        )
        # Creating the file if is necessary.
        # In the case that the file already exists, it is overwritten.
        if message.script_contents != "":
            if script.from_file_contents(message.script_contents):
                self._send_script_installation_status(file_name, exec_id, STATUS_INSTALLED)
            else:
                self._send_script_installation_status(file_name, exec_id, STATUS_NOT_INSTALLED)
                return
        max_parallel_execs = self._states.get(
            "script_max_parallel_execs", DEFAULT_MAX_PARALLEL_EXECS
        )
        if message.run:
            # Getting list of keys is atomic, so it's not necessary to
            # use mutex here. Please refer to
            # http://effbot.org/pyfaq/what-kinds-of-global-value-mutation-are-thread-safe.htm
            # for more information.
            if len(self._current_running_scripts.keys()) >= max_parallel_execs:
                self._send_script_execution_status(
                    file_name,
                    exec_id,
                    STATUS_ABORTED,
                    status_details="exceeded parallel script " "executions",
                )
                return
            else:
                # Run the script if requested
                timeout = self._states["script_execution_timeout"]
                concurrent_execs = self._states.get(
                    "script_concurrent_execs", DEFAULT_CONCURRENT_EXEC
                )
            with self._mutex:
                can_run_script = self._can_run_script(file_name, args, concurrent_execs)
            if not can_run_script:
                self._send_script_execution_status(
                    file_name, exec_id, STATUS_ABORTED, status_details="script already running"
                )
                return
            # Add another entry to the current running scripts
            # dictionary
            with self._mutex:
                self._current_running_scripts[exec_id] = script
            script.run(self._send_script_execution_status, timeout)
    def _can_run_script(self, file_name, args, concurrent_execs):
        """
        Checks if a script can be executed based on the concurrent execution
        settings and whether there is a script with the same file_name and
        args already running.
        This method accesses self._current_running_scripts and assumes
        self._mutex is already locked.
        """
        if concurrent_execs == CONCURRENT_EXEC_ENABLED:
            return True
        is_script_running, are_args_equal = self._is_script_running(file_name, args)
        # Script can't be ran if the file is already running and concurrent
        # executions flag is disabled.
        if concurrent_execs == CONCURRENT_EXEC_DISABLED and is_script_running:
            return False
        # Script can't be ran if same file + args is running and concurrent
        # executions with different arguments is enabled
        elif (
            concurrent_execs == CONCURRENT_EXEC_DISABLED_FOR_SAME_ARGS
            and is_script_running
            and are_args_equal
        ):
            return False
        return True
    def _is_script_running(self, file_name, args):
        """
        Returns an array of two booleans, the first one indicated whether
        the given file is being executed and the second one if arguments are
        equal to the ones provided.
        This method accesses self._current_running_scripts and assumes
        self._mutex is already locked.
        """
        active_exec_ids = self._current_running_scripts.keys()
        # Assuming the caller is sending either an array filled with args
        # or an empty array.
        args_string = " ".join(args)
        for exec_id in active_exec_ids:
            is_script_running = self._current_running_scripts[exec_id].get_name() == file_name
            are_args_equal = self._current_running_scripts[exec_id].get_args_string() == args_string
            if is_script_running:
                return [is_script_running, are_args_equal]
        return [False, False]
    def _store_concurrent_execs_state(self, script_concurrent_execs):
        """
        Checks if the script_concurrent_execs option provided is within the
        expected values. If it's not, assumes a default state and warns the
        user.
        """
        if script_concurrent_execs not in CONCURRENT_EXEC_OPTIONS:
            self._states["script_concurrent_execs"] = DEFAULT_CONCURRENT_EXEC
            self.logger.warning(
                f"'script_concurrent_execs' state: '{script_concurrent_execs}' is not valid. "
                f"Assuming '{DEFAULT_CONCURRENT_EXEC}' as default."
            )
        else:
            self._states["script_concurrent_execs"] = script_concurrent_execs
    def _configure_actions_path(self, path):
        """
        Configures the path where the agent will look for the action scripts.
        """
        # Set default path for actions script
        self._files_path = path
        self.logger.info(f"Looking for actions scripts in: {path}.")
````

## File: inorbit/agentlets/custom_data.py
````python
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
````

## File: inorbit/agentlets/databag.py
````python
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
````

## File: inorbit/agentlets/diagnostics.py
````python
# Copyright (c) 2018, InOrbit, Inc.
# All rights reserved.
# Agentlet module that publishes ROS diagnostics messages
# Depends on the ROS module.
import threading
from copy import deepcopy
from util.overrides import overrides
from .agentlet import Agentlet
from .agentlet import RUNLEVEL_FULL
from .agentlet import RUNLEVEL_PARTIAL
from .agentlet import RUNLEVEL_SILENT
from .inorbit_pb2 import KeyValueMessage
from .inorbit_pb2 import RosDiagnosticsField
from .inorbit_pb2 import RosDiagnosticsMessage
from .inorbit_pb2 import RosDiagnosticsStatusMessage
ROS_DIAGNOSTICS_MSG_TYPE = "diagnostic_msgs/msg/DiagnosticArray"
ROS_DIAGNOSTICS_TOPIC = "diagnostics"
ROS_DIAGNOSTICS_TOPIC_AGG = "diagnostics_agg"
ROS_DIAGNOSTICS_TOPIC_DEFAULT = ROS_DIAGNOSTICS_TOPIC_AGG
MQTT_DIAGNOSTICS_TOPIC = "ros/diagnostics2"
MQTT_DIAGNOSTICS_STATUS_TOPIC = "ros/diagnostics/status"
# Diagnostics publisher's period for default and minimal runlevel
# (seconds)
PUBLISHER_PERIOD_DEFAULT_RUNLEVEL = 10
PUBLISHER_PERIOD_FULL_RUNLEVEL = 1
DIAGNOSTIC_LEVEL_STALE = 3
class RosDiagnosticsAgentlet(Agentlet):
    def __init__(self, uplink, ros):
        super(RosDiagnosticsAgentlet, self).__init__(uplink)
        self._ros = ros
        # Publisher thread running state
        self._running = False
        # Dictionary of diagnostics data (level, message, nested
        # key/values), indexed by component name.
        self._last_diagnostics = dict()
        # Last update timestamp for diagnostics data (milliseconds)
        self._last_update_ts = 0.0
        # Mutex used to access _last_diagnostics.
        self._mutex = threading.Lock()
        # Condition to handle publisher's sleep/wake up timing
        self._condition = threading.Condition()
        # Is set to true when using diagnostics aggregator.
        # If not, bare diagnostics is used.
        self._using_diagnostics_aggregator = False
        # states of the agentlet
        self._states["diagnostics_topic"] = None
        self._states["available_diagnostics_topics"] = []
        # Flag to indicate that diagnostics keys state options have been
        # updated
        self._diagnostics_keys_updated = False
        # Array of objects with {key:<string>, name:<string>} elements. There
        # is one object per available key in the Diagnostics message.
        self._states["available_diagnostics_keys"] = []
        # Period at which the publisher loop sends update to the cloud
        self._publisher_period = PUBLISHER_PERIOD_DEFAULT_RUNLEVEL
    @overrides(Agentlet)
    def load(self, runlevel):
        self._states["runlevel"] = runlevel
        try:
            import diagnostic_msgs
            import diagnostic_msgs.msg
            global diagnostic_msgs
        except Exception as e:
            self.once_logger.exception(
                "diagnostics_msgs_load", "Exception loading diagnostics_msgs."
            )
            return False
        self._set_initial_diagnostics_topic()
        self._ros.add_submodule(
            "diagnostics",
            subs=(
                (
                    self._states["diagnostics_topic"],
                    diagnostic_msgs.msg.DiagnosticArray,
                    self._ros_on_diagnostics,
                ),
            ),
        )
        # Set the publisher period based on the runlevel
        self._set_publisher_period(runlevel)
        # If runlevel is non silent, start publisher thread
        if self.get_runlevel() != RUNLEVEL_SILENT:
            self._launch_publisher_thread()
        self._states["loaded"] = True
        self.publish_state(self.uplink, self._states)
        return True
    @overrides(Agentlet)
    def unload(self):
        # Shutdown publisher thread
        self._running = False
        # Remove ROS subscribers
        self._ros.remove_submodule("diagnostics")
        self._states["loaded"] = False
        self.once_logger.reset_all()
        # Notify that this agentlet was unloaded
        self.publish_state(self.uplink, self._states)
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
        # Set the publisher period based on the runlevel
        self._set_publisher_period(runlevel)
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
    @overrides(Agentlet)
    def set_state(self, state):
        if "diagnostics_topic" in state.keys():
            if self._states["loaded"]:
                # Reset initial state
                new_sub = (
                    state["diagnostics_topic"],
                    diagnostic_msgs.msg.DiagnosticArray,
                    self._ros_on_diagnostics,
                )
                self._ros.update_subscriber_topic(
                    "diagnostics", self._states["diagnostics_topic"], new_sub
                )
            self._states["diagnostics_topic"] = state["diagnostics_topic"]
        # Clean up available diagnostics keys
        self._diagnostics_keys_updated = False
        self._states["available_diagnostics_keys"] = []
        self._using_diagnostics_aggregator = (
            self._states["diagnostics_topic"] == ROS_DIAGNOSTICS_TOPIC_DEFAULT
        )
        # Send a state update
        self.publish_state(self.uplink, self._states)
    def get_diagnostics(self):
        """
        Returns and object with diagnostic data and timestamp of last
        update.
        Within each diagnostic data field, there is mandatory level, name
        and message and then optional 0 to N key value pairs
        """
        with self._mutex:
            last_diagnostics = deepcopy(self._last_diagnostics)
        return {"data": last_diagnostics, "ts": self._last_update_ts}
    def get_diagnostics_value(self, name, status_fields):
        """
        Returns a dictionary with diagnostics status key/value pairs for the
        selected name.
        """
        with self._mutex:
            if not self._last_diagnostics:
                return None
            diag_value = {}
            if name in self._last_diagnostics:
                kv = self._last_diagnostics.get(name, {}).get("key_values", {})
                if status_fields in kv:
                    last_diag_status_ts = self._last_diagnostics[name].get(
                        "last_diag_status_ts", self.get_ts()
                    )
                    if self.is_time_expired(last_diag_status_ts, self._publisher_period * 1000):
                        return None
                    diag_value[status_fields] = kv[status_fields]
                    diag_value["ts"] = last_diag_status_ts
                else:
                    return None
        return diag_value
    def _launch_publisher_thread(self):
        """
        Starts uplink publishing thread.
        """
        threading.Thread(target=self._publish_loop).start()
    def _shutdown_publisher_thread(self):
        """
        Shutdowns publisher thread by setting its state to not running.
        TODO (Flor_Grosso): Consider implementing a way to make sure that thread
        is killed properly
        """
        self._running = False
    def _publish_loop(self):
        """
        Runs on a separate thread. Publishes diagnostics at a rate dependent
        on the runlevel.
        """
        self._running = True
        while self._running:
            try:
                self._maybe_publish()
            except Exception as e:
                self.once_logger.exception("diagnostics_publish", "Exception publishing data.")
            self._condition.acquire()
            # Throttle differently depending on the module runlevel
            self._condition.wait(self._publisher_period)
            self._condition.release()
        self.logger.info("Publisher thread shutting down.")
    def _maybe_publish(self):
        """
        Publishes the latest recorded diagnostics data if possible and
        appropriate.
        """
        # Publish diagnostics messages only at default or minimal
        # runlevels.
        # TODO (Flor_Grosso): target directly runlevels that will
        # follow this rule instead of those that aren't partial.
        if self._states["runlevel"] != RUNLEVEL_PARTIAL:
            last_diagnostics = self._make_protobuf_message()
            if last_diagnostics:
                diagnostics_msg = RosDiagnosticsMessage()
                diagnostics_msg.ts = self._last_update_ts
                diagnostics_msg.fields.extend(last_diagnostics)
                self.uplink.publish_protobuf(MQTT_DIAGNOSTICS_TOPIC, diagnostics_msg)
        # Status is sent for all runlevels which allow publishing.
        status = self._merge_levels()
        if status is not None:
            status_msg = RosDiagnosticsStatusMessage()
            status_msg.ts = self.get_ts()
            status_msg.status = status
            status_msg.has_status = True
            self.uplink.publish_protobuf(MQTT_DIAGNOSTICS_STATUS_TOPIC, status_msg)
        with self._mutex:
            # Update diagnostics keys if necessary (only once, at the
            # beginning).
            # TODO (Flor_Grosso): This should be triggered by the server,
            # not here. State options need to be updated periodically,
            # or at least when focusing on its config.
            # NOTE that if the agentlet is always running at level 0, then
            # this is never sent.
            if not self._diagnostics_keys_updated:
                self._update_keys()
    def _ros_on_diagnostics(self, data):
        """
        Callback for diagnostics_agg messages.
        """
        # If we were using the aggregator, we serialize the container to the
        # message here. If not, we serialize when the message is needed, in
        # order to collect the diagnostics messages.
        if self._using_diagnostics_aggregator:
            def f_name(s):
                return s
        else:
            f_name = self._adapt_bare_name_like_aggregator
        with self._mutex:
            for s in data.status:
                if s.message is None:
                    s.message = ""
                name = f_name(s.name).replace("|", "/")
                self._last_diagnostics[name] = {}
                # NOTE(MarianoCereda): The incoming level format is bytes, then it needs to be
                # converted.
                self._last_diagnostics[name]["level"] = int.from_bytes(s.level, byteorder="little")
                self._last_diagnostics[name]["msg"] = s.message
                self._last_diagnostics[name]["key_values"] = {
                    key_val.key: key_val.value for key_val in s.values
                }
                self._last_diagnostics[name]["last_diag_status_ts"] = self.get_ts()
            self._last_update_ts = self.get_ts()
    def _make_protobuf_message(self):
        """
        Makes protobuf message data in order to be send via MQTT.
        """
        with self._mutex:
            diagnostics_array = []
            if self._last_diagnostics:
                for diag_name in self._last_diagnostics:
                    diag = RosDiagnosticsField()
                    diag.name = diag_name
                    data = self._last_diagnostics.get(diag_name, {})
                    # If data didn't update since the last publication,
                    # discard it
                    if self.is_time_expired(
                        ts_hint=data.get("last_diag_status_ts"),
                        max_delay=self._publisher_period * 1000,
                    ):
                        continue
                    diag.level = data.get("level")
                    if diag.level is None:
                        continue
                    diag.has_level = True
                    diag.msg = data.get("msg")
                    # Publish nested key-values
                    kv_array = []
                    for k, v in data.get("key_values", {}).items():
                        item = KeyValueMessage()
                        item.key = k
                        item.value = v
                        kv_array.append(item)
                    diag.key_values.extend(kv_array)
                    diagnostics_array.append(diag)
        return diagnostics_array
    def _adapt_bare_name_like_aggregator(self, name):
        # TODO(ivanpauno): A slash is inserted before the name, in order to
        # use the same state naming of "diagnostics aggregator"
        # To take advantage of bare diagnostics state naming as
        # "driver: device" and as we were taking advantage of aggregator naming
        # hierarchy with slashes, we first try doing
        # '/'+name.replace(': ', '/')
        # The problem is that you also need a status of "/driver" in order to
        # see them in the widget.
        # We could invent a status for "/driver", or change how the widget is
        # working for using this "/driver/device" hierarchy.
        return "/" + name
    def _get_available_diagnostics_topics(self):
        """
        Finds topics publishing ROS_DIAGNOSTICS_MSG_TYPE, populate and sort
        available_diagnostics_topics list.
        TODO (ivanpauno): A function called
        get_available_topics(topic_name, default_topic)
        sounds good in order to unify with teleop, etc.
        """
        # TODO (ivan): get_topics is a better name for this
        # method of the ros agentlet.
        diagnostics_topics = self._ros.get_topics_to_publish(ROS_DIAGNOSTICS_MSG_TYPE)
        # If diagnostics topics were found, then sort the list leaving
        # "diagnostics_agg" topic above.
        if diagnostics_topics:
            diagnostics_topics = sorted(
                diagnostics_topics, key=lambda x: (x != ROS_DIAGNOSTICS_TOPIC_DEFAULT, x)
            )
        return diagnostics_topics
    def _set_initial_diagnostics_topic(self):
        """
        Sets the initial diagnostics topic for when the agentlet is first
        loaded.
        """
        self._states["available_diagnostics_topics"] = self._get_available_diagnostics_topics()
        if not self._states["diagnostics_topic"]:
            if not self._states["available_diagnostics_topics"]:
                self.logger.warning(
                    "No ROS diagnostics topics available. "
                    f"Setting '{ROS_DIAGNOSTICS_TOPIC_DEFAULT}' as default."
                )
                self._states["diagnostics_topic"] = ROS_DIAGNOSTICS_TOPIC_DEFAULT
            else:
                # If there are cmd topics, set the first on the list as the
                # current one.
                self._states["diagnostics_topic"] = self._states["available_diagnostics_topics"][0]
        # "Aggregator mode" is set when listening to /diagnostics_agg.
        # In all other cases it is used the "bare mode".
        # Usually is /diagnostics topic, using other topic isn't usual.
        self._using_diagnostics_aggregator = (
            self._states["diagnostics_topic"] == ROS_DIAGNOSTICS_TOPIC_DEFAULT
        )
    def _update_keys(self):
        """
        Updates the list of available diagnostics keys. Note:
        self._last_diagnostics needs to be locked by caller method.
        """
        available_diagnostics_keys = []
        for name in self._last_diagnostics.keys():
            for key in self._last_diagnostics.get(name, {}).get("key_values", {}):
                available_diagnostics_keys.append({"name": name, "key": key})
        if not available_diagnostics_keys:
            return
        self._diagnostics_keys_updated = True
        self._states["available_diagnostics_keys"] = available_diagnostics_keys
        # Trigger a state update
        # TODO (Flor_Grosso): This state update needs to be triggered by the
        # server rather than here manually.
        self.publish_state(self.uplink, self._states)
    def _merge_levels(self):
        """
        Computes merged diagnostics level based on the current diagnostics
        data. Returns the max value among all components.
        """
        # Initialize to 0 (OK)
        total_level = 0
        with self._mutex:
            for name in self._last_diagnostics.keys():
                level = self._last_diagnostics.get(name, {}).get("level", 0)
                # Ignore stale messages
                if level != DIAGNOSTIC_LEVEL_STALE:
                    total_level = max(level, total_level)
        return total_level
    def _set_publisher_period(self, runlevel):
        """
        Checks the agentlet's runlevel and sets the period at which the publisher
        threads sends updates to the cloud accordingly.
        """
        # Runlevel silent doesn't publish data, skip it
        if runlevel == RUNLEVEL_SILENT:
            return
        elif runlevel == RUNLEVEL_FULL:
            self._publisher_period = PUBLISHER_PERIOD_FULL_RUNLEVEL
        # If the runlevel is not full publish at default
        else:
            self._publisher_period = PUBLISHER_PERIOD_DEFAULT_RUNLEVEL
````

## File: inorbit/agentlets/events.py
````python
# Copyright (c) 2020, InOrbit, Inc.
# All rights reserved.
# RobotEvents agentlet.
#
# Used by other agentlets, most notably custom_data to
# send events on-demand to the InOrbit cloud
#
# TODO(adamantivm) Upgrade to a more precise rate-limiting mechanism
import threading
import time
from util.overrides import overrides
from .agentlet import Agentlet
from .inorbit_pb2 import CustomDataMessage
from .inorbit_pb2 import KeyValueCustomElement
MQTT_EVENTS_TOPIC = "events"
DEFAULT_MAX_MSG_RATE = 10  # Maximum rate, in Hertz
DEFAULT_MAX_BYTES_PER_SECOND = 10000  # Maximum rate, in bytes per second
# Module state keys - configurable
MAX_MSG_RATE_KEY = "max_msg_rate_count"
MAX_MSG_BYTES_PER_SECOND_KEY = "max_msg_rate_bytes"
# Module state keys - output state
# Last time when a message was dropped due to reaching the configured
# maximum rate
LAST_MESSAGE_DROPPED_TIME = "msg_dropped_time"
# Hardcoded size in seconds of the time used to measure message rate and
# size for rate limiting purposes. See _is_over_limit
BUCKET_SIZE_S = 10
class RobotEventsAgentlet(Agentlet):
    def __init__(self, uplink):
        super(RobotEventsAgentlet, self).__init__(uplink)
        # Flag to manage publisher thread
        self._publisher_running = False
        # Custom queue. List of (ts, (event))
        self._events = []
        # Lock to control access to the _events and also used as a base
        # for the condition, which also access the same _events shared
        # variable
        # NOTE(adamantiv) Be careful to avoid deadlocks with self._rate_mutex
        self._events_mutex = threading.Lock()
        self._condition = threading.Condition(self._events_mutex)
        # Rate limiting variables. See _is_over_limit.
        # Accumulates number of messages sent during a 10 seconds time bucket
        self._rate_msg_sent_n_accum = 0
        self._rate_msg_sent_bytes_accum = 0
        # Keeps track of the currently running second
        self._rate_bucket = None
        # Control access to all _rate variables
        # NOTE(adamantivm) Be careful to avoid deadlocks with self._event_mutex
        self._rate_mutex = threading.Lock()
        # Keep track of last time we published a module state update because
        # a message was dropped, so that we can throttle it
        self._last_dropped_time_published_time = None
        # Module states
        self._states[MAX_MSG_BYTES_PER_SECOND_KEY] = DEFAULT_MAX_BYTES_PER_SECOND
        self._states[MAX_MSG_RATE_KEY] = DEFAULT_MAX_MSG_RATE
    @overrides(Agentlet)
    def load(self, runlevel):
        self._states["runlevel"] = runlevel
        self._states["loaded"] = True
        self._launch_publisher_thread()
        # Send a state update to the cloud
        self.publish_state(self.uplink, self._states)
        return True
    @overrides(Agentlet)
    def unload(self):
        # Shutdown publisher thread
        self._shutdown_publisher_thread()
        self._states["loaded"] = False
        self.once_logger.reset_all()
        # Send a state update to the cloud
        self.publish_state(self.uplink, self._states)
        return True
    @overrides(Agentlet)
    def set_runlevel(self, runlevel):
        self._states["runlevel"] = runlevel
        return
    @overrides(Agentlet)
    def set_state(self, state):
        if MAX_MSG_BYTES_PER_SECOND_KEY in state.keys():
            self._states[MAX_MSG_BYTES_PER_SECOND_KEY] = state[MAX_MSG_BYTES_PER_SECOND_KEY]
        if MAX_MSG_RATE_KEY in state.keys():
            self._states[MAX_MSG_RATE_KEY] = state[MAX_MSG_RATE_KEY]
        # Send a state update to the cloud
        self.publish_state(self.uplink, self._states)
    def _is_over_limit(self, ts, key, value):
        """
        Very basic rate limiting.
        Returns true if the message will be over the maximum transmit limits at
        this time.
        Keeps track of number of events already sent for the currently running
        block of ten seconds
        """
        # Each bucket is 10 seconds
        ts_bucket = round(ts / (1000 * BUCKET_SIZE_S))
        msg_size = len("{}{}".format(key, value).encode("utf-8"))
        with self._rate_mutex:
            if self._rate_bucket != ts_bucket:
                self._rate_msg_sent_n_accum = 0
                self._rate_msg_sent_bytes_accum = 0
                self._rate_bucket = ts_bucket
            # NOTE(adamantivm) We only break after we've surpassed the configured rates
            # for a 10 seconds bucket.
            if (
                self._rate_msg_sent_n_accum + 1 > self._states[MAX_MSG_RATE_KEY] * BUCKET_SIZE_S
                or self._rate_msg_sent_bytes_accum + msg_size
                > self._states[MAX_MSG_BYTES_PER_SECOND_KEY] * BUCKET_SIZE_S
            ):
                return True
            else:
                self._rate_msg_sent_n_accum += 1
                self._rate_msg_sent_bytes_accum += msg_size
                return False
    def put_key_value(self, key, value, custom_field):
        """
        Submits a new event.
        """
        ts = self.get_ts()
        if self._is_over_limit(ts, key, value):
            self.once_logger.warn("RobotEvents", f"Event rate limited / dropped. key = {key}")
            # Record the time when we dropped a message so that we can report it
            # to the server
            self._states[LAST_MESSAGE_DROPPED_TIME] = ts
            # NOTE(adamantivm) We should be careful not to publish each time we get
            # here as this could be an arbitrarily high rate. Instead, publish
            # this update at a later time, if it hasn't been published for a while
            if (
                self._last_dropped_time_published_time is None
                or ts - self._last_dropped_time_published_time > 1000 * 60 * BUCKET_SIZE_S
            ):
                self._last_dropped_time_published_time = ts
                self.publish_state(self.uplink, self._states)
        else:
            with self._events_mutex:
                self._events.append((ts, key, value, custom_field))
                self._condition.notify()
    def _launch_publisher_thread(self):
        """
        Starts uplink publishing thread.
        """
        threading.Thread(target=self._publish_loop).start()
    def _shutdown_publisher_thread(self):
        """
        Shuts down publisher thread by setting its state to not running.
        """
        self._publisher_running = False
        with self._events_mutex:
            self._condition.notify()
    def _publish_loop(self):
        """
        Publishes thread running on a separate thread.
        """
        self._publisher_running = True
        while self._publisher_running:
            try:
                # Wait a small amount of time to account for multiple messages coming
                # all at once
                time.sleep(0.1)
                self._publish_events_if_available()
            except Exception as e:
                self.once_logger.exception("RobotEvents", "Exception publishing events")
            with self._events_mutex:
                self._condition.wait()
        self.logger.info("Publisher thread shutting down")
    def _publish_events_if_available(self):
        """
        Does the actual publishing of events queued to be sent (if any).
        """
        with self._events_mutex:
            events_to_send = self._events
            self._events = []
        # First go through events to create KeyValueCustomElements
        # and store them by custom_field
        events_by_custom_field = {}
        for ts, key, value, custom_field in events_to_send:
            item_array = events_by_custom_field.get(custom_field, [])
            item = KeyValueCustomElement()
            item.key = key
            item.value = value
            item.ts = ts
            item_array.append(item)
            events_by_custom_field[custom_field] = item_array
        # Compose a send a message per custom_field
        for custom_field, item in events_by_custom_field.items():
            data = CustomDataMessage()
            data.custom_field = custom_field
            data.ts = self.get_ts()
            data.key_value_payload.pairs.extend(item)
            self.uplink.publish_protobuf(MQTT_EVENTS_TOPIC, data, qos=2)
````

## File: inorbit/agentlets/gps.py
````python
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
````

## File: inorbit/agentlets/inorbit_pb2.py
````python
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: inorbit.proto
import sys
_b=sys.version_info[0]<3 and (lambda x:x) or (lambda x:x.encode('latin1'))
from google.protobuf.internal import enum_type_wrapper
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
from google.protobuf import descriptor_pb2
# @@protoc_insertion_point(imports)
_sym_db = _symbol_database.Default()
DESCRIPTOR = _descriptor.FileDescriptor(
  name='inorbit.proto',
  package='inorbit',
  syntax='proto3',
  serialized_pb=_b('\n\rinorbit.proto\x12\x07inorbit\"?\n\x10\x44iskUsageMessage\x12\x11\n\tvolume_id\x18\x01 \x01(\t\x12\x18\n\x10usage_percentage\x18\x02 \x01(\x02\"C\n\x13NetworkStatsMessage\x12\x14\n\x0cinterface_id\x18\x01 \x01(\t\x12\n\n\x02tx\x18\x02 \x01(\x03\x12\n\n\x02rx\x18\x03 \x01(\x03\"\xe6\x03\n\x12SystemStatsMessage\x12\x11\n\ttimestamp\x18\x01 \x01(\x03\x12\x17\n\x0f\x65lapsed_seconds\x18\x02 \x01(\x02\x12\x1b\n\x13\x63pu_load_percentage\x18\x03 \x01(\x02\x12\x19\n\x11network_interface\x18\x04 \x01(\t\x12\x10\n\x08total_tx\x18\x05 \x01(\x03\x12\x10\n\x08total_rx\x18\x06 \x01(\x03\x12\x12\n\ninorbit_tx\x18\x07 \x01(\x03\x12\x12\n\ninorbit_rx\x18\x08 \x01(\x03\x12\x1c\n\x14hdd_usage_percentage\x18\t \x01(\x02\x12\x1c\n\x14inorbit_hdd_usage_mb\x18\n \x01(\x02\x12$\n\x1cinorbit_hdd_usage_percentage\x18\x0b \x01(\x02\x12\x36\n\x13optional_disks_data\x18\x0f \x03(\x0b\x32\x19.inorbit.DiskUsageMessage\x12\x46\n optional_network_interfaces_data\x18\x10 \x03(\x0b\x32\x1c.inorbit.NetworkStatsMessage\x12\x1c\n\x14ram_usage_percentage\x18\x11 \x01(\x02\x12\x0f\n\x07mqtt_tx\x18\x12 \x01(\x03\x12\x0f\n\x07mqtt_rx\x18\x13 \x01(\x03\"\xac\x01\n\x13OdometryDataMessage\x12\x10\n\x08ts_start\x18\x01 \x01(\x03\x12\n\n\x02ts\x18\x02 \x01(\x03\x12\x17\n\x0flinear_distance\x18\x03 \x01(\x02\x12\x18\n\x10\x61ngular_distance\x18\x04 \x01(\x02\x12\x14\n\x0clinear_speed\x18\x05 \x01(\x02\x12\x15\n\rangular_speed\x18\x06 \x01(\x02\x12\x17\n\x0fspeed_available\x18\x07 \x01(\x08\"!\n\tPathPoint\x12\t\n\x01x\x18\x01 \x01(\x02\x12\t\n\x01y\x18\x02 \x01(\x02\"n\n\x12\x44\x65ltaIntPathPoints\x12\"\n\x02xs\x18\x01 \x01(\x0b\x32\x16.inorbit.DeltaIntArray\x12\"\n\x02ys\x18\x02 \x01(\x0b\x32\x16.inorbit.DeltaIntArray\x12\x10\n\x08max_bits\x18\x03 \x01(\x05\"\xad\x01\n\tRobotPath\x12\"\n\x06points\x18\x01 \x03(\x0b\x32\x12.inorbit.PathPoint\x12\n\n\x02ts\x18\x02 \x01(\x03\x12\x0f\n\x07path_id\x18\x03 \x01(\t\x12\x10\n\x08\x66rame_id\x18\x04 \x01(\t\x12\x18\n\x10\x65ncoding_version\x18\x05 \x01(\x05\x12\x33\n\x0e\x65ncoded_points\x18\x06 \x01(\x0b\x32\x1b.inorbit.DeltaIntPathPoints\"\xa1\x01\n\rDeltaIntArray\x12\x0e\n\x06\x61nchor\x18\x01 \x01(\x02\x12\x10\n\x08\x65xponent\x18\x02 \x01(\x11\x12\x0e\n\x06\x64\x65ltas\x18\x03 \x03(\x11\x12\x30\n\x05stats\x18\x04 \x03(\x0b\x32!.inorbit.DeltaIntArray.StatsEntry\x1a,\n\nStatsEntry\x12\x0b\n\x03key\x18\x01 \x01(\t\x12\r\n\x05value\x18\x02 \x01(\x02:\x02\x38\x01\"d\n\x0fPathDataMessage\x12\"\n\x06points\x18\x01 \x03(\x0b\x32\x12.inorbit.PathPoint\x12\n\n\x02ts\x18\x02 \x01(\x03\x12!\n\x05paths\x18\x03 \x03(\x0b\x32\x12.inorbit.RobotPath\"\xd7\x01\n\nMapMessage\x12\r\n\x05width\x18\x01 \x01(\r\x12\x0e\n\x06height\x18\x02 \x01(\r\x12\x0e\n\x06pixels\x18\x03 \x01(\x0c\x12\t\n\x01x\x18\x04 \x01(\x02\x12\t\n\x01y\x18\x05 \x01(\x02\x12\r\n\x05theta\x18\x06 \x01(\x02\x12\x12\n\nresolution\x18\x07 \x01(\x02\x12\n\n\x02ts\x18\x08 \x01(\x03\x12\r\n\x05label\x18\t \x01(\t\x12\x11\n\tdata_hash\x18\n \x01(\x03\x12\x10\n\x08\x66rame_id\x18\x0b \x01(\t\x12\x0e\n\x06map_id\x18\x0c \x01(\t\x12\x11\n\tis_update\x18\r \x01(\x08\".\n\nMapRequest\x12\r\n\x05label\x18\t \x01(\t\x12\x11\n\tdata_hash\x18\n \x01(\x03\"g\n\x14Nav2DWaypointMessage\x12*\n\x05\x66rame\x18\x01 \x01(\x0e\x32\x1b.inorbit.Nav2DWaypointFrame\x12\t\n\x01x\x18\x02 \x01(\x02\x12\t\n\x01y\x18\x03 \x01(\x02\x12\r\n\x05theta\x18\x04 \x01(\x02\"\x81\x01\n\x10Nav2DPathMessage\x12*\n\x05\x66rame\x18\x01 \x01(\x0e\x32\x1b.inorbit.Nav2DWaypointFrame\x12\x0f\n\x07ts_hint\x18\x02 \x01(\x03\x12\x30\n\twaypoints\x18\x03 \x03(\x0b\x32\x1d.inorbit.Nav2DWaypointMessage\"n\n\x0c\x41lertMessage\x12\n\n\x02ts\x18\x01 \x01(\x03\x12\x14\n\x0c\x63omponent_id\x18\x02 \x01(\t\x12\x0e\n\x06status\x18\x03 \x01(\t\x12\r\n\x05level\x18\x04 \x01(\t\x12\x0c\n\x04name\x18\x05 \x01(\t\x12\x0f\n\x07message\x18\x06 \x01(\t\"f\n\x04\x45\x63ho\x12\x12\n\ntime_stamp\x18\x01 \x01(\x03\x12\r\n\x05topic\x18\x02 \x01(\t\x12\x18\n\x0estring_payload\x18\x03 \x01(\tH\x00\x12\x16\n\x0c\x62yte_payload\x18\x04 \x01(\x0cH\x00\x42\t\n\x07payload\"\x9f\x02\n\x14\x44\x61tabagUpdateMessage\x12\x0c\n\x04name\x18\x01 \x01(\t\x12\x17\n\x0fstored_in_robot\x18\x02 \x01(\x08\x12\x1a\n\x12uploading_to_cloud\x18\x03 \x01(\x08\x12\x0b\n\x03url\x18\x04 \x01(\t\x12\x10\n\x08start_ts\x18\x05 \x01(\x03\x12\x0e\n\x06\x65nd_ts\x18\x06 \x01(\x03\x12\x0f\n\x07size_kb\x18\x07 \x01(\x05\x12\x0e\n\x06topics\x18\t \x03(\t\x12\x41\n\nproperties\x18\x08 \x03(\x0b\x32-.inorbit.DatabagUpdateMessage.PropertiesEntry\x1a\x31\n\x0fPropertiesEntry\x12\x0b\n\x03key\x18\x01 \x01(\t\x12\r\n\x05value\x18\x02 \x01(\t:\x02\x38\x01\"\xa9\x02\n\x13RosbagUpdateMessage\x12\x0c\n\x04name\x18\x01 \x01(\t\x12\x17\n\x0fstored_in_robot\x18\x02 \x01(\x08\x12\x1a\n\x12uploading_to_cloud\x18\x03 \x01(\x08\x12\x0b\n\x03url\x18\x04 \x01(\t\x12\n\n\x02ts\x18\x05 \x01(\x03\x12\x10\n\x08start_ts\x18\x06 \x01(\x03\x12\x0e\n\x06\x65nd_ts\x18\x07 \x01(\x03\x12\x0f\n\x07size_kb\x18\x08 \x01(\x05\x12\x0e\n\x06topics\x18\t \x03(\t\x12@\n\nproperties\x18\n \x03(\x0b\x32,.inorbit.RosbagUpdateMessage.PropertiesEntry\x1a\x31\n\x0fPropertiesEntry\x12\x0b\n\x03key\x18\x01 \x01(\t\x12\r\n\x05value\x18\x02 \x01(\t:\x02\x38\x01\"H\n\x0cLaserMessage\x12\x0c\n\x04name\x18\x01 \x01(\t\x12*\n\x06ranges\x18\x02 \x01(\x0b\x32\x1a.inorbit.FloatingPointList\"1\n\x11\x46loatingPointList\x12\x0c\n\x04runs\x18\x01 \x03(\r\x12\x0e\n\x06values\x18\x02 \x03(\x02\"\xac\x01\n\x16LocationAndPoseMessage\x12\n\n\x02ts\x18\x01 \x01(\x03\x12\r\n\x05pos_x\x18\x02 \x01(\x02\x12\r\n\x05pos_y\x18\x03 \x01(\x02\x12\x0b\n\x03yaw\x18\x04 \x01(\x02\x12\x10\n\x08\x66rame_id\x18\x06 \x01(\t\x12%\n\x06lasers\x18\x05 \x03(\x0b\x32\x15.inorbit.LaserMessage\x12\x10\n\x08offset_x\x18\x07 \x01(\x02\x12\x10\n\x08offset_y\x18\x08 \x01(\x02\"6\n\x0bPoseMessage\x12\'\n\x05poses\x18\x01 \x03(\x0b\x32\x18.inorbit.PoseMessageData\"~\n\x0fPoseMessageData\x12\n\n\x02ts\x18\x01 \x01(\x03\x12\r\n\x05pos_x\x18\x02 \x01(\x02\x12\r\n\x05pos_y\x18\x03 \x01(\x02\x12\x0b\n\x03yaw\x18\x04 \x01(\x02\x12\x10\n\x08\x66rame_id\x18\x05 \x01(\t\x12\x10\n\x08offset_x\x18\x06 \x01(\x02\x12\x10\n\x08offset_y\x18\x07 \x01(\x02\"\\\n\rCameraMessage\x12\x11\n\tcamera_id\x18\x01 \x01(\t\x12\r\n\x05width\x18\x02 \x01(\r\x12\x0e\n\x06height\x18\x03 \x01(\r\x12\r\n\x05image\x18\x04 \x01(\x0c\x12\n\n\x02ts\x18\x05 \x01(\x03\"\x87\x01\n\rRobotFileData\x12\x0c\n\x04name\x18\x01 \x01(\t\x12\x17\n\x0fstored_in_robot\x18\x02 \x01(\x08\x12\x1a\n\x12uploading_to_cloud\x18\x03 \x01(\x08\x12\x0b\n\x03url\x18\x04 \x01(\t\x12\n\n\x02ts\x18\x05 \x01(\x03\x12\x0c\n\x04size\x18\x06 \x01(\x03\x12\x0c\n\x04type\x18\x07 \x01(\t\"P\n\x17RobotFilesUpdateMessage\x12)\n\tartifacts\x18\x01 \x03(\x0b\x32\x16.inorbit.RobotFileData\x12\n\n\x02ts\x18\x02 \x01(\x03\"?\n\x15KeyValueCustomElement\x12\x0b\n\x03key\x18\x01 \x01(\t\x12\r\n\x05value\x18\x02 \x01(\t\x12\n\n\x02ts\x18\x03 \x01(\x03\">\n\rKeyValuePairs\x12-\n\x05pairs\x18\x01 \x03(\x0b\x32\x1e.inorbit.KeyValueCustomElement\"?\n\x12\x44iagnosticsMessage\x12\r\n\x05label\x18\x01 \x01(\t\x12\x0b\n\x03key\x18\x02 \x01(\t\x12\r\n\x05value\x18\x03 \x01(\t\"`\n\x0fTextFileMessage\x12\x0c\n\x04\x64\x61ta\x18\x01 \x01(\x0c\x12\x13\n\x0b\x62lob_offset\x18\x02 \x01(\x05\x12\x11\n\tblob_size\x18\x03 \x01(\x05\x12\x17\n\x0ftotal_file_size\x18\x04 \x01(\x05\"\xa0\x02\n\x11\x43ustomDataMessage\x12\x14\n\x0c\x63ustom_field\x18\x01 \x01(\t\x12\x33\n\x11key_value_payload\x18\x02 \x01(\x0b\x32\x16.inorbit.KeyValuePairsH\x00\x12\x17\n\rimage_payload\x18\x03 \x01(\x0cH\x00\x12\x1b\n\x11text_file_payload\x18\x04 \x01(\x0cH\x00\x12:\n\x13\x64iagnostics_payload\x18\x05 \x01(\x0b\x32\x1b.inorbit.DiagnosticsMessageH\x00\x12\x37\n\x13text_file_payload_2\x18\x06 \x01(\x0b\x32\x18.inorbit.TextFileMessageH\x00\x12\n\n\x02ts\x18\x07 \x01(\x03\x42\t\n\x07payload\"C\n\x13TopicMonitorMessage\x12\x0c\n\x04name\x18\x01 \x01(\t\x12\x10\n\x08has_rate\x18\x02 \x01(\x08\x12\x0c\n\x04rate\x18\x03 \x01(\x05\"2\n\x13ParamMonitorMessage\x12\x0c\n\x04name\x18\x01 \x01(\t\x12\r\n\x05value\x18\x02 \x01(\t\"M\n\x12NodeMonitorMessage\x12\x0c\n\x04name\x18\x01 \x01(\t\x12\x19\n\x11has_ping_response\x18\x02 \x01(\x08\x12\x0e\n\x06pinged\x18\x03 \x01(\x08\"%\n\x15ServiceMonitorMessage\x12\x0c\n\x04name\x18\x01 \x01(\t\"\x8c\x02\n\x11RosMonitorMessage\x12\n\n\x02ts\x18\x01 \x01(\x03\x12\x15\n\rmaster_status\x18\x02 \x01(\x05\x12\x33\n\rtopics_update\x18\x03 \x03(\x0b\x32\x1c.inorbit.TopicMonitorMessage\x12\x33\n\rparams_update\x18\x04 \x03(\x0b\x32\x1c.inorbit.ParamMonitorMessage\x12\x31\n\x0cnodes_update\x18\x05 \x03(\x0b\x32\x1b.inorbit.NodeMonitorMessage\x12\x37\n\x0fservices_update\x18\x06 \x03(\x0b\x32\x1e.inorbit.ServiceMonitorMessage\"B\n\rRosOutMessage\x12\n\n\x02ts\x18\x01 \x01(\x03\x12\x0b\n\x03log\x18\x02 \x01(\x0c\x12\x18\n\x10has_skipped_msgs\x18\x03 \x01(\x08\"2\n\x17\x43ustomCommandRosMessage\x12\n\n\x02ts\x18\x01 \x01(\x03\x12\x0b\n\x03\x63md\x18\x02 \x01(\t\"\x8c\x01\n\x1a\x43ustomScriptCommandMessage\x12\n\n\x02ts\x18\x01 \x01(\x03\x12\x11\n\tfile_name\x18\x02 \x01(\t\x12\x13\n\x0b\x61rg_options\x18\x03 \x03(\t\x12\x17\n\x0fscript_contents\x18\x04 \x01(\t\x12\x0b\n\x03run\x18\x05 \x01(\x08\x12\x14\n\x0c\x65xecution_id\x18\x06 \x01(\t\"\xc1\x01\n\x19\x43ustomScriptStatusMessage\x12\n\n\x02ts\x18\x01 \x01(\x03\x12\x11\n\tfile_name\x18\x02 \x01(\t\x12\x18\n\x10\x65xecution_status\x18\x03 \x01(\t\x12\x13\n\x0breturn_code\x18\x04 \x01(\t\x12\x0e\n\x06stdout\x18\x05 \x01(\t\x12\x0e\n\x06stderr\x18\x06 \x01(\t\x12\x14\n\x0c\x65xecution_id\x18\x07 \x01(\t\x12 \n\x18\x65xecution_status_details\x18\x08 \x01(\t\"U\n\x0fTeleopGoCommand\x12\x0f\n\x07ts_hint\x18\x01 \x01(\x03\x12\x17\n\x0flinear_velocity\x18\x02 \x01(\x02\x12\x18\n\x10\x61ngular_velocity\x18\x03 \x01(\x02\"-\n\x0fKeyValueMessage\x12\x0b\n\x03key\x18\x01 \x01(\t\x12\r\n\x05value\x18\x02 \x01(\t\"\x80\x01\n\x13RosDiagnosticsField\x12\x0c\n\x04name\x18\x01 \x01(\t\x12\r\n\x05level\x18\x02 \x01(\x05\x12\x0b\n\x03msg\x18\x03 \x01(\t\x12,\n\nkey_values\x18\x04 \x03(\x0b\x32\x18.inorbit.KeyValueMessage\x12\x11\n\thas_level\x18\x05 \x01(\x08\"Q\n\x15RosDiagnosticsMessage\x12\n\n\x02ts\x18\x01 \x01(\x03\x12,\n\x06\x66ields\x18\x02 \x03(\x0b\x32\x1c.inorbit.RosDiagnosticsField\"M\n\x1bRosDiagnosticsStatusMessage\x12\n\n\x02ts\x18\x01 \x01(\x03\x12\x0e\n\x06status\x18\x02 \x01(\x05\x12\x12\n\nhas_status\x18\x03 \x01(\x08\"2\n\x0cStateOptions\x12\x12\n\nstate_name\x18\x01 \x01(\t\x12\x0e\n\x06values\x18\x02 \x03(\t\"^\n\x19ModuleStateOptionsMessage\x12\x13\n\x0bmodule_name\x18\x01 \x01(\t\x12,\n\rstate_options\x18\x02 \x03(\x0b\x32\x15.inorbit.StateOptions\"\xb9\x01\n\rGpsFixMessage\x12\n\n\x02ts\x18\x01 \x01(\x03\x12\x10\n\x08latitude\x18\x02 \x01(\x01\x12\x11\n\tlongitude\x18\x03 \x01(\x01\x12\x10\n\x08\x61ltitude\x18\x04 \x01(\x01\x12\x0f\n\x07\x62\x65\x61ring\x18\x05 \x01(\x01\x12\x13\n\x0bhas_bearing\x18\x06 \x01(\x08\x12\x14\n\x0chas_accuracy\x18\x07 \x01(\x08\x12\x17\n\x0f\x61\x63\x63uracy_meters\x18\x08 \x01(\x02\x12\x10\n\x08\x66rame_id\x18\t \x01(\t*5\n\x12Nav2DWaypointFrame\x12\x0b\n\x07UNKNOWN\x10\x00\x12\x07\n\x03MAP\x10\x01\x12\t\n\x05ROBOT\x10\x02\x62\x06proto3')
)
_NAV2DWAYPOINTFRAME = _descriptor.EnumDescriptor(
  name='Nav2DWaypointFrame',
  full_name='inorbit.Nav2DWaypointFrame',
  filename=None,
  file=DESCRIPTOR,
  values=[
    _descriptor.EnumValueDescriptor(
      name='UNKNOWN', index=0, number=0,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='MAP', index=1, number=1,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='ROBOT', index=2, number=2,
      options=None,
      type=None),
  ],
  containing_type=None,
  options=None,
  serialized_start=5835,
  serialized_end=5888,
)
_sym_db.RegisterEnumDescriptor(_NAV2DWAYPOINTFRAME)
Nav2DWaypointFrame = enum_type_wrapper.EnumTypeWrapper(_NAV2DWAYPOINTFRAME)
UNKNOWN = 0
MAP = 1
ROBOT = 2
_DISKUSAGEMESSAGE = _descriptor.Descriptor(
  name='DiskUsageMessage',
  full_name='inorbit.DiskUsageMessage',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='volume_id', full_name='inorbit.DiskUsageMessage.volume_id', index=0,
      number=1, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='usage_percentage', full_name='inorbit.DiskUsageMessage.usage_percentage', index=1,
      number=2, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=26,
  serialized_end=89,
)
_NETWORKSTATSMESSAGE = _descriptor.Descriptor(
  name='NetworkStatsMessage',
  full_name='inorbit.NetworkStatsMessage',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='interface_id', full_name='inorbit.NetworkStatsMessage.interface_id', index=0,
      number=1, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='tx', full_name='inorbit.NetworkStatsMessage.tx', index=1,
      number=2, type=3, cpp_type=2, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='rx', full_name='inorbit.NetworkStatsMessage.rx', index=2,
      number=3, type=3, cpp_type=2, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=91,
  serialized_end=158,
)
_SYSTEMSTATSMESSAGE = _descriptor.Descriptor(
  name='SystemStatsMessage',
  full_name='inorbit.SystemStatsMessage',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='timestamp', full_name='inorbit.SystemStatsMessage.timestamp', index=0,
      number=1, type=3, cpp_type=2, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='elapsed_seconds', full_name='inorbit.SystemStatsMessage.elapsed_seconds', index=1,
      number=2, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='cpu_load_percentage', full_name='inorbit.SystemStatsMessage.cpu_load_percentage', index=2,
      number=3, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='network_interface', full_name='inorbit.SystemStatsMessage.network_interface', index=3,
      number=4, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='total_tx', full_name='inorbit.SystemStatsMessage.total_tx', index=4,
      number=5, type=3, cpp_type=2, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='total_rx', full_name='inorbit.SystemStatsMessage.total_rx', index=5,
      number=6, type=3, cpp_type=2, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='inorbit_tx', full_name='inorbit.SystemStatsMessage.inorbit_tx', index=6,
      number=7, type=3, cpp_type=2, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='inorbit_rx', full_name='inorbit.SystemStatsMessage.inorbit_rx', index=7,
      number=8, type=3, cpp_type=2, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='hdd_usage_percentage', full_name='inorbit.SystemStatsMessage.hdd_usage_percentage', index=8,
      number=9, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='inorbit_hdd_usage_mb', full_name='inorbit.SystemStatsMessage.inorbit_hdd_usage_mb', index=9,
      number=10, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='inorbit_hdd_usage_percentage', full_name='inorbit.SystemStatsMessage.inorbit_hdd_usage_percentage', index=10,
      number=11, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='optional_disks_data', full_name='inorbit.SystemStatsMessage.optional_disks_data', index=11,
      number=15, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='optional_network_interfaces_data', full_name='inorbit.SystemStatsMessage.optional_network_interfaces_data', index=12,
      number=16, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='ram_usage_percentage', full_name='inorbit.SystemStatsMessage.ram_usage_percentage', index=13,
      number=17, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='mqtt_tx', full_name='inorbit.SystemStatsMessage.mqtt_tx', index=14,
      number=18, type=3, cpp_type=2, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='mqtt_rx', full_name='inorbit.SystemStatsMessage.mqtt_rx', index=15,
      number=19, type=3, cpp_type=2, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=161,
  serialized_end=647,
)
_ODOMETRYDATAMESSAGE = _descriptor.Descriptor(
  name='OdometryDataMessage',
  full_name='inorbit.OdometryDataMessage',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='ts_start', full_name='inorbit.OdometryDataMessage.ts_start', index=0,
      number=1, type=3, cpp_type=2, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='ts', full_name='inorbit.OdometryDataMessage.ts', index=1,
      number=2, type=3, cpp_type=2, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='linear_distance', full_name='inorbit.OdometryDataMessage.linear_distance', index=2,
      number=3, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='angular_distance', full_name='inorbit.OdometryDataMessage.angular_distance', index=3,
      number=4, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='linear_speed', full_name='inorbit.OdometryDataMessage.linear_speed', index=4,
      number=5, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='angular_speed', full_name='inorbit.OdometryDataMessage.angular_speed', index=5,
      number=6, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='speed_available', full_name='inorbit.OdometryDataMessage.speed_available', index=6,
      number=7, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=650,
  serialized_end=822,
)
_PATHPOINT = _descriptor.Descriptor(
  name='PathPoint',
  full_name='inorbit.PathPoint',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='x', full_name='inorbit.PathPoint.x', index=0,
      number=1, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='y', full_name='inorbit.PathPoint.y', index=1,
      number=2, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=824,
  serialized_end=857,
)
_DELTAINTPATHPOINTS = _descriptor.Descriptor(
  name='DeltaIntPathPoints',
  full_name='inorbit.DeltaIntPathPoints',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='xs', full_name='inorbit.DeltaIntPathPoints.xs', index=0,
      number=1, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='ys', full_name='inorbit.DeltaIntPathPoints.ys', index=1,
      number=2, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='max_bits', full_name='inorbit.DeltaIntPathPoints.max_bits', index=2,
      number=3, type=5, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=859,
  serialized_end=969,
)
_ROBOTPATH = _descriptor.Descriptor(
  name='RobotPath',
  full_name='inorbit.RobotPath',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='points', full_name='inorbit.RobotPath.points', index=0,
      number=1, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='ts', full_name='inorbit.RobotPath.ts', index=1,
      number=2, type=3, cpp_type=2, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='path_id', full_name='inorbit.RobotPath.path_id', index=2,
      number=3, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='frame_id', full_name='inorbit.RobotPath.frame_id', index=3,
      number=4, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='encoding_version', full_name='inorbit.RobotPath.encoding_version', index=4,
      number=5, type=5, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='encoded_points', full_name='inorbit.RobotPath.encoded_points', index=5,
      number=6, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=972,
  serialized_end=1145,
)
_DELTAINTARRAY_STATSENTRY = _descriptor.Descriptor(
  name='StatsEntry',
  full_name='inorbit.DeltaIntArray.StatsEntry',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='key', full_name='inorbit.DeltaIntArray.StatsEntry.key', index=0,
      number=1, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='value', full_name='inorbit.DeltaIntArray.StatsEntry.value', index=1,
      number=2, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=_descriptor._ParseOptions(descriptor_pb2.MessageOptions(), _b('8\001')),
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=1265,
  serialized_end=1309,
)
_DELTAINTARRAY = _descriptor.Descriptor(
  name='DeltaIntArray',
  full_name='inorbit.DeltaIntArray',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='anchor', full_name='inorbit.DeltaIntArray.anchor', index=0,
      number=1, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='exponent', full_name='inorbit.DeltaIntArray.exponent', index=1,
      number=2, type=17, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='deltas', full_name='inorbit.DeltaIntArray.deltas', index=2,
      number=3, type=17, cpp_type=1, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='stats', full_name='inorbit.DeltaIntArray.stats', index=3,
      number=4, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[_DELTAINTARRAY_STATSENTRY, ],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=1148,
  serialized_end=1309,
)
_PATHDATAMESSAGE = _descriptor.Descriptor(
  name='PathDataMessage',
  full_name='inorbit.PathDataMessage',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='points', full_name='inorbit.PathDataMessage.points', index=0,
      number=1, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='ts', full_name='inorbit.PathDataMessage.ts', index=1,
      number=2, type=3, cpp_type=2, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='paths', full_name='inorbit.PathDataMessage.paths', index=2,
      number=3, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=1311,
  serialized_end=1411,
)
_MAPMESSAGE = _descriptor.Descriptor(
  name='MapMessage',
  full_name='inorbit.MapMessage',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='width', full_name='inorbit.MapMessage.width', index=0,
      number=1, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='height', full_name='inorbit.MapMessage.height', index=1,
      number=2, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='pixels', full_name='inorbit.MapMessage.pixels', index=2,
      number=3, type=12, cpp_type=9, label=1,
      has_default_value=False, default_value=_b(""),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='x', full_name='inorbit.MapMessage.x', index=3,
      number=4, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='y', full_name='inorbit.MapMessage.y', index=4,
      number=5, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='theta', full_name='inorbit.MapMessage.theta', index=5,
      number=6, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='resolution', full_name='inorbit.MapMessage.resolution', index=6,
      number=7, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='ts', full_name='inorbit.MapMessage.ts', index=7,
      number=8, type=3, cpp_type=2, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='label', full_name='inorbit.MapMessage.label', index=8,
      number=9, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='data_hash', full_name='inorbit.MapMessage.data_hash', index=9,
      number=10, type=3, cpp_type=2, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='frame_id', full_name='inorbit.MapMessage.frame_id', index=10,
      number=11, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='map_id', full_name='inorbit.MapMessage.map_id', index=11,
      number=12, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='is_update', full_name='inorbit.MapMessage.is_update', index=12,
      number=13, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=1414,
  serialized_end=1629,
)
_MAPREQUEST = _descriptor.Descriptor(
  name='MapRequest',
  full_name='inorbit.MapRequest',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='label', full_name='inorbit.MapRequest.label', index=0,
      number=9, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='data_hash', full_name='inorbit.MapRequest.data_hash', index=1,
      number=10, type=3, cpp_type=2, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=1631,
  serialized_end=1677,
)
_NAV2DWAYPOINTMESSAGE = _descriptor.Descriptor(
  name='Nav2DWaypointMessage',
  full_name='inorbit.Nav2DWaypointMessage',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='frame', full_name='inorbit.Nav2DWaypointMessage.frame', index=0,
      number=1, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='x', full_name='inorbit.Nav2DWaypointMessage.x', index=1,
      number=2, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='y', full_name='inorbit.Nav2DWaypointMessage.y', index=2,
      number=3, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='theta', full_name='inorbit.Nav2DWaypointMessage.theta', index=3,
      number=4, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=1679,
  serialized_end=1782,
)
_NAV2DPATHMESSAGE = _descriptor.Descriptor(
  name='Nav2DPathMessage',
  full_name='inorbit.Nav2DPathMessage',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='frame', full_name='inorbit.Nav2DPathMessage.frame', index=0,
      number=1, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='ts_hint', full_name='inorbit.Nav2DPathMessage.ts_hint', index=1,
      number=2, type=3, cpp_type=2, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='waypoints', full_name='inorbit.Nav2DPathMessage.waypoints', index=2,
      number=3, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=1785,
  serialized_end=1914,
)
_ALERTMESSAGE = _descriptor.Descriptor(
  name='AlertMessage',
  full_name='inorbit.AlertMessage',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='ts', full_name='inorbit.AlertMessage.ts', index=0,
      number=1, type=3, cpp_type=2, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='component_id', full_name='inorbit.AlertMessage.component_id', index=1,
      number=2, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='status', full_name='inorbit.AlertMessage.status', index=2,
      number=3, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='level', full_name='inorbit.AlertMessage.level', index=3,
      number=4, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='name', full_name='inorbit.AlertMessage.name', index=4,
      number=5, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='message', full_name='inorbit.AlertMessage.message', index=5,
      number=6, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=1916,
  serialized_end=2026,
)
_ECHO = _descriptor.Descriptor(
  name='Echo',
  full_name='inorbit.Echo',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='time_stamp', full_name='inorbit.Echo.time_stamp', index=0,
      number=1, type=3, cpp_type=2, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='topic', full_name='inorbit.Echo.topic', index=1,
      number=2, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='string_payload', full_name='inorbit.Echo.string_payload', index=2,
      number=3, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='byte_payload', full_name='inorbit.Echo.byte_payload', index=3,
      number=4, type=12, cpp_type=9, label=1,
      has_default_value=False, default_value=_b(""),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
    _descriptor.OneofDescriptor(
      name='payload', full_name='inorbit.Echo.payload',
      index=0, containing_type=None, fields=[]),
  ],
  serialized_start=2028,
  serialized_end=2130,
)
_DATABAGUPDATEMESSAGE_PROPERTIESENTRY = _descriptor.Descriptor(
  name='PropertiesEntry',
  full_name='inorbit.DatabagUpdateMessage.PropertiesEntry',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='key', full_name='inorbit.DatabagUpdateMessage.PropertiesEntry.key', index=0,
      number=1, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='value', full_name='inorbit.DatabagUpdateMessage.PropertiesEntry.value', index=1,
      number=2, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=_descriptor._ParseOptions(descriptor_pb2.MessageOptions(), _b('8\001')),
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=2371,
  serialized_end=2420,
)
_DATABAGUPDATEMESSAGE = _descriptor.Descriptor(
  name='DatabagUpdateMessage',
  full_name='inorbit.DatabagUpdateMessage',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='name', full_name='inorbit.DatabagUpdateMessage.name', index=0,
      number=1, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='stored_in_robot', full_name='inorbit.DatabagUpdateMessage.stored_in_robot', index=1,
      number=2, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='uploading_to_cloud', full_name='inorbit.DatabagUpdateMessage.uploading_to_cloud', index=2,
      number=3, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='url', full_name='inorbit.DatabagUpdateMessage.url', index=3,
      number=4, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='start_ts', full_name='inorbit.DatabagUpdateMessage.start_ts', index=4,
      number=5, type=3, cpp_type=2, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='end_ts', full_name='inorbit.DatabagUpdateMessage.end_ts', index=5,
      number=6, type=3, cpp_type=2, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='size_kb', full_name='inorbit.DatabagUpdateMessage.size_kb', index=6,
      number=7, type=5, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='topics', full_name='inorbit.DatabagUpdateMessage.topics', index=7,
      number=9, type=9, cpp_type=9, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='properties', full_name='inorbit.DatabagUpdateMessage.properties', index=8,
      number=8, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[_DATABAGUPDATEMESSAGE_PROPERTIESENTRY, ],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=2133,
  serialized_end=2420,
)
_ROSBAGUPDATEMESSAGE_PROPERTIESENTRY = _descriptor.Descriptor(
  name='PropertiesEntry',
  full_name='inorbit.RosbagUpdateMessage.PropertiesEntry',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='key', full_name='inorbit.RosbagUpdateMessage.PropertiesEntry.key', index=0,
      number=1, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='value', full_name='inorbit.RosbagUpdateMessage.PropertiesEntry.value', index=1,
      number=2, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=_descriptor._ParseOptions(descriptor_pb2.MessageOptions(), _b('8\001')),
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=2371,
  serialized_end=2420,
)
_ROSBAGUPDATEMESSAGE = _descriptor.Descriptor(
  name='RosbagUpdateMessage',
  full_name='inorbit.RosbagUpdateMessage',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='name', full_name='inorbit.RosbagUpdateMessage.name', index=0,
      number=1, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='stored_in_robot', full_name='inorbit.RosbagUpdateMessage.stored_in_robot', index=1,
      number=2, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='uploading_to_cloud', full_name='inorbit.RosbagUpdateMessage.uploading_to_cloud', index=2,
      number=3, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='url', full_name='inorbit.RosbagUpdateMessage.url', index=3,
      number=4, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='ts', full_name='inorbit.RosbagUpdateMessage.ts', index=4,
      number=5, type=3, cpp_type=2, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='start_ts', full_name='inorbit.RosbagUpdateMessage.start_ts', index=5,
      number=6, type=3, cpp_type=2, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='end_ts', full_name='inorbit.RosbagUpdateMessage.end_ts', index=6,
      number=7, type=3, cpp_type=2, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='size_kb', full_name='inorbit.RosbagUpdateMessage.size_kb', index=7,
      number=8, type=5, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='topics', full_name='inorbit.RosbagUpdateMessage.topics', index=8,
      number=9, type=9, cpp_type=9, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='properties', full_name='inorbit.RosbagUpdateMessage.properties', index=9,
      number=10, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[_ROSBAGUPDATEMESSAGE_PROPERTIESENTRY, ],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=2423,
  serialized_end=2720,
)
_LASERMESSAGE = _descriptor.Descriptor(
  name='LaserMessage',
  full_name='inorbit.LaserMessage',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='name', full_name='inorbit.LaserMessage.name', index=0,
      number=1, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='ranges', full_name='inorbit.LaserMessage.ranges', index=1,
      number=2, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=2722,
  serialized_end=2794,
)
_FLOATINGPOINTLIST = _descriptor.Descriptor(
  name='FloatingPointList',
  full_name='inorbit.FloatingPointList',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='runs', full_name='inorbit.FloatingPointList.runs', index=0,
      number=1, type=13, cpp_type=3, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='values', full_name='inorbit.FloatingPointList.values', index=1,
      number=2, type=2, cpp_type=6, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=2796,
  serialized_end=2845,
)
_LOCATIONANDPOSEMESSAGE = _descriptor.Descriptor(
  name='LocationAndPoseMessage',
  full_name='inorbit.LocationAndPoseMessage',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='ts', full_name='inorbit.LocationAndPoseMessage.ts', index=0,
      number=1, type=3, cpp_type=2, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='pos_x', full_name='inorbit.LocationAndPoseMessage.pos_x', index=1,
      number=2, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='pos_y', full_name='inorbit.LocationAndPoseMessage.pos_y', index=2,
      number=3, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='yaw', full_name='inorbit.LocationAndPoseMessage.yaw', index=3,
      number=4, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='frame_id', full_name='inorbit.LocationAndPoseMessage.frame_id', index=4,
      number=6, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='lasers', full_name='inorbit.LocationAndPoseMessage.lasers', index=5,
      number=5, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='offset_x', full_name='inorbit.LocationAndPoseMessage.offset_x', index=6,
      number=7, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='offset_y', full_name='inorbit.LocationAndPoseMessage.offset_y', index=7,
      number=8, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=2848,
  serialized_end=3020,
)
_POSEMESSAGE = _descriptor.Descriptor(
  name='PoseMessage',
  full_name='inorbit.PoseMessage',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='poses', full_name='inorbit.PoseMessage.poses', index=0,
      number=1, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=3022,
  serialized_end=3076,
)
_POSEMESSAGEDATA = _descriptor.Descriptor(
  name='PoseMessageData',
  full_name='inorbit.PoseMessageData',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='ts', full_name='inorbit.PoseMessageData.ts', index=0,
      number=1, type=3, cpp_type=2, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='pos_x', full_name='inorbit.PoseMessageData.pos_x', index=1,
      number=2, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='pos_y', full_name='inorbit.PoseMessageData.pos_y', index=2,
      number=3, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='yaw', full_name='inorbit.PoseMessageData.yaw', index=3,
      number=4, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='frame_id', full_name='inorbit.PoseMessageData.frame_id', index=4,
      number=5, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='offset_x', full_name='inorbit.PoseMessageData.offset_x', index=5,
      number=6, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='offset_y', full_name='inorbit.PoseMessageData.offset_y', index=6,
      number=7, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=3078,
  serialized_end=3204,
)
_CAMERAMESSAGE = _descriptor.Descriptor(
  name='CameraMessage',
  full_name='inorbit.CameraMessage',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='camera_id', full_name='inorbit.CameraMessage.camera_id', index=0,
      number=1, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='width', full_name='inorbit.CameraMessage.width', index=1,
      number=2, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='height', full_name='inorbit.CameraMessage.height', index=2,
      number=3, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='image', full_name='inorbit.CameraMessage.image', index=3,
      number=4, type=12, cpp_type=9, label=1,
      has_default_value=False, default_value=_b(""),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='ts', full_name='inorbit.CameraMessage.ts', index=4,
      number=5, type=3, cpp_type=2, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=3206,
  serialized_end=3298,
)
_ROBOTFILEDATA = _descriptor.Descriptor(
  name='RobotFileData',
  full_name='inorbit.RobotFileData',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='name', full_name='inorbit.RobotFileData.name', index=0,
      number=1, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='stored_in_robot', full_name='inorbit.RobotFileData.stored_in_robot', index=1,
      number=2, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='uploading_to_cloud', full_name='inorbit.RobotFileData.uploading_to_cloud', index=2,
      number=3, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='url', full_name='inorbit.RobotFileData.url', index=3,
      number=4, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='ts', full_name='inorbit.RobotFileData.ts', index=4,
      number=5, type=3, cpp_type=2, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='size', full_name='inorbit.RobotFileData.size', index=5,
      number=6, type=3, cpp_type=2, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='type', full_name='inorbit.RobotFileData.type', index=6,
      number=7, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=3301,
  serialized_end=3436,
)
_ROBOTFILESUPDATEMESSAGE = _descriptor.Descriptor(
  name='RobotFilesUpdateMessage',
  full_name='inorbit.RobotFilesUpdateMessage',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='artifacts', full_name='inorbit.RobotFilesUpdateMessage.artifacts', index=0,
      number=1, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='ts', full_name='inorbit.RobotFilesUpdateMessage.ts', index=1,
      number=2, type=3, cpp_type=2, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=3438,
  serialized_end=3518,
)
_KEYVALUECUSTOMELEMENT = _descriptor.Descriptor(
  name='KeyValueCustomElement',
  full_name='inorbit.KeyValueCustomElement',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='key', full_name='inorbit.KeyValueCustomElement.key', index=0,
      number=1, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='value', full_name='inorbit.KeyValueCustomElement.value', index=1,
      number=2, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='ts', full_name='inorbit.KeyValueCustomElement.ts', index=2,
      number=3, type=3, cpp_type=2, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=3520,
  serialized_end=3583,
)
_KEYVALUEPAIRS = _descriptor.Descriptor(
  name='KeyValuePairs',
  full_name='inorbit.KeyValuePairs',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='pairs', full_name='inorbit.KeyValuePairs.pairs', index=0,
      number=1, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=3585,
  serialized_end=3647,
)
_DIAGNOSTICSMESSAGE = _descriptor.Descriptor(
  name='DiagnosticsMessage',
  full_name='inorbit.DiagnosticsMessage',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='label', full_name='inorbit.DiagnosticsMessage.label', index=0,
      number=1, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='key', full_name='inorbit.DiagnosticsMessage.key', index=1,
      number=2, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='value', full_name='inorbit.DiagnosticsMessage.value', index=2,
      number=3, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=3649,
  serialized_end=3712,
)
_TEXTFILEMESSAGE = _descriptor.Descriptor(
  name='TextFileMessage',
  full_name='inorbit.TextFileMessage',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='inorbit.TextFileMessage.data', index=0,
      number=1, type=12, cpp_type=9, label=1,
      has_default_value=False, default_value=_b(""),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='blob_offset', full_name='inorbit.TextFileMessage.blob_offset', index=1,
      number=2, type=5, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='blob_size', full_name='inorbit.TextFileMessage.blob_size', index=2,
      number=3, type=5, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='total_file_size', full_name='inorbit.TextFileMessage.total_file_size', index=3,
      number=4, type=5, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=3714,
  serialized_end=3810,
)
_CUSTOMDATAMESSAGE = _descriptor.Descriptor(
  name='CustomDataMessage',
  full_name='inorbit.CustomDataMessage',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='custom_field', full_name='inorbit.CustomDataMessage.custom_field', index=0,
      number=1, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='key_value_payload', full_name='inorbit.CustomDataMessage.key_value_payload', index=1,
      number=2, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='image_payload', full_name='inorbit.CustomDataMessage.image_payload', index=2,
      number=3, type=12, cpp_type=9, label=1,
      has_default_value=False, default_value=_b(""),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='text_file_payload', full_name='inorbit.CustomDataMessage.text_file_payload', index=3,
      number=4, type=12, cpp_type=9, label=1,
      has_default_value=False, default_value=_b(""),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='diagnostics_payload', full_name='inorbit.CustomDataMessage.diagnostics_payload', index=4,
      number=5, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='text_file_payload_2', full_name='inorbit.CustomDataMessage.text_file_payload_2', index=5,
      number=6, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='ts', full_name='inorbit.CustomDataMessage.ts', index=6,
      number=7, type=3, cpp_type=2, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
    _descriptor.OneofDescriptor(
      name='payload', full_name='inorbit.CustomDataMessage.payload',
      index=0, containing_type=None, fields=[]),
  ],
  serialized_start=3813,
  serialized_end=4101,
)
_TOPICMONITORMESSAGE = _descriptor.Descriptor(
  name='TopicMonitorMessage',
  full_name='inorbit.TopicMonitorMessage',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='name', full_name='inorbit.TopicMonitorMessage.name', index=0,
      number=1, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='has_rate', full_name='inorbit.TopicMonitorMessage.has_rate', index=1,
      number=2, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='rate', full_name='inorbit.TopicMonitorMessage.rate', index=2,
      number=3, type=5, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=4103,
  serialized_end=4170,
)
_PARAMMONITORMESSAGE = _descriptor.Descriptor(
  name='ParamMonitorMessage',
  full_name='inorbit.ParamMonitorMessage',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='name', full_name='inorbit.ParamMonitorMessage.name', index=0,
      number=1, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='value', full_name='inorbit.ParamMonitorMessage.value', index=1,
      number=2, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=4172,
  serialized_end=4222,
)
_NODEMONITORMESSAGE = _descriptor.Descriptor(
  name='NodeMonitorMessage',
  full_name='inorbit.NodeMonitorMessage',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='name', full_name='inorbit.NodeMonitorMessage.name', index=0,
      number=1, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='has_ping_response', full_name='inorbit.NodeMonitorMessage.has_ping_response', index=1,
      number=2, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='pinged', full_name='inorbit.NodeMonitorMessage.pinged', index=2,
      number=3, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=4224,
  serialized_end=4301,
)
_SERVICEMONITORMESSAGE = _descriptor.Descriptor(
  name='ServiceMonitorMessage',
  full_name='inorbit.ServiceMonitorMessage',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='name', full_name='inorbit.ServiceMonitorMessage.name', index=0,
      number=1, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=4303,
  serialized_end=4340,
)
_ROSMONITORMESSAGE = _descriptor.Descriptor(
  name='RosMonitorMessage',
  full_name='inorbit.RosMonitorMessage',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='ts', full_name='inorbit.RosMonitorMessage.ts', index=0,
      number=1, type=3, cpp_type=2, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='master_status', full_name='inorbit.RosMonitorMessage.master_status', index=1,
      number=2, type=5, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='topics_update', full_name='inorbit.RosMonitorMessage.topics_update', index=2,
      number=3, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='params_update', full_name='inorbit.RosMonitorMessage.params_update', index=3,
      number=4, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='nodes_update', full_name='inorbit.RosMonitorMessage.nodes_update', index=4,
      number=5, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='services_update', full_name='inorbit.RosMonitorMessage.services_update', index=5,
      number=6, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=4343,
  serialized_end=4611,
)
_ROSOUTMESSAGE = _descriptor.Descriptor(
  name='RosOutMessage',
  full_name='inorbit.RosOutMessage',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='ts', full_name='inorbit.RosOutMessage.ts', index=0,
      number=1, type=3, cpp_type=2, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='log', full_name='inorbit.RosOutMessage.log', index=1,
      number=2, type=12, cpp_type=9, label=1,
      has_default_value=False, default_value=_b(""),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='has_skipped_msgs', full_name='inorbit.RosOutMessage.has_skipped_msgs', index=2,
      number=3, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=4613,
  serialized_end=4679,
)
_CUSTOMCOMMANDROSMESSAGE = _descriptor.Descriptor(
  name='CustomCommandRosMessage',
  full_name='inorbit.CustomCommandRosMessage',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='ts', full_name='inorbit.CustomCommandRosMessage.ts', index=0,
      number=1, type=3, cpp_type=2, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='cmd', full_name='inorbit.CustomCommandRosMessage.cmd', index=1,
      number=2, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=4681,
  serialized_end=4731,
)
_CUSTOMSCRIPTCOMMANDMESSAGE = _descriptor.Descriptor(
  name='CustomScriptCommandMessage',
  full_name='inorbit.CustomScriptCommandMessage',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='ts', full_name='inorbit.CustomScriptCommandMessage.ts', index=0,
      number=1, type=3, cpp_type=2, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='file_name', full_name='inorbit.CustomScriptCommandMessage.file_name', index=1,
      number=2, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='arg_options', full_name='inorbit.CustomScriptCommandMessage.arg_options', index=2,
      number=3, type=9, cpp_type=9, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='script_contents', full_name='inorbit.CustomScriptCommandMessage.script_contents', index=3,
      number=4, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='run', full_name='inorbit.CustomScriptCommandMessage.run', index=4,
      number=5, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='execution_id', full_name='inorbit.CustomScriptCommandMessage.execution_id', index=5,
      number=6, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=4734,
  serialized_end=4874,
)
_CUSTOMSCRIPTSTATUSMESSAGE = _descriptor.Descriptor(
  name='CustomScriptStatusMessage',
  full_name='inorbit.CustomScriptStatusMessage',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='ts', full_name='inorbit.CustomScriptStatusMessage.ts', index=0,
      number=1, type=3, cpp_type=2, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='file_name', full_name='inorbit.CustomScriptStatusMessage.file_name', index=1,
      number=2, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='execution_status', full_name='inorbit.CustomScriptStatusMessage.execution_status', index=2,
      number=3, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='return_code', full_name='inorbit.CustomScriptStatusMessage.return_code', index=3,
      number=4, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='stdout', full_name='inorbit.CustomScriptStatusMessage.stdout', index=4,
      number=5, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='stderr', full_name='inorbit.CustomScriptStatusMessage.stderr', index=5,
      number=6, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='execution_id', full_name='inorbit.CustomScriptStatusMessage.execution_id', index=6,
      number=7, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='execution_status_details', full_name='inorbit.CustomScriptStatusMessage.execution_status_details', index=7,
      number=8, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=4877,
  serialized_end=5070,
)
_TELEOPGOCOMMAND = _descriptor.Descriptor(
  name='TeleopGoCommand',
  full_name='inorbit.TeleopGoCommand',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='ts_hint', full_name='inorbit.TeleopGoCommand.ts_hint', index=0,
      number=1, type=3, cpp_type=2, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='linear_velocity', full_name='inorbit.TeleopGoCommand.linear_velocity', index=1,
      number=2, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='angular_velocity', full_name='inorbit.TeleopGoCommand.angular_velocity', index=2,
      number=3, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=5072,
  serialized_end=5157,
)
_KEYVALUEMESSAGE = _descriptor.Descriptor(
  name='KeyValueMessage',
  full_name='inorbit.KeyValueMessage',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='key', full_name='inorbit.KeyValueMessage.key', index=0,
      number=1, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='value', full_name='inorbit.KeyValueMessage.value', index=1,
      number=2, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=5159,
  serialized_end=5204,
)
_ROSDIAGNOSTICSFIELD = _descriptor.Descriptor(
  name='RosDiagnosticsField',
  full_name='inorbit.RosDiagnosticsField',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='name', full_name='inorbit.RosDiagnosticsField.name', index=0,
      number=1, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='level', full_name='inorbit.RosDiagnosticsField.level', index=1,
      number=2, type=5, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='msg', full_name='inorbit.RosDiagnosticsField.msg', index=2,
      number=3, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='key_values', full_name='inorbit.RosDiagnosticsField.key_values', index=3,
      number=4, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='has_level', full_name='inorbit.RosDiagnosticsField.has_level', index=4,
      number=5, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=5207,
  serialized_end=5335,
)
_ROSDIAGNOSTICSMESSAGE = _descriptor.Descriptor(
  name='RosDiagnosticsMessage',
  full_name='inorbit.RosDiagnosticsMessage',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='ts', full_name='inorbit.RosDiagnosticsMessage.ts', index=0,
      number=1, type=3, cpp_type=2, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='fields', full_name='inorbit.RosDiagnosticsMessage.fields', index=1,
      number=2, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=5337,
  serialized_end=5418,
)
_ROSDIAGNOSTICSSTATUSMESSAGE = _descriptor.Descriptor(
  name='RosDiagnosticsStatusMessage',
  full_name='inorbit.RosDiagnosticsStatusMessage',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='ts', full_name='inorbit.RosDiagnosticsStatusMessage.ts', index=0,
      number=1, type=3, cpp_type=2, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='status', full_name='inorbit.RosDiagnosticsStatusMessage.status', index=1,
      number=2, type=5, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='has_status', full_name='inorbit.RosDiagnosticsStatusMessage.has_status', index=2,
      number=3, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=5420,
  serialized_end=5497,
)
_STATEOPTIONS = _descriptor.Descriptor(
  name='StateOptions',
  full_name='inorbit.StateOptions',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='state_name', full_name='inorbit.StateOptions.state_name', index=0,
      number=1, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='values', full_name='inorbit.StateOptions.values', index=1,
      number=2, type=9, cpp_type=9, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=5499,
  serialized_end=5549,
)
_MODULESTATEOPTIONSMESSAGE = _descriptor.Descriptor(
  name='ModuleStateOptionsMessage',
  full_name='inorbit.ModuleStateOptionsMessage',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='module_name', full_name='inorbit.ModuleStateOptionsMessage.module_name', index=0,
      number=1, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='state_options', full_name='inorbit.ModuleStateOptionsMessage.state_options', index=1,
      number=2, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=5551,
  serialized_end=5645,
)
_GPSFIXMESSAGE = _descriptor.Descriptor(
  name='GpsFixMessage',
  full_name='inorbit.GpsFixMessage',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='ts', full_name='inorbit.GpsFixMessage.ts', index=0,
      number=1, type=3, cpp_type=2, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='latitude', full_name='inorbit.GpsFixMessage.latitude', index=1,
      number=2, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='longitude', full_name='inorbit.GpsFixMessage.longitude', index=2,
      number=3, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='altitude', full_name='inorbit.GpsFixMessage.altitude', index=3,
      number=4, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='bearing', full_name='inorbit.GpsFixMessage.bearing', index=4,
      number=5, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='has_bearing', full_name='inorbit.GpsFixMessage.has_bearing', index=5,
      number=6, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='has_accuracy', full_name='inorbit.GpsFixMessage.has_accuracy', index=6,
      number=7, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='accuracy_meters', full_name='inorbit.GpsFixMessage.accuracy_meters', index=7,
      number=8, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='frame_id', full_name='inorbit.GpsFixMessage.frame_id', index=8,
      number=9, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=5648,
  serialized_end=5833,
)
_SYSTEMSTATSMESSAGE.fields_by_name['optional_disks_data'].message_type = _DISKUSAGEMESSAGE
_SYSTEMSTATSMESSAGE.fields_by_name['optional_network_interfaces_data'].message_type = _NETWORKSTATSMESSAGE
_DELTAINTPATHPOINTS.fields_by_name['xs'].message_type = _DELTAINTARRAY
_DELTAINTPATHPOINTS.fields_by_name['ys'].message_type = _DELTAINTARRAY
_ROBOTPATH.fields_by_name['points'].message_type = _PATHPOINT
_ROBOTPATH.fields_by_name['encoded_points'].message_type = _DELTAINTPATHPOINTS
_DELTAINTARRAY_STATSENTRY.containing_type = _DELTAINTARRAY
_DELTAINTARRAY.fields_by_name['stats'].message_type = _DELTAINTARRAY_STATSENTRY
_PATHDATAMESSAGE.fields_by_name['points'].message_type = _PATHPOINT
_PATHDATAMESSAGE.fields_by_name['paths'].message_type = _ROBOTPATH
_NAV2DWAYPOINTMESSAGE.fields_by_name['frame'].enum_type = _NAV2DWAYPOINTFRAME
_NAV2DPATHMESSAGE.fields_by_name['frame'].enum_type = _NAV2DWAYPOINTFRAME
_NAV2DPATHMESSAGE.fields_by_name['waypoints'].message_type = _NAV2DWAYPOINTMESSAGE
_ECHO.oneofs_by_name['payload'].fields.append(
  _ECHO.fields_by_name['string_payload'])
_ECHO.fields_by_name['string_payload'].containing_oneof = _ECHO.oneofs_by_name['payload']
_ECHO.oneofs_by_name['payload'].fields.append(
  _ECHO.fields_by_name['byte_payload'])
_ECHO.fields_by_name['byte_payload'].containing_oneof = _ECHO.oneofs_by_name['payload']
_DATABAGUPDATEMESSAGE_PROPERTIESENTRY.containing_type = _DATABAGUPDATEMESSAGE
_DATABAGUPDATEMESSAGE.fields_by_name['properties'].message_type = _DATABAGUPDATEMESSAGE_PROPERTIESENTRY
_ROSBAGUPDATEMESSAGE_PROPERTIESENTRY.containing_type = _ROSBAGUPDATEMESSAGE
_ROSBAGUPDATEMESSAGE.fields_by_name['properties'].message_type = _ROSBAGUPDATEMESSAGE_PROPERTIESENTRY
_LASERMESSAGE.fields_by_name['ranges'].message_type = _FLOATINGPOINTLIST
_LOCATIONANDPOSEMESSAGE.fields_by_name['lasers'].message_type = _LASERMESSAGE
_POSEMESSAGE.fields_by_name['poses'].message_type = _POSEMESSAGEDATA
_ROBOTFILESUPDATEMESSAGE.fields_by_name['artifacts'].message_type = _ROBOTFILEDATA
_KEYVALUEPAIRS.fields_by_name['pairs'].message_type = _KEYVALUECUSTOMELEMENT
_CUSTOMDATAMESSAGE.fields_by_name['key_value_payload'].message_type = _KEYVALUEPAIRS
_CUSTOMDATAMESSAGE.fields_by_name['diagnostics_payload'].message_type = _DIAGNOSTICSMESSAGE
_CUSTOMDATAMESSAGE.fields_by_name['text_file_payload_2'].message_type = _TEXTFILEMESSAGE
_CUSTOMDATAMESSAGE.oneofs_by_name['payload'].fields.append(
  _CUSTOMDATAMESSAGE.fields_by_name['key_value_payload'])
_CUSTOMDATAMESSAGE.fields_by_name['key_value_payload'].containing_oneof = _CUSTOMDATAMESSAGE.oneofs_by_name['payload']
_CUSTOMDATAMESSAGE.oneofs_by_name['payload'].fields.append(
  _CUSTOMDATAMESSAGE.fields_by_name['image_payload'])
_CUSTOMDATAMESSAGE.fields_by_name['image_payload'].containing_oneof = _CUSTOMDATAMESSAGE.oneofs_by_name['payload']
_CUSTOMDATAMESSAGE.oneofs_by_name['payload'].fields.append(
  _CUSTOMDATAMESSAGE.fields_by_name['text_file_payload'])
_CUSTOMDATAMESSAGE.fields_by_name['text_file_payload'].containing_oneof = _CUSTOMDATAMESSAGE.oneofs_by_name['payload']
_CUSTOMDATAMESSAGE.oneofs_by_name['payload'].fields.append(
  _CUSTOMDATAMESSAGE.fields_by_name['diagnostics_payload'])
_CUSTOMDATAMESSAGE.fields_by_name['diagnostics_payload'].containing_oneof = _CUSTOMDATAMESSAGE.oneofs_by_name['payload']
_CUSTOMDATAMESSAGE.oneofs_by_name['payload'].fields.append(
  _CUSTOMDATAMESSAGE.fields_by_name['text_file_payload_2'])
_CUSTOMDATAMESSAGE.fields_by_name['text_file_payload_2'].containing_oneof = _CUSTOMDATAMESSAGE.oneofs_by_name['payload']
_ROSMONITORMESSAGE.fields_by_name['topics_update'].message_type = _TOPICMONITORMESSAGE
_ROSMONITORMESSAGE.fields_by_name['params_update'].message_type = _PARAMMONITORMESSAGE
_ROSMONITORMESSAGE.fields_by_name['nodes_update'].message_type = _NODEMONITORMESSAGE
_ROSMONITORMESSAGE.fields_by_name['services_update'].message_type = _SERVICEMONITORMESSAGE
_ROSDIAGNOSTICSFIELD.fields_by_name['key_values'].message_type = _KEYVALUEMESSAGE
_ROSDIAGNOSTICSMESSAGE.fields_by_name['fields'].message_type = _ROSDIAGNOSTICSFIELD
_MODULESTATEOPTIONSMESSAGE.fields_by_name['state_options'].message_type = _STATEOPTIONS
DESCRIPTOR.message_types_by_name['DiskUsageMessage'] = _DISKUSAGEMESSAGE
DESCRIPTOR.message_types_by_name['NetworkStatsMessage'] = _NETWORKSTATSMESSAGE
DESCRIPTOR.message_types_by_name['SystemStatsMessage'] = _SYSTEMSTATSMESSAGE
DESCRIPTOR.message_types_by_name['OdometryDataMessage'] = _ODOMETRYDATAMESSAGE
DESCRIPTOR.message_types_by_name['PathPoint'] = _PATHPOINT
DESCRIPTOR.message_types_by_name['DeltaIntPathPoints'] = _DELTAINTPATHPOINTS
DESCRIPTOR.message_types_by_name['RobotPath'] = _ROBOTPATH
DESCRIPTOR.message_types_by_name['DeltaIntArray'] = _DELTAINTARRAY
DESCRIPTOR.message_types_by_name['PathDataMessage'] = _PATHDATAMESSAGE
DESCRIPTOR.message_types_by_name['MapMessage'] = _MAPMESSAGE
DESCRIPTOR.message_types_by_name['MapRequest'] = _MAPREQUEST
DESCRIPTOR.message_types_by_name['Nav2DWaypointMessage'] = _NAV2DWAYPOINTMESSAGE
DESCRIPTOR.message_types_by_name['Nav2DPathMessage'] = _NAV2DPATHMESSAGE
DESCRIPTOR.message_types_by_name['AlertMessage'] = _ALERTMESSAGE
DESCRIPTOR.message_types_by_name['Echo'] = _ECHO
DESCRIPTOR.message_types_by_name['DatabagUpdateMessage'] = _DATABAGUPDATEMESSAGE
DESCRIPTOR.message_types_by_name['RosbagUpdateMessage'] = _ROSBAGUPDATEMESSAGE
DESCRIPTOR.message_types_by_name['LaserMessage'] = _LASERMESSAGE
DESCRIPTOR.message_types_by_name['FloatingPointList'] = _FLOATINGPOINTLIST
DESCRIPTOR.message_types_by_name['LocationAndPoseMessage'] = _LOCATIONANDPOSEMESSAGE
DESCRIPTOR.message_types_by_name['PoseMessage'] = _POSEMESSAGE
DESCRIPTOR.message_types_by_name['PoseMessageData'] = _POSEMESSAGEDATA
DESCRIPTOR.message_types_by_name['CameraMessage'] = _CAMERAMESSAGE
DESCRIPTOR.message_types_by_name['RobotFileData'] = _ROBOTFILEDATA
DESCRIPTOR.message_types_by_name['RobotFilesUpdateMessage'] = _ROBOTFILESUPDATEMESSAGE
DESCRIPTOR.message_types_by_name['KeyValueCustomElement'] = _KEYVALUECUSTOMELEMENT
DESCRIPTOR.message_types_by_name['KeyValuePairs'] = _KEYVALUEPAIRS
DESCRIPTOR.message_types_by_name['DiagnosticsMessage'] = _DIAGNOSTICSMESSAGE
DESCRIPTOR.message_types_by_name['TextFileMessage'] = _TEXTFILEMESSAGE
DESCRIPTOR.message_types_by_name['CustomDataMessage'] = _CUSTOMDATAMESSAGE
DESCRIPTOR.message_types_by_name['TopicMonitorMessage'] = _TOPICMONITORMESSAGE
DESCRIPTOR.message_types_by_name['ParamMonitorMessage'] = _PARAMMONITORMESSAGE
DESCRIPTOR.message_types_by_name['NodeMonitorMessage'] = _NODEMONITORMESSAGE
DESCRIPTOR.message_types_by_name['ServiceMonitorMessage'] = _SERVICEMONITORMESSAGE
DESCRIPTOR.message_types_by_name['RosMonitorMessage'] = _ROSMONITORMESSAGE
DESCRIPTOR.message_types_by_name['RosOutMessage'] = _ROSOUTMESSAGE
DESCRIPTOR.message_types_by_name['CustomCommandRosMessage'] = _CUSTOMCOMMANDROSMESSAGE
DESCRIPTOR.message_types_by_name['CustomScriptCommandMessage'] = _CUSTOMSCRIPTCOMMANDMESSAGE
DESCRIPTOR.message_types_by_name['CustomScriptStatusMessage'] = _CUSTOMSCRIPTSTATUSMESSAGE
DESCRIPTOR.message_types_by_name['TeleopGoCommand'] = _TELEOPGOCOMMAND
DESCRIPTOR.message_types_by_name['KeyValueMessage'] = _KEYVALUEMESSAGE
DESCRIPTOR.message_types_by_name['RosDiagnosticsField'] = _ROSDIAGNOSTICSFIELD
DESCRIPTOR.message_types_by_name['RosDiagnosticsMessage'] = _ROSDIAGNOSTICSMESSAGE
DESCRIPTOR.message_types_by_name['RosDiagnosticsStatusMessage'] = _ROSDIAGNOSTICSSTATUSMESSAGE
DESCRIPTOR.message_types_by_name['StateOptions'] = _STATEOPTIONS
DESCRIPTOR.message_types_by_name['ModuleStateOptionsMessage'] = _MODULESTATEOPTIONSMESSAGE
DESCRIPTOR.message_types_by_name['GpsFixMessage'] = _GPSFIXMESSAGE
DESCRIPTOR.enum_types_by_name['Nav2DWaypointFrame'] = _NAV2DWAYPOINTFRAME
_sym_db.RegisterFileDescriptor(DESCRIPTOR)
DiskUsageMessage = _reflection.GeneratedProtocolMessageType('DiskUsageMessage', (_message.Message,), dict(
  DESCRIPTOR = _DISKUSAGEMESSAGE,
  __module__ = 'inorbit_pb2'
  # @@protoc_insertion_point(class_scope:inorbit.DiskUsageMessage)
  ))
_sym_db.RegisterMessage(DiskUsageMessage)
NetworkStatsMessage = _reflection.GeneratedProtocolMessageType('NetworkStatsMessage', (_message.Message,), dict(
  DESCRIPTOR = _NETWORKSTATSMESSAGE,
  __module__ = 'inorbit_pb2'
  # @@protoc_insertion_point(class_scope:inorbit.NetworkStatsMessage)
  ))
_sym_db.RegisterMessage(NetworkStatsMessage)
SystemStatsMessage = _reflection.GeneratedProtocolMessageType('SystemStatsMessage', (_message.Message,), dict(
  DESCRIPTOR = _SYSTEMSTATSMESSAGE,
  __module__ = 'inorbit_pb2'
  # @@protoc_insertion_point(class_scope:inorbit.SystemStatsMessage)
  ))
_sym_db.RegisterMessage(SystemStatsMessage)
OdometryDataMessage = _reflection.GeneratedProtocolMessageType('OdometryDataMessage', (_message.Message,), dict(
  DESCRIPTOR = _ODOMETRYDATAMESSAGE,
  __module__ = 'inorbit_pb2'
  # @@protoc_insertion_point(class_scope:inorbit.OdometryDataMessage)
  ))
_sym_db.RegisterMessage(OdometryDataMessage)
PathPoint = _reflection.GeneratedProtocolMessageType('PathPoint', (_message.Message,), dict(
  DESCRIPTOR = _PATHPOINT,
  __module__ = 'inorbit_pb2'
  # @@protoc_insertion_point(class_scope:inorbit.PathPoint)
  ))
_sym_db.RegisterMessage(PathPoint)
DeltaIntPathPoints = _reflection.GeneratedProtocolMessageType('DeltaIntPathPoints', (_message.Message,), dict(
  DESCRIPTOR = _DELTAINTPATHPOINTS,
  __module__ = 'inorbit_pb2'
  # @@protoc_insertion_point(class_scope:inorbit.DeltaIntPathPoints)
  ))
_sym_db.RegisterMessage(DeltaIntPathPoints)
RobotPath = _reflection.GeneratedProtocolMessageType('RobotPath', (_message.Message,), dict(
  DESCRIPTOR = _ROBOTPATH,
  __module__ = 'inorbit_pb2'
  # @@protoc_insertion_point(class_scope:inorbit.RobotPath)
  ))
_sym_db.RegisterMessage(RobotPath)
DeltaIntArray = _reflection.GeneratedProtocolMessageType('DeltaIntArray', (_message.Message,), dict(
  StatsEntry = _reflection.GeneratedProtocolMessageType('StatsEntry', (_message.Message,), dict(
    DESCRIPTOR = _DELTAINTARRAY_STATSENTRY,
    __module__ = 'inorbit_pb2'
    # @@protoc_insertion_point(class_scope:inorbit.DeltaIntArray.StatsEntry)
    ))
  ,
  DESCRIPTOR = _DELTAINTARRAY,
  __module__ = 'inorbit_pb2'
  # @@protoc_insertion_point(class_scope:inorbit.DeltaIntArray)
  ))
_sym_db.RegisterMessage(DeltaIntArray)
_sym_db.RegisterMessage(DeltaIntArray.StatsEntry)
PathDataMessage = _reflection.GeneratedProtocolMessageType('PathDataMessage', (_message.Message,), dict(
  DESCRIPTOR = _PATHDATAMESSAGE,
  __module__ = 'inorbit_pb2'
  # @@protoc_insertion_point(class_scope:inorbit.PathDataMessage)
  ))
_sym_db.RegisterMessage(PathDataMessage)
MapMessage = _reflection.GeneratedProtocolMessageType('MapMessage', (_message.Message,), dict(
  DESCRIPTOR = _MAPMESSAGE,
  __module__ = 'inorbit_pb2'
  # @@protoc_insertion_point(class_scope:inorbit.MapMessage)
  ))
_sym_db.RegisterMessage(MapMessage)
MapRequest = _reflection.GeneratedProtocolMessageType('MapRequest', (_message.Message,), dict(
  DESCRIPTOR = _MAPREQUEST,
  __module__ = 'inorbit_pb2'
  # @@protoc_insertion_point(class_scope:inorbit.MapRequest)
  ))
_sym_db.RegisterMessage(MapRequest)
Nav2DWaypointMessage = _reflection.GeneratedProtocolMessageType('Nav2DWaypointMessage', (_message.Message,), dict(
  DESCRIPTOR = _NAV2DWAYPOINTMESSAGE,
  __module__ = 'inorbit_pb2'
  # @@protoc_insertion_point(class_scope:inorbit.Nav2DWaypointMessage)
  ))
_sym_db.RegisterMessage(Nav2DWaypointMessage)
Nav2DPathMessage = _reflection.GeneratedProtocolMessageType('Nav2DPathMessage', (_message.Message,), dict(
  DESCRIPTOR = _NAV2DPATHMESSAGE,
  __module__ = 'inorbit_pb2'
  # @@protoc_insertion_point(class_scope:inorbit.Nav2DPathMessage)
  ))
_sym_db.RegisterMessage(Nav2DPathMessage)
AlertMessage = _reflection.GeneratedProtocolMessageType('AlertMessage', (_message.Message,), dict(
  DESCRIPTOR = _ALERTMESSAGE,
  __module__ = 'inorbit_pb2'
  # @@protoc_insertion_point(class_scope:inorbit.AlertMessage)
  ))
_sym_db.RegisterMessage(AlertMessage)
Echo = _reflection.GeneratedProtocolMessageType('Echo', (_message.Message,), dict(
  DESCRIPTOR = _ECHO,
  __module__ = 'inorbit_pb2'
  # @@protoc_insertion_point(class_scope:inorbit.Echo)
  ))
_sym_db.RegisterMessage(Echo)
DatabagUpdateMessage = _reflection.GeneratedProtocolMessageType('DatabagUpdateMessage', (_message.Message,), dict(
  PropertiesEntry = _reflection.GeneratedProtocolMessageType('PropertiesEntry', (_message.Message,), dict(
    DESCRIPTOR = _DATABAGUPDATEMESSAGE_PROPERTIESENTRY,
    __module__ = 'inorbit_pb2'
    # @@protoc_insertion_point(class_scope:inorbit.DatabagUpdateMessage.PropertiesEntry)
    ))
  ,
  DESCRIPTOR = _DATABAGUPDATEMESSAGE,
  __module__ = 'inorbit_pb2'
  # @@protoc_insertion_point(class_scope:inorbit.DatabagUpdateMessage)
  ))
_sym_db.RegisterMessage(DatabagUpdateMessage)
_sym_db.RegisterMessage(DatabagUpdateMessage.PropertiesEntry)
RosbagUpdateMessage = _reflection.GeneratedProtocolMessageType('RosbagUpdateMessage', (_message.Message,), dict(
  PropertiesEntry = _reflection.GeneratedProtocolMessageType('PropertiesEntry', (_message.Message,), dict(
    DESCRIPTOR = _ROSBAGUPDATEMESSAGE_PROPERTIESENTRY,
    __module__ = 'inorbit_pb2'
    # @@protoc_insertion_point(class_scope:inorbit.RosbagUpdateMessage.PropertiesEntry)
    ))
  ,
  DESCRIPTOR = _ROSBAGUPDATEMESSAGE,
  __module__ = 'inorbit_pb2'
  # @@protoc_insertion_point(class_scope:inorbit.RosbagUpdateMessage)
  ))
_sym_db.RegisterMessage(RosbagUpdateMessage)
_sym_db.RegisterMessage(RosbagUpdateMessage.PropertiesEntry)
LaserMessage = _reflection.GeneratedProtocolMessageType('LaserMessage', (_message.Message,), dict(
  DESCRIPTOR = _LASERMESSAGE,
  __module__ = 'inorbit_pb2'
  # @@protoc_insertion_point(class_scope:inorbit.LaserMessage)
  ))
_sym_db.RegisterMessage(LaserMessage)
FloatingPointList = _reflection.GeneratedProtocolMessageType('FloatingPointList', (_message.Message,), dict(
  DESCRIPTOR = _FLOATINGPOINTLIST,
  __module__ = 'inorbit_pb2'
  # @@protoc_insertion_point(class_scope:inorbit.FloatingPointList)
  ))
_sym_db.RegisterMessage(FloatingPointList)
LocationAndPoseMessage = _reflection.GeneratedProtocolMessageType('LocationAndPoseMessage', (_message.Message,), dict(
  DESCRIPTOR = _LOCATIONANDPOSEMESSAGE,
  __module__ = 'inorbit_pb2'
  # @@protoc_insertion_point(class_scope:inorbit.LocationAndPoseMessage)
  ))
_sym_db.RegisterMessage(LocationAndPoseMessage)
PoseMessage = _reflection.GeneratedProtocolMessageType('PoseMessage', (_message.Message,), dict(
  DESCRIPTOR = _POSEMESSAGE,
  __module__ = 'inorbit_pb2'
  # @@protoc_insertion_point(class_scope:inorbit.PoseMessage)
  ))
_sym_db.RegisterMessage(PoseMessage)
PoseMessageData = _reflection.GeneratedProtocolMessageType('PoseMessageData', (_message.Message,), dict(
  DESCRIPTOR = _POSEMESSAGEDATA,
  __module__ = 'inorbit_pb2'
  # @@protoc_insertion_point(class_scope:inorbit.PoseMessageData)
  ))
_sym_db.RegisterMessage(PoseMessageData)
CameraMessage = _reflection.GeneratedProtocolMessageType('CameraMessage', (_message.Message,), dict(
  DESCRIPTOR = _CAMERAMESSAGE,
  __module__ = 'inorbit_pb2'
  # @@protoc_insertion_point(class_scope:inorbit.CameraMessage)
  ))
_sym_db.RegisterMessage(CameraMessage)
RobotFileData = _reflection.GeneratedProtocolMessageType('RobotFileData', (_message.Message,), dict(
  DESCRIPTOR = _ROBOTFILEDATA,
  __module__ = 'inorbit_pb2'
  # @@protoc_insertion_point(class_scope:inorbit.RobotFileData)
  ))
_sym_db.RegisterMessage(RobotFileData)
RobotFilesUpdateMessage = _reflection.GeneratedProtocolMessageType('RobotFilesUpdateMessage', (_message.Message,), dict(
  DESCRIPTOR = _ROBOTFILESUPDATEMESSAGE,
  __module__ = 'inorbit_pb2'
  # @@protoc_insertion_point(class_scope:inorbit.RobotFilesUpdateMessage)
  ))
_sym_db.RegisterMessage(RobotFilesUpdateMessage)
KeyValueCustomElement = _reflection.GeneratedProtocolMessageType('KeyValueCustomElement', (_message.Message,), dict(
  DESCRIPTOR = _KEYVALUECUSTOMELEMENT,
  __module__ = 'inorbit_pb2'
  # @@protoc_insertion_point(class_scope:inorbit.KeyValueCustomElement)
  ))
_sym_db.RegisterMessage(KeyValueCustomElement)
KeyValuePairs = _reflection.GeneratedProtocolMessageType('KeyValuePairs', (_message.Message,), dict(
  DESCRIPTOR = _KEYVALUEPAIRS,
  __module__ = 'inorbit_pb2'
  # @@protoc_insertion_point(class_scope:inorbit.KeyValuePairs)
  ))
_sym_db.RegisterMessage(KeyValuePairs)
DiagnosticsMessage = _reflection.GeneratedProtocolMessageType('DiagnosticsMessage', (_message.Message,), dict(
  DESCRIPTOR = _DIAGNOSTICSMESSAGE,
  __module__ = 'inorbit_pb2'
  # @@protoc_insertion_point(class_scope:inorbit.DiagnosticsMessage)
  ))
_sym_db.RegisterMessage(DiagnosticsMessage)
TextFileMessage = _reflection.GeneratedProtocolMessageType('TextFileMessage', (_message.Message,), dict(
  DESCRIPTOR = _TEXTFILEMESSAGE,
  __module__ = 'inorbit_pb2'
  # @@protoc_insertion_point(class_scope:inorbit.TextFileMessage)
  ))
_sym_db.RegisterMessage(TextFileMessage)
CustomDataMessage = _reflection.GeneratedProtocolMessageType('CustomDataMessage', (_message.Message,), dict(
  DESCRIPTOR = _CUSTOMDATAMESSAGE,
  __module__ = 'inorbit_pb2'
  # @@protoc_insertion_point(class_scope:inorbit.CustomDataMessage)
  ))
_sym_db.RegisterMessage(CustomDataMessage)
TopicMonitorMessage = _reflection.GeneratedProtocolMessageType('TopicMonitorMessage', (_message.Message,), dict(
  DESCRIPTOR = _TOPICMONITORMESSAGE,
  __module__ = 'inorbit_pb2'
  # @@protoc_insertion_point(class_scope:inorbit.TopicMonitorMessage)
  ))
_sym_db.RegisterMessage(TopicMonitorMessage)
ParamMonitorMessage = _reflection.GeneratedProtocolMessageType('ParamMonitorMessage', (_message.Message,), dict(
  DESCRIPTOR = _PARAMMONITORMESSAGE,
  __module__ = 'inorbit_pb2'
  # @@protoc_insertion_point(class_scope:inorbit.ParamMonitorMessage)
  ))
_sym_db.RegisterMessage(ParamMonitorMessage)
NodeMonitorMessage = _reflection.GeneratedProtocolMessageType('NodeMonitorMessage', (_message.Message,), dict(
  DESCRIPTOR = _NODEMONITORMESSAGE,
  __module__ = 'inorbit_pb2'
  # @@protoc_insertion_point(class_scope:inorbit.NodeMonitorMessage)
  ))
_sym_db.RegisterMessage(NodeMonitorMessage)
ServiceMonitorMessage = _reflection.GeneratedProtocolMessageType('ServiceMonitorMessage', (_message.Message,), dict(
  DESCRIPTOR = _SERVICEMONITORMESSAGE,
  __module__ = 'inorbit_pb2'
  # @@protoc_insertion_point(class_scope:inorbit.ServiceMonitorMessage)
  ))
_sym_db.RegisterMessage(ServiceMonitorMessage)
RosMonitorMessage = _reflection.GeneratedProtocolMessageType('RosMonitorMessage', (_message.Message,), dict(
  DESCRIPTOR = _ROSMONITORMESSAGE,
  __module__ = 'inorbit_pb2'
  # @@protoc_insertion_point(class_scope:inorbit.RosMonitorMessage)
  ))
_sym_db.RegisterMessage(RosMonitorMessage)
RosOutMessage = _reflection.GeneratedProtocolMessageType('RosOutMessage', (_message.Message,), dict(
  DESCRIPTOR = _ROSOUTMESSAGE,
  __module__ = 'inorbit_pb2'
  # @@protoc_insertion_point(class_scope:inorbit.RosOutMessage)
  ))
_sym_db.RegisterMessage(RosOutMessage)
CustomCommandRosMessage = _reflection.GeneratedProtocolMessageType('CustomCommandRosMessage', (_message.Message,), dict(
  DESCRIPTOR = _CUSTOMCOMMANDROSMESSAGE,
  __module__ = 'inorbit_pb2'
  # @@protoc_insertion_point(class_scope:inorbit.CustomCommandRosMessage)
  ))
_sym_db.RegisterMessage(CustomCommandRosMessage)
CustomScriptCommandMessage = _reflection.GeneratedProtocolMessageType('CustomScriptCommandMessage', (_message.Message,), dict(
  DESCRIPTOR = _CUSTOMSCRIPTCOMMANDMESSAGE,
  __module__ = 'inorbit_pb2'
  # @@protoc_insertion_point(class_scope:inorbit.CustomScriptCommandMessage)
  ))
_sym_db.RegisterMessage(CustomScriptCommandMessage)
CustomScriptStatusMessage = _reflection.GeneratedProtocolMessageType('CustomScriptStatusMessage', (_message.Message,), dict(
  DESCRIPTOR = _CUSTOMSCRIPTSTATUSMESSAGE,
  __module__ = 'inorbit_pb2'
  # @@protoc_insertion_point(class_scope:inorbit.CustomScriptStatusMessage)
  ))
_sym_db.RegisterMessage(CustomScriptStatusMessage)
TeleopGoCommand = _reflection.GeneratedProtocolMessageType('TeleopGoCommand', (_message.Message,), dict(
  DESCRIPTOR = _TELEOPGOCOMMAND,
  __module__ = 'inorbit_pb2'
  # @@protoc_insertion_point(class_scope:inorbit.TeleopGoCommand)
  ))
_sym_db.RegisterMessage(TeleopGoCommand)
KeyValueMessage = _reflection.GeneratedProtocolMessageType('KeyValueMessage', (_message.Message,), dict(
  DESCRIPTOR = _KEYVALUEMESSAGE,
  __module__ = 'inorbit_pb2'
  # @@protoc_insertion_point(class_scope:inorbit.KeyValueMessage)
  ))
_sym_db.RegisterMessage(KeyValueMessage)
RosDiagnosticsField = _reflection.GeneratedProtocolMessageType('RosDiagnosticsField', (_message.Message,), dict(
  DESCRIPTOR = _ROSDIAGNOSTICSFIELD,
  __module__ = 'inorbit_pb2'
  # @@protoc_insertion_point(class_scope:inorbit.RosDiagnosticsField)
  ))
_sym_db.RegisterMessage(RosDiagnosticsField)
RosDiagnosticsMessage = _reflection.GeneratedProtocolMessageType('RosDiagnosticsMessage', (_message.Message,), dict(
  DESCRIPTOR = _ROSDIAGNOSTICSMESSAGE,
  __module__ = 'inorbit_pb2'
  # @@protoc_insertion_point(class_scope:inorbit.RosDiagnosticsMessage)
  ))
_sym_db.RegisterMessage(RosDiagnosticsMessage)
RosDiagnosticsStatusMessage = _reflection.GeneratedProtocolMessageType('RosDiagnosticsStatusMessage', (_message.Message,), dict(
  DESCRIPTOR = _ROSDIAGNOSTICSSTATUSMESSAGE,
  __module__ = 'inorbit_pb2'
  # @@protoc_insertion_point(class_scope:inorbit.RosDiagnosticsStatusMessage)
  ))
_sym_db.RegisterMessage(RosDiagnosticsStatusMessage)
StateOptions = _reflection.GeneratedProtocolMessageType('StateOptions', (_message.Message,), dict(
  DESCRIPTOR = _STATEOPTIONS,
  __module__ = 'inorbit_pb2'
  # @@protoc_insertion_point(class_scope:inorbit.StateOptions)
  ))
_sym_db.RegisterMessage(StateOptions)
ModuleStateOptionsMessage = _reflection.GeneratedProtocolMessageType('ModuleStateOptionsMessage', (_message.Message,), dict(
  DESCRIPTOR = _MODULESTATEOPTIONSMESSAGE,
  __module__ = 'inorbit_pb2'
  # @@protoc_insertion_point(class_scope:inorbit.ModuleStateOptionsMessage)
  ))
_sym_db.RegisterMessage(ModuleStateOptionsMessage)
GpsFixMessage = _reflection.GeneratedProtocolMessageType('GpsFixMessage', (_message.Message,), dict(
  DESCRIPTOR = _GPSFIXMESSAGE,
  __module__ = 'inorbit_pb2'
  # @@protoc_insertion_point(class_scope:inorbit.GpsFixMessage)
  ))
_sym_db.RegisterMessage(GpsFixMessage)
_DELTAINTARRAY_STATSENTRY.has_options = True
_DELTAINTARRAY_STATSENTRY._options = _descriptor._ParseOptions(descriptor_pb2.MessageOptions(), _b('8\001'))
_DATABAGUPDATEMESSAGE_PROPERTIESENTRY.has_options = True
_DATABAGUPDATEMESSAGE_PROPERTIESENTRY._options = _descriptor._ParseOptions(descriptor_pb2.MessageOptions(), _b('8\001'))
_ROSBAGUPDATEMESSAGE_PROPERTIESENTRY.has_options = True
_ROSBAGUPDATEMESSAGE_PROPERTIESENTRY._options = _descriptor._ParseOptions(descriptor_pb2.MessageOptions(), _b('8\001'))
# @@protoc_insertion_point(module_scope)
````

## File: inorbit/agentlets/localization.py
````python
# Copyright (c) 2020, InOrbit, Inc.
# All rights reserved.
# Localization agentlet.
# Depends on the ROS module.
#
# TODO:
# - Implement shutdown
# - Implement clearing of retained messages (map and laser config)
# - (Flor_Grosso) Implement subscriptions/unsubscriptions to laser topics
#   listeners properly. Check how camera topic updates are handled.
#
# NOTE(herchu) Some of this agentlet's functionality is now emulated
# in a new (experimental) job, job-rosbag-importer (under /ingest):
# TF processing and poses report/saving.import base64
import io
import math
import threading
import time
import numpy
import png
from util.array_util import delta_int_encode_points
from util.math_util import downsample_array
from util.math_util import get_position_with_offset
from util.math_util import path_distance
from util.overrides import overrides
from util.rate_limiter import RateLimiter
from util.simplify import simplify
from .agentlet import Agentlet
from .agentlet import RUNLEVEL_DEFAULT
from .agentlet import RUNLEVEL_MINIMAL
from .inorbit_pb2 import LaserMessage
from .inorbit_pb2 import LocationAndPoseMessage
from .inorbit_pb2 import MapMessage
from .inorbit_pb2 import Nav2DPathMessage
from .inorbit_pb2 import Nav2DWaypointFrame
from .inorbit_pb2 import PathDataMessage
from .inorbit_pb2 import PathPoint
from .inorbit_pb2 import RobotPath
from .ros import RosPublisher
# ---------------------- ROS TOPICS AND TYPES ------------------------
# Maps:
ROS_MAP_MSG_TYPE = "nav_msgs/msg/OccupancyGrid"  # Both for map and costmap
ROS_COSTMAP_TOPIC_DEFAULT = "move_base_node/local_costmap/costmap"
# Nav paths:
ROS_PATH_TOPIC_DEFAULT = "/plan"
ROS_PATH_MSG_TYPE = "nav_msgs/msg/Path"
# Lasers:
ROS_LASER_TOPIC_DEFAULT = "scan"
ROS_LASER_MSG_TYPE = "sensor_msgs/msg/LaserScan"
# Pose:
ROS_SET_POSE_MSG_TYPE = "geometry_msgs/msg/PoseWithCovarianceStamped"
ROS_SET_POSE_TOPIC_DEFAULT = "initialpose"
# Navigation Goal:
ROS_NAV_GOAL_MSG_TYPE = "geometry_msgs/msg/PoseStamped"
ROS_NAV_GOAL_TOPIC_DEFAULT = "/goal_pose"
# Path-based navigation:
ROS_NAV_PATH_MSG_TYPE = "geometry_msgs/msg/PoseArray"
ROS_NAV_PATH_TOPIC_DEFAULT = "inorbit/nav2d/goal_path"
# Localization publisher's period for default and minimal runlevel
# (seconds)
PUBLISHER_PERIOD_DEFAULT_RUNLEVEL = 1
PUBLISHER_PERIOD_MINIMAL_RUNLEVEL = 10
# MQTT Topics
# - outgoing
MQTT_COSTMAP_TOPIC = "ros/loc/costmap"
MQTT_PATH_TOPIC = "ros/loc/path"
MQTT_LASERS_AND_POSE_TOPIC_V2 = "ros/loc/data2"
# - incoming:
MQTT_SET_POSE = "ros/loc/set_pose"  # Relocalization
MQTT_NAV_GOAL = "ros/loc/nav_goal"  # Navigate to goal
MQTT_NAV_PATH = "ros/nav/goal_path"  # Navigate through waypoints
# Cancel current navigation goal by sending a goal to the current pose
MQTT_CANCEL_GOAL = "ros/nav/goal_to_current_pose"
# ------- ROBOT PATH PROCESSING (paths published by the robot) -------
# How many nav path points to send; very few for a default
# runlevel and more (but not too many) when running higher priority
MAX_NAV_PATH_POINTS_DEFAULT = 20
MAX_NAV_PATH_POINTS_DETAILED = 50
# The max number of points that will be processed for a received robot
# path
DEFAULT_MAX_ROBOT_PATH_LENGTH = 5000
# Simplify paths with a smart decimation algorithm
DEFAULT_SIMPLIFY_PATH = True
# Parameters to be used when simplifying paths
# Tolerance: min squared distance a point can be from the curve to be
# included (those closer from that value are discarded)
DEFAULT_SIMPLIFY_PATH_TOLERANCE = 0.1
# High quality: excludes distance-based preprocessing step which leads
# to highest quality simplification but runs ~10-20 times slower.
DEFAULT_SIMPLIFY_PATH_HIGH_QUALITY = True
# Path updates closer than every 1 seconds are discarded
PATH_MAX_RATE_DEFAULT = 1
# ----------------------- NAVIGATION GOALS --------------------------
# Maximum acceptable delay between ts_hint and current time to be able to send a nav goal
DEFAULT_NAV_GOAL_MAX_DELAY_MS = 5000
# Time interval in seconds. When a 'cancel nav goal' action is received,
# the agent computes how much the robot has moved in this last interval
# and uses that value to predict where the robot will be at the same
# interval in the future.
DEFAULT_CANCEL_GOAL_LAG_SECS = 2
# ---------------------------- LASER SCAN ----------------------------
# Minimum number of valid (non infinite) laser ranges after downsampling
MIN_VALID_LASER_RANGES = 360
# ------------------------------ COSTMAP -----------------------------
# Minimum time interval before sending local costmap updates (in milliseconds)
MIN_COSTMAP_INTERVAL = 1000 * 10
# Path encoding options: NONE to list path points as floats (x, y), and
# DELTA_INT to use DeltaInt encoding (see array_util.py) using more compact protobuf
# representation (this version is newer).
# In all cases, path maximum input points, simplify options and maximum output paths apply.
PATH_ENCODING_NONE = 0
PATH_ENCODING_DELTA_INT = 1
PATH_ENCODING_DEFAULT = PATH_ENCODING_NONE  # The default is still the old representation
PATH_DELTA_INT_MAX_BITS_DEFAULT = 8
class RosLocalizationAgentlet(Agentlet):
    def __init__(self, uplink, ros, pose_agentlet, map_agentlet):
        super(RosLocalizationAgentlet, self).__init__(uplink)
        self._ros = ros
        # create frame ids
        self.base_link_frame_id = self.robot_namespace + "base_link"
        self.laser_frame_id = self.robot_namespace + "laser"
        # Keep a reference to pose_agentlet to make sure poses reporting do not
        self._pose_agentlet = pose_agentlet
        self._map_agentlet = map_agentlet
        self.inf = numpy.inf
        self.inf = math.inf
        self._states["available_costmap_topics"] = []
        self._states["available_path_topics"] = []
        self._states["available_laser_topics"] = []
        self._states["available_set_pose_topics"] = []
        self._states["available_nav_goal_topics"] = []
        self._states["available_nav_path_topics"] = []
        self._states["costmap_topic"] = None
        self._states["path_topics"] = {}
        """
        Dictionary of path topics, indexed by path id
        {  <path_id>: {
                topic: <ROS topic>
            }
        }
        """
        self._states["paths_config"] = {}
        """
        Dictionary of paths processing options
        {
            should_downsample: <flag indicating whether paths should be downsampled>,
            max_rate: <max rate the agent will be capturing path updates>,
            max_length: <max length of path message to be published by the agent>,
            simplify_high_quality: <Indicates whether the path simplification should preserve high
                                    quality or not (apply an extra decimation algorithm)>.
            simplify_tolerance: <Tolerance for the simplification algorithm>
        }
        """
        self._states["laser_topic"] = None
        self._states["laser2_topic"] = None
        self._states["set_pose_topic"] = None
        self._states["nav_goal_topic"] = None
        self._states["nav_path_topic"] = None
        self._states["nav_goal_max_delay_ms"] = DEFAULT_NAV_GOAL_MAX_DELAY_MS
        self._states["nav_goal_cancel_lag_secs"] = DEFAULT_CANCEL_GOAL_LAG_SECS
        self._states["robot_frame"] = self.base_link_frame_id
        # TODO(adamantivm) Make this part of the superclass
        self._states["runlevel"] = RUNLEVEL_DEFAULT
        # WARNING - HACK (FlorGrosso): Use this with care and if you really
        # know what you are doing. Ignores the 'map_published' flag and processes
        # localization data even if the map hasn't been published by the agent.
        self._states["disable_map_published_flag"] = False
        # Store a dictionary of the current laser topics configured, indexed
        # by laser id (0,1)
        self._configured_laser_topics = {}
        # State variables
        # TODO(adamantivm) Consider making these module states
        self._frame_ids = {  # Frame IDs for robot, map and lasers
            "robot": self._states["robot_frame"],
            "laser.0": self.laser_frame_id,
            "laser.1": self.laser_frame_id,
        }
        self._reset()
        # ROS Publisher for initial pose
        self._ros_initialpose = RosPublisher()
        # ROS Publisher for navigation goal
        self._ros_nav_goal = RosPublisher()
        # ROS Publisher for navigation path
        self._ros_nav_path = RosPublisher()
        # Condition to handle publisher's sleep/wake up timing
        self._condition = threading.Condition()
        # Lock to use _last_data and _path_rate_limiter (which implicitly
        # reads and modifies _path_data)
        self._path_data_mutex = threading.Lock()
    def _reset(self):
        # Whether the laser configuration has been already sent
        self._laser_config_published = {0: False, 1: False}
        # Latest recorded laser data
        self._last_laser = {0: None, 1: None}
        # Publisher thread running state
        self._running = False
        # Costmap data
        self._last_costmap = None  # Last costmap data
        self._last_costmap_ts = None  # Timestamp of last received costmap
        self._sent_costmap_ts = None  # Timestamp of last costmap data
        # Reset paths data (no need to lock _path_data_mutex; it replaces the entire instance var)
        self._last_path = {}
        max_path_rate = self._states.get("paths_config", {}).get("max_rate", PATH_MAX_RATE_DEFAULT)
        self._path_rate_limiter = RateLimiter(max_path_rate, dict_data=self._last_path)
        """
        Dictionary of path data, indexed by topic name.
        { <topic> :{
          msg: <raw path data, as published on topic>,
          msg_ts: <ts when ros msg was received> -- updated and checked by a RateLimiter
          raw_points: <array of all positions from a msg which was flagged to be published>,
          points: <processed -downsampled- points from msg>,
          publish: <flag indicated whether path data should be published or not>
        }}
        """
    @overrides(Agentlet)
    def load(self, runlevel):
        self._states["runlevel"] = runlevel
        # TODO(adamantivm) Check that ROS module was loaded?
        # TODO: Generalize this as a 'try to load dependency' kind of thing
        try:
            import sensor_msgs
            import sensor_msgs.msg
            global sensor_msgs
        except Exception as e:
            self.once_logger.exception("sensor_msgs_load", "Exception loading sensor_msgs.")
            return False
        try:
            import nav_msgs
            import nav_msgs.msg
            global nav_msgs
        except Exception as e:
            self.once_logger.exception("nav_msgs_load", "Exception loading nav_msgs.")
            return False
        try:
            import transformations
            global transformations
        except Exception as e:
            self.once_logger.exception(
                "tf_transformations_load", "Exception loading tf.transformations."
            )
            return False
        try:
            import geometry_msgs
            import geometry_msgs.msg
            global geometry_msgs
        except Exception as e:
            self.once_logger.exception("geometry_msgs_load", "Exception loading geometry_msgs.")
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
            global ROS_LASER_QOS_PROFILE_DEFAULT
            ROS_LASER_QOS_PROFILE_DEFAULT = qos.qos_profile_sensor_data
        except Exception as e:
            return False
        self._set_initial_topics()
        subs = self._create_ros_subs()
        pubs = [
            (
                self._states["set_pose_topic"],
                geometry_msgs.msg.PoseWithCovarianceStamped,
                self._ros_initialpose,
            ),
            (self._states["nav_goal_topic"], geometry_msgs.msg.PoseStamped, self._ros_nav_goal),
            (self._states["nav_path_topic"], geometry_msgs.msg.PoseArray, self._ros_nav_path),
        ]
        self._ros.add_submodule("localization", subs=subs, pubs=pubs)
        # Register for upstream incoming pose requests
        self.uplink.add_listener(MQTT_SET_POSE, self._set_pose)
        # Register for upstream incoming navigation goals
        self.uplink.add_listener(MQTT_NAV_GOAL, self._send_nav_goal)
        # Register for upstream incoming waypoint navigation path goals
        self.uplink.add_listener(MQTT_NAV_PATH, self._send_nav_path)
        # Register for upstream incoming cancel navigation goal requests
        self.uplink.add_listener(MQTT_CANCEL_GOAL, self._cancel_nav_goal)
        # Inform the Pose agentlet that this object is starting to report poses
        self._pose_agentlet.inform_override(
            True, self._runlevel_to_sleep_period(self.get_runlevel())
        )
        # Start uplink publishing thread
        threading.Thread(target=self._publish_loop).start()
        # AGENT_VER_1.1.5 Make absolutely sure older topics are cleared
        # and we don't have older retained messages left to confuse us.
        self.uplink.publish("ros/loc/config", None, qos=1, retain=True)
        self._states["loaded"] = True
        self.publish_state(self.uplink, self._states)
        return True
    @overrides(Agentlet)
    def unload(self):
        # Remove ROS publishers and subscribers
        self._ros.remove_submodule("localization")
        # Remove listeners from MQTT uplink
        self.uplink.remove_listener(MQTT_SET_POSE)
        self.uplink.remove_listener(MQTT_NAV_GOAL)
        self.uplink.remove_listener(MQTT_NAV_PATH)
        # TODO(adamantivm) Clear laser configuration topics properly.
        # This next line is wrong. Instead, it should go over each configured
        # laser and clear ros/loc/config/<laser_topic>.
        # For now clearing two lasers as that covers 99% of our use cases
        self.uplink.publish("ros/loc/config/0", None, qos=1, retain=True)
        self.uplink.publish("ros/loc/config/1", None, qos=1, retain=True)
        # Reset initial state
        self._reset()
        # Re-initialize exception reporting
        self.once_logger.reset_all()
        # Mark as unloaded
        self._states["loaded"] = False
        # Inform the Pose agentlet that this object is no longer going to
        # report poses
        self._pose_agentlet.inform_override(False, None)
        self.publish_state(self.uplink, self._states)
        return True
    @overrides(Agentlet)
    def set_runlevel(self, runlevel):
        self._states["runlevel"] = runlevel
        # Force costmap update when updating the runlevel
        self._sent_costmap_ts = None
        self.wake_up_publisher(self._condition)
        # Inform the Pose agentlet that this object is starting to report poses
        self._pose_agentlet.inform_override(
            self._states["loaded"], self._runlevel_to_sleep_period(self.get_runlevel())
        )
    def set_state(self, state):
        """
        Called whenever a set_module command is received.
        TODO (Flor_Grosso): move this setter to a proper method in parent,
        to avoid duplicating code per new topic to set.
        """
        if "costmap_topic" in state.keys():
            # If costmap is changed while the module is loaded, then we need
            # to switch the currently subscribed topic and reset the
            # agentlet state so that it published the new map.
            if self._states["loaded"]:
                # Reset initial state
                self._last_costmap = None
                self._last_costmap_ts = None
                self._sent_costmap_ts = None
                new_costmap_topic = (
                    state["costmap_topic"],
                    nav_msgs.msg.OccupancyGrid,
                    self._ros_on_costmap,
                )
                self._ros.update_subscriber_topic(
                    "localization", self._states["costmap_topic"], new_costmap_topic
                )
            self._states["costmap_topic"] = state["costmap_topic"]
        if "path_topics" in state.keys():
            if self._states["loaded"]:
                # Update ros subscribers with the new configured topics
                self._update_path_subs(state["path_topics"])
                # Assuming path topics have changed, reset the rate limiter.
                self._path_rate_limiter.reset()
            self._states["path_topics"] = state["path_topics"]
        if "paths_config" in state.keys():
            self._states["paths_config"] = state["paths_config"]
            # If paths config change, reset the rate limiter to accept a new max_rate
            # (This may publish an extra path msg even if max_rate did not change, but only once)
            max_path_rate = self._states.get("paths_config", {}).get(
                "max_rate", PATH_MAX_RATE_DEFAULT
            )
            self._path_rate_limiter = RateLimiter(max_path_rate, dict_data=self._last_path)
        if "laser_topic" in state.keys():
            if self._states["loaded"]:
                # Reset initial state
                self._laser_config_published[0] = False
                self._update_laser_topic(self._states["laser_topic"], state["laser_topic"])
                self._configured_laser_topics[0] = state["laser_topic"]
            self._states["laser_topic"] = state["laser_topic"]
        if "laser2_topic" in state.keys():
            if self._states["loaded"]:
                # Reset initial state
                self._laser_config_published[1] = False
                self._update_laser_topic(self._states["laser2_topic"], state["laser2_topic"])
                self._configured_laser_topics[1] = state["laser2_topic"]
            self._states["laser2_topic"] = state["laser2_topic"]
        if "set_pose_topic" in state.keys():
            if self._states["loaded"]:
                new_pub = (
                    state["set_pose_topic"],
                    geometry_msgs.msg.PoseWithCovarianceStamped,
                    self._ros_initialpose,
                )
                self._ros.update_publisher_topic(
                    "localization", self._states["set_pose_topic"], new_pub
                )
            self._states["set_pose_topic"] = state["set_pose_topic"]
        if "nav_goal_topic" in state.keys():
            if self._states["loaded"]:
                new_pub = (
                    state["nav_goal_topic"],
                    geometry_msgs.msg.PoseStamped,
                    self._ros_nav_goal,
                )
                self._ros.update_publisher_topic(
                    "localization", self._states["nav_goal_topic"], new_pub
                )
            self._states["nav_goal_topic"] = state["nav_goal_topic"]
        if "nav_path_topic" in state.keys():
            if self._states["loaded"]:
                new_pub = (state["nav_path_topic"], geometry_msgs.msg.PoseArray, self._ros_nav_path)
                self._ros.update_publisher_topic(
                    "localization", self._states["nav_path_topic"], new_pub
                )
            self._states["nav_path_topic"] = state["nav_path_topic"]
        if "robot_frame" in state.keys():
            self._states["robot_frame"] = state["robot_frame"]
            self._frame_ids["robot"] = self._states["robot_frame"]
        if "nav_goal_max_delay_ms" in state.keys():
            self._states["nav_goal_max_delay_ms"] = state["nav_goal_max_delay_ms"]
        if "nav_goal_cancel_lag_secs" in state.keys():
            self._states["nav_goal_cancel_lag_secs"] = state["nav_goal_cancel_lag_secs"]
        if "disable_map_published_flag" in state.keys():
            self._states["disable_map_published_flag"] = state["disable_map_published_flag"]
        # Send a state update
        self.publish_state(self.uplink, self._states)
        # Reset logger to get new exception after changed state
        self.once_logger.reset_all()
    def _set_pose(self, payload):
        """
        Called from the server.
        Updates the robot pose.
        """
        self.logger.info("Adjusting robot pose: {:s}.".format(payload.decode("utf-8")))
        # Check that we have a valid ROS publisher registered
        if self._ros_initialpose.pub is None:
            self.logger.warning("ROS publisher not set. Aborting.")
            return
        p, nq = self._compute_destination_pose_from_delta(payload)
        if p is None or nq is None:
            return
        # Compose initialpose message
        msg = geometry_msgs.msg.PoseWithCovarianceStamped()
        msg.header.stamp = self._ros.ros_now().to_msg()
        msg.header.frame_id = self._map_agentlet.map_frame_id
        # Set pose from results
        msg.pose = geometry_msgs.msg.PoseWithCovariance()
        msg.pose.pose = geometry_msgs.msg.Pose()
        msg.pose.pose.position.x = p.x
        msg.pose.pose.position.y = p.y
        msg.pose.pose.position.z = p.z
        msg.pose.pose.orientation.x = nq[0]
        msg.pose.pose.orientation.y = nq[1]
        msg.pose.pose.orientation.z = nq[2]
        msg.pose.pose.orientation.w = nq[3]
        # NOTE: hardcoded covariance taken from RVIZ "2D Pose Estimate" button
        msg.pose.covariance = [
            0.25,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.25,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.06853891945200942,
        ]
        # Publish new pose now
        self._ros_initialpose.pub.publish(msg)
        self.logger.info("Pose published.")
    def _send_nav_goal(self, payload):
        """
        Called from the server.
        Sends a navigation goal to the robot.
        """
        self.logger.info("Sending navigation goal: {:s}.".format(payload.decode("utf-8")))
        # Check that we have a valid ROS publisher registered
        if self._ros_nav_goal.pub is None:
            self.logger.warning("ROS publisher not set. Aborting.")
            return
        p, q = self._compute_destination_pose_from(payload)
        if p is None or q is None:
            return
        # Compose move_base_simple/goal message
        msg = geometry_msgs.msg.PoseStamped()
        msg.header.stamp = self._ros.ros_now().to_msg()
        msg.header.frame_id = self._map_agentlet.map_frame_id
        # Set pose from results
        msg.pose = geometry_msgs.msg.Pose()
        msg.pose.position.x = p.x
        msg.pose.position.y = p.y
        msg.pose.position.z = p.z
        msg.pose.orientation.x = q[0]
        msg.pose.orientation.y = q[1]
        msg.pose.orientation.z = q[2]
        msg.pose.orientation.w = q[3]
        # Publish new pose now
        self._ros_nav_goal.pub.publish(msg)
    def _cancel_nav_goal(self, payload):
        """
        Called from the server.
        Cancels a navigation goal by sending a new goal to the robot to its
        "current" pose.
        NOTE(Flor_Grosso): this is a simple version of the action. Ideally,
        the agent should know the goal ID and cancel it directly through
        move_base action API.
        """
        self.logger.info("Canceling current navigation goal")
        # Lag of the cancel goal action in seconds. The agent will
        # use this value to "guess" the robot's pose 'lag_s' seconds
        # from now and send a move base goal to it.
        lag_s = self._states.get("nav_goal_cancel_lag_secs", DEFAULT_CANCEL_GOAL_LAG_SECS)
        # Get robot's pose as of 'lag_s' secs ago
        robot_pose_past = self._get_robot_pose(self._ros.ros_secs_ago(lag_s))
        # Get robot's current pose
        robot_pose_now = self._get_robot_pose()
        # Avoid crashing if any of the queried poses are None
        if robot_pose_past is None or robot_pose_now is None:
            self.logger.warning("Can't cancel current goal. Robot's pose is not available.")
            return
        # Compute translation difference between poses.
        dx = robot_pose_now.translation.x - robot_pose_past.translation.x
        dy = robot_pose_now.translation.y - robot_pose_past.translation.y
        dz = robot_pose_now.translation.z - robot_pose_past.translation.z
        # Compose move_base_simple/goal message
        msg = geometry_msgs.msg.PoseStamped()
        msg.header.stamp = self._ros.ros_now().to_msg()
        msg.header.frame_id = self._map_agentlet.map_frame_id
        # The new pose is a "guess" of where the robot will be in 'lag_s'
        # secs from now.
        msg.pose = geometry_msgs.msg.Pose()
        # Translation: add computed delta to current translation
        # NOTE(Flor_Grosso): this is considering that the robot will
        # move the same distance in the future as it did in the computed
        # interval.
        msg.pose.position.x = robot_pose_now.translation.x + dx
        msg.pose.position.y = robot_pose_now.translation.y + dy
        msg.pose.position.z = robot_pose_now.translation.z + dz
        # Use the 'current' rotation as the goal orientation
        # NOTE (FlorGrosso): in the worst case, the robot will rotate
        # in place to reach this orientation. Quaternion diff to predict
        # future orientation can be skipped.
        msg.pose.orientation = robot_pose_now.rotation
        # Publish target pose now
        self._ros_nav_goal.pub.publish(msg)
    def _send_nav_path(self, payload):
        """
        Called from the server.
        Sends a navigation path to the robot.
        """
        # Parse payload
        try:
            message = Nav2DPathMessage()
            message.ParseFromString(payload)
            frame_name = Nav2DWaypointFrame.Name(message.frame)
        except Exception as e:
            self.logger.warning("Failed to parse Nav2DPathMessage")
            return None
        self.logger.info(f"Received nav_path message.\n{message}")
        # Check that the message is correct and can be processed
        if len(message.waypoints) < 1:
            self.logger.warning("No waypoints provided")
            return None
        # Check if the ts_hint (timestamp of the last seen camera image by the operator) is
        # older than the maximum allowed delay, to prevent sending commands with too high delay
        if self.is_time_expired(message.ts_hint, self._states["nav_goal_max_delay_ms"]):
            self.logger.warning("Message too old, discarding for safety reasons")
            return None
        # Get the latest robot pose, which we will need to calculate the paths to suggest
        robot_pose = self._get_robot_pose()
        if robot_pose is None:
            self.logger.warning("Can't get current robot pose, will not send a goal path.")
            return None
        # Calculate target frame ID
        # TODO(adamantivm) Don't send a goal if the frame_ids were changed recently
        # TODO(adamantivm) Update to protoc 3.8.0 and use ROBOT and MAP constants from the
        # proto definition - see https://github.com/protocolbuffers/protobuf/issues/6028
        if frame_name == "ROBOT":
            frame_id = self._frame_ids["robot"]
            # Set Z to 0 on all poses: same current height as the robot
            pose_z = 0
        elif frame_name == "MAP":
            frame_id = self._map_agentlet.map_frame_id
            # Set Z to the same as the robot currently has for all poses
            pose_z = robot_pose.translation.z
        else:
            self.logger.warning(
                f"Frame reference for message unsupported or unspecified: {message.frame}"
            )
            return None
        # Compose the ROS message
        msg = geometry_msgs.msg.PoseArray()
        msg.header.stamp = self._ros.ros_now()
        msg.header.frame_id = frame_id
        msg.poses = []
        # Go through all waypoints and add as poses
        for waypoint in message.waypoints:
            pose = self._waypoint_to_pose(waypoint, pose_z)
            msg.poses.append(pose)
        # Publish to ROS
        self._ros_nav_path.pub.publish(msg)
        self.logger.info("Path published to ROS")
    def _waypoint_to_pose(self, waypoint, pose_z):
        """
        Returns a ROS Pose given a provided Nav2DWaypointMessage.
        pose_z is the target z for all poses, calculated based on the desired frame of
        reference, outside of this method.
        """
        pose = geometry_msgs.msg.Pose()
        pose.position.x = waypoint.x
        pose.position.y = waypoint.y
        pose.position.z = pose_z
        # Make quaternion from the given target angle
        q = transformations.quaternion_from_euler(waypoint.theta, 0, 0)
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]
        return pose
    def _compute_destination_pose_from_delta(self, payload):
        """
        Computes a target pose from a given delta received in payload.
        Valid for set_pose commands.
        """
        # Parse payload
        # TODO(adamantivm) Sanity check (and get serious about protocol)
        [dx, dy, dtheta] = self._parse_pose_payload(payload)
        # Get the latest robot pose
        robot_pose = self._get_robot_pose()
        if robot_pose is None:
            self.logger.warning("Can't get current robot pose, aborting update.")
            return None
        # NOTE: The provided delta x and y are with respect to the map
        # i.e.: in map coordinates. The rotation is around the robot.
        # We apply this correction by simply updating the translation and
        # rotation components of the map to robot transform directly.
        # Rotate current pose rotation by the given delta theta
        dq = transformations.quaternion_from_euler(dtheta, 0, 0)
        q = robot_pose.rotation
        nq = transformations.quaternion_multiply((q.x, q.y, q.z, q.w), dq)
        # Shift current pose translation by the given delta x and y
        p = robot_pose.translation
        p.x += dx
        p.y += dy
        return p, nq
    def _compute_destination_pose_from(self, payload):
        """
        Computes a target pose from a given pose received in payload.
        Valid for nav_goal commands.
        """
        # Parse payload
        [x, y, theta] = self._parse_pose_payload(payload)
        # Get the latest robot pose
        robot_pose = self._get_robot_pose()
        if robot_pose is None:
            self.logger.warning("Can't get current robot pose, " "aborting navigation.")
            return None
        # Make quaternion from the given target angle
        q = transformations.quaternion_from_euler(theta, 0, 0)
        # Set pose translation from the given x and y targets
        p = robot_pose.translation
        p.x = x
        p.y = y
        return p, q
    def _parse_pose_payload(self, payload):
        """
        Parses a pose message payload coming from the UI and returns x, y, theta.
        """
        [seq, timestamp, x, y, theta] = payload.decode("utf-8").split("|")
        timestamp = int(timestamp)
        x = float(x)
        y = float(y)
        theta = float(theta)
        return [x, y, theta]
    def _publish_loop(self):
        """
        Runs on a separate thread. Publishes robot pose + TF at a rate dependent on
        the runlevel.
        """
        self._running = True
        while self._running is True:
            try:
                # Publish data if ready
                self._maybe_publish()
            except Exception as e:
                self.once_logger.exception(
                    "localization_maybe_publish", "Exception publishing data."
                )
            # Throttle according to runlevel
            sleep_sec = self._runlevel_to_sleep_period(self.get_runlevel())
            self._condition.acquire()
            self._condition.wait(sleep_sec)
            self._condition.release()
        self.logger.info("Publisher thread shutting down.")
    def _runlevel_to_sleep_period(self, runlevel):
        """
        Returns the publish frequency (or rather: period, in seconds) associated
        to a runlevel. For now only runlevel DEFAULT is specified; while any
        other would trigger a MINIMAL report rate.
        """
        if runlevel == RUNLEVEL_DEFAULT:
            return PUBLISHER_PERIOD_DEFAULT_RUNLEVEL
        # If the runlevel is not recognized, publish at minimum
        else:
            return PUBLISHER_PERIOD_MINIMAL_RUNLEVEL
    def _get_robot_pose(self, time=None):
        """
        Helper method to get the latest known robot pose.
        Returns None if the pose can't be found.
        """
        map_T_robot = self._ros.lookup_transform(
            self._map_agentlet.map_frame_id, self._frame_ids["robot"], time
        )
        return map_T_robot.transform if map_T_robot is not None else None
    def _maybe_publish(self):
        """
        Publishes map and laser date if appropriate and possible.
        """
        # Don't do anything if map data is required and there isn't a map
        if not self._should_process_localization_data():
            return
        ranges = {0: [], 1: []}
        ts_msec = None
        # We can only publish laser if we have data
        # and configuration
        if self._laser_config_published[0] and self._last_laser[0] is not None:
            ranges[0] = self._last_laser[0]["ranges"]
            ts_msec = self._last_laser[0]["ts_msec"]
        else:
            ranges[0] = []
        # Same with second laser
        if self._laser_config_published[1] and self._last_laser[1] is not None:
            ranges[1] = self._last_laser[1]["ranges"]
            if ts_msec is None:
                ts_msec = self._last_laser[1]["ts_msec"]
        else:
            ranges[1] = []
        if ts_msec is None:
            ts_msec = self.get_ts()
        # If we have a robot_pose alongside a laser scan, use that
        robot_pose = None
        if (
            ranges[0] is not None
            and self._last_laser[0] is not None
            and len(ranges[0]) != 0
            and "robot_pose" in self._last_laser[0]
        ):
            robot_pose = self._last_laser[0]["robot_pose"]
        # Otherwise try with the secondary laser
        if (
            robot_pose is None
            and self._last_laser[1] is not None
            and ranges[1] is not None
            and len(ranges[1]) != 0
            and "robot_pose" in self._last_laser[1]
        ):
            robot_pose = self._last_laser[1]["robot_pose"]
        # If we got here with no robot pose, try to get one now
        if robot_pose is None:
            robot_pose = self._get_robot_pose()
        # TODO(adamantivm) Clear used laser data so that it's not sent
        # again. Also, don't query old data.
        # Test this by running a small bag and then making sure old
        # data doesn't keep being published as new.
        # If we can't get the robot pose, there is nothing to show
        if robot_pose is None:
            self.once_logger.warn("localization_no_robot_pose", "No robot pose available.")
            return
        # Get robot position and orientation
        p = robot_pose.translation
        # Get Yaw from quaternion
        o = robot_pose.rotation
        q = (o.x, o.y, o.z, o.w)
        euler = transformations.euler_from_quaternion(q)
        yaw = euler[0]
        # TODO(adamantivm) Downsample by only leaving one every N samples
        # when in a low-frequency mode
        pose_with_offset = get_position_with_offset(p.x, p.y)
        data = LocationAndPoseMessage()
        data.ts = ts_msec
        data.pos_x = pose_with_offset["pos_x"]
        data.pos_y = pose_with_offset["pos_y"]
        data.offset_x = pose_with_offset["offset_x"]
        data.offset_y = pose_with_offset["offset_y"]
        data.yaw = yaw
        data.frame_id = self._map_agentlet.inorbit_frame_id
        lasers = []
        for key in ranges:
            # Does not send empty laser data if not present in message
            if len(ranges[key]):
                laser = LaserMessage()
                laser.name = str(key)
                downsampled = self._downsample_ranges_list(ranges[key])
                self._encode_floating_point_list(laser.ranges, downsampled)
                lasers.append(laser)
        data.lasers.extend(lasers)
        self.uplink.publish_protobuf(MQTT_LASERS_AND_POSE_TOPIC_V2, data)
        self._last_laser = {0: None, 1: None}
        # Handle path data processing, build protobuf message and publish
        # to mqtt topic.
        self._publish_path_data()
        # Publish costmap: If there is data that was not yet sent.
        # Frequency is determined by calls to this maybe_publish, with a cap
        # in rate of MIN_COSTMAP_INTERVAL
        if self._last_costmap is not None and (
            self._sent_costmap_ts is None
            or self._last_costmap_ts - self._sent_costmap_ts > MIN_COSTMAP_INTERVAL
        ):
            mqttData = self._costmap_to_mqtt(self._last_costmap, self._last_costmap_ts)
            self.uplink.publish_protobuf(MQTT_COSTMAP_TOPIC, mqttData)
            self._sent_costmap_ts = self._last_costmap_ts
    def _downsample_ranges_list(self, ranges):
        """
        Downsamples (or not!) a list of laser ranges which contains numbers and some Infinite
        values. It will attempt to leave no less than MIN_VALID_LASER_RANGES
        """
        count_valid = lambda downsample: len(
            list(filter(lambda x: x != self.inf, ranges[0::downsample]))
        )
        down = 1
        # Calculate downsampling (as long as the *next* downsampling level still leaves at
        # least MIN_VALID_LASER_RANGES useful laser readings)
        while count_valid(down + 1) >= MIN_VALID_LASER_RANGES:
            down += 1
        # Return.
        # If down==1, do not even copy the array.
        # Otherwise, downsampling works here as just replacing some positions by Infinite, since
        # this gets compacted very efficiently in our protobuf implementation.
        # (A simpler downsampling would just be ranges[0::down] but this requires more logic in the
        # client to handle the new indices and new number of elements)
        return (
            ranges
            if down <= 1
            else [(ranges[i] if i % down == 0 else self.inf) for i in range(len(ranges))]
        )
    def _encode_floating_point_list(self, fp_list, ranges):
        """
        Encodes a list of float numbers (which may contain infinite values) into a
        FloatingPointList which has a compact representation for runs of
        consecutive inf and non-inf values.
        """
        # Encode the numbers in runs of infinite and non-infinite sequences
        last_was_infinite = True
        current_run_length = 0
        values = []
        runs = []
        for r in ranges:
            if (r == self.inf) == last_was_infinite:
                # Current and last were both infinite, or both non-infinite
                current_run_length += 1
            else:
                # Current=inf, last was not inf; switch and output the last run
                runs.append(current_run_length)
                current_run_length = 1
                last_was_infinite = r == self.inf
            # Now process the number (if not infinite)
            if r != self.inf:
                values.append(r)
        # Finally output the last run length
        runs.append(current_run_length)
        fp_list.runs.extend(runs)
        fp_list.values.extend(values)
        # Tricky code above... Do some validations for invariants
        # (see declaration of FloatingPointList)
        if sum(runs) != len(ranges):
            raise Exception(
                f"Sum of encoded runs is {sum(runs)}, must be equal to original list "
                f"length {len(ranges)}"
            )
        # Only the first element can be 0
        if len(list(filter(lambda x: x <= 0, runs[1:]))) > 0:
            raise Exception("There are zero or negative elements in runs!")
        if sum(runs[1::2]) != len(values):
            raise Exception(
                f"Sum of non-inf runs is {sum(runs[1::2])}, must be equal to number of "
                f"encoded values {len(values)}"
            )
    def _ros_on_laser(self, data, laser_topic):
        """
        Callback for scan messages.
        """
        # Collect, process and publish laser data for every laser id
        # linked to laser_topic
        for laser_id in self._configured_laser_topics:
            if self._configured_laser_topics[laser_id] == laser_topic:
                self._ros_on_laser_all(data, laser_id)
    def _ros_on_laser_all(self, data, laser_id):
        """
        Callback for scan messages.
        TODO (Flor_Grosso): if more than one laser_id points to the same topic,
        then laser data will be processed more than one. Consider making each
        processing unique per topic, and reuse that data to be published by the
        associated laser ids.
        """
        # Don't do anything if map data is required and there isn't a map
        if not self._should_process_localization_data():
            return
        # TODO(adamantivm) General time management review
        now = time.time()
        frame_id_key = f"laser.{laser_id}"
        # Capture laser frame ID
        # Clean-up, sometimes it comes with a leadning /
        if data.header.frame_id:
            if data.header.frame_id[0] == "/":
                self._frame_ids[frame_id_key] = data.header.frame_id[1:]
            else:
                self._frame_ids[frame_id_key] = data.header.frame_id
        # Query transform to robot frame
        robot_T_laser = self._ros.lookup_transform(
            self._frame_ids["robot"], self._frame_ids[frame_id_key]
        )
        # If it hasn't been published yet, publish the laser configuration
        # TODO(adamantivm) Also publish if it changed significantly
        if robot_T_laser is not None and not self._laser_config_published[laser_id]:
            p = robot_T_laser.transform.translation
            o = robot_T_laser.transform.rotation
            q = (o.x, o.y, o.z, o.w)
            euler = transformations.euler_from_quaternion(q)
            yaw = euler[0]
            # HACK Assume upside-down laser if a rotation around X is present
            if abs(euler[0]) > 3:
                angle_min = data.angle_max
                angle_max = data.angle_min
            else:
                angle_min = data.angle_min
                angle_max = data.angle_max
            self.uplink.publish(
                "ros/loc/config/{:d}".format(laser_id),
                "{:d}|{:.4g}|{:.4g}|{:.6g}|{:.6g}|{:.6g}|{:.4g}|{:.4g}|{:d}".format(
                    int(now),
                    p.x,
                    p.y,
                    yaw,
                    angle_min,
                    angle_max,
                    data.range_min,
                    data.range_max,
                    len(data.ranges),
                ),
                qos=1,
                retain=True,
            )
            self._laser_config_published[laser_id] = True
        # Query the robot position at the time the laser was published
        robot_pose = self._get_robot_pose(data.header.stamp)
        # Record latest laser information for publishing
        self._last_laser[laser_id] = {
            "ts_msec": int(now * 1000),
            "ranges": data.ranges,
            "robot_pose": robot_pose,
        }
    def _ros_on_costmap(self, msg):
        """
        Callback for costmap messages.
        """
        # Save this costmap. It might not get sent at all (depends on
        # state)
        ts = self.get_ts()
        # Attempt to transform to map coordinates that the client can handle,
        # if not already there
        # (This is done now even if costmap gets never sent; so coordinate is
        # transformed right after it was received)
        target_frame_id = self._map_agentlet.map_frame_id
        if msg.header.frame_id != target_frame_id:
            # Use ros module to transform to target frame
            pose_in_map = self._ros.transform_pose(
                msg.info.origin, msg.header.frame_id, target_frame_id
            )
            if pose_in_map is None:
                # It was not converted to the target (map) frame_id. Ignore it!
                return
            # Now replace the position with map coordinates
            msg.header.frame_id = target_frame_id
            msg.info.origin = pose_in_map
        # Save this costmap to be sent later
        self._last_costmap = msg
        self._last_costmap_ts = ts
    def _map_to_pixels(self, msg, msgdata, fn):
        """
        Converts an OccupancyGrid map
        http://docs.ros.org/api/nav_msgs/html/msg/OccupancyGrid.html
        into a matrix of pixels.
        This method just creates the matrix and iterates it; the lambda function in
        the fn argument must care of creating each actual pixel (grayscal, rgb, etc)
        NOTE(adamantivm) The map data itself is passed as a separate argument in case
        the publisher thread needs to truncate the map while we're publishing.
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
    def _costmap_to_mqtt(self, msg, ts):
        # Record the map frame ID
        self._frame_ids["costmap"] = msg.header.frame_id.split("/")[-1]
        # Color to represent transparent / unknown
        TRANSP = 255
        # Note(FlorGrosso): this is ignoring a range of values from the
        # occupancy grid which are considered illegal:
        #   - Negative values: shade from red to yellow in rviz.
        #   - 101-127: shown in green in rviz.
        #
        # Reference: http://docs.ros.org/en/jade/api/rviz/html/c++/map__display_8cpp_source.html
        #
        # Consider keeping this data and sending it to the server in a way
        # that it can be displayed in both the default InOrbit format
        # (wine-color in gradient of alpha), but also that can be
        # converted into RVIZ format if the user chooses that.
        # Use TRANSP for "transparent" (costmap: unknown)
        fn = lambda data: TRANSP if data < 0 or data > 100 else int(max(data * 254 / 100, 0))
        pixels = self._map_to_pixels(msg, msg.data, fn)
        # Encode into PNG format
        buf = io.BytesIO()
        w = png.Writer(msg.info.width, msg.info.height, greyscale=True, transparent=TRANSP)
        w.write(buf, pixels)
        buf.flush()
        euler = transformations.euler_from_quaternion(
            (
                msg.info.origin.orientation.x,
                msg.info.origin.orientation.y,
                msg.info.origin.orientation.z,
                msg.info.origin.orientation.w,
            )
        )
        yaw = euler[0]
        # Build the protobuf message object
        data = MapMessage()
        data.width = msg.info.width
        data.height = msg.info.height
        data.pixels = buf.getvalue()
        data.x = msg.info.origin.position.x
        data.y = msg.info.origin.position.y
        data.theta = yaw
        data.resolution = msg.info.resolution
        data.ts = ts
        return data
    def _downsample_path(self, points_arr, maxn):
        """
        Downsamples a path applying a combination of:
        1) Radial distance algorithms (optional)
        2) A decimation algorithm (Douglas-Peucker)
        3) [if the output array from (2) is still larger than maxn)
            a simple downsampling method taking elements at regular intervals.
        First and last element are always returned (provided N>=2).
        """
        if len(points_arr) <= maxn or maxn <= 1:
            # It is assumed that _downsample will return a new array,
            # so just save the iterators logic but still return a copy
            return points_arr[:]
        # Should simplify?
        should_simplify = self._states["paths_config"].get("should_simplify", DEFAULT_SIMPLIFY_PATH)
        # Turn on logging for path processing stats (time)
        log_path_processing_stats = self._states["paths_config"].get("log_processing_stats", False)
        # Array containing a reduced set of path points
        reduced_path = []
        try:
            start_time = time.time()
            if should_simplify:
                # Get parameters for the algorithm simplifying the curve
                tolerance = self._states["paths_config"].get(
                    "simplify_tolerance", DEFAULT_SIMPLIFY_PATH_TOLERANCE
                )
                high_quality = self._states["paths_config"].get(
                    "simplify_high_quality", DEFAULT_SIMPLIFY_PATH_HIGH_QUALITY
                )
                log_msg = f"simplify=(tolerance {tolerance}, high_quality {high_quality}"
                # Use simplify method (a combination of DP + radial distance
                # algorithms)
                reduced_path = simplify(points_arr, tolerance, high_quality)
                if len(reduced_path) > maxn:
                    reduced_path = downsample_array(reduced_path, maxn)
                    log_msg += f", reduced to {len(reduced_path)}"
            else:
                # If there was no simplification or simplified array is still
                #  larger than limits, procede with a regular downsampling.
                reduced_path = downsample_array(points_arr, maxn)
                log_msg += f"simple downsample from {len(points_arr)} to {len(reduced_path)}"
            elapsed_time = time.time() - start_time
            if log_path_processing_stats:
                self.logger.info(f"Path processing took {elapsed_time:.4f} secs: {log_msg}")
        except Exception as e:
            self.once_logger.info("Unable to downsample path")
        return reduced_path
    def _build_path_msg(self, points, path_id):
        """
        Converts a ROS PoseStamped array to protobuf PathPoint to be sent
        through mqtt.
        """
        # Create path object to store locally and send on next publish
        path = RobotPath()
        # Max number of points that will be processed for the robot path.
        encoding = self._states["paths_config"].get("encoding_version", PATH_ENCODING_DEFAULT)
        if encoding == PATH_ENCODING_DELTA_INT:
            # Encode path points in "DeltaInt" representation
            max_bits = self._states["paths_config"].get(
                "encoding_max_bits", PATH_DELTA_INT_MAX_BITS_DEFAULT
            )
            compute_stats = self._states["paths_config"].get("encoding_stats", False)
            if not isinstance(max_bits, int) or max_bits <= 0:
                self.once_logger.warn(
                    "invalid_encoding_max_bits",
                    f"Invalid encoding_max_bits value: {max_bits}",
                )
                max_bits = PATH_DELTA_INT_MAX_BITS_DEFAULT
            (x, y) = delta_int_encode_points(points, max_bits, compute_stats)
            path.encoding_version = PATH_ENCODING_DELTA_INT
            path.encoded_points.max_bits = max_bits
            path.encoded_points.xs.anchor = x[0]
            path.encoded_points.xs.deltas.extend(x[1])
            path.encoded_points.xs.exponent = x[2]
            path.encoded_points.ys.anchor = y[0]
            path.encoded_points.ys.deltas.extend(y[1])
            path.encoded_points.ys.exponent = y[2]
            if compute_stats:
                for k, v in x[3].items():
                    path.encoded_points.xs.stats[k] = v
                for k, v in y[3].items():
                    path.encoded_points.ys.stats[k] = v
        else:
            # Default encoding: path points as a list of (x, y) float values
            # (Note: Setting value to 0 in protobuf is not actually needed)
            path.encoding_version = PATH_ENCODING_NONE
            # Normally PATH_ENCODING_NONE, but also catch-all for any invalid state value
            ps = []
            # Collect coords of all points
            for p in points:
                # TODO(herchu) how to get 'secs' element?
                point = PathPoint()
                point.x = p.x
                point.y = p.y
                ps.append(point)
            path.points.extend(ps)
        # Send a timestamp so server can decide when data gets old and rusty
        path.ts = self.get_ts()
        path.path_id = path_id
        path.frame_id = self._map_agentlet.inorbit_frame_id
        return path
    def _ros_on_path(self, msg, topic):
        """
        Callback for path messages. Listens to path updates and stores data
        separately, depending on the source topic.
        """
        current_ts = self.get_ts()
        try:
            with self._path_data_mutex:
                if topic not in self._last_path.keys():
                    self._last_path[topic] = {}
                map_frame_id = self._map_agentlet.map_frame_id
                if msg.header.frame_id != "" and msg.header.frame_id != map_frame_id:
                    self.once_logger.warn(
                        "localization_path_frame_diff_map_frame",
                        "Warning, path frame_id does not match map frame_id."
                        "This case isn't handled well by the agent. Path will be reported using "
                        f"the map frame. '{msg.header.frame_id}' != '{map_frame_id}'.",
                    )
                # Calculate time difference between this message and the
                # last one. Discard new messages if the update rate is higher
                # than the maximum configured.
                if self._path_rate_limiter.accepts(topic, current_ts):
                    self._last_path[topic]["msg"] = msg
        except Exception as e:
            self.once_logger.exception(
                f"ros_callback for {topic}",
                f"Exception receiving path data from: '{topic}'.",
            )
    def _get_available_map_topics(self, is_costmap):
        """
        Finds topics publishing ROS_MAP_MSG_TYPE, and returns a sorted list
        When is_costmap is false, regular maps are left on top. When true, costmap
        will be at start of the list.
        """
        map_topics = self._ros.get_topics_publishing(ROS_MAP_MSG_TYPE)
        # If map topics were found, then sort the list leaving "costmap" topics
        # below.
        if map_topics:
            map_topics = sorted(
                map_topics, key=lambda x: (-1 if is_costmap else 1) * (x.find("costmap"), x)
            )
        return map_topics
    def _get_available_path_topics(self):
        """
        Finds topics publishing ROS_PATH_MSG_TYPE.
        If the topic we know to work is in the list, it is returned first
        (so it will be used as default).
        """
        topics = self._ros.get_topics_publishing(ROS_PATH_MSG_TYPE)
        if topics:
            topics = sorted(topics, key=lambda x: (x.find(ROS_PATH_TOPIC_DEFAULT), x), reverse=True)
        return topics
    def _get_available_laser_topics(self):
        """
        Finds topics publishing ROS_LASER_MSG_TYPE.
        """
        return self._ros.get_topics_publishing(ROS_LASER_MSG_TYPE)
    def _get_available_set_pose_topics(self):
        """
        Finds topics to publish ROS_SET_POSE_MSG_TYPE to.
        """
        set_pose_topics = self._ros.get_topics_to_publish(ROS_SET_POSE_MSG_TYPE)
        # If set_pose topics were found, then sort the list leaving
        # "initialpose" topic first.
        if set_pose_topics:
            set_pose_topics = sorted(
                set_pose_topics, key=lambda x: (x.find(ROS_SET_POSE_TOPIC_DEFAULT), x), reverse=True
            )
        return set_pose_topics
    def _get_available_nav_goal_topics(self):
        """
        Finds topics to publish ROS_NAV_GOAL_MSG_TYPE to.
        """
        nav_goal_topics = self._ros.get_topics_to_publish(ROS_NAV_GOAL_MSG_TYPE)
        # If nav_goal topics were found, then sort the list leaving
        # "move_base_simple/goal" topic first.
        if nav_goal_topics:
            nav_goal_topics = sorted(
                nav_goal_topics, key=lambda x: (x.find(ROS_NAV_GOAL_TOPIC_DEFAULT), x), reverse=True
            )
        return nav_goal_topics
    def _get_available_nav_path_topics(self):
        """
        Finds topics which subscribe to ROS_NAV_PATH_MSG_TYPE.
        """
        nav_path_topics = self._ros.get_topics_to_publish(ROS_NAV_PATH_MSG_TYPE)
        # If nav_path topics were found, make sure the agentlet default is
        # included and provided first in the list.
        if nav_path_topics:
            if ROS_NAV_PATH_TOPIC_DEFAULT not in nav_path_topics:
                nav_path_topics.insert(0, ROS_NAV_PATH_TOPIC_DEFAULT)
            else:
                nav_path_topics = sorted(
                    nav_path_topics,
                    key=lambda x: (x.find(ROS_NAV_PATH_TOPIC_DEFAULT), x),
                    reverse=True,
                )
        return nav_path_topics
    def _query_available_topics(self):
        """
        Queries available topics related to localization features and sets their
        corresponding states.
        """
        # COSTMAP
        # HACK(herchu) Adding a default and invalid topic first to costmaps are
        # not enabled "accidentally" by default yet they report the list of
        # available topics
        avail_topics = self._get_available_map_topics(True)
        self._states["available_costmap_topics"] = (
            None if not avail_topics else ["DISABLED"] + avail_topics
        )
        # PATH
        self._states["available_path_topics"] = self._get_available_path_topics()
        # LASERS
        self._states["available_laser_topics"] = self._get_available_laser_topics()
        # SET_POSE
        self._states["available_set_pose_topics"] = self._get_available_set_pose_topics()
        # NAV GOAL
        self._states["available_nav_goal_topics"] = self._get_available_nav_goal_topics()
        # NAV PATH GOAL
        self._states["available_nav_path_topics"] = self._get_available_nav_path_topics()
    def _set_initial_topics(self):
        """
        Sets initial topics based on available values and default values.
        """
        # Query all available topics
        self._query_available_topics()
        # List of topics to set for localization
        topics_to_set = ["costmap", "laser", "set_pose", "nav_goal", "nav_path"]
        for topic_key in topics_to_set:
            state_to_topic = f"{topic_key}_topic"
            state_to_available_topics = f"available_{topic_key}_topics"
            default_topic = eval(f"ROS_{topic_key.upper()}_TOPIC_DEFAULT")
            if not self._states[state_to_topic]:
                if not self._states[state_to_available_topics]:
                    self.logger.warning(
                        f"No ROS {state_to_topic} available. "
                        f"Setting '{default_topic}' as default."
                    )
                    self._states[state_to_topic] = default_topic
                else:
                    # If there are available topics, set the first on the list
                    # as the current one.
                    self._states[state_to_topic] = self._states[state_to_available_topics][0]
        # There can be more than one path configured. Handle default
        # topics for these separately.
        self._set_initial_path_topics()
    def _create_ros_subs(self):
        """
        Returns an array of ros subscribers to the current topics under states.
        """
        # Default subscriptions
        subs = [
            (
                self._states["costmap_topic"],
                nav_msgs.msg.OccupancyGrid,
                self._ros_on_costmap,
                ROS_MAP_QOS_PROFILE_DEFAULT,
            ),
            (
                self._states["laser_topic"],
                sensor_msgs.msg.LaserScan,
                lambda msg: self._ros_on_laser(msg, self._states["laser_topic"]),
                ROS_LASER_QOS_PROFILE_DEFAULT,
            ),
        ]
        self._configured_laser_topics[0] = self._states["laser_topic"]
        # Now for those topics which can repeat previous subscriptions
        # Secondary laser: subscribe to topic only if there are no
        # subscriptions yet
        if self._states["laser2_topic"] is not None:
            if self._states["laser2_topic"] != self._states["laser_topic"]:
                subs.append(
                    [
                        self._states["laser2_topic"],
                        sensor_msgs.msg.LaserScan,
                        lambda msg: self._ros_on_laser(msg, self._states["laser2_topic"]),
                    ]
                )
            self._configured_laser_topics[1] = self._states["laser2_topic"]
        # Make ROS subs list for path topics
        current_path_topics = set(
            self._states["path_topics"][path_id].get("topic")
            for path_id in self._states["path_topics"].keys()
        )
        for topic in current_path_topics:
            subs.append(
                [topic, nav_msgs.msg.Path, lambda msg, topic=topic: self._ros_on_path(msg, topic)]
            )
        return subs
    def _update_laser_topic(self, current_topic, new_topic):
        """
        Updates laser config from current_topic to new_topic topic when a new
        state is received.
        """
        if current_topic == new_topic:
            return
        if (self._configured_laser_topics.values()).count(current_topic) > 1:
            # Do not unsuscribe from current topic, just add the new
            # subscription
            new_subs = [
                new_topic,
                sensor_msgs.msg.LaserScan,
                lambda msg: self._ros_on_laser(msg, new_topic),
            ]
            self._ros.add_submodule_listeners("localization", subs=[new_subs], pubs=[])
        else:
            # Unregister from old topic
            if new_topic in self._configured_laser_topics.values():
                self._ros.unregister_subscriber_to(current_topic, "localization")
            else:
                # Unregister from old topic, and register to new one
                new_subs = [
                    new_topic,
                    sensor_msgs.msg.LaserScan,
                    lambda msg: self._ros_on_laser(msg, new_topic),
                ]
                self._ros.update_subscriber_topic("localization", current_topic, new_subs)
    def _update_path_subs(self, new_topics):
        """
        Updates ros subscribers for path data when a new state is received.
        """
        current_path_topics = set(new_topics[path_id].get("topic") for path_id in new_topics.keys())
        previous_path_topics = set(
            self._states["path_topics"][path_id].get("topic")
            for path_id in self._states["path_topics"].keys()
        )
        deleted_topics = previous_path_topics - current_path_topics
        new_topics = current_path_topics - previous_path_topics
        # Remove listeners for deleted topics
        for topic in deleted_topics:
            self._ros.unregister_subscriber_to(topic, "localization")
            # Remove key from raw path storage
            with self._path_data_mutex:
                self._last_path.pop(topic, None)
        # Add new listeners
        for topic in new_topics:
            self._ros.add_submodule_listeners(
                "localization",
                subs=[
                    [
                        topic,
                        nav_msgs.msg.Path,
                        lambda msg, topic=topic: self._ros_on_path(msg, topic),
                    ]
                ],
                pubs=[],
            )
    def _set_initial_path_topics(self):
        """
        Sets the initial path topics for when the agentlet is first loaded.
        """
        # If there is no configuration provided, create sensible defaults
        # trying to fetch one path with the first available path topic
        if not self._states.get("path_topics"):
            # One path
            self._states["path_topics"]["0"] = {}
            # Try to detect a topic, otherwise use a hardcoded value
            if not self._states["available_path_topics"]:
                self.logger.warning(
                    f"No ROS path topics available. Setting '{ROS_PATH_TOPIC_DEFAULT}' as "
                    "default."
                )
                topic = ROS_PATH_TOPIC_DEFAULT
            else:
                # If there are path topics, set the first on the list as
                # the current one.
                topic = self._states["available_path_topics"][0]
            # set-up this topic as default
            self._states["path_topics"]["0"]["topic"] = topic
            # Trigger a state update
            self.publish_state(self.uplink, self._states)
    def _publish_path_data(self):
        """
        Publishes path data. It first processes all stored paths and then
        build the PathDataMessage, which is sent through MQTT.
        """
        # Process stored path msgs if necessary.
        self._process_paths()
        # Build RobotPath message for every path_id
        path_data = []
        for path_id in self._states["path_topics"]:
            topic = self._states["path_topics"][path_id].get("topic")
            if self._last_path.get(topic, {}).get("publish", True):
                points = self._last_path.get(topic, {}).get("points")
                if points is not None:
                    path = self._build_path_msg(points, path_id)
                    path_data.append(path)
        if not path_data:
            return
        data = PathDataMessage()
        data.ts = self.get_ts()
        data.paths.extend(path_data)
        self.uplink.publish_protobuf(MQTT_PATH_TOPIC, data)
    def _process_paths(self):
        """
        Path msg processor. This does some some downsampling if configured
        and necessary.
        """
        # Check if downsample flag is on (true by default)
        should_downsample = self._states["paths_config"].get("should_downsample", True)
        # Max number of points that will be processed for the robot path.
        max_input_path_length = self._states["paths_config"].get(
            "max_input_path_length", DEFAULT_MAX_ROBOT_PATH_LENGTH
        )
        # Process every path per topic (this is done so that in case
        # more than one path id share the same topic, processing is
        # not replicated).
        with self._path_data_mutex:
            for topic in self._last_path:
                # Get ROS msg
                last_msg = self._last_path.get(topic, {}).get("msg")
                if last_msg is not None:
                    poses = last_msg.poses
                    # First check if the path's length is larger than the
                    # max allowed for processing. If so, cap the poses array.
                    if len(poses) > max_input_path_length:
                        self.once_logger.warn(
                            "max_robot_nav_path_length",
                            f"Max path length reached. Keeping {max_input_path_length} the "
                            "initial points from now on",
                        )
                        poses = poses[: max_input_path_length - 1]
                    # Make an array of points (x,y,z) from
                    # PoseStamped array
                    points = [p.pose.position for p in poses]
                    # Before the intense downsampling, check if the path
                    # changed or if it doesn't require an update.
                    self._last_path[topic]["publish"] = self._should_update_path(points, topic)
                    if not self._last_path[topic]["publish"]:
                        # Abort further processing this path, it won't be
                        # published.
                        break
                    # Last published points, raw (with no processing)
                    self._last_path[topic]["raw_points"] = points
                    if should_downsample:
                        # Get the max number of points this path can have
                        max_points = self._states["paths_config"].get("max_length")
                        # If max_points wasn't configured, get defaults based on
                        # current runlevel.
                        if not max_points:
                            max_points = (
                                MAX_NAV_PATH_POINTS_DETAILED
                                if (self.get_runlevel() > RUNLEVEL_MINIMAL)
                                else MAX_NAV_PATH_POINTS_DEFAULT
                            )
                        # Too many points? downsample
                        if len(points) > max_points:
                            points = self._downsample_path(points, max_points)
                    self._last_path[topic]["points"] = points
    def _should_update_path(self, points, topic):
        """
        Decides whether a path should be updated by computing the distance
        between the current path and the previous path sent. If this is less
        than 1m, the output is False.
        NOTE: It uses self._last_path and assumes that _path_data_mutex is locked.
        """
        return (
            path_distance(self._last_path.get(topic, {}).get("raw_points"), points) > 1
        )  # TODO(herchu) is this 1meter?
    def _should_process_localization_data(self):
        """
        Checks whether localization data should be processed or not, based
        on the 'map published' flag:
        - If map data is required (disable_map_published_flag = False),
            process localization data only if a map has been published by
            the agent.
        - If map data isn't required (disable_map_published_flag = True),
            process localization data always.
        """
        return self._states["disable_map_published_flag"] or self._map_agentlet.map_published
````

## File: inorbit/agentlets/map.py
````python
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
````

## File: inorbit/agentlets/meters.py
````python
# Copyright (c) 2018, InOrbit, Inc.
# All rights reserved.
import time
class Meters:
    """
    A Meters object saves an snapshot of some counters at a given timestamp,
    and can perform basic arithmetic on them -- e.g. calculating a delta
    over two different Meters snapshots. Moreover, Meters are created with a
    definition (a list of meter names) and consistency is ensured across
    accesses: a valid meter value can be absent, but trying to set or get
    a meter name not in the Meters definition raises an error. Better than
    just using a raw dictionary!
    TODO(herchu) At some point this class will be able to "serialize" into
    an mqtt packet.
    Members:
    _names: dictionary whose keys are meter names.
    TODO(herchu) expand these 'schema' definition with datatypes,
    e.g. { "count": int, "weight": float } (values are unused)
    _values: meter values. Keys are also in _names. Some may be missing
    _ts: timestamp (seconds since epoch)
    _duration: Only for "delta" Meters; the difference in time between the
    two Meters compared
    """
    def __init__(self, ts=None, template=None, names=None):
        """
        Constructs a Meters object.
        It receives a timestamp (optional) and one of two optional parameters:
        - a meters object, used as template (only for meters names)
        - a list or a dictionary of meter names
        The Meters object starts with no values (not the same as being zero
        valued).
        """
        self._ts = ts if (ts is not None) else time.time()
        self._values = {}
        if template is not None:
            self._names = template._names
        elif names is not None:
            if isinstance(names, list):
                self._names = dict.fromkeys(names)
            else:
                self._names = names  # A dictionary
        else:
            raise Exception("Incomplete meters definition; lacking meter names")
    def set(self, name, value):
        """
        Sets the value of a meter.
        """
        if name not in self._names:
            raise Exception("set: Invalid meter name")
        self._values[name] = value
    def get(self, name, default=0):
        """
        Gets the value of a meter.
        """
        if name in self._values:
            return self._values[name]
        elif name not in self._names:
            raise Exception("get: Invalid meter name")
        else:
            return default
    def ts(self):
        """
        Gets the timestamp (float value, python 'time' module).
        """
        return self._ts
    def ts_as_ms(self):
        """
        Gets the timestamp (int value, milliseconds).
        """
        return int(self.ts() * 1000)
    def duration(self):
        """
        Returns the duration (in seconds, float value), only if this is a delta
        object - otherwise it returns None.
        """
        return self._duration
    def delta(self, other):
        """
        Constructs a new Meters object as the delta from 'other' to 'this' object.
        """
        m = Meters(self._ts, template=self)
        for k in self._names:
            m.set(k, self._values[k] - other.get(k, 0))
        m._duration = self._ts - other._ts
        return m
    def is_empty(self):
        """
        Tells if this Meters object holds no values.
        """
        return len(self._values) == 0
    def __str__(self):
        """
        String representation - for debugging.
        """
        return (
            "{@"
            + str(self._ts)
            + ": "
            + "; ".join(
                [
                    k + "=" + (str(self._values[k]) if (k in self._values) else "?")
                    for k in self._names
                ]
            )
            + "}"
        )
````

## File: inorbit/agentlets/odometry.py
````python
# Copyright (c) 2020, InOrbit, Inc.
# All rights reserved.
# Odometry agentlet.
# Depends on the ROS module.
#
# TODO:
# - Discover the possible frame IDs
# - Review data publishing for minimal runlevel
import math
import threading
import time
import transformations
from util.overrides import overrides
from .agentlet import Agentlet
from .agentlet import RUNLEVEL_DEFAULT
from .agentlet import RUNLEVEL_FULL
from .agentlet import RUNLEVEL_SILENT
from .inorbit_pb2 import OdometryDataMessage
ODOM_FRAME = "odom"
BASE_FRAME = "base_link"
ODOM_QUERY_RATE_HZ = 5
# Odometry publisher's period for different runlevels
# (seconds)
PUBLISHER_PERIOD_FULL_RUNLEVEL = 0.5
PUBLISHER_PERIOD_DEFAULT_RUNLEVEL = 1
PUBLISHER_PERIOD_MINIMAL_RUNLEVEL = 10
# Timeout to compute speed from acquired data (seconds)
SPEED_CALCULATION_TIMEOUT = 3
MQTT_ODOMETRY_TOPIC = "ros/odometry/data"
MQTT_STATE_TOPIC = "ros/odometry/state"
# Max acceptable speed to filter out invalid distances, in m/s
MAX_VALID_SPEED = 3.0
class RosOdometryAgentlet(Agentlet):
    def __init__(self, uplink, ros):
        super(RosOdometryAgentlet, self).__init__(uplink)
        self._ros = ros
        self._states["base_frame"] = self.robot_namespace + BASE_FRAME
        self._states["odom_frame"] = self.robot_namespace + ODOM_FRAME
        # Timestamp, in milliseconds, when the agentlet started
        # accumulating odometry. Access to this variable must be
        # guaranteed by _mutex
        self._ts_start = 0
        # Timestamp, in milliseconds, of the last time odometry
        # accumulator was updated. Access to this variable must
        # be guaranteed by _mutex
        self._ts = 0
        # Accumulated displacement (meters). Access to this
        # variable must be guaranteed by _mutex
        self._linear_distance = 0
        # Accumulated rotation (radians). Access to this
        # variable must be guaranteed by _mutex
        self._angular_distance = 0
        # Flag to compute and publish speed
        self._should_compute_speed = False
        # Odometry data needed to compute speed
        self._last_linear_distance = 0
        self._last_angular_distance = 0
        self._last_ts = 0
        self._initial_pose = None
        self._last_pose = None
        # Mutex used to access _ts_start, _ts, _linear_distance
        # and _angular_distance
        self._mutex = threading.Lock()
        # Odometry listener thread running state
        self._odom_listener_running = False
        # Uplink publisher thread running state
        self._odom_publisher_running = False
        # Condition to handle publisher's sleep/wake up timing
        self._condition = threading.Condition()
    @overrides(Agentlet)
    def load(self, runlevel):
        self._states["runlevel"] = runlevel
        self._ros.add_submodule("odometry")
        # Reset the initial pose
        self._initial_pose = self._get_robot_pose()
        # Save the start time
        self._ts_start = self.get_ts()
        self._last_ts = self._ts_start
        # Define if speed should be computed or not
        self._set_speed_status()
        # Start odometry listener thread
        threading.Thread(target=self._odom_listener).start()
        # Start uplink publishing thread if runlevel is other than silent
        if self.get_runlevel() != RUNLEVEL_SILENT:
            self._launch_publisher_thread()
        self._states["loaded"] = True
        # Send a state update to the cloud
        self.publish_state(self.uplink, self._states)
        return True
    @overrides(Agentlet)
    def unload(self):
        # Shutdown odometry listener thread
        self._odom_listener_running = False
        # Shutdown uplink publisher thread
        self._odom_publisher_running = False
        # Re-initialize exception reporting
        self.once_logger.reset_all()
        # Remove ROS subscribers
        self._ros.remove_submodule("odometry")
        self._states["loaded"] = False
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
        # Define if speed should be computed or not with the new runlevel
        self._set_speed_status()
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
    def get_odometry(self):
        """
        Returns odometry data as an array: [ts_start, ts, linear, angular].
        """
        self._mutex.acquire()
        ts_start = self._ts_start
        ts = self._ts
        linear = self._linear_distance
        angular = self._angular_distance
        self._mutex.release()
        return [ts_start, ts, linear, angular]
    def is_odometry_valid(self, odometry_data):
        """
        Checks odometry data as obtained with get_odometry() is valid.
        """
        # A valid odometry is a four-element array with non-empty values
        return (
            odometry_data is not None
            and len(odometry_data) == 4
            and odometry_data[0] is not None
            and odometry_data[1] is not None
            and odometry_data[2] is not None
            and odometry_data[3] is not None
        )
    def _is_distance_valid(self, distance, new_pose, ts_delta_ms):
        """
        Validates a traveled distance appears to be physically plausible. We have
        seen cases with 100s of m/s that we now just filter out.
        """
        # Avoid zero division. 1 ms is short enough to filter almost any
        # non-zero traveled distance if the two snapshots were too close
        if ts_delta_ms <= 0:
            ts_delta_ms = 1
        if distance / (ts_delta_ms * 0.001) > MAX_VALID_SPEED:
            # Print this error to logs for anomaly analysis (at most once)
            error_msg = (
                f"Odometry: Traveled distance is too far: {distance}m at "
                f"{distance / (ts_delta_ms * 0.001)}m/s. Resetting odometry state."
                f"\nPrev pose:\n{self._initial_pose}"
                f"\nNew pose:\n{new_pose}\n"
            )
            self.once_logger.error("odometry_out_of_range", error_msg)
            return False
        return True
    def set_state(self, state):
        """
        Called whenever a set_module command is received.
        """
        if "base_frame" in state:
            self._states["base_frame"] = state["base_frame"]
        if "odom_frame" in state:
            self._states["odom_frame"] = state["odom_frame"]
        # Send a state update
        self.publish_state(self.uplink, self._states)
    def _odom_listener(self):
        """
        Runs on a separate thread. Listens to odometry msgs and updates the
        traversed distance.
        TODO (Flor_Grosso): odom listener and distance computations have been
        taken from teleop module. Consider removing these methods from there.
        """
        self._odom_listener_running = True
        while self._odom_listener_running:
            # If the initial pose was not obtained correctly when loading
            # the module, retry getting it.
            if self._initial_pose is None:
                self._initial_pose = self._get_robot_pose()
                if self._ts_start == 0:  # Start ts was lost due to a reset
                    self._mutex.acquire()
                    self._ts_start = self.get_ts()
                    self._mutex.release()
            else:
                last_pose = self._get_robot_pose()
                if last_pose is not None:
                    self._update_traversed_distance(last_pose)
            time.sleep(1.0 / ODOM_QUERY_RATE_HZ)
    def _get_robot_pose(self, time=None):
        """
        Helper method to get the latest known robot pose.
        Returns None if the pose can't be found.
        """
        odom_T_robot = self._ros.lookup_transform(
            self._states["odom_frame"], self._states["base_frame"]
        )
        return odom_T_robot.transform if odom_T_robot is not None else None
    def _angular_distance_between(self, start_pose, end_pose):
        """
        Computes the angular distance between two poses.
        """
        # Get Yaw from quaternion for the start pose
        start_rotation = start_pose.rotation
        start_quaternion = (start_rotation.x, start_rotation.y, start_rotation.z, start_rotation.w)
        euler_start = transformations.euler_from_quaternion(start_quaternion)
        yaw_start = euler_start[2]
        # Get Yaw from quaternion for the end pose
        end_rotation = end_pose.rotation
        end_quaternion = (end_rotation.x, end_rotation.y, end_rotation.z, end_rotation.w)
        euler_end = transformations.euler_from_quaternion(end_quaternion)
        yaw_end = euler_end[2]
        return yaw_end - yaw_start
    def _linear_distance_squared_between(self, start_pose, end_pose):
        """
        Computes the linear distance (squared) between two poses.
        """
        # Get robot position
        p_start = start_pose.translation
        p_end = end_pose.translation
        delta_x = p_end.x - p_start.x
        delta_y = p_end.y - p_start.y
        return delta_x * delta_x + delta_y * delta_y
    def _linear_distance_between(self, start_pose, end_pose):
        """
        Computes the linear distance between two poses.
        """
        # Get squared distance first
        distance_squared = self._linear_distance_squared_between(start_pose, end_pose)
        return math.sqrt(distance_squared)
    def _update_traversed_distance(self, last_pose):
        """
        Updates the traversed distance with the last odometry data.
        """
        distance = self._linear_distance_between(self._initial_pose, last_pose)
        ts = self.get_ts()
        self._mutex.acquire()
        if self._is_distance_valid(distance, last_pose, ts - self._ts):
            self._ts = ts
            # Compute accumulated linear and angular distances
            self._linear_distance += distance
            # Computing absolute value as the angular distance sign
            # represents direction. Negative is clockwise rotation.
            self._angular_distance += abs(
                self._angular_distance_between(self._initial_pose, last_pose)
            )
            self._initial_pose = last_pose
        else:
            # Poses are too far and distance suggests there is something wrong.
            # Reset the state
            self._ts_start = 0
            self._ts = 0
            self._linear_distance = 0
            self._angular_distance = 0
            self._initial_pose = None
        self._mutex.release()
    def _compute_speed(self):
        """
        Computes the average linear and angular speed between consecutive calls.
        """
        self._mutex.acquire()
        current_linear_distance = self._linear_distance
        current_angular_distance = self._angular_distance
        current_ts = self._ts
        self._mutex.release()
        current_pose = self._get_robot_pose()
        # Compute time difference and convert it to seconds
        delta_ts = float(current_ts - self._last_ts) * 0.001
        # If last data is too old or elapsed time is zero, don't compute speed
        if delta_ts > SPEED_CALCULATION_TIMEOUT or delta_ts == 0:
            speed = None
        else:
            linear_speed = (current_linear_distance - self._last_linear_distance) / delta_ts
            angular_speed = (
                -self._angular_distance_between(self._last_pose, current_pose) / delta_ts
                if self._last_pose
                else 0
            )
            speed = [linear_speed, angular_speed]
        # Store initial values for the next call
        self._last_linear_distance = current_linear_distance
        self._last_angular_distance = current_angular_distance
        self._last_ts = current_ts
        self._last_pose = current_pose
        return speed
    def _set_speed_status(self):
        """
        Sets flag to compute speed only if current runlevel is FULL.
        """
        # If agentlet is loaded in full runlevel, compute and publish speed
        self._should_compute_speed = self.get_runlevel() == RUNLEVEL_FULL
    def _publish_loop(self):
        """
        Runs on a separate thread. Publishes odometry data at a rate dependent
        on the runlevel.
        """
        self._odom_publisher_running = True
        while self._odom_publisher_running:
            try:
                self._publish_odometry_if_available()
            except Exception as e:
                self.once_logger.exception("odometry_publish", "Exception publishing data.")
            self._condition.acquire()
            # Throttle differently depending on the module runlevel
            if self.get_runlevel() == RUNLEVEL_DEFAULT:
                self._condition.wait(PUBLISHER_PERIOD_DEFAULT_RUNLEVEL)
            elif self.get_runlevel() == RUNLEVEL_FULL:
                self._condition.wait(PUBLISHER_PERIOD_FULL_RUNLEVEL)
            # If the runlevel is not recognized, publish at minimum
            # TODO (Flor_Grosso): publish data together with the ROS master
            # status for minimal runlevel
            else:
                self._condition.wait(PUBLISHER_PERIOD_MINIMAL_RUNLEVEL)
            self._condition.release()
        self.logger.info("Publisher thread shutting down.")
    def _publish_odometry_if_available(self):
        """
        Publishes the latest update of the traversed distance if possible.
        """
        odometry = self.get_odometry()
        if odometry is not None:
            odom_data = OdometryDataMessage()
            odom_data.ts_start = odometry[0]
            odom_data.ts = odometry[1]
            odom_data.linear_distance = odometry[2]
            odom_data.angular_distance = odometry[3]
            odom_data.speed_available = False
            if self._should_compute_speed:
                speed = self._compute_speed()
                if speed is not None:
                    odom_data.linear_speed = speed[0]
                    odom_data.angular_speed = speed[1]
                    odom_data.speed_available = True
            self.uplink.publish_protobuf(MQTT_ODOMETRY_TOPIC, odom_data)
    def _launch_publisher_thread(self):
        """
        Starts uplink publishing thread.
        """
        threading.Thread(target=self._publish_loop).start()
    def _shutdown_publisher_thread(self):
        """
        Shuts down publisher thread by setting its state to not running.
        TODO (Flor_Grosso): Consider implementing a way to make sure that thread
        is killed properly.
        """
        self._odom_publisher_running = False
````

## File: inorbit/agentlets/pose.py
````python
# Copyright (c) 2020, InOrbit, Inc.
# All rights reserved.
#
# Pose agentlet: Responsible for calculating poses (transformations between frames).
# It depends on the ROS module, and acts as helper for Localization agentlet.
#
# The Localization agentlet interacts with this agentlet by notifying each
# change in runlevel or loaded/unloaded state. When the LocalizationAgentlet
# is known to be publishing (which includes pose AND lasers, in addition to
# other data like path or map) this agentlet will decide to stop publishing
# so that data is not sent duplicated in 2 channels, with also the chance of
# ending up with slightly unsynchronized poses and lasers
#
#
# TODO(herchu) Support multiple poses / frames. The protobuf format allows it;
#   already; it may require adding one additional field (see .proto file),
#   but the code for now sends only one pose.
#
#
import threading
from util.overrides import overrides
from util.math_util import get_position_with_offset
from .agentlet import Agentlet
from .agentlet import RUNLEVEL_DEFAULT
from .agentlet import RUNLEVEL_FULL
from .agentlet import RUNLEVEL_SILENT
from .inorbit_pb2 import PoseMessage
from .inorbit_pb2 import PoseMessageData
# Default frame names for pose calculation
ROBOT_FRAME = "base_link"
# Publisher's period for different runlevels (seconds)
# Note that DEFAULT and MINIMAL need to meet the values
PUBLISHER_PERIOD_FULL_RUNLEVEL = 1
PUBLISHER_PERIOD_DEFAULT_RUNLEVEL = 10
PUBLISHER_PERIOD_MINIMAL_RUNLEVEL = 60
#
MIN_POSE_REPORT_THRESHOLD_SEC = 60
# MQTT pose topic is not shared with Localization agentlet; it only sends
# pose data. Since both topics' data overlap, they should not be published
# at the same time.
MQTT_POSE_TOPIC = "ros/loc/pose"
class RosPoseAgentlet(Agentlet):
    def __init__(self, uplink, ros, map_agentlet):
        super(RosPoseAgentlet, self).__init__(uplink)
        self._ros = ros
        self._map_agentlet = map_agentlet
        self._states["robot_frame"] = self.robot_namespace + ROBOT_FRAME
        # Condition to handle publisher's sleep/wake up timing
        self._condition = threading.Condition()
        # Mutex for controlling publisher thread variables and create or stop
        # threads. Controlled variables:
        #  - _publisher_thread
        #  - _publisher_period_sec_effective
        self._control_mutex = threading.Lock()
        # Sleep period for the publisher thread. This combines this agentlet
        # runlevel with an "override" from LocalizationAgentlet (its
        # _publisher_period_sec_override) into the _publisher_period_sec_effective.
        # The publisher thread (when it exists) will publish data only if
        # _publisher_period_sec_effective > 0. See _apply_runlevel()
        self._publisher_period_sec_override = 0
        self._publisher_period_sec_effective = 0
        self._publisher_thread = None
    @overrides(Agentlet)
    def load(self, runlevel):
        try:
            import transformations
            global transformations
        except Exception as e:
            self.once_logger.exception(
                "tf_transformations_load", "Exception loading tf.transformations."
            )
            return False
        self._states["runlevel"] = runlevel
        self._states["loaded"] = True
        # Start uplink publishing thread if runlevel is other than silent
        if self.get_runlevel() != RUNLEVEL_SILENT:
            self._apply_runlevel()
        # Send a state update to the cloud
        self.publish_state(self.uplink, self._states)
        return True
    @overrides(Agentlet)
    def unload(self):
        # Re-initialize exception reporting
        self.once_logger.reset_all()
        # Shutdown uplink publisher thread
        self._shutdown_publisher_thread()
        self._states["loaded"] = False
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
        # If old and new runlevels match, do nothing
        if runlevel == self.get_runlevel():
            return
        # Update runlevel
        self._states["runlevel"] = runlevel
        # Make this runlevel effective - see comment in that method
        self._apply_runlevel()
    def _apply_runlevel(self):
        """
        Combines the requested runlevel with
        the runlevel obtained by "override modules" (LocalizationAgentlet) to
        come up with the "effective" run level this agentlet should run.
        We do this since if LocalizationAgentlet is already reporting poses
        (loaded and runlevel > 0) then this module should not be repeating the
        same job.
        """
        # Current effective publish rate
        with self._control_mutex:
            last_publisher_period = self._publisher_period_sec_effective
        # Requested publish rate defined by runlevel (not taking into account
        # the LocalizationAgentlet)
        requested_publisher_period = self._runlevel_to_publisher_period(self.get_runlevel())
        if (
            self._publisher_period_sec_override > 0
            and self._publisher_period_sec_override <= requested_publisher_period
        ):
            # Localization module is already reporting data at a greater
            next_publisher_period = 0
        else:
            # else: LocalizationAgentlet is not publishing, or is publishing
            # at a low rate (while this module is requested to publish at
            # higher rate
            next_publisher_period = requested_publisher_period
        # If previously active and no longer publishing, stop thread
        if next_publisher_period == 0 and last_publisher_period > 0:
            self._shutdown_publisher_thread()
        # Conversely, if not publishing earlier and starting to publish
        # now, start a thread
        elif next_publisher_period > 0 and last_publisher_period == 0:
            self._launch_publisher_thread(next_publisher_period)
        # else, if publish period is different (and nonzero), just
        # wake up the thread so it shanges it publish frequency
        elif next_publisher_period != last_publisher_period:
            with self._control_mutex:
                self._publisher_period_sec_effective = next_publisher_period
            self.wake_up_publisher(self._condition)
        # The only remaining case: next_publisher_period == last_publisher_period,
        # there no changes ar eneeded
    def inform_override(self, loaded, period_secs):
        """
        Called from LocalizationAgenlet, to inform its runlevel and loaded state.
        To avoid sending poses through 2 different channels, it may decide to
        stop publishing (done in _apply_runlevel).
        """
        self._publisher_period_sec_override = period_secs if loaded else 0
        self._apply_runlevel()
    def set_state(self, state):
        """
        Called whenever a set_module command is received.
        """
        if "robot_frame" in state:
            self._states["robot_frame"] = state["robot_frame"]
        # Send a state update
        self.publish_state(self.uplink, self._states)
    def _get_robot_pose(self, time=None):
        """
        Helper method to get the latest known robot pose.
        Returns None if the pose can't be found.
        """
        map_frame = self._map_agentlet.map_frame_id
        # TODO(herchu) Cache last pose lookup and return it again if recent
        map_T_robot = self._ros.lookup_transform(map_frame, self._states["robot_frame"])
        ts = self.get_ts()
        return (map_T_robot.transform if map_T_robot is not None else None, ts)
    def _publish_loop(self):
        """
        Runs on a separate thread. Publishes data at a rate depending on runlevel.
        """
        while self._publisher_period_sec_effective > 0:
            try:
                self._publish_pose_if_available()
            except Exception as e:
                self.once_logger.exception("pose_publish", "Exception publishing data.")
            # Throttle differently depending on the module runlevel
            self._condition.acquire()
            self._condition.wait(self._publisher_period_sec_effective)
            self._condition.release()
        self.logger.info("Publisher thread shutting down.")
    def _runlevel_to_publisher_period(self, runlevel):
        """
        Returns the publish frequency (or rather: period, in seconds) associated
        to a runlevel. For now only runlevel DEFAULT is specified; while any
        other would trigger a MINIMAL report rate.
        """
        if runlevel == RUNLEVEL_SILENT:
            # 0: do not publish
            return 0
        if runlevel == RUNLEVEL_DEFAULT:
            return PUBLISHER_PERIOD_DEFAULT_RUNLEVEL
        if runlevel == RUNLEVEL_FULL:
            return PUBLISHER_PERIOD_FULL_RUNLEVEL
        # If the runlevel is not recognized, publish at minimum
        else:
            return PUBLISHER_PERIOD_MINIMAL_RUNLEVEL
    def _publish_pose_if_available(self):
        """
        Publishes the latest pose update.
        """
        (robot_pose, ts_msec) = self._get_robot_pose()
        if robot_pose is not None:
            # Get robot position and orientation
            p = robot_pose.translation
            # Get Yaw from quaternion
            o = robot_pose.rotation
            q = (o.x, o.y, o.z, o.w)
            euler = transformations.euler_from_quaternion(q)
            yaw = euler[2]
            # Create the pose message. For now we are sending ONE pose, but the
            # message has support for sending multiple poses eventually.
            # Note that when this is done, an additional field should identify
            # what each pose represents.
            pose_with_offset = get_position_with_offset(p.x, p.y)
            data = PoseMessageData()
            data.ts = ts_msec
            data.pos_x = pose_with_offset['pos_x']
            data.pos_y = pose_with_offset['pos_y']
            data.offset_x = pose_with_offset['offset_x']
            data.offset_y = pose_with_offset['offset_y']
            data.yaw = yaw
            data.frame_id = self._map_agentlet.inorbit_frame_id
            message = PoseMessage()
            message.poses.extend([data])
            self.uplink.publish_protobuf(MQTT_POSE_TOPIC, message)
    def _launch_publisher_thread(self, publish_period):
        """
        Starts uplink publishing thread.
        """
        with self._control_mutex:
            # in case another concurrent call to _launch_publisher_thread()
            if self._publisher_thread is None:
                self._publisher_period_sec_effective = publish_period
                self._publisher_thread = threading.Thread(
                    target=self._publish_loop, name="RosPose-publisher"
                )
                self._publisher_thread.start()
    def _shutdown_publisher_thread(self):
        """
        Shuts down publisher thread by setting its state to not running.
        """
        with self._control_mutex:
            if self._publisher_thread is not None:
                self._publisher_period_sec_effective = 0
                self.wake_up_publisher(self._condition)
                # The next join() should always succeed immediately as
                # _publisher_period_sec_effective is 0, but give it a timeout
                # of 1 sec anyway
                self._publisher_thread.join(1)
                self._publisher_thread = None
````

## File: inorbit/agentlets/ros_monitoring.py
````python
# Copyright (c) 2018, InOrbit, Inc.
# All rights reserved.
# Ros Monitoring agentlet. Monitors requested ROS processes and sends updates
# to the cloud.
import threading
from inorbit_pb2 import RosMonitorMessage
from ros_monitor.nodes_monitor import NodeMonitor
from ros_monitor.params_monitor import ParamMonitor
from ros_monitor.topics_monitor import TopicMonitor
from util.overrides import overrides
from .agentlet import Agentlet
MQTT_ROS_UPDATES_TOPIC = "ros/monitor/update"
# Updates publisher's period for minimal runlevel
# (seconds)
PUBLISHER_PERIOD_MINIMAL_RUNLEVEL = 10
class RosMonitoringAgentlet(Agentlet):
    def __init__(self, uplink, ros):
        super(RosMonitoringAgentlet, self).__init__(uplink)
        self._ros = ros
        # Dictionary of actions to perform indexed by
        # topic/node/param name
        self._states["ros_topic_monitor"] = {}
        self._states["ros_node_monitor"] = {}
        self._states["ros_param_monitor"] = {}
        # Flag to indicate if ros monitoring is being performed or not.
        self._is_topic_monitor_enabled = True
        self._topic_monitor = TopicMonitor(ros, self._states["ros_topic_monitor"])
        self._param_monitor = ParamMonitor(ros, self._states["ros_param_monitor"])
        self._node_monitor = NodeMonitor(ros, self._states["ros_node_monitor"])
        # Flag to indicate that tf tree data has been logged
        self._tf_tree_logged = False
        # Condition to handle publisher's sleep/wake up timing
        self._condition = threading.Condition()
    @overrides(Agentlet)
    def load(self, runlevel):
        self._states["runlevel"] = runlevel
        self._load_topic_monitor()
        self._load_param_monitor()
        self._load_node_monitor()
        # Start ros monitoring thread
        threading.Thread(target=self._ros_monitor).start()
        # Send a state update to the cloud
        self._states["loaded"] = True
        self.publish_state(self.uplink, self._states)
        return True
    def set_state(self, state):
        """
        Called whenever a set_module command is received.
        """
        if "ros_topic_monitor" in state.keys():
            # Sanity checks. If the state received is not a dict,
            # skip it.
            topics = state["ros_topic_monitor"]
            if isinstance(topics, dict):
                self._states["ros_topic_monitor"] = topics
                self._topic_monitor.update_topics(topics)
        if "ros_param_monitor" in state.keys():
            params = state["ros_param_monitor"]
            if isinstance(params, dict):
                self._states["ros_param_monitor"] = params
                self._param_monitor.update_params(params)
        if "ros_node_monitor" in state.keys():
            nodes = state["ros_node_monitor"]
            if isinstance(nodes, dict):
                self._states["ros_node_monitor"] = nodes
                self._node_monitor.update_nodes(nodes)
        # Send a state update to the cloud
        self.publish_state(self.uplink, self._states)
    @overrides(Agentlet)
    def unload(self):
        self._running = False
        self._states["loaded"] = False
        self.once_logger.reset_all()
        return True
    @overrides(Agentlet)
    def set_runlevel(self, runlevel):
        self._states["runlevel"] = runlevel
        self.wake_up_publisher(self._condition)
    def _ros_monitor(self):
        """
        Runs on a separate thread. Publishes ros processes updates at a rate
        dependent on the runlevel.
        """
        self._running = True
        while self._running is True:
            try:
                # Query and log TF tree (only once, at start up)
                self._log_tf_tree_config()
                data = RosMonitorMessage()
                data.master_status = self._ros.get_master_status()
                if self._is_topic_monitor_enabled:
                    data.topics_update.extend(self._topic_monitor.make_update_msg())
                data.params_update.extend(self._param_monitor.make_update_msg())
                data.nodes_update.extend(self._node_monitor.make_update_msg())
                # Publish to cloud
                self.uplink.publish_protobuf(MQTT_ROS_UPDATES_TOPIC, data)
                self._condition.acquire()
                self._condition.wait(PUBLISHER_PERIOD_MINIMAL_RUNLEVEL)
                self._condition.release()
            except Exception as e:
                self.once_logger.exception("ros_monitor_publish", "Exception publishing data.")
        # Stop monitoring topics when unloaded
        if self._is_topic_monitor_enabled:
            self._topic_monitor.stop()
        self.logger.info("Publisher thread shutting down.")
    def get_state_options(self, state_name=None):
        """
        Returns available topics, nodes or params.
        """
        if not self._states["loaded"]:
            self.logger.warning("Requested state_options but module isn't loaded")
            return
        if state_name == "ros_topic_monitor":
            if self._is_topic_monitor_enabled:
                return self._topic_monitor.list_available_topics()
        elif state_name == "ros_param_monitor":
            return self._param_monitor.list_available_params()
        elif state_name == "ros_node_monitor":
            return self._node_monitor.list_available_nodes()
    def _load_topic_monitor(self):
        """
        Loads topic monitor by first triggering a dependencies load. If this fails,
        topics monitoring is aborted.
        """
        # Sanity check to avoid using topic monitoring tool if dependencies
        # can't be loaded
        if not self._topic_monitor.load_dependencies():
            self._is_topic_monitor_enabled = False
            return
        # Updates topics to monitor with the current state
        self._topic_monitor.update_topics(self._states["ros_topic_monitor"])
        self._topic_monitor.start()
    def _load_param_monitor(self):
        """
        Loads param monitor by updating params to monitor with the received state.
        """
        # Updates params to monitor with the current state
        self._param_monitor.update_params(self._states["ros_param_monitor"])
    def _load_node_monitor(self):
        """
        Loads node monitor by updating nodes to monitor with the received state.
        """
        # Updates nodes to monitor with the current state
        self._node_monitor.update_nodes(self._states["ros_node_monitor"])
    def _log_tf_tree_config(self):
        """
        Queries and logs the tf tree description as a dictionary of child (key) to
        parent (value) frame ids.
        """
        if not self._tf_tree_logged:
            tf_tree = self._ros.get_tf_tree()
            if tf_tree:
                self.logger.info("TF tree loaded")
                self.logger.info(tf_tree)
                self._tf_tree_logged = True
````

## File: inorbit/agentlets/ros.py
````python
# Copyright (c) 2018, InOrbit, Inc.
# All rights reserved.
# Agentlet that provides access to the ROS environment.
import os
import threading
import time
import rclpy
import yaml
from inorbit.rossetup.ros import ros_autodetect
from util.overrides import overrides
from .agentlet import Agentlet
# from util.suppress_stderr import suppress_stderr
NODE_NAME = "inorbit_agent"
SPIN_TIMEOUT = 0.01  # sec
class RosPublisher:
    """
    Simple POD that can hold a publisher, for submodules
    to provide and get their publishers back
    """
    pub = None
class RosSubmodule:
    """
    Simple POD to hold submodule registrations and their
    associated pub and sub handles
    """
    def __init__(self, subs=(), pubs=()):
        self.subs = subs
        self.pubs = pubs
        self.sub_handles = []
        self.pub_handles = []
class RosAgentlet(Agentlet):
    def __init__(self, uplink):
        super(RosAgentlet, self).__init__(uplink)
        self.ros_master_status = 0
        self._ros_thread = None
        self._watchdog_thread = None
        self.running = True
        self._node_handle = None
        # TF support
        self._tf_buffer = None
        self._tf_listener = None
        # Modules
        self._tf2_ros_module = None
        # Registered submodules, by module name
        self._submodules = {}
        # Mechanism to work around a known bug in rospy where unregistering
        # publishers may cause spam in the console and rosout.
        # See: https://github.com/ros/ros_comm/issues/111
        # Only unregister publishers if it is explicitly enabled
        # NOTE(adamantivm) This was the default behavior in agents prior to
        # 3.21.0
        self._unregister_publishers = (
            os.environ.get("INORBIT_ROS_UNREGISTER_PUBLISHERS", "FALSE") == "TRUE"
        )
        # Keep a list of existing publishers per topic to avoid duplication
        self._all_pub_handles = {}
    @overrides(Agentlet)
    def load(self, runlevel):
        self._states["loaded"] = self._discover_ros()
        self.logger.info(
            "rospy Publisher unregister workaround {}".format(
                "DISABLED" if self._unregister_publishers else "ENABLED"
            )
        )
        if self._states["loaded"]:
            # Start watchdog, which in turn starts ROS threads
            self._watchdog_thread = threading.Thread(target=self._watchdog_run)
            self._watchdog_thread.start()
        # Wait before returning control to allow the rospy client to
        # start and avoid subsequent dependent modules to fail on load
        time.sleep(2)
        return self._states["loaded"]
    def _discover_ros(self):
        # Find a system installation of ROS where rospy can be used from
        # TODO(adamantivm) Look in more places and test more thoroughly
        ros_autodetect()
        try:
            import rclpy
            global rclpy
        except Exception as e:
            self.once_logger.exception("rclpy", "Exception loading rclpy.")
            return False
        try:
            import tf2_ros
            self._tf2_ros_module = tf2_ros
        except Exception as e:
            self.once_logger.exception("tf2_ros_load", "Exception loading tf2_ros.")
        try:
            import transformations
            global transformations
        except Exception as e:
            self.once_logger.exception(
                "tf_tranformations_load", "Exception loading tf.transformations."
            )
        # Also import our tf_util module. This is in our local path, but it
        # depends on correct ROS paths to work, so this is the only module that
        # imports it.
        try:
            import util.tf_util
            global tf_util
            self._tf_util_module = util.tf_util
        except Exception as e:
            self.once_logger.exception("tf_util_load", "Exception loading tf_util.")
        return True
    def add_submodule(self, name, subs=(), pubs=()):
        """
        Registers a new submodule that uses ROS publishers or subscribers.
        """
        submodule = RosSubmodule(subs, pubs)
        # Register the newly added listeners
        self._register_listeners(submodule)
        # Save the reference for future unloading, etc.
        self._submodules[name] = submodule
    def add_submodule_listeners(self, name, subs=(), pubs=()):
        """
        Adds publishers and/or subscribers to a submodule.
        """
        submodule = self._submodules[name]
        self._add_listeners(submodule, subs, pubs)
    def remove_submodule(self, name):
        """
        Unregisters a submodule and shuts down publishers and subscribers.
        """
        if name not in self._submodules:
            self.logger.warning(f"Attempt to remove non-registered module: {name}.")
            return
        submodule = self._submodules[name]
        self._unregister_listeners(submodule)
        del self._submodules[name]
    def lookup_transform(self, child, parent, time=None):
        """
        Gets the latest available transform for two given frames.
        """
        # TODO(adamantivm) Proper exception design and raising
        # If the ROS master is not available, return None
        if self._tf_listener is None or self._tf_buffer is None or self._tf2_ros_module is None:
            return None
        # If no timestamp is requested, get the latest available one
        if time is None:
            # NOTE(herchu) Changing this from self.ros_now() to 0, meaning "latest" and
            # not "now". With simulations, if amcl is not keeping up to speed, we may get
            # something like :
            # Requested time [X] but the latest data is at time [almost X], when looking up
            # transform from frame [A] to frame [B]
            # See also:
            # https://github.com/ros2/geometry2/blob/8a3db872eaa01e59a98f65dd7ddce8b55788bda3/tf2_ros/include/tf2_ros/buffer.h#L80
            # What we could do is to discard the obtained transformation is its time
            # is too far in the past from self.ros_now() at the time of calling the function.
            time_to_request = rclpy.time.Time()  # This is equivalent to `time = 0`
        else:
            # Just request the transform at the time indicated as argument
            time_to_request = time
        try:
            request_stamp = self.ros_now()
            trans = self._tf_buffer.lookup_transform(child, parent, time_to_request)
        except Exception as e:
            # Keep track of reported exceptions to avoid sending
            # repeated log reports.
            # This will be a dict with key = "<child> to <parent>" and value
            # "number of times occurred".
            exc_key = f"{child} to {parent}"
            self.once_logger.exception(exc_key, f"Exception looking up transform: {exc_key}.")
            return None
        # TODO(herchu) calculate time difference between trans.header.stamp (a
        # builtin_interfaces.msg.Time object) and request_stamp (a Time object). Do this
        # when `time is None`, meaning we are requesting the latest transform, and if the
        # time difference is great than, say, a few milliseconds, discard it and return
        # "no pose". But if it is up to 1 or 5 ms, accept it -- assume a simulator is not
        # keeping up to speed.
        # if time is None:
        #     ....
        return trans
    def ros_now(self):
        """
        Helper method to get the 'now' time in ROS format.
        """
        return self._node_handle.get_clock().now()
    def ros_secs_ago(self, secs):
        """
        Helper method to get a past time in ROS format as the subtraction
        from 'now' and a ROS Duration in seconds.
        """
        return self._node_handle.get_clock().now() - rclpy.duration.Duration(seconds=secs)
    def get_published_topics(self):
        """
        Returns a list of topic names and types being published.
        """
        topics = self._node_handle.get_topic_names_and_types()
        return topics
    def get_topics_publishing(self, message_type):
        """
        Gets the topics where the robot is publishing messages of a given type.
        """
        published_topics = self.get_published_topics()
        matching_topics = []
        if published_topics:
            for topic in published_topics:
                if topic[1][0] == message_type:
                    matching_topics.append(topic[0][1:])
        return matching_topics
    def get_msg_type_for(self, topic):
        """
        Gets the msg type for a given topic. If topic not found, returns None.
        """
        # get topics
        # api returns the topic msg as a 1 item list
        topics = {topic[0]: topic[1][0] for topic in self._node_handle.get_topic_names_and_types()}
        try:
            if topic[0] != "/":
                return topics["/" + topic]
            else:
                return topics[topic]
        except Exception as e:
            return None
    def get_topics_to_publish(self, message_type):
        """
        Gets the topics where the agent can publish messages of a given type
        from a list of all available topics. Each item on the list is a key - value
        pair as for [topic_name, message_type].
        """
        # api returns the topic msg as a 1 item list
        topics = [
            (topic[0], topic[1][0]) for topic in self._node_handle.get_topic_names_and_types()
        ]
        # Get the list of topics where the robot publishes data which
        # corresponds to message_type
        published_topics = self.get_topics_publishing(message_type)
        # Save ros topics which have a message_type as desired
        matching_topics = [item[0][1:] for item in topics if item[1] == message_type]
        # Return list of topics leaving those with publishers at the end
        # TODO (Flor_Grosso): define a priority level or identifier for the
        # caller to distinguish this classification (only publishers, only
        # subscribers or both)
        return sorted(matching_topics, key=lambda x: (x in published_topics, x))
    def update_subscriber_topic(self, submodule_name, old_topic, new_sub):
        """
        Updates a topic for a subscriber. new_sub is an array of strings
        consisting of [new_topic_name, topic_message_type, callback_method].
        """
        if old_topic is not None:
            self.unregister_subscriber_to(old_topic, submodule_name)
        submodule = self._submodules[submodule_name]
        try:
            qos = new_sub[3]
        except IndexError:
            qos = 10
        self.logger.info("Registering subscriber to topic: %s." % new_sub[0])
        sub_handle = self._node_handle.create_subscription(new_sub[1], new_sub[0], new_sub[2], qos)
        submodule.sub_handles.append(sub_handle)
    def update_publisher_topic(self, submodule_name, old_topic, new_pub):
        """
        Updates a topic for a publisher. new_pub is an array of strings
        consisting of [new_topic_name, topic_message_type, publisher_method].
        """
        # Avoid updating publisher if ros master is down
        if old_topic is not None:
            self._unregister_publisher_to(old_topic, submodule_name)
        submodule = self._submodules[submodule_name]
        try:
            qos = new_pub[3]
        except IndexError:
            qos = 10
        self.logger.info("Registering publisher to topic: %s." % new_pub[0])
        new_pub[2].pub = self._register_publisher(new_pub[0], new_pub[1], qos)
        submodule.sub_handles.append(new_pub[2].pub)
    def transform_pose(self, pose, frame_id_from, frame_id_to, time=None):
        """
        Transforms a geom_msgs.Pose object from a given frame_id to a target
        frame_id. If no transform can be obtained from ros or any error occurs, it
        returns None.
        """
        tf = self.lookup_transform(frame_id_to, frame_id_from, time=time)
        # If no transform could be obtained, ignore this map
        if tf is None:
            self.once_logger.warn("transform_pose", "No transform found; cannot transform pose.")
            return None
        map_T_odom = self._tf_util_module.transform_to_matrix(tf.transform)
        odom_T_costmap = self._tf_util_module.pose_to_matrix(
            self._tf_util_module.geommsgpose_to_pose(pose)
        )
        map_T_costmap = transformations.concatenate_matrices(odom_T_costmap, map_T_odom)
        pose_arr_in_target = self._tf_util_module.matrix_to_pose(map_T_costmap)
        pose_in_target = self._tf_util_module.pose_to_geommsgpose(pose_arr_in_target)
        return pose_in_target
    def get_param_names(self):
        """
        Returns a list of parameters names available in the node
        """
        return [*self._node_handle._parameters]
    def get_param(self, name):
        """
        Returns a parameter value from the node.
        """
        return self._node_handle.get_parameter(name)._value
    def get_node_names(self):
        """
        Returns a list of active node names.
        """
        return self._node_handle.get_node_names()
    # RECHECK IF NEEDED, AND IF SO, LOOK FOR A ROS 2 ALTERNATIVE
    # """
    # Tests connectivity to node and returns True if node pinged.
    # """
    # def rosnode_ping(self, node_name):
    #     # Note (Flor_Grosso): Even if verbose is explicitly set to False,
    #     # this method prints to console when the node to be pinged is unknown.
    #     # stderr needs to be suppressed to avoid spamming the log.
    #     with suppress_stderr():
    #         return rosnode.rosnode_ping(node_name, 1, False)
    # """
    # Receives only one message from a given topic. It has a default timeout of 1
    # minute.
    # """
    # def wait_for_message(self, topic, msg_type):
    #     return rospy.wait_for_message("/" + topic, msg_type, timeout=60)
    def _watchdog_run(self):
        """
        Watchdog thread run method.
        It has the high-level logic that constantly checks for ROS connection
        status
        """
        while self.running:
            # If the ROS thread is not created or dead, launch a new one
            if self._ros_thread is None:
                self._ros_thread = threading.Thread(target=self._ros_run)
                self._ros_thread.start()
            time.sleep(10)
    def _ros_run(self):
        """
        ROS thread run method.
        All ROS functions are done here on a separate thread to avoid
        locking the main agentlet thread.
        """
        self.ros_master_status = 0
        rclpy.init()
        self._node_handle = rclpy.create_node(NODE_NAME)
        # ROS starting ###
        self.logger.info("Initializing ROS processes.")
        # Set use_sim_time if needed
        self._determine_ros_time_source()
        # Register message publishers and subscribers
        for submodule in self._submodules.values():
            self._register_listeners(submodule)
        # Checks if tf2 module is available
        if self._tf2_ros_module is not None:
            # Create TF buffers
            self._tf_buffer = self._tf2_ros_module.Buffer()
            self._tf_listener = self._tf2_ros_module.TransformListener(
                self._tf_buffer, self._node_handle
            )
        # ROS master is now alive
        self.ros_master_status = 1
        self.logger.info("ROS processes initialized.")
        # Keep testing ROS connection
        while self._ros_ping():
            try:
                rclpy.spin_once(self._node_handle, timeout_sec=SPIN_TIMEOUT)
            except rclpy.exceptions.InvalidHandle:
                self.once_logger.error(
                    "spin_once_exception", "InvalidHandle exception while running spin_once method"
                )
                break
        # ROS dying ###
        self.logger.info("ROS processes shutting down.")
        self.ros_master_status = 0
        self._ros_thread = None
        # Clean-up topic subscribers
        for submodule in self._submodules.values():
            self._unregister_listeners(submodule)
        # Clean-up TF buffers
        # NOTE: tf2-py 0.5.13 doesn't have an unregister method
        if hasattr(self._tf_listener, "unregister"):
            self._tf_listener.unregister()
        self._tf_listener = None
        # TODO(adamantivm) Look more thoroughly for how to clear the
        # TF buffer
        self._tf_buffer = None
        self.logger.info("ROS processes disabled.")
        # HACK: restarting this thread is not functioning properly.
        # Commit suicide as a workaround to allow the agent to restart
        # fresh.
        # see https://trello.com/c/Hqpqdggx/183-proper-reconnection-on-ros-shutdown-and-restart
        # TODO(adamantivm) Fix the underlying issue and remove this hack
        self.logger.warning("Killing agent to force a full fresh restart.")
        os._exit(0)
    def _ros_ping(self):
        """
        Returns True if context is still up.
        """
        return rclpy.ok()
    def _register_listeners(self, submodule):
        """
        Registers all publishers and subscribers for the given submodule. Called
        when creating a submodule.
        """
        self._add_listeners(submodule, submodule.subs, submodule.pubs)
    def _add_listeners(self, submodule, subs, pubs):
        """
        Adds new publishers and subscribers to a given submodule.
        """
        # Subscribers
        for sub in subs:
            if sub[1] == "":
                self.logger.warning("Skipping subscriber to empty topic.")
                continue
            self.logger.info("Registering subscriber to topic: %s." % sub[0])
            try:
                qos = sub[3]
            except IndexError:
                qos = 10
            sub_handle = self._node_handle.create_subscription(sub[1], sub[0], sub[2], qos)
            submodule.sub_handles.append(sub_handle)
        # Publishers
        for pub in pubs:
            if pub[0] == "":
                self.logger.warning("Skipping publisher to empty topic.")
                continue
            try:
                qos = pub[3]
            except IndexError:
                qos = 10
            pub[2].pub = self._register_publisher(pub[1], pub[0], qos)
            submodule.pub_handles.append(pub[2].pub)
    def _unregister_listeners(self, submodule):
        """
        Unregisters publishers and subscribers for the given submodule.
        """
        # Subscribers
        for sub_handle in submodule.sub_handles:
            self.logger.info(f"Unregistering subscriber to {sub_handle.topic}")
            self._node_handle.destroy_subscription(sub_handle)
        submodule.sub_handles = []
        # Publishers
        for pub_handle in submodule.pub_handles:
            if self._unregister_publishers:
                self._node_handle.destroy_publisher(pub_handle)
        submodule.pub_handles = []
        for pub in submodule.pubs:
            pub[2].pub = None
    def unregister_subscriber_to(self, topic_name, submodule_name):
        """
        Unregisters a listener to a topic for a subscriber.
        """
        if submodule_name not in self._submodules:
            self.logger.warning(
                "Attempt to unregister a subscriber "
                f"for a non-registered module: {submodule_name}."
            )
            return
        submodule = self._submodules[submodule_name]
        for sub_handle in submodule.sub_handles:
            # Support both topic names with an initial '/' and those with
            # no initial '/'
            if topic_name[0] != "/":
                topic_name = "/" + topic_name
            if sub_handle.topic == topic_name:
                self.logger.info("Unregistering subscriber " "to: %s." % (sub_handle.topic))
                self._node_handle.destroy_subscription(sub_handle)
                # Remove sub_handle for current topic
                submodule.sub_handles.remove(sub_handle)
    def _register_publisher(self, topic, message_type, qos):
        """
        Registers a new publisher with the given configuration.
        Normally, in order to avoid a known rospy bug, we avoid publisher
        unregisters and also re-use previously registered publishers.
        Returns a publisher object, which can be used to publish messages.
        """
        # If we're not unregistering publishers AND there is an existing one,
        # re-use it
        if not self._unregister_publishers and topic in self._all_pub_handles:
            publisher = self._all_pub_handles[topic]
            self.logger.info("Re-using publisher for topic: %s." % topic)
        # Otherwise, we must create a new one
        else:
            self.logger.info("Registering publisher for topic: %s." % topic)
            publisher = self._node_handle.create_publisher(topic, message_type, qos)
            if not self._unregister_publishers:
                # If we're not registering publishers and had to register a
                # new one, save it for future use
                self._all_pub_handles[topic] = publisher
        return publisher
    def _unregister_publisher_to(self, topic_name, submodule_name):
        """
        Unregisters a listener to a topic for a publisher.
        """
        # Avoid unregistering publisher if ros master is down
        # TODO (Flor_Grosso): consider notifying the caller that the publisher
        # couldn't be unregistered, in case it wants to retry later.
        if submodule_name not in self._submodules:
            self.logger.warning(
                "Attempt to unregister a publisher "
                f"for a non-registered module: {submodule_name}."
            )
            return
        submodule = self._submodules[submodule_name]
        for pub_handle in submodule.pub_handles:
            if pub_handle.topic == "/" + topic_name:
                if self._unregister_publishers:
                    self.logger.info("Unregistering publisher " "to: %s." % (pub_handle.topic))
                    self._node_handle.destroy_publisher(pub_handle)
                # Remove pub_handle for current topic
                submodule.pub_handles.remove(pub_handle)
    def get_tf_tree(self):
        """
        Returns the tf tree description as a dictionary of child (key) to parent
        (value) frame ids.
        """
        if self._tf_listener is None or self._tf_buffer is None or self._tf2_ros_module is None:
            return None
        frame_config = {}
        # Query tf tree
        try:
            tf_frames = yaml.load(self._tf_buffer.all_frames_as_yaml())
            # Extract only child and parent frame ids values from the raw
            # yaml data. Make a sub dict and log it.
            for child_id in tf_frames.keys():
                frame_config[child_id] = tf_frames[child_id]["parent"]
        except Exception as e:
            self.once_logger.warn("tf_frames", "Exception getting tf tree data")
        return frame_config
    def _determine_ros_time_source(self):
        """
        Determines if simulation time should be used and set the parameter on the node.
        """
        env_var = os.environ.get("USE_SIM_TIME")
        if env_var:
            use_sim_time = True if str(env_var).upper() in ["TRUE", "1"] else False
            self._node_handle.set_parameters(
                [
                    rclpy.parameter.Parameter(
                        "use_sim_time", rclpy.parameter.Parameter.Type.BOOL, use_sim_time
                    )
                ]
            )
            if use_sim_time:
                self.logger.info("Using simulation time")
````

## File: inorbit/agentlets/rosbag.py
````python
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
````

## File: inorbit/agentlets/rosout.py
````python
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
````

## File: inorbit/agentlets/spatial_annotations.py
````python
# Copyright (c) 2018, InOrbit, Inc.
# All rights reserved.
# Spatial annotations agentlet.
# Depends on the ROS module.
import threading
import agentlet
import numpy as np
from ros import RosPublisher
from util.overrides import overrides
NO_GO_ZONE_COST = 100
PUBLISHER_PERIOD_DEFAULT_RUNLEVEL = 10
# Topic to publish ros data to, if none is provided on the config.
DEFAULT_OUTPUT_TOPIC = "inorbit/map"
class SpatialAnnotationsAgentlet(agentlet.Agentlet):
    def __init__(self, uplink, ros, localization):
        super(SpatialAnnotationsAgentlet, self).__init__(uplink)
        self._ros = ros
        self._localization = localization
        # ROS publisher for designed global maps/costmaps
        self._ros_map_publisher = RosPublisher()
        # Spatial annotations object. It contains publication params required
        # per publication mode and a description of the zones to trace (polygon
        # coordinates and cost to fill the areas with).
        #
        # Complete schema for this module state can be found under:
        # inorbit/web/lib/collections.js
        self._states["spatial_annotations"] = {}
        # Flag to indicate whether a map/costmap has already been published.
        self._costmap_published = False
        # Condition to handle publisher's sleep/wake up timing
        self._condition = threading.Condition()
    @overrides(agentlet.Agentlet)
    def load(self, runlevel):
        self._states["runlevel"] = runlevel
        try:
            from util.Image import Image
            global Image
        except Exception as e:
            self.once_logger.exception(
                "image_available", "Exception when loading Image package: " + str(e)
            )
            return False
        else:
            self.logger.info("Using %s as image processing package." % Image._PACKAGE)
        try:
            import nav_msgs
            import nav_msgs.msg
            global nav_msgs
        except Exception as e:
            self.once_logger.exception("nav_msgs_load", "Exception loading nav_msgs.")
            return False
        # Create the ROS publisher for costmap mode
        self._add_ros_submodule()
        threading.Thread(target=self._map_publisher).start()
        self._states["loaded"] = True
        # Send a state update to the cloud
        self.publish_state(self.uplink, self._states)
        return True
    @overrides(agentlet.Agentlet)
    def unload(self):
        # Re-initialize exception reporting
        self.once_logger.reset_all()
        # Remove ROS subscribers
        self._ros.remove_submodule("spatial_annotations")
        self._costmap_published = False
        self._map_publisher_running = False
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
        if "spatial_annotations" in state.keys():
            # First check if there's an output topic configured, or we should
            # use defaults.
            output_topic = (
                state["spatial_annotations"].get("publication_params", {}).get("output_topic")
            )
            if not output_topic:
                output_topic = DEFAULT_OUTPUT_TOPIC
                state["spatial_annotations"]["publication_params"]["output_topic"] = output_topic
            if self._states["loaded"]:
                # Update publisher with output topic
                self._update_ros_publisher(output_topic)
                self._costmap_published = False
                self.once_logger.reset_all()
            self._states["spatial_annotations"] = state["spatial_annotations"]
        # Send a state update
        self.publish_state(self.uplink, self._states)
    def _add_ros_submodule(self):
        """
        Adds the spatial_annotations submodule to ROS agentlet when loading. Note
        that the map publisher will be added if there's a valid state only.
        """
        output_topic = (
            self._states["spatial_annotations"]
            .get("publication_params", {})
            .get("output_topic", DEFAULT_OUTPUT_TOPIC)
        )
        if output_topic:
            self._ros.add_submodule(
                "spatial_annotations",
                pubs=((output_topic, nav_msgs.msg.OccupancyGrid, self._ros_map_publisher, True),),
            )
        else:
            self._ros.add_submodule("spatial_annotations")
    def _update_ros_publisher(self, new_topic):
        """
        Updates ros publisher when a new state is received.
        """
        # Do not update publisher if the new output topic is not valid or if it
        # didn't update.
        if (
            not new_topic
            or self._states["spatial_annotations"].get("publication_params", {}).get("output_topic")
            == new_topic
        ):
            return
        new_pub = (new_topic, nav_msgs.msg.OccupancyGrid, self._ros_map_publisher, True)
        self._ros.update_publisher_topic("spatial_annotations", new_topic, new_pub)
    def _map_publisher(self):
        """
        Map publisher loop. It runs on a separate thread.
        """
        self._map_publisher_running = True
        while self._map_publisher_running:
            try:
                self._maybe_publish()
            except Exception as e:
                self.once_logger.exception(
                    "spatial_annotations_publish", "Exception publishing data."
                )
            self._condition.acquire()
            self._condition.wait(PUBLISHER_PERIOD_DEFAULT_RUNLEVEL)
            self._condition.release()
        self.logger.info("Publisher thread shutting down.")
    def _maybe_publish(self):
        """
        Creates and publishes the map/costmap if possible and appropriate.
        """
        # If the costmap is already published, skip it
        if self._costmap_published:
            return
        # TODO (Flor_Grosso): Consider checking whether the source map data
        # received matches the localization config. Use
        # `_is_source_map_current()` for that.
        if self._ros_map_publisher.pub is None:
            self.once_logger.warn("ros_publisher_not_set", "ROS publisher not set. Aborting.")
            return
        publication_params = self._states["spatial_annotations"].get("publication_params", {})
        # Get current map to merge with the given polygons.
        source_topic = publication_params.get("source_topic")
        base_map = None
        try:
            base_map = self._ros.wait_for_message(source_topic, nav_msgs.msg.OccupancyGrid)
        except Exception as e:
            self.once_logger.warn("read_source_topic", f"Couldn't get data from {source_topic}")
            return
        # Create custom map by tracing the zones
        # TODO (Flor_Grosso): Create the rest of the zones here too.
        custom_map = self._add_no_go_zones_to(base_map)
        # Publish ROS Message if a valid map was created only
        # TODO (Flor_Grosso): send a notification to the client about this error.
        if custom_map is None:
            self.once_logger.warn(
                "map_design_error", "Map with spatial annotations couldn't be created."
            )
            return
        # Create the OccupancyGrid Message
        msg = nav_msgs.msg.OccupancyGrid()
        msg.header.stamp = self._ros.ros_now()
        msg.header.frame_id = base_map.header.frame_id
        # Preserve the map metadata from the base map (resolution,
        # width, height, origin)
        msg.info = base_map.info
        msg.data = custom_map
        # Publish the map
        self._ros_map_publisher.pub.publish(msg)
        self._costmap_published = True
        self.logger.info("Global map published.")
    def _add_no_go_zones_to(self, base_map):
        """
        Creates a customized map/costmap with no go zones in it, parting from the
        base_map.
        """
        # If there is no base map, abort the design.
        # NOTE (Flor_Grosso): consider supporting creating maps from scratch,
        # with no base and just the polygons.
        if not base_map:
            return None
        annotations = self._states["spatial_annotations"].get("annotations", [])
        # Get contours for no go zones only. This will return a list of objects
        # with 'data' (coordinates) and 'cost' fields.
        contours = filter(lambda x: x["type"] == "NO_GO_ZONE", annotations)
        # If there are no no go zones on the spatial annotations, return the
        # original map.
        if not contours:
            self.logger.warning(
                "No polygon coordinates received. Output map " "will be the same as input map."
            )
            return base_map.data
        # If there is a valid source map, use it as a base to create the
        # new one.
        map_width = base_map.info.width
        map_height = base_map.info.height
        # The data from ros comes as a 1d array, but Image
        # needs it as a 2d array.
        base_map_2d = np.reshape(base_map.data, (map_width, map_height))
        im = Image()
        updated_map_2d = im.fillPolygons(base_map_2d, contours, NO_GO_ZONE_COST)
        return updated_map_2d.flatten()
    def _is_source_map_current(self):
        """
        Checks if source data is current, by comparing the map checksum with the
        checksum of the data provided by the localizationAgentlet.
        """
        source_map_checksum = self._states["spatial_annotations"].get("map", {}).get("checksum")
        return (
            source_map_checksum is not None
            and source_map_checksum == self._localization.get_map_checksum()
        )
````

## File: inorbit/agentlets/state_manager.py
````python
# Copyright (c) 2021, InOrbit, Inc.
# All rights reserved.
# Module States Manager
#
import os
import shelve
import threading
import time
import inorbit.logger  # Import before other modules to set logging format
from inorbit import INORBIT_MODULE_STATES_CACHE_FILE
from util.once_logger import OnceLogger
# TODO(diegobatt): Make this threshold configurable
MAX_CACHE_AGE = 60 * 60 * 24 * 2  # 2 days
class ModuleStateManager(object):
    """
    Manager for module states.
    This class provides tools for persisting module states into disk, loading and
    setting their state
    TODO(diegobatt): This module should overtake most of the concerns in the Agent's
    main entrypoint. Such as defining default modules and ros dependant ones. Also,
    Subscribing to modules/set_state and modules/get_state_options topics.
    TODO(diegobatt): This module uses shelve as the cache backend, that is proving to be not as
    reliable as we would want. Consider replacing it with another disk-persistent data structure.
    """
    def __init__(self, modules):
        self._modules = modules
        self.logger = inorbit.logger.getLog(self.__class__.__name__)
        self.once_logger = OnceLogger(self.logger)
        # Lock for accessing the modules cache as it is not natively thread-safe
        # for write operations
        self._cache_mutex = threading.Lock()
        # Dict like object persisted into disk
        try:
            self._cache = shelve.open(INORBIT_MODULE_STATES_CACHE_FILE)
        except Exception:
            # If for some reason the cache is corrupted and can't be opened, force
            # the creation of a new one
            self._force_cache_refresh()
    def _force_cache_refresh(self):
        """
        Forces the creation of a new cache in case it is corrupted for some reason.
        """
        self.logger.info("Clearing corrupted module states cache")
        try:
            if os.path.exists(INORBIT_MODULE_STATES_CACHE_FILE):
                os.remove(INORBIT_MODULE_STATES_CACHE_FILE)
            self._cache = shelve.open(INORBIT_MODULE_STATES_CACHE_FILE, "n")
        except Exception as e:
            self.logger.error("Failed to clear module states cache: %s", str(e))
    def save_module_state(self, module_name):
        """
        Persists a module's state into a disk cache.
        """
        # Python 2/3 compatibility: if in Python 2 type(module_name) is <type 'unicode'>
        # The cache won't work, so we convert it to string
        module_name = str(module_name)
        module = self._modules.get(module_name)
        if not module:
            return
        state = module.get_state()
        with self._cache_mutex:
            try:
                self._cache[module_name] = (time.time(), state)
                # Flush content to disk
                self._cache.sync()
            except Exception as e:
                self.logger.error("Failed to save module state for %s: %s", module_name, str(e))
                self.logger.error(e, exc_info=True)
                self._force_cache_refresh()
    def load_module_state(self, module_name):
        """
        Loads a module's state from a disk cache and apply it
        NOTE(diegobatt): This method is awfully similar to inorbit.py's load_module,
        when this manager overtakes all the module-related responsibilities from inorbit.py
        this method should have the exact same functionality as that one. For instance,
        this is not currently checking that ros is enabled for the ros dependant modules
        """
        # Python 2/3 compatibility: if in Python 2 type(module_name) is <type 'unicode'>
        # The cache won't work, so we convert it to string
        module_name = str(module_name)
        with self._cache_mutex:
            try:
                ts, state = self._cache.get(module_name, (None, None))
                # If module is not in the cache, return
                if state is None:
                    return
                if time.time() - ts > MAX_CACHE_AGE:
                    self.logger.info(
                        "Avoiding to load module %s from cache as it is too old", module_name
                    )
                    # If key is too old, remove it
                    del self._cache[module_name]
                    return
            except Exception as e:
                self.logger.error("Failed to load module %s from cache: %s", module_name, str(e))
                return
        if state["loaded"]:
            current_state = self._modules[module_name].get_state()
            # Avoid loading a module that is already loaded as module loading is not
            # always idempotent.
            if not current_state["loaded"]:
                self._modules[module_name].load(state["runlevel"])
                self.logger.info(
                    "Module %s loaded from cache with runlevel %s", module_name, state["runlevel"]
                )
            elif current_state["runlevel"] != state["runlevel"]:
                self._modules[module_name].set_runlevel(state["runlevel"])
                self.logger.info(
                    "Setting Module %s loaded from cache to runlevel %s",
                    module_name,
                    state["runlevel"],
                )
        # NOTE(diegobatt): Not all modules have set_state implemented
        try:
            self.logger.info("Module %s setting state from cache: %s", module_name, state)
            self._modules[module_name].set_state(state)
        except NotImplementedError:
            pass
    def load_module_states(self):
        """
        Loads every available module in the cache.
        """
        with self._cache_mutex:
            try:
                keys = list(self._cache.keys())
            except Exception:
                self.logger.exception("Failed to read cache keys from disk")
                return
        for module_name in keys:
            self.load_module_state(module_name)
    def clear_module_states(self):
        """
        Clears all modules from the cache.
        """
        self.logger.info("Clearing module states cache")
        with self._cache_mutex:
            try:
                self._cache.clear()
            except Exception:
                # If gracefully clearing fails, force it by opening a new empty db
                self._force_cache_refresh()
    def stop(self):
        """
        Gracefully stops manager.
        """
        self._cache.close()
````

## File: inorbit/agentlets/system.py
````python
# Copyright (c) 2018, InOrbit, Inc.
# All rights reserved.
# Agentlet that provides system information (CPU load, network stats, HDD usage)
import os
import subprocess
import threading
import time
import psutil
from util.overrides import overrides
from .agentlet import Agentlet
from .inorbit_pb2 import DiskUsageMessage
from .inorbit_pb2 import NetworkStatsMessage
from .inorbit_pb2 import SystemStatsMessage
from .meters import Meters
# Internal keys for the Meters object (any string works)
TOTAL_TX = "t_tx"
TOTAL_RX = "t_rx"
INORBIT_TX = "o_tx"
INORBIT_RX = "o_rx"
MQTT_TX = "m_tx"
MQTT_RX = "m_rx"
SLEEP_INTERVAL_S = 10
# If hdd source is undefined, compute usage from all the aggregated storage in
# the robot
HDD_SOURCE_DEFAULT = "/"
class SystemAgentlet(Agentlet):
    def __init__(self, uplink, diagnostics):
        super(SystemAgentlet, self).__init__(uplink)
        self._loop_thread = None
        self._run = False
        self._inorbit_path = os.getenv(
            "INORBIT_HOME", os.path.join(os.path.expanduser("~"), ".inorbit")
        )
        self._stats = None
        self._diagnostics = diagnostics
        self._detailed_net_stats = {}
        # Primary disk to report usage of
        self._primary_disk_partition = HDD_SOURCE_DEFAULT
        # Optional disk sources to report. This is a dictionary of partition
        # paths indexed by volume id.
        self._states["optional_disk_sources"] = {}
        # Optional network interfaces to monitor. This is a dictionary of
        # mount points indexed by interfaces ids.
        self._states["optional_network_interfaces"] = {}
        # List of disk partitions names
        self._states["available_hdd_partitions"] = self._list_disk_partitions()
        # List of network interfaces
        self._states["available_network_interfaces"] = self._list_network_interfaces()
        # Initialize the list of "networking" modules (those who transmit/receive data) with
        # the MQTT uplink, which is always metered
        self._networking_modules = [uplink]
    @overrides(Agentlet)
    def load(self, runlevel):
        self._run = True
        self._loop_thread = threading.Thread(target=self._loop_main)
        self._loop_thread.start()
        self._states["loaded"] = True
        self._load_states()
        # Send a state update to the cloud
        self.publish_state(self.uplink, self._states)
        return self._states["loaded"]
    def set_state(self, state):
        """
        Called whenever a set_module command is received.
        """
        if "optional_disk_sources" in state.keys():
            # Sanity checks. If the state received is not a dictionary,
            # skip it.
            if isinstance(state["optional_disk_sources"], dict):
                self._states["optional_disk_sources"] = state["optional_disk_sources"]
                # Restart exception reporting
                self.once_logger.reset_all()
        if "optional_network_interfaces" in state.keys():
            if isinstance(state["optional_network_interfaces"], dict):
                self._states["optional_network_interfaces"] = state["optional_network_interfaces"]
                self.once_logger.reset_all()
        # Send a state update to the cloud
        self.publish_state(self.uplink, self._states)
    def get_stats(self):
        return self._stats
    def _loop_main(self):
        """
        Main agentlet thread.
        Periodically reads and publishes system information.
        """
        self.logger.info("Starting system agentlet main loop.")
        # Initial (empty) Meters object to hold the metrics names
        # No timestamp, this object is not sent
        meters = Meters(names=self._get_meters_names())
        while self._run:
            metersNow = Meters(template=meters)
            # Read total CPU load average within a small interval
            # No interval=x in this call; CPU load since last call
            load_average = psutil.cpu_percent()
            # Get HDD usage (from all the aggregated storage in the robot)
            total_disk_usage = self._compute_disk_usage_for(HDD_SOURCE_DEFAULT)
            # Get the storage used by InOrbit
            inorbit_hdd_usage_mb, inorbit_hdd_usage_percentage = self._compute_inorbit_disk_usage(
                total_disk_usage
            )
            # Get network I/O statistics per interface
            net = self._get_detailed_net_stats()
            # Total network metrics
            total_net = self._get_total_net_stats_from(net)
            metersNow.set(TOTAL_TX, total_net[0])
            metersNow.set(TOTAL_RX, total_net[1])
            # InOrbit network metrics
            metersNow.set(INORBIT_TX, self.get_inorbit_tx_bytes())
            metersNow.set(INORBIT_RX, self.get_inorbit_rx_bytes())
            metersNow.set(MQTT_TX, self.uplink.get_tx_bytes())
            metersNow.set(MQTT_RX, self.uplink.get_rx_bytes())
            # Optional network interfaces metrics
            self._set_optional_net_meters_from(net, metersNow)
            # Get RAM usage - percentage in [0...1] range
            memory_usage_percent = self._get_ram_usage_percent()
            if not meters.is_empty():  # Report the delta - only done
                # ... after a second pass on this loop
                delta = metersNow.delta(meters)
                # Publish all stats (in a single message)\
                self._stats = SystemStatsMessage()
                self._stats.timestamp = delta.ts_as_ms()
                self._stats.elapsed_seconds = delta.duration()
                self._stats.cpu_load_percentage = load_average / 100
                if total_disk_usage:
                    self._stats.hdd_usage_percentage = total_disk_usage.percent / 100
                self._stats.inorbit_hdd_usage_mb = inorbit_hdd_usage_mb
                self._stats.inorbit_hdd_usage_percentage = inorbit_hdd_usage_percentage
                self._stats.optional_disks_data.extend(self._create_optional_disks_msg_array())
                self._stats.optional_network_interfaces_data.extend(
                    self._optional_net_interfaces_msg_from_delta(delta)
                )
                self._stats.total_tx = delta.get(TOTAL_TX)
                self._stats.total_rx = delta.get(TOTAL_RX)
                self._stats.inorbit_tx = delta.get(INORBIT_TX)
                self._stats.inorbit_rx = delta.get(INORBIT_RX)
                self._stats.mqtt_tx = delta.get(MQTT_TX)
                self._stats.mqtt_rx = delta.get(MQTT_RX)
                self._stats.ram_usage_percentage = memory_usage_percent
                self.uplink.publish_protobuf("system/stats", self._stats)
            # Save last timestamp and sleep for a while
            meters = metersNow
            time.sleep(SLEEP_INTERVAL_S)
        self.logger.info("Ending system agentlet main loop.")
    def _get_meters_names(self):
        """
        Returns the list of names used to build a meters template.
        """
        names = [TOTAL_TX, TOTAL_RX, INORBIT_TX, INORBIT_RX, MQTT_TX, MQTT_RX]
        for net_interface in self._states["available_network_interfaces"]:
            names.extend([f"{net_interface}_TX", f"{net_interface}_RX"])
        return names
    def _load_states(self):
        """
        Sets initial state values when loading the agentlet.
        """
        # No state loaded (battery handling removed)
        pass
    def _list_disk_partitions(self):
        """
        Returns a list with the mountpoints for disk partitions available.
        """
        return [partition.mountpoint for partition in psutil.disk_partitions()]
    def _compute_disk_usage_for(self, partition):
        """
        Returns disk usage statistics about the partition which contains the given
        path as a named tuple including total, used and free space expressed in
        bytes, plus the percentage usage.
        """
        # Get HDD usage from the source specified under states
        try:
            return psutil.disk_usage(partition)
        except Exception as e:
            self.once_logger.warn(
                f"disk_usage for {partition}",
                f"Failed to compute disk usage for partition: '{partition}'.",
            )
            return None
    def _compute_inorbit_disk_usage(self, primary_disk_data):
        """
        Returns inorbit's hdd usage in mb and percentage. This last value is
        calculated based on the total space of the primary disk.
        """
        # TODO (Flor_Grosso): Implement a proper locking mechanism to
        # avoid race conditions while computing hdd usage and deleting
        # data files under the .inorbit directory.
        inorbit_hdd_usage_mb = 0
        inorbit_hdd_usage_percentage = 0
        try:
            with open(os.devnull, "w") as devnull:
                inorbit_hdd_usage = float(
                    subprocess.check_output(
                        ["du", "-sb", self._inorbit_path], stderr=devnull
                    ).split()[0]
                )
            # Convert bytes to MB
            inorbit_hdd_usage_mb = inorbit_hdd_usage / (1024 * 1024)
            # Get inorbit disk usage as a percentage of disk's total
            if primary_disk_data and primary_disk_data.total > 0:
                inorbit_hdd_usage_percentage = inorbit_hdd_usage / primary_disk_data.total
        except Exception as e:
            self.once_logger.warn("inorbit_disk_usage", "Could not compute inorbit disk usage.")
        return inorbit_hdd_usage_mb, inorbit_hdd_usage_percentage
    def _create_optional_disks_msg_array(self):
        """
        Returns a list of DiskUsageMessage() elements, which contains information
        about optional disk partitions (id and usage percents).
        """
        optional_disk_data = []
        for volume_id in self._states["optional_disk_sources"]:
            volume_path = self._states["optional_disk_sources"][volume_id]
            disk_usage = self._compute_disk_usage_for(volume_path)
            if disk_usage:
                data = DiskUsageMessage()
                data.volume_id = volume_id
                # Don't send a message with empty data
                if not disk_usage.total:
                    continue
                data.usage_percentage = disk_usage.percent / 100
                optional_disk_data.append(data)
        return optional_disk_data
    def _list_network_interfaces(self):
        """
        Returns a list with the names of the network interfaces available.
        """
        return list(psutil.net_if_addrs().keys())
    def _get_detailed_net_stats(self):
        """
        Returns dictionary of complete network stats per available interface.
        """
        return psutil.net_io_counters(pernic=True)
    def _get_total_net_stats_from(self, net_stats):
        """
        Returns the total of bytes sent and received by all interfaces.
        """
        bytes_sent = 0
        bytes_recv = 0
        for interface in net_stats:
            # Get data traffic for all interfaces but localhost
            if interface != "lo":
                bytes_sent += net_stats[interface].bytes_sent
                bytes_recv += net_stats[interface].bytes_recv
        return (bytes_sent, bytes_recv)
    def _get_net_stats_for(self, detailed_stats, interface):
        """
        Returns the number of bytes sent and received by a specific interface
        """
        bytes_sent = 0
        bytes_recv = 0
        interface_stats = detailed_stats.get(interface)
        if interface_stats:
            bytes_sent = detailed_stats[interface].bytes_sent
            bytes_recv = detailed_stats[interface].bytes_recv
        return (bytes_sent, bytes_recv)
    def _set_optional_net_meters_from(self, detailed_net_stats, meters):
        """
        Sets meters values for optional network interfaces
        (bytes sent and bytes received per id) from the current net stats.
        """
        for net_interface in self._states["available_network_interfaces"]:
            (bytes_sent, bytes_recv) = self._get_net_stats_for(detailed_net_stats, net_interface)
            meters.set(f"{net_interface}_TX", bytes_sent)
            meters.set(f"{net_interface}_RX", bytes_recv)
    def _optional_net_interfaces_msg_from_delta(self, delta):
        """
        Returns a list of NetworkStatsMessage() elements, which contain information
        about optional network interfaces (id, bytes sent and bytes received).
        Numeric values are obtained from the deltas of the previous pass.
        """
        optional_network_interfaces_data = []
        for net_interface in self._states["optional_network_interfaces"]:
            mountpoint = self._states["optional_network_interfaces"][net_interface]
            try:
                bytes_sent = delta.get(f"{mountpoint}_TX")
                bytes_recv = delta.get(f"{mountpoint}_RX")
                if bytes_sent or bytes_recv:
                    data = NetworkStatsMessage()
                    data.interface_id = net_interface
                    data.tx = bytes_sent
                    data.rx = bytes_recv
                    optional_network_interfaces_data.append(data)
            except Exception as e:
                self.once_logger.warn(
                    f"net_stats for {mountpoint}",
                    "Attempt to get network stats for a non-available "
                    f"interface: '{mountpoint}'.",
                )
        return optional_network_interfaces_data
    def _get_ram_usage_percent(self):
        """
        Computes and returns RAM usage percent in a range from 0 to 1, based on
        total and available memory stats.
        """
        try:
            memory_stats = psutil.virtual_memory()
            used_memory = memory_stats.total - memory_stats.available
            return float(used_memory) / memory_stats.total
        except Exception as e:
            return 0
    def register_networking_module(self, module):
        """
        Registers a new module (normally, an Agentlet) as a module that uses and tracks use of
        networking: an util.NetworkingMixin object. It will be queried to report the total
        number of bytes transmitted and received by InOrbit agent.
        """
        self._networking_modules.append(module)
        # List the tracked modules in a module state, mostly for server-side debugging to know
        # which modules are already enabled (and tracked) by each agent.
        self._states["networking_modules"] = [
            m.__class__.__name__ for m in self._networking_modules
        ]
    def get_inorbit_tx_bytes(self):
        """
        Returns total number of bytes transmitted by InOrbit agent. This includes MQTT traffic
        and also traffic from other modules registered in register_networking_module()
        """
        return sum([m.get_tx_bytes() for m in self._networking_modules])
    def get_inorbit_rx_bytes(self):
        """
        Returns total number of bytes received by InOrbit agent. This includes MQTT traffic
        and also traffic from other modules registered in register_networking_module()
        """
        return sum([m.get_rx_bytes() for m in self._networking_modules])
````

## File: inorbit/agentlets/teleop.py
````python
# Copyright (c) 2020, InOrbit, Inc.
# All rights reserved.
# Teleop agentlet.
# Depends on the ROS module.
#
# TODO:
# - Serialize messages to cloud using protobuf
import threading
import time
from util.overrides import overrides
from .agentlet import Agentlet
from .inorbit_pb2 import TeleopGoCommand
from .ros import RosPublisher
MQTT_TELEOP_TOPIC_STEP = "ros/teleop/step"
MQTT_TELEOP_TOPIC_GO = "ros/teleop/go"
MQTT_STATE_TOPIC = "ros/teleop/state"
ROS_CMD_VEL_MSG_TYPE = "geometry_msgs/Twist"
ROS_CMD_VEL_TOPIC_DEFAULT = "cmd_vel"
# Direction values allowed
DIRECTION_UP = 0
DIRECTION_DOWN = 2
DIRECTION_LEFT = 1
DIRECTION_RIGHT = -1
# Default angular and linear velocities, in meters and radians per second
DEFAULT_LINEAR_VEL = 0.15
DEFAULT_ANGULAR_VEL = 0.3925  # 1/16th of a turn per second
# Maximum allowed values to be set
MAX_LINEAR_VEL = 0.5
MAX_ANGULAR_VEL = 0.785  # 1/8th of a turn per second
VEL_PUBLISH_RATE_HZ = 10
TELEOP_CMD_TIMEOUT_MS = 2000
MAX_PUBLICATIONS_PER_CMD = VEL_PUBLISH_RATE_HZ * TELEOP_CMD_TIMEOUT_MS / 1000
# Hack (Pisti) read _drive for how this variable is used
ZERO_VEL_THRESHOLD = 5
# Timeout for odometry data to be considered valid
ODOM_DATA_TIMEOUT_MS = 500
# Maximum linear distance traveled allowed per command sent
# (in meters)
MAX_LINEAR_DISTANCE_PER_CMD = 1.0
# Maximum angular distance allowed per command sent (in radians)
MAX_ANGULAR_DISTANCE_PER_CMD = 3.14
class RosTeleopAgentlet(Agentlet):
    def __init__(self, uplink, ros, odometry):
        super(RosTeleopAgentlet, self).__init__(uplink)
        self._ros = ros
        self._odom = odometry
        self._states["cmd_vel_topic"] = None
        self._states["angular_vel"] = DEFAULT_ANGULAR_VEL
        self._states["linear_vel"] = DEFAULT_LINEAR_VEL
        self._states["available_cmd_vel_topics"] = []
        self._states["publish_zero_vel"] = False
        self._states["zero_vel_threshold"] = ZERO_VEL_THRESHOLD
        self._direction = None
        self._publish_zero_vel = False
        # Access to this variable must be guaranteed by _mutex
        self._remaining_commands = 0
        # Access to this variable must be guaranteed by _mutex
        self._current_vel_command = None
        # Odometry data when the a step command is received
        self._initial_odometry = None
        # Mutex used to access _remaining_commands and _current_vel_command
        self._mutex = threading.Lock()
        # Publisher thread running state
        self._vel_publisher_running = False
        # ROS Publisher for velocity commands
        self._ros_cmd_vel_publisher = RosPublisher()
    @overrides(Agentlet)
    def load(self, runlevel):
        self._states["runlevel"] = runlevel
        try:
            import geometry_msgs
            import geometry_msgs.msg
            global geometry_msgs
        except Exception as e:
            self.once_logger.exception("geometry_msgs_load", "Exception loading geometry_msgs.")
            return False
        self._set_initial_teleop_topic()
        self._ros.add_submodule(
            "teleop",
            pubs=(
                (
                    self._states["cmd_vel_topic"],
                    geometry_msgs.msg.Twist,
                    self._ros_cmd_vel_publisher,
                ),
            ),
        )
        # Send a state update to the cloud
        self.publish_state(self.uplink, self._states)
        # Register for upstream incoming teleop STEP commands (step-by-step)
        self.uplink.add_listener(MQTT_TELEOP_TOPIC_STEP, self._set_vel)
        # Register for upstream incoming teleop GO commands (continuous teleop)
        self.uplink.add_listener(MQTT_TELEOP_TOPIC_GO, self._start_go)
        # Start velocity commands publishing thread
        threading.Thread(target=self._drive).start()
        self._states["loaded"] = True
        return True
    @overrides(Agentlet)
    def unload(self):
        # Shutdown publisher thread
        self._vel_publisher_running = False
        # Remove ROS subscribers
        self._ros.remove_submodule("teleop")
        self._states["loaded"] = False
        return True
    @overrides(Agentlet)
    def set_runlevel(self, runlevel):
        self._states["runlevel"] = runlevel
    def set_state(self, state):
        """
        Called whenever a set_module command is received.
        """
        if "cmd_vel_topic" in state.keys():
            if self._states["loaded"]:
                # Reset initial state
                new_pub = (
                    state["cmd_vel_topic"],
                    geometry_msgs.msg.Twist,
                    self._ros_cmd_vel_publisher,
                )
                self._ros.update_publisher_topic("teleop", self._states["cmd_vel_topic"], new_pub)
            self._states["cmd_vel_topic"] = state["cmd_vel_topic"]
        if "publish_zero_vel" in state.keys():
            self._states["publish_zero_vel"] = state["publish_zero_vel"]
        if "zero_vel_threshold" in state.keys():
            self._states["zero_vel_threshold"] = state["zero_vel_threshold"]
        self.set_float_state_value(state, "angular_vel", MAX_ANGULAR_VEL)
        self.set_float_state_value(state, "linear_vel", MAX_LINEAR_VEL)
        # Send a state update
        self.publish_state(self.uplink, self._states)
    def set_float_state_value(self, state, state_key, max_value):
        """
        Aux method to parse and set a float state and cap it to a max value.
        It will only set in case the value exists, can be parsed, is positive
        and less than the maximum.
        """
        if state_key in state.keys():
            value = state[state_key]
            try:
                float_value = float(value)
                if float_value < 0:
                    self.logger.warning(
                        f"Attempt to set state: {state_key} to negative " f"value: {float_value}."
                    )
                elif float_value > max_value:
                    self.logger.warning(
                        f"Attempt to set state: {state_key} to value: "
                        f"{float_value}, higher than the maximum allowed: "
                        f"{max_value}."
                    )
                else:
                    self._states[state_key] = float_value
            except Exception as e:
                self.logger.exception(f"Exception parsing state: {state_key}, value: {value}.")
    def _set_vel(self, payload):
        """
        Called from the server.
        Receives velocity step commands.
        """
        if self._ros_cmd_vel_publisher.pub is None:
            self.logger.warning("ROS publisher not set. Aborting.")
            return
        # Parse payload
        [seq, ts_hint, direction] = payload.split(b"|")
        # Estimated ts in milliseconds of the user's perception
        self._direction = int(direction)
        if not self.is_time_expired(int(ts_hint), TELEOP_CMD_TIMEOUT_MS):
            # Create Twist message
            twist_msg = self._create_empty_twist_msg()
            if self._direction == DIRECTION_UP:
                twist_msg.linear.x = self._states["linear_vel"]
            elif self._direction == DIRECTION_DOWN:
                twist_msg.linear.x = -1 * self._states["linear_vel"]
            elif self._direction == DIRECTION_LEFT:
                twist_msg.angular.z = self._states["angular_vel"]
            elif self._direction == DIRECTION_RIGHT:
                twist_msg.angular.z = -1 * self._states["angular_vel"]
            self._initial_odometry = self._odom.get_odometry()
            self._mutex.acquire()
            self._current_vel_command = twist_msg
            self._remaining_commands = MAX_PUBLICATIONS_PER_CMD
            self._mutex.release()
    def _start_go(self, payload):
        """
        Called from the server.
        Receives velocity GO commands with velocity information
        """
        if self._ros_cmd_vel_publisher.pub is None:
            self.logger.warning("ROS publisher not set. Aborting.")
            return
        # Parse payload
        try:
            message = TeleopGoCommand()
            message.ParseFromString(payload)
            self.linear_vel_command = float(message.linear_velocity)
            self.angular_vel_command = float(message.angular_velocity)
        except Exception as e:
            self.logger.warning("Failed to parse TeleopGoCommand")
            return
        # Estimated ts in millisecs of the user's perception
        if not self.is_time_expired(message.ts_hint, TELEOP_CMD_TIMEOUT_MS):
            # Create Twist message
            twist_msg = self._create_empty_twist_msg()
            twist_msg.linear.x = self.linear_vel_command * self._states["linear_vel"]
            twist_msg.angular.z = self.angular_vel_command * self._states["angular_vel"]
            self._initial_odometry = self._odom.get_odometry()
            self._mutex.acquire()
            self._current_vel_command = twist_msg
            self._remaining_commands = 1
            self._mutex.release()
    def _create_empty_twist_msg(self):
        twist_msg = geometry_msgs.msg.Twist()
        twist_msg.linear.x = 0.0
        twist_msg.linear.y = 0.0
        twist_msg.linear.z = 0.0
        twist_msg.angular.x = 0.0
        twist_msg.angular.y = 0.0
        twist_msg.angular.z = 0.0
        return twist_msg
    def _drive(self):
        """
        Runs on a separate thread. Publishes velocity commands at a fixed rate.
        """
        self._vel_publisher_running = True
        _zero_vel_count = 0
        while self._vel_publisher_running:
            self._mutex.acquire()
            local_command = self._current_vel_command
            local_counter = self._remaining_commands
            send_zero_vel = self._publish_zero_vel
            self._mutex.release()
            if self._states["publish_zero_vel"] is True:
                # HACK (Pisti) There are scenarios in which a robot might not
                # automatically stop moving when we stop publishing velocity commands
                # due to the robot missing a check for velocity command publishing
                # this results in robots being "stuck" with the last velocity message
                # The fix below attempts to prevent the robot being "stuck" with the latest
                # teleop command by publishing a (0,0,0) velocity command
                if local_counter == 0 and _zero_vel_count < self._states["zero_vel_threshold"]:
                    _zero_vel_count += 1
                elif _zero_vel_count == self._states["zero_vel_threshold"] and send_zero_vel:
                    self._publish_vel(self._create_empty_twist_msg())
                    self._publish_zero_vel = self._set_zero_vel_flag(False)
            if local_counter > 0:
                if self._should_halt_movement():
                    self._mutex.acquire()
                    self._remaining_commands = 0
                    self._mutex.release()
                    self.logger.info("Halting current teleoperation.")
                else:
                    try:
                        self._publish_vel(local_command)
                        _zero_vel_count = 0
                        self._mutex.acquire()
                        # Do not decrease remaining commands if a new command
                        # has arrived
                        if local_counter == self._remaining_commands:
                            self._remaining_commands -= 1
                        if self._remaining_commands <= 0:
                            # Change flag if state allows it
                            self._publish_zero_vel = self._set_zero_vel_flag(True)
                        self._mutex.release()
                    except Exception as e:
                        self.logger.exception("Exception publishing data.")
            time.sleep(1.0 / VEL_PUBLISH_RATE_HZ)
        self.logger.info("Publisher thread shutting down.")
    def _publish_vel(self, msg):
        """
        Publishes the latest velocity command if possible.
        """
        self._ros_cmd_vel_publisher.pub.publish(msg)
    def _set_zero_vel_flag(self, bool):
        """
        Shorthand function for checking a state before setting a value.
        """
        if self._states["publish_zero_vel"] is False:
            return False
        else:
            return bool
    def _should_halt_movement(self):
        """
        Checks if the movement should be halted based on odometry feedback.
        """
        # Get initial and last odometry, and check that they both have
        # a valid value
        initial_odometry = self._initial_odometry
        last_odometry = self._odom.get_odometry()
        # Don't allow teleoperations if odometry data is not valid
        if not self._odom.is_odometry_valid(initial_odometry) and not self._odom.is_odometry_valid(
            last_odometry
        ):
            self.logger.warning("Odometry data is not valid.")
            return True
        # Don't allow teleoperations if last odometry data is outdated
        if self._is_odometry_outdated(last_odometry):
            self.logger.warning("Odometry data is outdated.")
            return True
        # If odometry readings are valid, then check if the distance
        # traveled exceeds limits.
        if self._distance_traveled_exceeds_max(initial_odometry, last_odometry):
            self.logger.warning("Robot movement is not as planned.")
            return True
        return False
    def _is_odometry_outdated(self, odom_data):
        """
        Checks if the data received from the odometry module is outdated.
        """
        ts = odom_data[1]
        current_time = self.get_ts()
        # follow the same pattern with a this.newCostmap flag which is updated in
        # componentWillUpdate
        return current_time - ts > ODOM_DATA_TIMEOUT_MS
    def _distance_traveled_exceeds_max(self, initial_odometry, last_odometry):
        """
        Checks if the distance traveled by the robot has exceeded either
        the linear or angular limits.
        """
        # Odometry is a four element array composed of
        # [ts_start, ts, linear_distance, angular_distance]
        initial_linear_distance = initial_odometry[2]
        initial_angular_distance = initial_odometry[3]
        last_linear_distance = last_odometry[2]
        last_angular_distance = last_odometry[3]
        # Check if traversed distances exceed limits
        linear_distance_over_limit = (
            last_linear_distance - initial_linear_distance > MAX_LINEAR_DISTANCE_PER_CMD
        )
        angular_distance_over_limit = (
            last_angular_distance - initial_angular_distance > MAX_ANGULAR_DISTANCE_PER_CMD
        )
        return linear_distance_over_limit or angular_distance_over_limit
    def _get_available_cmd_vel_topics(self):
        """
        Finds topics publishing ROS_CMD_VEL_MSG_TYPE, populate and sort
        available_cmd_vel_topics list.
        """
        cmd_vel_topics = self._ros.get_topics_to_publish(ROS_CMD_VEL_MSG_TYPE)
        # If cmd vel topics were found, then sort the list leaving "cmd_vel"
        # topic above.
        if cmd_vel_topics:
            cmd_vel_topics = sorted(
                cmd_vel_topics, key=lambda x: (x != ROS_CMD_VEL_TOPIC_DEFAULT, x)
            )
        return cmd_vel_topics
    def _set_initial_teleop_topic(self):
        """
        Sets the initial teleop topic for when the agentlet is first loaded.
        """
        self._states["available_cmd_vel_topics"] = self._get_available_cmd_vel_topics()
        if not self._states["cmd_vel_topic"]:
            if not self._states["available_cmd_vel_topics"]:
                self.logger.warning(
                    "No ROS cmd topics available. "
                    f"Setting '{ROS_CMD_VEL_TOPIC_DEFAULT}' as default."
                )
                self._states["cmd_vel_topic"] = ROS_CMD_VEL_TOPIC_DEFAULT
            else:
                # If there are cmd topics, set the first on the list as the
                # current one.
                self._states["cmd_vel_topic"] = self._states["available_cmd_vel_topics"][0]
````

## File: inorbit/msg/__init__.py
````python

````

## File: inorbit/msg/_InOrbitOut.py
````python
# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from inorbit/InOrbitOut.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct
class InOrbitOut(genpy.Message):
  _md5sum = "9ebbdd1a417b231c022fa114239492b6"
  _type = "inorbit/InOrbitOut"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """string topic
uint8[] data
bool sent
"""
  __slots__ = ['topic','data','sent']
  _slot_types = ['string','uint8[]','bool']
  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.
    The available fields are:
       topic,data,sent
    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(InOrbitOut, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.topic is None:
        self.topic = ''
      if self.data is None:
        self.data = b''
      if self.sent is None:
        self.sent = False
    else:
      self.topic = ''
      self.data = b''
      self.sent = False
  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types
  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      _x = self.topic
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self.data
      length = len(_x)
      # - if encoded as a list instead, serialize as bytes instead of string
      if type(_x) in [list, tuple]:
        buff.write(struct.Struct('<I%sB'%length).pack(length, *_x))
      else:
        buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self.sent
      buff.write(_get_struct_B().pack(_x))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))
  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.topic = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.topic = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      self.data = str[start:end]
      start = end
      end += 1
      (self.sent,) = _get_struct_B().unpack(str[start:end])
      self.sent = bool(self.sent)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill
  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self.topic
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self.data
      length = len(_x)
      # - if encoded as a list instead, serialize as bytes instead of string
      if type(_x) in [list, tuple]:
        buff.write(struct.Struct('<I%sB'%length).pack(length, *_x))
      else:
        buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self.sent
      buff.write(_get_struct_B().pack(_x))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))
  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.topic = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.topic = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      self.data = str[start:end]
      start = end
      end += 1
      (self.sent,) = _get_struct_B().unpack(str[start:end])
      self.sent = bool(self.sent)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill
_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_B = None
def _get_struct_B():
    global _struct_B
    if _struct_B is None:
        _struct_B = struct.Struct("<B")
    return _struct_B
````

## File: inorbit/rossetup/__init__.py
````python

````

## File: inorbit/rossetup/ros.py
````python
# Copyright (c) 2021, InOrbit, Inc.
# All rights reserved.
#
# Various utility functions to handle high level ROS setup.
import os
import sys
# Flag to mark that ROS autodetection already ran.
ros_autodetect_ran = False
# Keep this list updated with the ROS 2 versions supported.
SUPPORTED_ROS_VERSIONS = ["foxy", "humble", "iron", "jazzy"]
ros_to_python_version = {
    "foxy": "python3.8",
    "humble": "python3.10",
    "iron": "python3.10",
    "jazzy": "python3.12",
}
def ros_autodetect():
    """
    Autodetects and configures the environment to use the existing ROS
    installation, if any. This includes:
      - Patch python path to find ROS libraries.
      - Setup ROS master URI
      - Setup other ROS related environment variables
    This simple implementation just attempts to find ROS in various known
    places from /opt/ros or based on INORBIT_ROS_PATH environment variable.
    It could fail (add wrong paths) if multiple ROS installations exist, as
    in the case of ROS1 + ROS2; see note below.
    Used from ROS agentlet and artifacts (rosbags) utilities.
    """
    global ros_autodetect_ran
    if ros_autodetect_ran:
        return
    ros_autodetect_ran = True
    ros_version = os.environ.get("INORBIT_ROS")
    # NOTE(FlorGrosso): It is unlikely that we get here with an unknown ROS
    # version. The inorbit.py entry script already checks if this variable
    # is set and doesn't load any ROS-dependant module if not.
    if ros_version == "unknown":
        print(
            "INORBIT_ROS env var not set on local/agent.env.sh. "
            "Please set it to use the correct ROS 2 version."
        )
    ros_path = os.getenv("INORBIT_ROS_PATH", None)
    # If the ROS version was set and is one of the supported ones, add the paths to site
    # and dist packages using the correct ROS version + python version combination.
    # If it is not known, then use all the possible combination of ROS and python versions.
    # For any of these cases, use a custom ROS path if provided. Otherwise use the
    # default opt/ros/<version> one.
    if ros_version in ros_to_python_version:
        python_ver = ros_to_python_version[ros_version]
        libs_path = ros_path if ros_path is not None else "/opt/ros/{ros_version}"
        sys.path.append(f"{libs_path}/lib/{python_ver}/site-packages")
        sys.path.append(f"{libs_path}/local/lib/{python_ver}/dist-packages")
        print(sys.path)
    else:
        for ros_ver, python_ver in ros_to_python_version.items():
            libs_path = ros_path if ros_path is not None else "/opt/ros/{ros_ver}"
            sys.path.append(f"{libs_path}/lib/{python_ver}/site-packages")
            sys.path.append(f"{libs_path}/local/lib/{python_ver}/dist-packages")
    if "ROS_PACKAGE_PATH" not in os.environ:
        if ros_path is not None:
            os.environ["ROS_PACKAGE_PATH"] = f"{ros_path}/share"
        else:
            if ros_version in SUPPORTED_ROS_VERSIONS:
                os.environ["ROS_PACKAGE_PATH"] = ":".join(f"/opt/ros/{ros_version}/share")
            else:
                os.environ["ROS_PACKAGE_PATH"] = ":".join(
                    ["/opt/ros/foxy/share", "/opt/ros/humble/share", "/opt/ros/jazzy/share"]
                )
    if "ROS_MASTER_URI" not in os.environ:
        os.environ["ROS_MASTER_URI"] = "http://localhost:11311"
````

## File: inorbit/__init__.py
````python
# Copyright (c) 2018, InOrbit, Inc.
# All rights reserved.
from os import environ
from os.path import expanduser
INORBIT_HOME = environ.get("INORBIT_HOME", expanduser("~") + "/.inorbit")
INORBIT_LOG_PATH = INORBIT_HOME + "/local"
INORBIT_LOG_FILE = INORBIT_LOG_PATH + "/inorbit_agent.log"
INORBIT_MODULE_STATES_CACHE_FILE = INORBIT_HOME + "/local" + "/.module_states"
INORBIT_MQTT_BROKER_ADDRESS_FILE = INORBIT_LOG_PATH + "/.mqtt_broker_address"
INORBIT_AGENT_ROSBAGS_PATH = INORBIT_HOME + "/local/rosbags"
INORBIT_ACTIONS_PATH_DEFAULT = INORBIT_HOME + "/local/user_scripts"
INORBIT_CONNECTION_FILE_DEFAULT = INORBIT_LOG_PATH + "/.connection"
VERSION = "4.22.0.ros2"
VARIANT = "ros2"
````

## File: inorbit/blackbox.py
````python
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
````

## File: inorbit/link.py
````python
# Copyright (c) 2018, InOrbit, Inc.
# All rights reserved.
#
# NOTE(adamantivm) Always return from MQTT client callbacks as soon as possible
# to avoid concurrency problems and deadlocks
# @see https://inorbit.atlassian.net/browse/IO-6927
# NOTE(Flor_Grosso): all features related to the blackbox and rosbag recorder
# modules are currently disabled. Consider re-enabling them when adding
# support for ROS 2 bags.
import json
import os
import signal
import socket
import ssl
import sys
import threading
import time
import urllib
import inorbit.logger
import paho.mqtt.client as mqtt
import socks
from inorbit import INORBIT_CONNECTION_FILE_DEFAULT
from inorbit import INORBIT_MQTT_BROKER_ADDRESS_FILE
from inorbit import VERSION
from util.networking_mixin import NetworkingMixin
from util.once_logger import OnceLogger
from .agentlets.inorbit_pb2 import Echo
# from inorbit.blackbox import Recorder
class Link(NetworkingMixin):
    def __init__(self):
        NetworkingMixin.__init__(self)
        self.logger = inorbit.logger.getLog(__name__)
        self.once_logger = OnceLogger(self.logger)
        # TODO: re-enable _blackbox. Disabled until properly
        # implementing support for data-fillback on ROS2
        # self._blackbox = Recorder()
        # None before it was even attempted to connect
        self._connected = None
        # Callbacks registered by other modules to be executed on
        # connection/disconnection
        # <subscriber, callback>
        self._on_connect_callbacks = {}
        self._on_disconnect_callbacks = {}
        self._listeners = {}
        # Current QoS for subscriptions (used for reconnects/reloads)
        # The dict is indexed by mqtt subtopic (exactly like this.listeners)
        self._qos = {}
        self.meters = {}
        # List of topics using protobuf
        # TODO(Flor_Grosso): Consider including wildcards for families of
        # commands
        self._protobuf_topics = (
            "custom_command/script/command",
            "custom_command/ros",
            "ros/teleop/go",
            "ros/loc/mapreq",
            "ros/nav/goal_path",
        )
        # Read configuration from the environment
        if "INORBIT_KEY" not in os.environ:
            raise RuntimeError("Missing INORBIT_KEY environment variable")
        self.api_key = os.environ["INORBIT_KEY"]
        if "INORBIT_ID" not in os.environ:
            raise RuntimeError("Missing INORBIT_ID environment variable")
        self.robot_id = os.environ["INORBIT_ID"]
        if "INORBIT_URL" not in os.environ:
            raise RuntimeError("Missing INORBIT_URL environment variable")
        self.inorbit_url = os.environ["INORBIT_URL"]
        # Read optional proxy configuration from the environment
        # We use self.http_proxy == None to indicate if proxy configuration
        # should be used
        self.http_proxy = os.getenv("HTTP_PROXY")
        if self.http_proxy == "":
            self.http_proxy = None
        if self.http_proxy is not None:
            self.logger.info(
                "Found HTTP_PROXY environment configuration "
                f" = {self.http_proxy}. Will use WebSockets transport."
            )
        # Number of times to retry getting MQTT credentials from InOrbit app server
        self.config_retries = 25
        # If MQTT gets disconnected and can't reconnect after this many seconds, commit suicide
        # to force a restart and attempt to get new MQTT credentials.
        self.disconnect_reboot_seconds = 300  # 5 minutes
        # Allow overriding link configuration values with environment variables
        try:
            if "INORBIT_CONFIG_RETRIES" in os.environ:
                self.config_retries = int(os.environ["INORBIT_CONFIG_RETRIES"])
            if "INORBIT_DISCONNECT_REBOOT_SECONDS" in os.environ:
                self.disconnect_reboot_seconds = int(
                    os.environ["INORBIT_DISCONNECT_REBOOT_SECONDS"]
                )
        except Exception:
            self.logger.exception(
                "Exception reading environment variables for link settings. Using defaults."
            )
        # We need to defer the creation of the MQTT client object until we get the
        # config from the server and know if we will be using websockets or MQTT
        self.client = None
    def _create_client(self):
        """
        Creates a new MQTT client instance and assigns it to self.client.
        """
        # Proxy access requires using websockets protocol
        if self.http_proxy is not None:
            self.use_websocket = True
        if self.use_websocket:
            self.client = mqtt.Client(protocol=mqtt.MQTTv311, transport="websockets")
        else:
            self.client = mqtt.Client(protocol=mqtt.MQTTv311)
        if self.http_proxy is not None:
            # Configure proxy hostname and port
            parts = urllib.parse.urlsplit(self.http_proxy)
            proxy_hostname = parts.hostname
            proxy_port = parts.port
            self.client.proxy_set(
                proxy_type=socks.HTTP, proxy_addr=proxy_hostname, proxy_port=proxy_port
            )
        # Connect callbacks
        self.client.on_message = self.on_message
        self.client.on_connect = self.on_connect
        self.client.on_disconnect = self.on_disconnect
    def _configure(self):
        """
        Configure MQTT connection parameters by retrieving them from InOrbit
        """
        # TODO(adamantivm) Rethink this configuration mechanism.
        # It is extremely important
        # that it is robust. Consider using MQTT or another
        # self-reconnecting client instead
        # of this custom logic.
        # TODO(adamantivm) Move this to a proper module
        self.my_hostname = os.getenv("INORBIT_ROBOT_NAME")
        if not self.my_hostname:
            # If the INORBIT_ROBOT_NAME is not provided in the env file
            # it tries to detect the hostname
            try:
                self.my_hostname = socket.gethostname()
                self.logger.info("Got hostname: %s" % self.my_hostname)
            except Exception:
                self.my_hostname = None
        # TODO(adamantivm) Use constants for cache file name
        cache_filename = "%s/local/.cache" % os.getenv(
            "INORBIT_HOME", os.getenv("HOME") + "/.inorbit"
        )
        # Start with an empty dict an update each time with the latest
        # information available
        # NOTE: python's dict.update is "later always wins"
        config = {}
        # Read pre-provisioned credentials
        try:
            connection_file_name = os.environ.get(
                "INORBIT_CONNECTION_CONFIG_FILE", INORBIT_CONNECTION_FILE_DEFAULT
            )
            with open(connection_file_name, "r") as f:
                config.update(json.loads(f.read()))
            # If the username is available, store it as it might be used later
            # as a credential_id
            if "username" in config:
                self.username = config["username"]
            self.logger.info("Pre-provisioned configuration applied")
        except Exception:
            pass
        # Try to connect for at least 10 times to read the MQTT configuration
        server_config_ready = False
        for i in range(0, self.config_retries):
            try:
                proxy_msg = "" if self.http_proxy is None else " (using HTTP_PROXY)"
                self.logger.info(
                    "Attempt %d to fetch MQTT configuration from " "InOrbit.%s" % (i, proxy_msg)
                )
                data_params = {
                    "apiKey": self.api_key,
                    "robotId": self.robot_id,
                    "agentVersion": VERSION,
                    "hostname": self.my_hostname,
                }
                if self.credential_id:
                    data_params["credentialId"] = self.credential_id
                data = urllib.parse.urlencode(data_params).encode("utf-8")
                req = urllib.request.Request(url=f"{self.inorbit_url}mqtt_config", data=data)
                # NOTE(adamantivm) 10 seconds timeout per attempt to avoid
                # blocking forever
                res = urllib.request.urlopen(url=req, timeout=10).read()
                config.update(json.loads(res))
                server_config_ready = True
                # Cache values for next time
                try:
                    cache_file = open(cache_filename, "w")
                    cache_file.write(json.dumps(config))
                    cache_file.close()
                except Exception:
                    self.logger.exception("Exception writing configuration " "to cache.")
            except Exception:
                # If it fails, wait 5 seconds and retry
                self.logger.exception("Exception reading MQTT configuration " "from InOrbit.")
                time.sleep(5)
            if server_config_ready:
                self.logger.info("Got MQTT configuration from InOrbit.")
                break
        if not server_config_ready:
            # Try reading last cached value
            try:
                cache_file = open(cache_filename, "r")
                cache = cache_file.read()
                cache_file.close()
                config.update(json.loads(cache))
                self.logger.warn("Using MQTT configuration from cache.")
            except Exception:
                self.logger.exception("Exception reading MQTT configuration " "from cache.")
        # In order to proceed, we need at least hostname, port, username and
        # password
        if (
            "hostname" in config
            and "port" in config
            and "username" in config
            and "password" in config
        ):
            self.hostname = config["hostname"]
            self.port = config["port"]
            self.username = config["username"]
            self.password = config["password"]
            self.websocket_port = config.get("websocket_port", config["port"])
            self.use_websocket = config.get("use_websocket", False)
        else:
            raise Exception("FATAL ERROR: Couldn't get MQTT configuration.")
        # Save the broker's address as '<broker IP>:<port>' for use by the watchdog
        try:
            mqtt_boker_ip = socket.gethostbyname(config["hostname"])
            mqtt_broker_address = open(INORBIT_MQTT_BROKER_ADDRESS_FILE, "w")
            mqtt_broker_address.write(str(mqtt_boker_ip + ":" + str(config["port"])))
            mqtt_broker_address.close()
        except Exception:
            self.logger.exception(
                "Exception writing MQTT broker's IP address to mqtt_broker_address."
            )
        # Enable echo of incoming commands for debugging purposes
        self.enable_echo = True
    def add_listener(self, subtopic, callback, qos=0):
        """
        Adds a new callback to be called whenever a message is received
        in the given subtopic.
        Callbacks are in the form callback(payload).
        The qos argument is optional and determines Quality of Service level for
        the subscription.
        """
        self._listeners[subtopic] = callback
        self._qos[subtopic] = qos
        topic = f"r/{self.robot_id}/{subtopic}"
        if self.client is not None:
            self.client.subscribe(topic, qos=qos)
        self.add_tx_bytes(len(topic))
    def remove_listener(self, subtopic):
        """
        Removes an already registered callback for incoming messages
        from the cloud.
        """
        if subtopic not in self._listeners:
            self.logger.warn(f"Subtopic '{subtopic}' not registered.")
            return
        topic = f"r/{self.robot_id}/{subtopic}"
        del self._listeners[subtopic]
        del self._qos[subtopic]
        if self.client is not None:
            self.client.unsubscribe(topic)
        self.add_tx_bytes(len(topic))
    def start(self):
        # Start a disconnected-reboot timer
        self.reboot_timer_start()
        # Get the MQTT and AWS credentials configuration for this Robot ID and
        # API key
        self._configure()
        # Create client object
        self._create_client()
        self.client.username_pw_set(self.username, self.password)
        # Establish connection status
        # Taken from here: https://stackoverflow.com/a/19071979
        # TODO(adamantivm) Update connection state on clean disconnect
        will_topic = "r/%s/state" % self.robot_id
        will_payload = "0|%s" % self.api_key
        self.client.will_set(will_topic, will_payload, qos=1, retain=True)
        self._mqtt_will_size = self._estimate_mqtt_message_size(will_topic, will_payload)
        # Enable tls for mqtt, except only for local development
        if self.hostname != "localdev.com" and self.hostname != "localhost":
            self.client.tls_set(
                "/etc/ssl/certs/ca-certificates.crt", tls_version=ssl.PROTOCOL_TLSv1_2
            )
        # Choose websocket port if using websocket transport
        port = self.websocket_port if self.use_websocket else self.port
        # TODO(adamantivm) Make keepalive value configurable
        self.client.connect_async(self.hostname, port, keepalive=10)
        self.client.loop_start()
        # TODO(herchu) Absolutely arbitrary counters
        self.add_tx_bytes(100)
        self.add_rx_bytes(20)
        self.logger.info(
            "MQTT connection initiated. "
            "{}:{} ({})".format(self.hostname, port, "websocket" if self.use_websocket else "MQTT")
        )
    def stop(self):
        # # gracefully stop blackbox
        # self._blackbox.stop()
        pass
    def on_disconnect(self, client, userdata, rc):
        self.logger.info("Disconnected from MQTT.")
        self._connected = False
        # Execute all callbacks registered to the disconnection event
        for subscriber, callback in iter(self._on_disconnect_callbacks.items()):
            try:
                callback()
            except Exception as e:
                self.once_logger.exception(
                    "on_disconnect_callback",
                    "Callback from subscriber '%s' failed: %s" % (subscriber, str(e)),
                )
        self.reboot_timer_start()
    def on_connect(self, client, userdata, flags, rc):
        # Only assume that the robot is connected if return code is 0.
        # Other values are taken as Errors (check here:
        # http://docs.oasis-open.org/mqtt/mqtt/v3.1.1/os/mqtt-v3.1.1-os.html#_Toc398718035)
        # so connection process needs to be aborted.
        if rc == 0:
            self.logger.info("Connected to MQTT")
        else:
            self.logger.warning(f"Unable to connect. rc = {rc}.")
            return
        self._connected = True
        # Execute all callbacks registered to the connection event
        self.reboot_timer_stop()
        # Count bytes transmitted to connect:
        # Estimate the outgoing CONNECT message, from the last will
        # saved, and a shorter CONNACK response. If reconnects happen,
        # we will count the same will and CONNACK every time.
        self.add_tx_bytes(self._mqtt_will_size)
        self.add_rx_bytes(4)  # CONNACK
        # Send online status.
        self.send_online_status()
        # Re-register subscribers
        for subtopic in self._listeners:
            topic = f"r/{self.robot_id}/{subtopic}"
            qos = self._qos[subtopic]
            self.logger.info(f"Subscribing to {topic}.")
            self.client.subscribe(topic, qos=qos)
            self.add_tx_bytes(len(topic))
        # Execute callbacks on a separate thread
        threading.Thread(target=self._execute_callbacks).start()
    def _execute_callbacks(self):
        """
        Executes all callbacks registered to the connection event
        """
        for subscriber, callback in iter(self._on_connect_callbacks.items()):
            try:
                callback()
            except Exception as e:
                self.once_logger.exception(
                    "on_connect_callback",
                    "Callback from subscriber '%s' failed: %s" % (subscriber, str(e)),
                )
    def reboot_timer_start(self):
        """
        Starts a timer to reboot the agent if it hasn't been able to connect after
        self.disconnect_reboot_seconds seconds.
        """
        if not hasattr(self, "_reboot_timer"):
            self.logger.info("starting disconnection timer")
            self._reboot_timer = threading.Timer(self.disconnect_reboot_seconds, self.do_reboot)
            self._reboot_timer.start()
    def reboot_timer_stop(self):
        """
        Clears any pending reboot timers.
        """
        if hasattr(self, "_reboot_timer"):
            self.logger.info("stopping disconnection timer")
            try:
                self._reboot_timer.cancel()
                del self._reboot_timer
            except Exception:
                self.logger.exception("Exception cancelling reboot timer")
    def do_reboot(self):
        """
        Kills the agent to force a reboot.
        """
        self.logger.error("Reboot timer expired. Exiting to force restart.")
        # Kill process with a user-defined signal
        # NOTE(diegobatt): This user-defined signal is caught by inorbit.py
        os.kill(os.getpid(), signal.SIGUSR1)
    def send_online_status(self):
        """
        Sends online status message.
        """
        # Every time we connect to the service, send updated status,
        # including online bit
        if self.my_hostname is not None:
            status_message = "1|%s|%s|%s" % (self.api_key, VERSION, self.my_hostname)
        else:
            status_message = "1|%s|%s" % (self.api_key, VERSION)
        ret = self.publish("state", status_message, qos=1, retain=True)
        self.logger.info(f"Publishing online status. ret = {ret}.")
    def _send_echo(self, topic, payload):
        """
        Sends an Echo message back to the application server.
        """
        # NOTE: sending back incoming messages which use protobuf breaks.
        # TODO (Flor_Grosso): fix this and enable echo for those cases.
        topic_id = topic.split("/", 2)[2]
        if topic_id in self._protobuf_topics:
            return
        # TODO(adamantivm) Protect this with try/catch and once_logger
        echo = Echo()
        echo.time_stamp = int(time.time() * 1000)
        echo.topic = topic
        echo.string_payload = payload
        self.publish_protobuf("echo", echo)
    def on_message(self, client, userdata, msg):
        try:
            self._rx_bytes += self._estimate_mqtt_message_size(msg.topic, msg.payload)
            if self.enable_echo:
                self._send_echo(msg.topic, msg.payload)
            # Attempt to extract subtopic
            subtopic = msg.topic.split(f"r/{self.robot_id}/")[1]
            # Debug MQTT incoming messages
            self.logger.debug("Received MQTT message")
            self.logger.debug("Topic: " + str(msg.topic))
            # Log payload from incoming messages, except module states
            if subtopic != "modules/set_state":
                self.logger.debug("Payload: " + str(msg.payload))
            if subtopic in self._listeners:
                try:
                    self._listeners[subtopic](msg.payload)
                except Exception:
                    self.logger.exception("Exception on command callback.")
        except Exception as ex:
            self.once_logger.exception(
                "on_message",
                f"Caught an exception while executing on_message callback: {ex}",
            )
    def publish_protobuf(self, subtopic, msg, *args, **kwargs):
        """
        Helper method to publish a protobuf message in binary format
        """
        if msg is None:
            self.publish(subtopic, None, *args, **kwargs)
        elif hasattr(msg, "SerializeToString"):
            self.publish(subtopic, bytearray(msg.SerializeToString()), *args, **kwargs)
    def publish(self, subtopic, msg, *args, **kwargs):
        topic = "r/%s/%s" % (self.robot_id, subtopic)
        res = None
        if self.client is not None:
            estimated_size = self._estimate_mqtt_message_size(topic, msg)
            self.add_tx_bytes(estimated_size)
            res = self.client.publish(topic, msg, *args, **kwargs)
        # Dump this message to persistent local storage
        # TODO(herchu) decide WHICH messages or which frequency to record
        # NOTE(diegobatt) we use the connection status as proxy for the sent
        # flag
        sent = res is not None and res.rc == mqtt.MQTT_ERR_SUCCESS and self.connected
        # self._blackbox.mqtt_out_write(topic, msg, sent)
        return res
    def _compute_mqtt_int_size(self, value):
        """
        Returns the number of bytes to use to represent the int 'value' in 7-bit,
        variable byte integers as used in MQTT.
        """
        if value < 0:
            raise RuntimeError("MQTT variable length cannot encode negative numbers")
        ret = 1
        value = value >> 7
        while value > 0:
            ret += 1
            value = value >> 7
        return ret
    def _compute_topic_size(self, topic):
        """
        Returns the size of the byte array used to represent a topic. This is 2
        length bytes, plus UTF8 encoding.
        """
        return 2 + len(topic.encode("utf-8"))
    def _estimate_mqtt_message_size(self, topic, arg):
        """
        Estimate the transferred size of a message, structured like a PUBLISH.
        See http://public.dhe.ibm.com/software/dw/webservices/ws-mqtt/mqtt-v3r1.html#publish
        """
        remaining_size = self._compute_topic_size(topic)
        # The payload is normally a byte string, but we also call it
        # with ints or floats as they are accepted in Paho.
        # For conversions below, see
        # https://www.eclipse.org/paho/clients/python/docs/
        if arg is None:
            # "f not given, or set to None a zero length message
            # will be used."
            return 0
        if isinstance(arg, int) or isinstance(arg, float):
            # "assing an int or float will result in the payload being
            # converted to a string representing that number. If you wish to
            # send a true int/float, use struct.pack() to create the
            # payload you require"
            arg = str(arg)
        if isinstance(arg, str):
            remaining_size += len(arg)
        else:
            remaining_size += sys.getsizeof(arg)  # We don't really know how
            # other types are converted. Just do something with sys!
            # TODO(herchu) this will be better when this code gets
            # into the paho client
        # NOTE: If we were using Qos > 0, we should add a message ID field.
        # Knowing the "remaining length" (payload + topic + no-ID)
        # We can add the remaining length field (variable length)
        # and the fixed header altogether.
        # Fixed header + Remaining length + Actual topic+payload
        return 2 + self._compute_mqtt_int_size(remaining_size) + remaining_size
    def add_connection_listener(self, callback, on="connect", subscriber=None):
        """
        Register a callback to be executed either on connect or disconnect.
        Used by other modules as a way of subscribing to a change in connection
        status.
        NOTE(diegobatt): This implementation is rudimentary, if multiple listeners
        want to subscribe to the same topic-key tuple they will need to provide a
        subscriber value so they don't overwrite each other.
        """
        if on == "connect":
            self._on_connect_callbacks[subscriber] = callback
        elif on == "disconnect":
            self._on_disconnect_callbacks[subscriber] = callback
        else:
            self.logger.warning(
                "Callback can only be registered on either 'connect' " "or 'disconnect'"
            )
    def remove_connection_listener(self, on="connect", subscriber=None):
        """
        Unregister a callback subscribed with add_connection_listener.
        Used by other modules as the counterpart of add_connection_listener.
        """
        if on == "connect":
            self._on_connect_callbacks.pop(subscriber, None)
        elif on == "disconnect":
            self._on_disconnect_callbacks.pop(subscriber, None)
        else:
            self.logger.warning(
                "Callback can only be unregistered on either 'connect' " "or 'disconnect'"
            )
    # """
    # Returns total blackbox recorder
    # """
    # @property
    # def blackbox(self):
    #     return self._blackbox
    @property
    def connected(self):
        """
        Returns connection state with mqtt.
        """
        return self._connected
    @property
    def credential_id(self):
        """
        Returns the credential_id.
        For a first iteration, this is just the username (if configured), but in
        the future as we might support different types of credentials, this method
        could start to gain more complexity.
        """
        return self.username if hasattr(self, "username") else None
````

## File: inorbit/log_manager.py
````python
# Copyright (c) 2018, InOrbit, Inc.
# All rights reserved.
# Log Manager class.
import glob
import os
import threading
import time
from queue import Queue
from inorbit.agentlets.inorbit_pb2 import RobotFileData
from inorbit.agentlets.inorbit_pb2 import RobotFilesUpdateMessage
from inorbit.logger import getLog
from inorbit.logger import INORBIT_LOG_PATH
from util.artifacts import Artifact
from util.concurrency import Interval
MQTT_UPDATES_TOPIC = "logfiles_update"
LOG_FILE_TYPE = "log"
UPLOAD_FILE_RATE_HZ = 1.0
class LogManager(object):
    def __init__(self, uplink):
        self._link = uplink
        self.logger = getLog(__name__)
        self._logs = {}
        self._logs_mutex = threading.Lock()
        self._upload_requests_queue = Queue()
        self._uploader_thread = None
    def upload_log(self, file_name):
        """
        Adds an upload request to the queue, when an upload command is received.
        """
        if not self._uploader_thread:
            self._uploader_thread = Interval(self._upload_logs, 1.0 / UPLOAD_FILE_RATE_HZ).start()
        self._upload_requests_queue.put(file_name)
    def _create_update_msg(self):
        """
        Returns a RobotFilesUpdateMessage, made from the current logs.
        """
        files_update = RobotFilesUpdateMessage()
        files_messages = []
        with self._logs_mutex:
            for log in self._logs.values():
                file_msg = RobotFileData()
                file_msg.type = LOG_FILE_TYPE
                file_msg.name = log.name
                file_msg.stored_in_robot = log.in_robot
                file_msg.uploading_to_cloud = log.uploading
                file_msg.url = log.url
                file_msg.ts = log.ts
                file_msg.size = log.size_kb
                files_messages.append(file_msg)
        files_update.artifacts.extend(files_messages)
        files_update.ts = int(time.time() * 1000)
        return files_update
    def _publish_logs_update(self):
        """
        Publishes an update message using protobuf. The whole log list is sent
        back again.
        """
        self._link.publish_protobuf(MQTT_UPDATES_TOPIC, self._create_update_msg(), qos=1)
    def send_list_update(self):
        """
        Cleans up current file handlers and refills it with current data to publish
        it through protobuf.
        NOTE: this is specific for log files, due to the way we are handling UI
        updates.
        """
        self._sync_logs()
        self._publish_logs_update()
    def _sync_logs(self):
        """
        Returns a dictionary of robot log files, indexed by file name.
        """
        current_logs = glob.glob(os.path.join(INORBIT_LOG_PATH, "*.log*"))
        with self._logs_mutex:
            self._logs = {}
            for filepath in current_logs:
                filename = os.path.basename(filepath)
                self._logs[filename] = Artifact(filepath, compress=True)
    def _upload_logs(self):
        """
        Runs on a separate thread. Checks uploads requests buffer and uploads the
        logs if possible.
        """
        while self._upload_requests_queue.qsize() != 0:
            filename = self._upload_requests_queue.get()
            self.logger.info("Uploading %s to cloud.", filename)
            try:
                self._logs[filename].upload()
            except Exception as e:
                self.logger.exception("Could not upload %s to cloud.", filename)
                # Check that the file exists before updating its uploading status
                if filename in self._logs:
                    self._logs[filename].set_uploading(False)
            # Send update to cloud
            self._publish_logs_update()
````

## File: inorbit/logger.py
````python
#!/usr/bin/env python
#
# Copyright (c) 2018, InOrbit, Inc.
# All rights reserved.
#
# InOrbit logging functions
import logging.handlers
import os.path
from inorbit import INORBIT_HOME
INORBIT_LOG_PATH = os.path.join(INORBIT_HOME, "local")
INORBIT_LOG_FILENAME = "inorbit_agent.log"
INORBIT_LOG_FILE = os.path.join(INORBIT_LOG_PATH, INORBIT_LOG_FILENAME)
LOG_FILE_SIZE = 1000000  # 1 Mb
LOG_FILES_COUNT = 9  # Current file plus N older logs
# Set overall logging level to INFO
# Possible log levels
LOG_LEVELS = {
    "debug": logging.DEBUG,
    "info": logging.INFO,
    "warning": logging.WARNING,
    "error": logging.ERROR,
    "critical": logging.CRITICAL,
}
# Set overall logging level to INFO by default
level = "info"
# Check if the user has configured a different log level
if "INORBIT_LOG_LEVEL" in os.environ:
    try:
        level = (os.environ["INORBIT_LOG_LEVEL"]).lower()
    except Exception as e:
        print("Unable to set custom logging level")
logging.basicConfig(level=LOG_LEVELS.get(level, logging.INFO))
# create a file handler
handler = logging.handlers.RotatingFileHandler(
    INORBIT_LOG_FILE, "a", LOG_FILE_SIZE, LOG_FILES_COUNT
)
# ...and a logging format
formatter = logging.Formatter("%(asctime)s %(levelname)-7s %(name)s: %(message)s")
handler.setFormatter(formatter)
def getLog(name):
    """
    Builds (or gets) a logger for a given module.
    It adds the appropriate handlers to this logging is sent to InOrbit's log
    files.
    """
    # create a new logger
    logger = logging.getLogger(name)
    # Disable propagating to root logger's handler (console)
    logger.propagate = False
    # add the handlers to the new logger
    logger.addHandler(handler)
    logger.setLevel(LOG_LEVELS.get(level, logging.INFO))
    return logger
````

## File: ros_monitor/__init__.py
````python
# Copyright (c) 2018, InOrbit, Inc.
# All rights reserved.
````

## File: ros_monitor/nodes_monitor.py
````python
# Copyright (c) 2018, InOrbit, Inc.
# All rights reserved.
# NodeMonitor class. Keeps a list of rosnodes to monitor and creates
# a list of update messages with their ping response.
import threading
import inorbit.logger
from inorbit.agentlets.inorbit_pb2 import NodeMonitorMessage
from util.once_logger import OnceLogger
class NodeMonitor:
    def __init__(self, ros, nodes_to_monitor):
        self._ros = ros
        self.logger = inorbit.logger.getLog(self.__class__.__name__)
        # Logger for high rate exceptions
        self.once_logger = OnceLogger(self.logger)
        # Dictionary of actions to perform indexed by node name
        self._nodes_to_monitor = nodes_to_monitor
        # Mutex used to access _nodes_to_monitor
        self._mutex = threading.Lock()
    def update_nodes(self, new_nodes):
        """
        Updates nodes to monitor with the of new nodes received. This is a
        dictionary which contains a list of nodes to monitor (key: "list"), as well
        as a dictionary of actions to perform indexed by node name (key: "values")
        NOTE (Flor_Grosso): actions to perform are not configurable for now and the
        same one (default: ping) is used for all nodes.
        """
        with self._mutex:
            self._nodes_to_monitor = new_nodes
        # Reset exceptions tracking
        self.once_logger.reset_all()
    def make_update_msg(self):
        """
        Returns a list of NodeMonitorMessage, for the current nodes being
        monitored.
        """
        node_update_array = []
        # TODO (Flor_Grosso): Use only keys for a start. Consider checking
        # values and iterating among the actions to perform on each item.
        with self._mutex:
            nodes_to_monitor = self._nodes_to_monitor.keys()
        for node in nodes_to_monitor:
            item = NodeMonitorMessage()
            item.name = node
            try:
                item.pinged = self._ros.rosnode_ping(node)
                item.has_ping_response = True
                node_update_array.append(item)
            except Exception as e:
                self.once_logger.warn(node, f"Exception monitoring node '{node}'.")
                continue
        return node_update_array
    def list_available_nodes(self):
        """
        Returns a list of all the available node names.
        """
        return self._ros.get_node_names()
````

## File: ros_monitor/params_monitor.py
````python
# Copyright (c) 2018, InOrbit, Inc.
# All rights reserved.
# ParamMonitor class. Keeps a list of rosparams to monitor and creates
# a list of update messages with their current values.
import threading
import inorbit.logger
from inorbit.agentlets.inorbit_pb2 import ParamMonitorMessage
from util.once_logger import OnceLogger
class ParamMonitor:
    def __init__(self, ros, params_to_monitor):
        self._ros = ros
        self.logger = inorbit.logger.getLog(self.__class__.__name__)
        # Logger for high rate exceptions
        self.once_logger = OnceLogger(self.logger)
        # List of available ros params on parameter server.
        self._available_ros_params = []
        # Dictionary of actions to perform indexed by param name
        self._params_to_monitor = params_to_monitor
        # Mutex used to access _params_to_monitor
        self._mutex = threading.Lock()
    def update_params(self, new_params):
        """
        Updates params to monitor with the of new params received. This is a
        dictionary which contains a list of params to monitor (key: "list"), as
        well as a dictionary of actions to perform indexed by param name
        (key: "values").
        NOTE (Flor_Grosso): actions to perform are not configurable for now and the
        same one (default: return value) is used for all params.
        """
        with self._mutex:
            self._params_to_monitor = new_params
        # Reset exceptions tracking
        self.once_logger.reset_all()
    def make_update_msg(self):
        """
        Returns a list of ParamMonitorMessage, for the current params being
        monitored.
        """
        param_update_array = []
        # TODO (Flor_Grosso): Use only keys for a start. Consider checking
        # values and iterating among the actions to perform on each item.
        with self._mutex:
            params_to_monitor = self._params_to_monitor.keys()
        for param in params_to_monitor:
            item = ParamMonitorMessage()
            item.name = param
            try:
                value = self._ros.get_param(param)
                if value:
                    item.value = str(value)
                param_update_array.append(item)
            except Exception as e:
                self.once_logger.warn(param, f"Exception monitoring param '{param}'.")
                continue
        return param_update_array
    def list_available_params(self):
        """
        Returns a list of all the available param names.
        """
        return self._ros.get_param_names()
````

## File: ros_monitor/topics_monitor.py
````python
# Copyright (c) 2018, InOrbit, Inc.
# All rights reserved.
# TopicMonitor class. Keeps a list of rostopics to monitor, subscribes to them
# and performs operations to get requested parameters (rate).
import threading
import inorbit.logger
from inorbit.agentlets.inorbit_pb2 import TopicMonitorMessage
from util.once_logger import OnceLogger
class TopicMonitor:
    def __init__(self, ros, topics_to_monitor):
        self._ros = ros
        self.logger = inorbit.logger.getLog(self.__class__.__name__)
        # Logger for high rate exceptions
        self.once_logger = OnceLogger(self.logger)
        # Dictionary of topic info objects types indexed by topic names.
        self._topic_info = {}
        # Monitoring state
        self._monitoring = False
        # List of available topic names.
        self._available_ros_topics = []
        # Dictionary of actions to perform indexed by topic name
        self._topics_to_monitor = topics_to_monitor
        # Mutex used to access _topics_to_monitor
        self._mutex = threading.Lock()
        # Flag that indicates whether a dependencies load request has been
        # already issued.
        self._has_tried_to_load_dependencies = False
    def load_dependencies(self):
        """
        Tries to load topic info module and returns False if it fails.
        """
        # Avoid loading dependencies more than once
        if self._has_tried_to_load_dependencies:
            return "TopicInfo" in globals()
        # Set flag to true after first run
        self._has_tried_to_load_dependencies = True
        try:
            from util.topic_info import TopicInfo
            global TopicInfo
        except Exception as e:
            self.logger.warning(f"Unable to monitor topics: {e}")
            return False
        return True
    def start(self):
        """
        Starts monitoring processes over the list of topics to monitor.
        """
        for topic in self._topics_to_monitor:
            # Get the topic type and setup the TopicInfo.
            topic_type = self._ros.get_msg_type_for(topic)
            self._topic_info[topic] = TopicInfo(topic, topic_type=topic_type)
            self._topic_info[topic].start_monitoring()
        self._monitoring = True
    def stop(self):
        """
        Stops monitoring topics.
        """
        if self._monitoring:
            for topic in self._topics_to_monitor:
                self._topic_info[topic].stop_monitoring()
            self._monitoring = False
    def update_topics(self, new_topics):
        """
        Updates topics to monitor with the of new topics received. This is a
        dictionary which of actions to perform indexed by topic name.
        NOTE (Flor_Grosso): actions to perform are not configurable for now and the
        same one (default: rate) is used for all topics.
        NOTE 2 (Flor_Grosso): to accomplish this, monitoring processes are stopped
        before and then resumed later with the new list.
        TODO (Flor_Grosso): implement the logic to update topics, leaving those
        that will continue to be monitored unchanged, removing the deleted ones
        and adding the appended ones.
        """
        should_update = self._monitoring
        if should_update:
            self.stop()
        with self._mutex:
            self._topics_to_monitor = new_topics
        if should_update:
            self.start()
        # Reset exceptions tracking
        self.once_logger.reset_all()
    def make_update_msg(self):
        """
        Returns a list of TopicMonitorMessage, for the current topics being
        monitored.
        """
        topic_update_array = []
        # TODO (Flor_Grosso): Use only keys for a start. Consider checking
        # values and iterating among the actions to perform on each item.
        with self._mutex:
            topics_to_monitor = self._topics_to_monitor.keys()
        for topic in topics_to_monitor:
            item = TopicMonitorMessage()
            item.name = topic
            try:
                rate = self._get_topic_rate(topic)
                item.has_rate = True
                item.rate = int(round(rate * 1000))
                topic_update_array.append(item)
            except Exception as e:
                self.once_logger.warn(topic, f"Exception monitoring topic '{topic}'.")
                continue
        return topic_update_array
    def _get_topic_rate(self, topic):
        """
        Returns topic rate in Hz.
        """
        rate = self._topic_info.get(topic).get_hz()[0]
        return rate if rate is not None else 0
    def list_available_topics(self):
        """
        Returns a list of all the available topic names.
        """
        topics = self._ros.get_published_topics()
        if topics:
            return [item[0] for item in topics]
        return []
    def _create_topics_dict(self):
        """
        Returns a dictionary of topic types indexed by topic names.
        """
        available_ros_topics = self._ros.get_published_topics()
        topics_dict = {}
        if available_ros_topics:
            for topic in available_ros_topics:
                topics_dict[topic[0]] = topic[1]
        return topics_dict
````

## File: scripts/agent_watchdog.sh
````bash
#!/bin/bash
# Helper script that checks INORBIT's agent status and kills
# it if the socket is stuck in CLOSE_WAIT state for more than
# WAIT_INTERVAL seconds.
#
# Assumptions
# - This is run under the same user as the inorbit agent (inorbit user for Debian installs)
# - The following tools are present: netstat, lsof (optional)
LOOP_INTERVAL=1    # seconds
WAIT_INTERVAL=60   # seconds
MY_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
INORBIT_DIST="$(cd ${MY_DIR}/.. && pwd)"
INORBIT_HOME="$(cd ${INORBIT_DIST}/.. && pwd)"
# When installed as a deb package, the `dist` folder in `INORBIT_DIST`
# ends up being `/usr/local/inorbit` and `INORBIT_HOME` renders as
# `/usr/local`, making subsequent commands to fail. In that case,
# fallback to the HOME folder of the user running the start.sh script. 
if [ $INORBIT_HOME -ef "/usr/local" ]
then
  INORBIT_HOME="$HOME/.inorbit"
fi
echo "InOrbit agent watchdog starting"
date
while true; do
  # Try to get mqtt port number from .mqtt_broker_address file
  MQTT_BROKER_ADDRESS_FILENAME=${INORBIT_HOME}/local/.mqtt_broker_address
  read -r MQTT_BROKER_ADDRESS < ${MQTT_BROKER_ADDRESS_FILENAME}
  # Get the output of all TCP connections to mqtt port
  # and check if there is any instance in CLOSE_WAIT state
  if [[ ! -z ${MQTT_BROKER_ADDRESS} ]] && [[ $(netstat -tn | grep ${MQTT_BROKER_ADDRESS}) =~ "CLOSE_WAIT" ]]; then
    # Wait and try again to confirm the process continues to stay in CLOSE_WAIT state
    sleep ${WAIT_INTERVAL}
    if [[ $(netstat -tn | grep ${MQTT_BROKER_ADDRESS}) =~ "CLOSE_WAIT" ]]; then
      # At this point, we can conclude that the connection is stuck
      # for at least LOOP_INTERVAL
      # Print a status report for logging purposes
      echo "----------------------------------"
      echo "Agent's MQTT broker address: ${MQTT_BROKER_ADDRESS}"
      echo "Socket in CLOSE_WAIT status"
      # Record all commands for clarity
      set -x
      # Timestamp
      date
      # Agent process status
      ps aux | grep inorbit.py
      # Last 10 lines of InOrbit's agent log
      tail -n 10 ${INORBIT_HOME}/local/inorbit_agent.log
      # Number of CLOSE_WAIT connections
      netstat -nap | grep CLOSE_WAIT | wc -l
      echo "Killing InOrbit's agent to force a restart"
      # Get pid based on process status output for inorbit.py
      AGENT_PID=$(ps -ef | grep inorbit.py | grep -v grep | awk '{ print $2 }')
      # Alternative PID search method in case ps doesn't work
      # MQTT_PORT=$(awk -F_ '{print $2}' <<<"$MQTT_BROKER_ADDRESS")
      # AGENT_PID=$(lsof -i:$MQTT_PORT -t)
      kill -9 $AGENT_PID
      set +x
    fi
  fi
  sleep ${LOOP_INTERVAL}
done
````

## File: scripts/start.sh
````bash
#!/bin/sh
# InOrbit agent upstart script. It keeps the agent running and
# automatically performs updates between respawns.
# This is to be replaced by a proper upstart script.
# TODO(adamantivm) Sanity checks of all sorts:
# - That we are in the correct directory (or else go there)
# - That the virtualenv exists
# - That activating the virtualenv was successful
# - That the agent and requirements.txt (full and core) are there
# TODO(adamantivm) Parameterize properly:
# - Virtualenv dir, retry interval
# Calculate INORBIT_DIST based on the directory of the current script
MY_DIR="$(dirname $0)"
INORBIT_DIST="$(cd ${MY_DIR}/.. && pwd)"
INORBIT_HOME="$(cd ${INORBIT_DIST}/.. && pwd)"
# When installed as a deb package, the `dist` folder in `INORBIT_DIST`
# ends up being `/usr/local/inorbit` and `INORBIT_HOME` renders as
# `/usr/local`, making subsequent commands to fail. In that case,
# fallback to the HOME folder of the user running the start.sh script. 
if [ $INORBIT_HOME -ef "/usr/local" ]
then
  INORBIT_HOME="$HOME/.inorbit"
fi
INORBIT_ENV=${INORBIT_HOME}/local/agent.env.sh
INORBIT_LOG=${INORBIT_HOME}/local/inorbit.log
# Run the agent
run_agent()
{
  cd ${INORBIT_DIST}
  # Activate agent virtualenv
  . ./venv/bin/activate
  # HACK(adamantivm) Set-up LD_LIBRARY_PATH to make sure ROS libs are found by python
  # This should be replaced by a dynamic loading mechanism inside the ros.py agentlet
  SUPPORTED_ROS2_DISTROS="foxy humble iron jazzy"
  UBUNTU_CODE=$(lsb_release -cs)
  # At this point we could assume that the 'INORBIT_ROS' env variable was set at
  # installation time, but if the user installed the agent using Debian, for example,
  # it could not be available.
  if [ `echo ${SUPPORTED_ROS2_DISTROS} | grep -o ${INORBIT_ROS}` ]
  then
    export LD_LIBRARY_PATH=/opt/ros/${INORBIT_ROS}/lib:/opt/ros/${INORBIT_ROS}/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH
  else
    echo 2>&1
    echo "INORBIT_ROS env variable not set on local/agent.env.sh." 2>&1
    echo "Using default LD_LIBRARY_PATH=/opt/ros/jazzy/lib:/opt/ros/jazzy/lib/x86_64-linux-gnu:/opt/ros/iron/lib:/opt/ros/iron/lib/x86_64-linux-gnu:/opt/ros/humble/lib:/opt/ros/humble/lib/x86_64-linux-gnu:/opt/ros/foxy/lib:/opt/ros/foxy/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH" 2>&1
    export LD_LIBRARY_PATH=/opt/ros/jazzy/lib:/opt/ros/jazzy/lib/x86_64-linux-gnu:/opt/ros/iron/lib:/opt/ros/iron/lib/x86_64-linux-gnu:/opt/ros/humble/lib:/opt/ros/humble/lib/x86_64-linux-gnu:/opt/ros/foxy/lib:/opt/ros/foxy/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH
  fi
  # Agent watchdog
  if [ "${INORBIT_ENABLE_WATCHDOG}" = "yes" ]; then
    WATCHDOG_PID=`pidof -x "agent_watchdog.sh"`
    if [ -z $WATCHDOG_PID ]; then
      echo "Starting agent watchdog" >> ${INORBIT_LOG} 2>&1
      ./scripts/agent_watchdog.sh >> ${INORBIT_HOME}/local/agent_watchdog.log &
    fi
  fi
  # Only log stdout and stderr if the INORBIT_LOG_LEVEL is set to DEBUG, otherwise
  # send it to /dev/null to avoid potential spam from ROS and other lower level
  # libraries
  if [ "${INORBIT_LOG_LEVEL}" = "debug" ]; then
    INORBIT_STDOUT="${INORBIT_LOG}"
  else
    INORBIT_STDOUT="/dev/null"
  fi
  # Start the agent
  python3 -u inorbit.py >> ${INORBIT_STDOUT} 2>&1
}
# In case this is the very first time the agent is started and
# no configuration has been created yet, create local working
# directory now
mkdir -p "${INORBIT_HOME}/local"
# HACK: Very basic log rotation to avoid excesive log file size
# TODO(adamantivm) Implement proper log rotation and centralization
rm "${INORBIT_LOG}.1" > /dev/null 2>&1
mv "${INORBIT_LOG}" "${INORBIT_LOG}.1" > /dev/null 2>&1
# TODO(adamantivm) Unify this with the agent's python-side logging
# (@see inorbit/__init__.py)
AGENT_LOG="${INORBIT_HOME}/local/inorbit_agent.log"
rm "${AGENT_LOG}.1" > /dev/null 2>&1
mv "${AGENT_LOG}" "${AGENT_LOG}.1" > /dev/null 2>&1
# Proactively delete any leftover badly closed module state file, see IO-6890
rm "${INORBIT_HOME}/local/__db..module_states" > /dev/null 2>&1
echo "---------------------------------------" >> ${INORBIT_LOG} 2>&1
echo "InOrbit Agent startup." >> ${INORBIT_LOG} 2>&1
date >> ${INORBIT_LOG} 2>&1
echo "INORBIT_DIST=${INORBIT_DIST}" >> ${INORBIT_LOG} 2>&1
echo "---------------------------------------" >> ${INORBIT_LOG} 2>&1
# Load the environment variables
if [ ! -f ${INORBIT_ENV} ]
then
  echo "Agent configuration file missing. Will keep retrying." >> ${INORBIT_LOG} 2>&1
  while [ ! -f ${INORBIT_ENV} ]
  do
    sleep 5
  done
fi
. ${INORBIT_ENV}
echo "Current configuration:" >> ${INORBIT_LOG} 2>&1
cat ${INORBIT_ENV} >> ${INORBIT_LOG} 2>&1
grep VERSION ${INORBIT_DIST}/inorbit/__init__.py >> ${INORBIT_LOG} 2>&1
grep VARIANT ${INORBIT_DIST}/inorbit/__init__.py >> ${INORBIT_LOG} 2>&1
echo "---------------------------------------" >> ${INORBIT_LOG} 2>&1
run_agent
````

## File: scripts/uninstall.sh
````bash
#/bin/sh
echo
echo "Shutting down and completely removing inOrbit Agent from your system."
echo
# Get the location of the current script
# https://stackoverflow.com/questions/59895/get-the-source-directory-of-a-bash-script-from-within-the-script-itself
MY_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
INORBIT_DIST="$(cd ${MY_DIR}/.. && pwd)"
INORBIT_HOME="$(cd ${INORBIT_DIST}/.. && pwd)"
# Stop inorbit agent through the appropriate init system
if [ -e /etc/systemd/system/inorbit.service ]
then
  sudo systemctl stop inorbit
  sudo systemctl disable inorbit
  sudo rm /etc/systemd/system/inorbit.service
elif [ -e /etc/init/inorbit.conf ]
then
  sudo initctl stop inorbit
  sudo rm /etc/init/inorbit.conf
fi
echo "Deleting agent files: ${INORBIT_HOME}"
rm -r ${INORBIT_HOME}
````

## File: util/__init__.py
````python
# Copyright (c) 2018, InOrbit, Inc.
# All rights reserved.
````

## File: util/array_util.py
````python
# Copyright (c) 2022, InOrbit, Inc.
# All rights reserved.
# Array utility functions, including our own encodings and compression of arrays.
# TODO(herchu) Move the encoding used for laser ranges from
# LocalizationAgentlet#_encode_floating_point_list to this file.
# (This encoding is based on runs of equal values, unlike the DeltaInt encoding below.)
"""
The delta_int_* functions below implement our "DeltaInt" encoding:
Used to encode arrays of floating point numbers which are relatively close to each other
in a more integer-friendly form, with minimal precision loss.
This encoding allows serializing values in protobuf in a much more compact precision
than floats, as small integers (protobuf: signed ints, `sint32`) use fewer bits
(proportional to log2(x)), while floats always use 32 bits (or 64 for double precision).
The input numbers (floating point) are encoded in a "fixed point" representation,
first computing some deltas (as they are all assumed to come from nearer spatial
coordinates). Then a fixed exponent is shared across all deltas, and each of them
is represented as a mantissa. See delta_int_encode implementation for details.
This idea is extensible also for coordinates by simply encoding X's and Y'x separately;
for this the wrapper functions delta_int_encode_points and delta_int_decode_points
are implemented.
Note that protobuf messages are not part of these functions, but it is trivial
to serialize encoded values in any message with 3 fields of type
(float, repeated sint32, sint32).
NOTE: This module is also implemented in nodejs codebase: arrayUtils.js
"""
def delta_int_encode(values, max_bits=15, compute_stats=False):
    """
    Encode a list of floating point values in DeltaInt form (see this file's header).
    The return value is a 4-element tuple:
    - A float `anchor`
    - A list of integer `deltas` (as a difference from `anchor`), scaled up or down by a given
      power of 2: 2^p
    - An exponent `p` which determines how to scale `deltas` back.
      This exponent is set to be 0. The return value 0 is reserved to represent an empty
      input array (since protobuf cannot represent None or null values)
    - An object with some stats if compute_stats is True, or None otherwise.
      This object includes for now values avg_abs_error and avg_rel_error with the (averages)
      absolute and relative errors that would result from decoding the received values.
      More keys could be added to this object in the future.
    Argument max_bits controls how many significant bits are kept to represent each
    output integer (the deltas). The higher maxBits value, the more precision is obtained
    when decoding values. But also, the deltas will be larger numbers, meaning their
    protobuf encoding will occupy more bits (roughly: maxBits).
    :param values is a list or sequence of floating point numbers
    :param max_bits controls the magnitude of returned deltas (more bits: more precision
    in the decoded results; but also more space occupied in protobuf serialization)
    """
    # When the input is an empty array, skip all the math below
    if len(values) == 0:
        return (0, [], 0)
    # The anchor is chosen to generate small distances to all other values
    anchor = sum(values) / len(values)  # empty list case is handled above
    # Estimate the maximum magnitude to represent, as the difference between
    # the minimum and maximum deltas.
    magnitude = abs(max(values, key=lambda x: abs(x - anchor)) - anchor)
    # Find the exponent that brings values closest to 2^max_bits
    power = 1 << max_bits
    if magnitude <= 0:
        scale = 0
        exponent = 0
    elif power > magnitude:
        # Max value is less than 2^max_bits, so we will scale up numbers before casting to int
        exponent = 1
        while power > magnitude * (1 << exponent):
            exponent += 1
        scale = 1 << exponent
    else:
        # Max value is greater than 2^max_bits, so we will scale down numbers before casting to int
        exponent = -1
        while power < magnitude / (1 << -exponent):
            exponent -= 1
        scale = 1 / (1 << -exponent)
    # Finally scale up (or down) all deltas with multiplying or dividing by 2^abs(exponent)
    int_deltas = [int(round((x - anchor) * scale)) for x in values]
    if not compute_stats:
        return (anchor, int_deltas, exponent, None)
    else:
        # Compute and return some statistics in the last tuple element.
        # These calculations should not be enabled by default.
        # For any calculation below, always prefer NOT allocating memory (use iterators)
        avg_abs_error = 0
        avg_rel_error = 0
        for v, d in zip(values, int_deltas):
            # delta / scale + anchor is the decoding function (see delta_int_decode)
            decoded = d / scale + anchor
            avg_abs_error += abs(decoded - v)
            avg_rel_error += abs((decoded - v) / v)
        avg_abs_error /= len(values)
        avg_rel_error /= len(values)
        stats = {
            "avg_abs_error": avg_abs_error,
            "avg_rel_error": avg_rel_error,
        }
        return (anchor, int_deltas, exponent, stats)
def delta_int_decode(anchor, deltas, exponent, options=None):
    """
    Decodes a DeltaInt encoding (see above) given as a tuple back into an float array.
    Note that the three arguments are exactly the elements returned by delta_int_encode;
    there is normally no need to manipulate them explicitly.
    :param anchor A floating point number (normally the mean of the input numbers)
    :param deltas A list of integers being the differences between each value and the anchor;
    representing fixed-point numbers
    :param exponent An value to scale all deltas by 2^exponent (fixed point representation)
    """
    # Determine scaling used and re-build deltas with scaling and adding back
    # the anchor value to become points.
    # This up- or down-scales values depending if exponent is greater or less than 0
    scale = 1 << exponent if exponent >= 0 else 1 / (1 << -exponent)
    values = [d / scale + anchor for d in deltas]
    return values
def delta_int_encode_points(points, max_bits=10, compute_stats=False):
    """
    Encodes a list of points { x, y } using DeltaInt
    and returns the 2 encodings as a pair (each element is a DeltaInt tuple, see above).
    :param points is a list or sequence of points (x, y)
    :param max_bits controls the magnitude of returned deltas (more bits: more precision
    in the decoded results; but also more space occupied in protobuf serialization)
    """
    xs = [p.x for p in points]
    ys = [p.y for p in points]
    return (
        delta_int_encode(xs, max_bits, compute_stats),
        delta_int_encode(ys, max_bits, compute_stats),
    )
def delta_int_decode_points(encoded_xs, encoded_ys):
    """
    Decodes two DeltaInt encodings (see above) of the same length into a list of pairs (x, y).
    :param encoded_xs An encoding of all X coordinates
    :param encoded_ys An encoding of all Y coordinates
    """
    xs = delta_int_decode(*encoded_xs)
    ys = delta_int_decode(*encoded_ys)
    return list(zip(xs, ys))
````

## File: util/artifacts.py
````python
# Copyright (c) 2021, InOrbit, Inc.
# All rights reserved.
# Utility classes representing arbitrary artifacts in the robot.
# An artifact is a file in the robot that we might want to persist at some
# point in time. So an Artifact class is mainly a File system object
# abstraction with uploading capabilities and extra metadata.
# Special kind of Artifacts are Databags (associated with a time frame and
# with extra properties comming from metadata) and Rosbags (Databags that are
# recorded listening to the robot ros topics)
import json
import os
import signal
import subprocess
import sys
import tarfile
import threading
import time
import inorbit.logger
import yaml
from inorbit.rossetup.ros import ros_autodetect
from util.s3_upload_helper import S3_Client
from util.concurrency import Interval
# Some functions from this module (rosbags recording) depend on 'rosbag'
# module from ROS. Try a soft-import
try:
    ros_autodetect()
    import rosbag
except ImportError:
    pass  # Error is ignored. Rosbag class functions can report if they fail
# Using kilobyte = 1024 since it is prevalent in filesystem terminology
KB = 1024
MS = 1000
class Artifact(object):
    """
    Class to handle arbitrary artifacts.
    An Artifact represents an arbitrary file than can be uploaded to the cloud
    to be persisted by InOrbit
    """
    def __init__(self, path, compress=False):
        self.logger = inorbit.logger.getLog(__name__)
        # TODO(herchu) Do not read this information from a file, but receive it
        # from a higher level module instead.
        try:
            self._s3config = self._read_aws_credentials()
        except Exception as e:
            self._s3config = None
            self.logger.warning("Couldn't obtain AWS credentials: %s", str(e))
        # Mutex used to protect uploading/deleting tasks
        self._mutex = threading.Lock()
        # Split the path of the artifact into folder and name
        self._dir, self._name = os.path.split(path)
        self._path = path
        # If the artifact exists in the filesystem, it is in_robot
        self._in_robot = os.path.isfile(path)
        self._uploading = False
        self._compress = compress
        self._url = ""
        # Use try-except with filesystem operations
        try:
            self._ts = int(os.path.getmtime(path) * MS) if self._in_robot else 0
        except Exception:
            self._ts = 0
        try:
            self._size_kb = int(os.path.getsize(path) / KB) if self._in_robot else 0
        except Exception:
            self._size_kb = 0
    def upload(self, subpath=None, callback=None):
        """
        Uploads the artifact to cloud.
        The target location is defined by the S3 config, which contains a company ID as part of the
        bucket name (and credentials provisioned with permissions on this bucket). A sub-path
        within this bucket can be given as optional argument. Finally, the object name will
        be the same as this artifact name in the filesystem
        The (optional) callback argument gets called periodically with number of bytes transferred
        since last call. It must be a function that receives a single numeric argument (bytes). See:
        https://boto3.amazonaws.com/v1/documentation/api/latest/reference/services/s3.html#S3.Bucket.upload_fileobj
        """
        if self._s3config is None:
            raise Exception("Couldn't upload %s because S3 is not configured" % self._path)
        s3_uploader = S3_Client(self._s3config)
        # TODO (Flor_Grosso): check whether the file should be compressed or
        # not, based on the mission status/ resource usage.
        with self._mutex:
            path = self.compress() if self._compress else self._path
            self._url = s3_uploader.upload_file(path, subpath, callback=callback)
        # Delete compressed (temporary) file
        try:
            if self._compress:
                os.remove(path)
        except Exception:
            raise Exception("Could not delete file: %s" % path)
        self._uploading = False
    def delete_from_robot(self):
        """
        Deletes the artifact from the robot.
        TODO (Flor_Grosso): Consider extending the locking to prevent race
        conditions while computing hdd usage in the SystemAgentlet
        TODO(herchu) This method ought to have some validations! Otherwise
        anyone hacking the mqtt channel somehow and sending a command to delete
        a not-rosbag might delete anything in the robot!
        """
        try:
            with self._mutex:
                os.remove(self._path)
            # Mark file as not stored in robot
            self._in_robot = False
        except Exception:
            raise Exception("Could not delete file: %s." % self._path)
    def compress(self):
        """
        Creates a tar file on the same working directory.
        TODO (Flor_Grosso): consider creating this file on a temporary directory,
        rather than using /local.
        """
        compressed_path = self._path + ".tar.gz"
        with tarfile.open(compressed_path, "w:gz") as tar:
            tar.add(self._path, arcname=self._name)
        return compressed_path
    def _read_aws_credentials(self):
        """
        Reads the cached aws credentials file.
        """
        inorbit_home = os.getenv("INORBIT_HOME", os.path.join(os.getenv("HOME"), ".inorbit"))
        aws_cache_filename = os.path.join(inorbit_home, "local", ".cache")
        try:
            aws_file = open(aws_cache_filename, "r")
            cache = aws_file.read()
            keys = json.loads(cache)
            return keys["awsUploadCredentials"]
        except Exception as e:
            raise e
    @property
    def path(self):
        return self._path
    @property
    def name(self):
        return self._name
    @property
    def ts(self):
        return self._ts
    @property
    def in_robot(self):
        return self._in_robot
    @property
    def uploading(self):
        return self._uploading
    @property
    def url(self):
        return self._url
    @property
    def size_kb(self):
        return self._size_kb
    def set_uploading(self, uploading):
        self._uploading = uploading
    def set_in_robot(self, in_robot):
        self._in_robot = in_robot
class Databag(Artifact):
    """
    Class to handle databag artifacts.
    A Databag represents an specific kind of artifact. It contains information
    for a given period of time (between start_ts and end_ts) and could be
    enriched with a properties yaml file
    """
    def __init__(self, path, compress=False):
        super(Databag, self).__init__(path, compress)
        # Default start_ts to last modification time
        try:
            self._start_ts = int(os.path.getmtime(path) * MS) if self._in_robot else 0
        except Exception:
            self._start_ts = 0
        self._end_ts = 0
        self._properties = {}
        self.refresh_properties()
    @property
    def start_ts(self):
        return self._start_ts
    @property
    def end_ts(self):
        return self._end_ts
    @property
    def properties(self):
        return self._properties
    def refresh_properties(self):
        """
        Load databag properties from an associated YAML file. Returns true if properties
        have changed since the last refresh.
        """
        # Replace the file path with a yml extension for the properties
        # HACK: mimics os.path.splitext but works with composed extensions
        # Such as .tar.xz. Will fail if the filename has dots besides extension
        properties_path = os.path.join(self._dir, self._name.split(".")[0] + ".yml")
        prev_properties = self._properties
        if os.path.isfile(properties_path):
            try:
                with open(properties_path, "r") as f:
                    self._properties = yaml.safe_load(f)
            except Exception as e:
                self.logger.warning(
                    "Couldn't parse properties yaml file %s: %s",
                    properties_path,
                    str(e),
                )
        return self._properties != prev_properties
class Rosbag(Databag):
    """
    Class to handle rosbag artifacts.
    A Rosbag is a special kind of databag that is recorded by the robot and has
    all the messages that went through a given set of topics for the recording
    period
    """
    def __init__(self, path, compress=False):
        super(Rosbag, self).__init__(path, compress)
        self._topics = []
        self._subprocess = None
        # Recording state
        self._recording = False
        self._recording_halted = False
        # Recording state
        self._bag_writer = None
        # Thread to stop 'ros2 bag record' subprocess after duration
        self._check_rosbag_recording_time_thread = None
        # Read rosbag metadata file, if present
        self.rosbag_metadata = self.read_metadata_file()
        # Calculate end_ts with the duration from rosbag metadata file
        self._end_ts = self._start_ts + self.get_rosbag_duration_from_metadata()
    def __str__(self) -> str:
        return f"_name: {self._name}, _path: {self._path}, _in_robot: {self._in_robot}"
    def __repr__(self) -> str:
        return self.__str__()
    def start_writer(self):
        if "rosbag" not in sys.modules:  # Depends on rosbag module from ROS
            return False
        try:
            with self._mutex:
                self._bag_writer = rosbag.Bag(self._path, "w")
        except Exception:
            return False
        return True
    def stop_writer(self):
        with self._mutex:
            if self._bag_writer:
                self._bag_writer.close()
                self._bag_writer = None
            else:
                print("error, calling write() without open rosbag")
    def add_topic(self, topic, msg_type):
        # Placeholder. In "rosbags" library we need to create a topic with its
        # message type def; but it is not necessary in ROS "rosbag" library.
        pass
    def read_metadata_file(self):
        """
        Reads ROS2 bag metadata.yaml file.
        Returns dictionary with yaml data if the file exists. If not,
        it returns an empty dict object '{}'
        """
        metadata_path = os.path.join(self._dir, "metadata.yaml")
        if os.path.exists(metadata_path):
            with open(metadata_path, "r") as f:
                return yaml.safe_load(f)
        return {}
    def get_rosbag_duration_from_metadata(self):
        """
        Reads ROS2 bag duration in nanoseconds from rosbag metadata.
        Returns rosbag duration in miliseconds. If there's no metadata
        or there's no duration it returns 0.
        """
        rosbag2_bagfile_information = self.rosbag_metadata.get("rosbag2_bagfile_information", {})
        duration_ns =  rosbag2_bagfile_information.get("duration", {}).get("nanoseconds", 0)
        return int(duration_ns / 1_000_000)
    def write_message(self, topic, ros_message, ts=None):
        if ts:
            # With the "rosbags" library we can tune the timestamp of the ROS
            # message, which we could ideally set to the original MQTT's
            # message timestamp. That is not possible in ROS' "rosbag" library,
            # so raise an error for now until we resolve it.
            raise Exception("User-defined timestamps are not yet implemented")
        with self._mutex:
            if self._bag_writer:
                self._bag_writer.write(topic, ros_message)
            else:
                print("error, calling write() without open rosbag")
    def _check_rosbag_recording_time(self, duration):
        """
        Monitors 'ros2 bag record' subprocess elapsed time and halts
        recording when duration is exceeded.
        It runs periodically when the recording starts, and stopped when
        the recording is halted.
        """
        if self._subprocess:
            elapsed = int(time.time() * MS) - self.start_ts
            if elapsed > duration:
                self.halt_recording()
    def record(self, topics, duration):
        """
        Performs a rosbag record process with the options given as arguments
        and the path defined in the constructor. Waits for completion and checks
        output.
        """
        # TODO check that master is running
        # TODO(adamantivm) Take this path from the ros agentlet instead
        if "INORBIT_ROS_PATH" in os.environ:
            source_ros_env_command = ". $INORBIT_ROS_PATH/setup.sh"
        else:
            source_ros_env_command = ". /opt/ros/$INORBIT_ROS/setup.sh"
        # NOTE: the ros2 bag recording command interface changes from ros1 to ros2.
        # With ros1 the 'rosbag record' command ends when the recording reaches
        # certain duration (specified with the "--duration" parameter).
        # 'ros2 bag record' doesn't allow setting a duration or timecap. Instead,
        # it only allows setting the maximum size in bytes before the bagfile will
        # be split (see "--max-bag-size MAX_BAG_SIZE" flag). The command doesn't
        # stop after reaching said size but it creates multiple rosbags files and
        # split the recording into chunk ending with _0, _1, _2, ...
        #   - rosbag2_2023_08_15-13_39_42/rosbag2_2023_08_15-13_39_42_0.db3
        #   - rosbag2_2023_08_15-13_39_42/rosbag2_2023_08_15-13_39_42_1.db3
        #   - rosbag2_2023_08_15-13_39_42/rosbag2_2023_08_15-13_39_42_2.db3
        # To maintain consistency and to enable recording multiple bags overtime,
        # this implementation assumes 'ros2 bag record' only outputs a single bag
        # file, which is accomplished by NOT setting the MAX_BAG_SIZE parameter.
        # 
        # TODO(@lpineda.io): implement a mechanism to make `ros2 bag record` process
        # stop after X time (using record's method parameter 'duration').
        record_command = (
            f"ros2 bag record --output={self._dir} {' '.join(topics)}"
        )
        self.logger.debug(f"Rosbag: record command '{record_command}'")
        rosbag_record = f"{source_ros_env_command}; {record_command}"
        self._recording = True
        self._topics = topics
        self._start_ts = int(time.time() * MS)
        # Start a thread to monitor 'ros2 bag record' subprocess elapsed time
        # and to kill the subprocess if the elapsed time exceeded duration.
        self._check_rosbag_recording_time_thread = Interval(
            self._check_rosbag_recording_time, 1, duration * MS
        )
        self._check_rosbag_recording_time_thread.start()
        self.logger.debug(f"Subprocess for rosbag record: start_ts {self._start_ts}")
        with open(os.devnull, "w") as devnull:
            self._subprocess = subprocess.Popen(
                rosbag_record,
                shell=True,
                stderr=subprocess.PIPE,
                stdout=devnull,
                preexec_fn=os.setsid,
            )
        self._subprocess.wait()
        self._check_rosbag_recording_time_thread.stop()
        self._check_rosbag_recording_time_thread = None
        self._recording = False
        self._end_ts = int(time.time() * MS)
        self.logger.debug(f"Subprocess for rosbag record: end_ts {self._end_ts}")
        # Raise an exception if process didn't finish correctly, except for
        # those cases when recording is halted.
        if self._subprocess.returncode != 0 and not self._recording_halted:
            output, error = self._subprocess.communicate()
            self._subprocess = None
            raise Exception(error)
        self._subprocess = None
        # Mark bag as stored in robot
        self._in_robot = True
    def halt_recording(self):
        """
        Halts a rosbag recording.
        """
        self.logger.debug(f"Halting rosbag recording")
        if self.recording:
            self._recording_halted = True
            os.killpg(self._subprocess.pid, signal.SIGINT)
        else:
            self.logger.warning("Trying to halt a non-existent recording.")
        self.logger.debug(f"Halting rosbag recording: Done")
    @property
    def topics(self):
        return self._topics
    @property
    def recording(self):
        return self._recording
````

## File: util/concurrency.py
````python
# Copyright (c) 2021, InOrbit, Inc.
# All rights reserved.
# Utility classes for working with threads
import threading
class Interval(threading.Thread):
    def __init__(self, callback, seconds, *args, **kwargs):
        """
        New thread that runs a callback every given seconds.
        """
        self._callback = callback
        self.__stop = threading.Event()
        self._seconds = seconds
        super(Interval, self).__init__(target=self._run, args=args, kwargs=kwargs)
    def _run(self, *args, **kwargs):
        """
        Overwrites threading.Thread()._run method with an infinite loop
        of the callback function spaced by the amount of seconds passed to the
        constructor.
        """
        # Run the callback one time immediately
        self._callback(*args, **kwargs)
        # and every self._seconds after that
        while not self.__stop.wait(self._seconds):
            self._callback(*args, **kwargs)
    def start(self):
        """
        Overwrites threading.Thread().start method. The only difference is that
        this method also returns the instance.
        """
        super(Interval, self).start()
        return self
    def stop(self):
        """
        Stops the thread by setting the stop event and breaking the while loop
        in _run method.
        """
        self.__stop.set()
    def set_interval_seconds(self, interval_seconds):
        """
        Changes the interval at which this Interval thread ticks. It takes effect after the next
        callback call; it does not cancel the current wait()
        """
        self._seconds = interval_seconds
````

## File: util/CvBridgeCustom.py
````python
# Copyright (c) 2018, InOrbit, Inc.
# All rights reserved.
# The work in this file is based on cv_bridge:
# https://github.com/ros-perception/vision_opencv/tree/kinetic/cv_bridge
# Parts of the code were taken and modified. The original license
# is reproduced below.
# Software License Agreement (BSD License)
#
# Copyright (c) 2011, Willow Garage, Inc.
# Copyright (c) 2016, Tal Regev.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
import sys
import cv2
import numpy
CV_CN_SHIFT = 3
CV_CN_MAX = 512
# CV types
CV_8U = 0
CV_8S = 1
CV_16U = 2
CV_16S = 3
CV_32S = 4
CV_32F = 5
CV_64F = 6
class CvBridgeCustom:
    def __init__(self):
        self.cvdepth_to_numpy_depth = {
            cv2.CV_8U: "uint8",
            cv2.CV_8S: "int8",
            cv2.CV_16U: "uint16",
            cv2.CV_16S: "int16",
            cv2.CV_32S: "int32",
            cv2.CV_32F: "float32",
            cv2.CV_64F: "float64",
        }
        self._encoding_flags = {
            "mono8": "GRAY",
            "mono16": "GRAY",
            "bgr8": "BGR",
            "bgr16": "BGR",
            "rgb8": "RGB",
            "rgb16": "RGB",
            "bgra8": "BGRA",
            "bgra16": "BGRA",
            "yuv422": "YUV422",
            "bayer_rggb8": "BAYER_RGGB",
            "bayer_rggb16": "BAYER_RGGB",
            "bayer_bggr8": "BAYER_BGGR",
            "bayer_bggr16": "BAYER_BGGR",
            "bayer_gbrg8": "BAYER_GBRG",
            "bayer_gbrg16": "BAYER_GBRG",
            "bayer_grbg8": "BAYER_GRBG",
            "bayer_grbg16": "BAYER_GRBG",
        }
        self._color_conversion_codes = {
            "GRAY2RGB": cv2.COLOR_GRAY2RGB,
            "GRAY2BGR": cv2.COLOR_GRAY2BGR,
            "GRAY2RGBA": cv2.COLOR_GRAY2RGBA,
            "GRAY2BGRA": cv2.COLOR_GRAY2BGRA,
            "RGB2GRAY": cv2.COLOR_RGB2GRAY,
            "RGB2BGR": cv2.COLOR_RGB2BGR,
            "RGB2RGBA": cv2.COLOR_RGB2RGBA,
            "RGB2BGRA": cv2.COLOR_RGB2BGRA,
            "BGR2GRAY": cv2.COLOR_BGR2GRAY,
            "BGR2RGB": cv2.COLOR_BGR2RGB,
            "BGR2RGBA": cv2.COLOR_BGR2RGBA,
            "BGR2BGRA": cv2.COLOR_BGR2BGRA,
            "RGBA2GRAY": cv2.COLOR_RGBA2GRAY,
            "RGBA2RGB": cv2.COLOR_RGBA2RGB,
            "RGBA2BGR": cv2.COLOR_RGBA2BGR,
            "RGBA2BGRA": cv2.COLOR_RGBA2BGRA,
            "BGRA2GRAY": cv2.COLOR_BGRA2GRAY,
            "BGRA2RGB": cv2.COLOR_BGRA2RGB,
            "BGRA2BGR": cv2.COLOR_BGRA2BGR,
            "BGRA2RGBA": cv2.COLOR_BGRA2RGBA,
            "YUV2GRAY_UYVY": cv2.COLOR_YUV2GRAY_UYVY,
            "YUV2RGB_UYVY": cv2.COLOR_YUV2RGB_UYVY,
            "YUV2BGR_UYVY": cv2.COLOR_YUV2BGR_UYVY,
            "YUV2RGBA_UYVY": cv2.COLOR_YUV2RGBA_UYVY,
            "YUV2BGRA_UYVY": cv2.COLOR_YUV2BGRA_UYVY,
        }
    def imgmsg_to_cv2(self, img_msg, desired_encoding="passthrough"):
        """
        Converts a ros image message to an openCV format.
        If desired_encoding is "passthrough", then the returned image has the same
        format as img_msg. Otherwise desired_encoding must be one of the standard
        image encodings.
        """
        dtype, n_channels = self._encoding_to_dtype_with_channels(img_msg.encoding)
        dtype = numpy.dtype(dtype)
        dtype = dtype.newbyteorder(">" if img_msg.is_bigendian else "<")
        # Full row length in bytes
        step = img_msg.step
        if n_channels == 1:
            strides = (step, dtype.itemsize)
            im = numpy.ndarray(
                shape=(img_msg.height, img_msg.width),
                dtype=dtype,
                buffer=img_msg.data,
                strides=strides,
            )
        else:
            strides = (step, n_channels, dtype.itemsize)
            im = numpy.ndarray(
                shape=(img_msg.height, img_msg.width, n_channels),
                dtype=dtype,
                buffer=img_msg.data,
                strides=strides,
            )
        # If the byt order is different between the message and the system.
        if img_msg.is_bigendian == (sys.byteorder == "little"):
            im = im.byteswap().newbyteorder()
        # Add hack to avoid degrading colors when working with opencv
        if desired_encoding == "rgb8":
            desired_encoding = "bgr8"
        if desired_encoding == "passthrough" or img_msg.encoding == desired_encoding:
            return im
        try:
            img_encoding = img_msg.encoding
            # GRAYSCALE DEPTH IMAGES
            # Return these as they come from source, without attempting
            # to do any conversion to rgb/bgr encodings.
            # 8UC1 encoding can be considered equal to mono8 as stated here
            # https://github.com/ros-perception/vision_opencv/blob/melodic/cv_bridge/src/cv_bridge.cpp#L671
            if img_encoding == "8UC1":
                return im
            # Do scaling between CV_16U/mono16 [0,65535] and mono8 [0,255] images.
            if img_encoding == "16UC1":
                return numpy.uint8(im / 257)
            # If the data type is single precision float, convert it to
            # uint8 (single channel).
            if img_encoding == "32FC1":
                return numpy.uint8(im)
            conversion_flag = self._get_conversion_flag(img_encoding, desired_encoding)
            if conversion_flag == "SAME_FORMAT":
                image_converted = im
            else:
                image_converted = cv2.cvtColor(im, conversion_flag)
        except Exception as e:
            raise Exception(f"Could not encode {img_encoding} image to {desired_encoding}")
        return image_converted
    def compressed_imgmsg_to_cv2(self, cmprs_img_msg, desired_encoding="passthrough"):
        """
        Converts a ros compressed image message to an openCV format.
        If desired_encoding is "passthrough", then the returned image has the same
        format as img_msg. Otherwise desired_encoding must be one of the standard
        image encodings.
        """
        str_msg = cmprs_img_msg.data
        buf = numpy.ndarray(shape=(1, len(str_msg)), dtype=numpy.uint8, buffer=cmprs_img_msg.data)
        im = cv2.imdecode(buf, cv2.IMREAD_ANYCOLOR)
        # Assume the data encoding is rgb8, since we don't have information
        # on it.
        img_encoding = "rgb8"
        if desired_encoding == "passthrough" or img_encoding == desired_encoding:
            return im
        try:
            conversion_flag = self._get_conversion_flag(img_encoding, desired_encoding)
            if conversion_flag == "SAME_FORMAT":
                image_converted = im
            else:
                image_converted = cv2.cvtColor(im, conversion_flag)
        except Exception as e:
            raise Exception(f"Could not encode compressed image to {desired_encoding}")
        return image_converted
    def _encoding_to_dtype_with_channels(self, encoding):
        return self._cvtype2_to_dtype_with_channels(self._encoding_to_cvtype2(encoding))
    def _cvtype2_to_dtype_with_channels(self, cvtype):
        return self.cvdepth_to_numpy_depth[self._cv_mat_depth(cvtype)], self._cv_mat_cn(cvtype)
    def _encoding_to_cvtype2(self, encoding):
        encoding_to_type_params = {
            "bgr8": (CV_8U, 3),
            "mono8": (CV_8U, 1),
            "rgb8": (CV_8U, 3),
            "mono16": (CV_16U, 1),
            "bgr16": (CV_16U, 3),
            "rgb16": (CV_16U, 3),
            "bgra8": (CV_8U, 4),
            "rgba8": (CV_8U, 4),
            "bgra16": (CV_16U, 4),
            "rgba16": (CV_16U, 4),
            "bayer_rggb8": (CV_8U, 1),
            "bayer_bggr8": (CV_8U, 1),
            "bayer_gbrg8": (CV_8U, 1),
            "bayer_grbg8": (CV_8U, 1),
            "bayer_rggb16": (CV_16U, 1),
            "bayer_bggr16": (CV_16U, 1),
            "bayer_gbrg16": (CV_16U, 1),
            "bayer_grbg16": (CV_16U, 1),
            "yuv422": (CV_8U, 2),
            "8UC1": (CV_8U, 1),
            "8UC2": (CV_8U, 2),
            "8UC3": (CV_8U, 3),
            "8UC4": (CV_8U, 4),
            "8SC1": (CV_8S, 1),
            "8SC2": (CV_8S, 2),
            "8SC3": (CV_8S, 3),
            "8SC4": (CV_8S, 4),
            "16UC1": (CV_16U, 1),
            "16UC2": (CV_16U, 2),
            "16UC3": (CV_16U, 3),
            "16UC4": (CV_16U, 4),
            "16SC1": (CV_16S, 1),
            "16SC2": (CV_16S, 2),
            "16SC3": (CV_16S, 3),
            "16SC4": (CV_16S, 4),
            "32FC1": (CV_32F, 1),
            "32FC2": (CV_32F, 2),
            "32FC3": (CV_32F, 3),
            "32FC4": (CV_32F, 4),
            "32SC1": (CV_32S, 1),
            "32SC2": (CV_32S, 2),
            "32SC3": (CV_32S, 3),
            "32SC4": (CV_32S, 4),
            "64FC1": (CV_64F, 1),
            "64FC2": (CV_64F, 2),
            "64FC3": (CV_64F, 3),
            "64FC4": (CV_64F, 4),
        }
        params = encoding_to_type_params.get(encoding)
        if params is not None:
            return self._cv_make_type(*params)
        else:
            raise Exception("Unrecognized image encoding [" + encoding + "]")
    def _cv_make_type(self, depth, cn):
        return self._cv_mat_depth(depth) + (((cn) - 1) << CV_CN_SHIFT)
    def _cv_mat_depth(self, flags):
        return (flags) & ((1 << CV_CN_SHIFT) - 1)
    def _cv_mat_cn(self, flags):
        return (((flags) & ((CV_CN_MAX - 1) << CV_CN_SHIFT)) >> CV_CN_SHIFT) + 1
    def _get_conversion_flag(self, source_encoding, dest_encoding):
        """
        Returns a flag used to convert an image from one cv2 encoding format to
        another. If the source or destiny encodings are not supported it returns
        None.
        """
        source_fmt = self._encoding_flags.get(source_encoding)
        dest_fmt = self._encoding_flags.get(dest_encoding)
        if source_fmt is None or dest_fmt is None:
            return None
        if source_fmt == dest_fmt:
            return "SAME_FORMAT"
        common_fmts = ["GRAY", "RGB", "BGR"]
        common_fmts_with_opacity = ["RGBA", "BGRA"]
        if (
            source_fmt in common_fmts
            or source_fmt in common_fmts_with_opacity
            and dest_fmt in common_fmts
            or dest_fmt in common_fmts_with_opacity
        ):
            return self._color_conversion_codes[f"{source_fmt}2{dest_fmt}"]
        elif (
            source_fmt == "YUV422"
            and dest_fmt in common_fmts
            or dest_fmt in common_fmts_with_opacity
        ):
            return self._color_conversion_codes[f"YUV2{dest_fmt}_UYVY"]
        elif "BAYER" in source_fmt:
            try:
                special_conversion_codes = {
                    "BayerBG2GRAY": cv2.COLOR_BayerBG2GRAY,
                    "BayerBG2RGB": cv2.COLOR_BayerBG2RGB,
                    "BayerBG2BGR": cv2.COLOR_BayerBG2BGR,
                    "BayerRG2GRAY": cv2.COLOR_BayerRG2GRAY,
                    "BayerRG2RGB": cv2.COLOR_BayerRG2RGB,
                    "BayerRG2BGR": cv2.COLOR_BayerRG2BGR,
                    "BayerGR2GRAY": cv2.COLOR_BayerGR2GRAY,
                    "BayerGR2RGB": cv2.COLOR_BayerGR2RGB,
                    "BayerGR2BGR": cv2.COLOR_BayerGR2BGR,
                    "BayerGB2GRAY": cv2.COLOR_BayerGB2GRAY,
                    "BayerGB2RGB": cv2.COLOR_BayerGB2RGB,
                    "BayerGB2BGR": cv2.COLOR_BayerGB2BGR,
                }
            except Exception as e:
                raise Exception("Current OpenCV distribution does not support " "Bayer format.")
            if source_fmt == "BAYER_RGGB" and dest_fmt in common_fmts:
                return special_conversion_codes[f"BayerBG2{dest_fmt}"]
            elif source_fmt == "BAYER_BGGR" and dest_fmt in common_fmts:
                return special_conversion_codes[f"BayerRG2{dest_fmt}"]
            elif source_fmt == "BAYER_GBRG" and dest_fmt in common_fmts:
                return special_conversion_codes[f"BayerGR2{dest_fmt}"]
            elif source_fmt == "BAYER_GRBG" and dest_fmt in common_fmts:
                return special_conversion_codes[f"BayerGB2{dest_fmt}"]
        else:
            return None
````

## File: util/Image.py
````python
# Copyright (c) 2018, InOrbit, Inc.
# All rights reserved.
# Image processing wrapper.
# Uses cv2 if available, if not uses Pillow (PIL fork).
import imghdr
import numpy
from util.overrides import overrides
# Package for image processing used (cv2 or PIL)
PACKAGE = None
# Checking OpenCv availability
try:
    import cv2
except Exception as e:
    pass
else:
    from util.CvBridgeCustom import CvBridgeCustom
    PACKAGE = "cv2"
# If not available, use PIL
if not PACKAGE:
    try:
        from PIL import Image as Im
        from PIL import ImageDraw
        from PIL import ImageEnhance
    except Exception as e:
        pass
    else:
        import io
        import sys
        PACKAGE = "PIL"
supported_img_encodings = ["jpg", "jpeg", "png"]
"""
This is the class to be used to process images. The interface is defined in
ImageBase class.
If OpenCv is available, cv2ImageWrapper is used as Image class. If not,
PILImageWrapper is used as image class.
If none of them is available, an ImageError exception is raised.
"""
Image = None
class ImageError(Exception):
    """
    This is the error raised by :class:`cv_bridge.CvBridge` methods when
    failing.
    """
    pass
class ImageBase:
    """
    Pure virtual class used to define the Image class interface
    """
    # Indicates the package being used
    _PACKAGE = None
    # Default parameters
    DEFAULT_CONTRAST = None
    DEFAULT_BRIGHTNESS = None
    def __init__(self):
        # Object containing the opened image.
        self._im = None
        # Memory buffer with the image encoded to jpeg.
        self._buf = None
        # Source (original) encoding of the image.
        self._source_format = None
    def open(self, image_path):
        """
        Opens a image from image_path, and saves the object in _im.
        Raises an IOError if the image can't be opened.
        """
        raise NotImplementedError()
    def resize(self, target_size):
        """
        Resizes the image, keeping its aspect ratio.
        target_size: Targeted (width, height).
        TODO(ivanpauno): Actually when using cv2, we are expanding images if they
                        are smaller than the specified size, and when using PIL
                        we aren't.
        """
        raise NotImplementedError()
    def encode(self, skip_conversion, quality):
        """
        Saves an internal buffer with the image encoded to jpg by default (if
        skip_conversion flag is false) or to the source format (skip_conversion
        is True), with a given compression quality.
        quality:
            - JPG/JPEG: compression quality of the image (0 --> 100 -highest-).
            - PNG: compression level (0 --> 9 -not compressed-)
        """
        raise NotImplementedError()
    def toBytes(self):
        """
        Returns saved memory buffer _buf as bytes.
        """
        raise NotImplementedError()
    def fromImgMsg(self, img_msg, output_encoding):
        """
        Opens an image from a ROS message. Also loaded in _im.
        img_msg: ROS message of type sensor_msgs/Image
        output_encoding: the encoding to convert the image to.
        """
        raise NotImplementedError()
    def fromCompressedImgMsg(self, img_msg, output_encoding):
        """
        Opens an image from a ROS message (COMPRESSED). Also loaded in _im.
        img_msg: ROS message of type sensor_msgs/CompressedImage
        output_encoding: the encoding to convert the image to.
        TODO (Flor_Grosso): implement this method for PILImageWrapper.
        """
        raise NotImplementedError()
    def fillPolygons(self, base_img, contours, default_color):
        """
        It receives an image to use as background, a list of objects (contours)
        with 'data' (polygon's vertices) and 'cost' values to fill the polygons
        with, and a default color to be used in case no fill color is
        provided.
        With this input, it draws and fills the polygons over the base image and
        returns the resulting one.
        """
        raise NotImplementedError()
    def isFormatSupported(self, fmt):
        """
        Checks if an image format is supported by the Image module.
        """
        return fmt in supported_img_encodings
    def enhance(self, contrast, brightness):
        """
        Adjusts contrast and brightness of the image.
        """
        raise NotImplementedError()
class PILImageWrapper(ImageBase):
    """
    This class allows simple image processing with PIL.
    """
    _PACKAGE = "PIL"
    DEFAULT_CONTRAST = 1.0
    DEFAULT_BRIGHTNESS = 1.0
    @overrides(ImageBase)
    def open(self, image_path):
        self._im = Im.open(image_path)
        self._source_format = self._im.format
        return self._source_format.lower()
    @overrides(ImageBase)
    def resize(self, target_size):
        # NOTE (Flor_Grosso): this PIL's method already takes the aspect ratio
        # preservation into account, so there's no need to handle that
        # externally.
        # For reference, check:
        # http://effbot.org/imagingbook/image.htm#tag-Image.Image.thumbnail
        # TODO (FlorGrosso): check whether the image needs to be resized
        # or if it has already the expected size, avoiding unnecessary
        # processing.
        self._im.thumbnail(target_size)
    @overrides(ImageBase)
    def encode(self, skip_conversion, quality):
        if self._im is None:
            return
        # If it is not a JPEG supported encoding, convert the encoding to RGB.
        # PIL doesn't convert it automaticaly when saving to JPEG.
        if self._im.mode not in ("L", "RGB", "CMYK"):
            self._im = self._im.convert("RGB")
        # If the no conversion flag is active, preserve source encoding.
        # If not, convert to jpg by default.
        if skip_conversion and self._source_format:
            out_format = self._source_format
        else:
            out_format = "JPEG"
        # Check if the output encoding is supported by this module
        if not out_format or out_format.lower() not in supported_img_encodings:
            return None
        with io.BytesIO() as output:
            if out_format == "JPEG" or out_format == "JPG":
                self._im.save(output, format=out_format, quality=quality)
            elif out_format == "PNG":
                self._im.save(output, format=out_format, compress_level=quality)
            self._buf = output.getvalue()
    @overrides(ImageBase)
    def toBytes(self):
        if self._buf is None:
            return None
        return self._buf
    @overrides(ImageBase)
    def fromImgMsg(self, img_msg, output_encoding):
        """
        This code was adapted from CvBridgeCustom.imgmsg_to_cv2 method
        (check ./CvBridgeCustom.py) in order to work with PIL.
        """
        # Map encoding from sensor_msgs/Image to
        # (numpy.dtype, channels, PILmode).
        # TODO(ivanpauno): Add support to other encoding types.
        encoding_to_dtype_Nchannels_PILmode = {
            "bgr8": ("uint8", 3, "RGB"),
            "mono8": ("uint8", 1, "L"),
            "rgb8": ("uint8", 3, "RGB"),
            "mono16": ("uint16", 1, "L"),
            "bgr16": ("uint16", 3, "RGB"),
            "rgb16": ("uint16", 3, "RGB"),
            "bgra8": None,
            "rgba8": ("uint8", 4, "RGBA"),
            "bgra16": None,
            "rgba16": None,
            "bayer_rggb8": None,
            "bayer_bggr8": None,
            "bayer_gbrg8": None,
            "bayer_grbg8": None,
            "bayer_rggb16": None,
            "bayer_bggr16": None,
            "bayer_gbrg16": None,
            "bayer_grbg16": None,
            "yuv422": None,
            "8UC1": ("uint8", 1, "L"),
            "8UC2": ("uint8", 2, "L"),
            "8UC3": ("uint8", 3, "L"),
            "8UC4": ("uint8", 4, "L"),
            "8SC1": ("int8", 1, "IS;8"),
            "8SC2": ("int8", 2, "IS;8"),
            "8SC3": ("int8", 3, "IS;8"),
            "8SC4": ("int8", 4, "IS;8"),
            "16UC1": ("uint16", 1, "I;16"),
            "16UC2": ("uint16", 2, "I;16"),
            "16UC3": ("uint16", 3, "I;16"),
            "16UC4": ("uint16", 4, "I;16"),
            "16SC1": ("int16", 1, "IS;16"),
            "16SC2": ("int16", 2, "IS;16"),
            "16SC3": ("int16", 3, "IS;16"),
            "16SC4": ("int16", 4, "IS;16"),
            "32FC1": ("float32", 1, "F"),
            "32FC2": ("float32", 2, "F"),
            "32FC3": ("float32", 3, "F"),
            "32FC4": ("float32", 4, "F"),
            "32SC1": ("int32", 1, "I"),
            "32SC2": ("int32", 2, "I"),
            "32SC3": ("int32", 3, "I"),
            "32SC4": ("int32", 4, "I"),
            "64FC1": ("float64", 1, "F"),
            "64FC2": ("float64", 2, "F"),
            "64FC3": ("float64", 3, "F"),
            "64FC4": ("float64", 4, "F"),
        }
        # Get the encoding of the image
        rv = encoding_to_dtype_Nchannels_PILmode[img_msg.encoding]
        # Raise NotImplementedError if encoding wasn't supported.
        if rv is None:
            raise ImageError(f"Encoding {img_msg.encoding}: not supported.")
        dtype, n_channels, mode = rv
        # Get appropiated data type for numpy
        dtype = numpy.dtype(dtype)
        dtype = dtype.newbyteorder(">" if img_msg.is_bigendian else "<")
        # Full row length in bytes
        step = img_msg.step
        # Create array depending on number of channels (mono=1, rgb=3, rgba=4)
        if n_channels == 1:
            strides = (step, dtype.itemsize)
            im = numpy.ndarray(
                shape=(img_msg.height, img_msg.width),
                dtype=dtype,
                buffer=img_msg.data,
                strides=strides,
            )
        else:
            strides = (step, n_channels, dtype.itemsize)
            im = numpy.ndarray(
                shape=(img_msg.height, img_msg.width, n_channels),
                dtype=dtype,
                buffer=img_msg.data,
                strides=strides,
            )
        # Swap bytes if endian type is different
        if img_msg.is_bigendian == (sys.byteorder == "little"):
            im = im.byteswap().newbyteorder()
        try:
            # Do scaling between CV_16U [0,65535] and mono8 [0,255] images.
            # TODO (Flor_Grosso): this is a special case when converting
            # depth images to mono8. Consider extending this to other formats.
            if img_msg.encoding == "16UC1":
                im = im / 257
            # Create PIL Image object from the array data.
            im = Im.fromarray(im, mode)
            # Convert to output_encoding if necessary.
            if img_msg.encoding != output_encoding:
                if output_encoding == "mono8":
                    im = im.convert("L")
                elif output_encoding == "rgb8":
                    im = im.convert("RGB")
        except Exception as e:
            raise ImageError(f"Could not encode {img_msg.encoding} image to {output_encoding}.")
        self._im = im
    @overrides(ImageBase)
    def fillPolygons(self, base_img, contours, default_cost):
        # Create PIL Image object
        base = Im.fromarray(base_img.astype("uint8"))
        # PIL supports drawing a polygon at a time.
        for polygon in contours:
            # Convert list of list into list of tupples, to match required
            # input format.
            coords = list(map(tuple, polygon.get("data", [])))
            color = polygon.get("color", default_cost)
            ImageDraw.Draw(base).polygon(coords, outline=color, fill=color)
        return numpy.array(base)
    @overrides(ImageBase)
    def enhance(self, brightness, contrast):
        """
        Adjust brightness and contrast of the image
        """
        if contrast != self.DEFAULT_CONTRAST:
            """
            A factor of less than 1.0 makes the image darker (and a value of
            0.0 makes the image completely black). A factor of greater than
            1.0 makes the image brighter. A factor of exactly 1.0 leaves the
            original image unchanged.
            """
            self._im = ImageEnhance.Contrast(self._im).enhance(contrast)
        if brightness != self.DEFAULT_BRIGHTNESS:
            """
            Factors greater than 1.0 make the image brighter, less than 1.0
            makes the image darker. A factor of 0.0 results in a completely
            black image:
            """
            self._im = ImageEnhance.Contrast(self._im).enhance(brightness)
class cv2ImageWrapper(ImageBase):
    """
    This class allows simple image processing with cv2 (OpenCV).
    """
    _PACKAGE = "cv2"
    DEFAULT_CONTRAST = 1.0
    DEFAULT_BRIGHTNESS = 0
    def __init__(self):
        ImageBase.__init__(self)
        # HACK(herchu): Initialize the cv2 encodings only if an object of this class
        # is ever constructed. Otherwise, it will fail if `cv` is not available
        self._cv2_encodings_to_compression_quality = {
            "jpg": int(cv2.IMWRITE_JPEG_QUALITY),  # 0(lowest) to 100(highest)
            "jpeg": int(cv2.IMWRITE_JPEG_QUALITY),
            "png": int(
                cv2.IMWRITE_PNG_COMPRESSION
            ),  # 0 (no compression) to 9 (highest compression)
        }
    @overrides(ImageBase)
    def open(self, image_path):
        self._im = cv2.imread(image_path)
        self._source_format = imghdr.what(image_path)
        return self._source_format
    @overrides(ImageBase)
    def resize(self, target_size):
        if self._im is None:
            return
        if self._should_resize_to(target_size):
            width, height = self._compute_output_image_size(target_size)
            # Check if resizing should take place based on the image's
            # current dimensions and the expected size.
            self._im = cv2.resize(self._im, (width, height), interpolation=cv2.INTER_LINEAR)
    @overrides(ImageBase)
    def encode(self, skip_conversion, quality):
        if self._im is None:
            return
        # If the no conversion flag is active, preserve source encoding.
        # If not, convert to jpg by default.
        if skip_conversion and self._source_format:
            out_format = self._source_format
        else:
            out_format = "jpg"
        if out_format not in supported_img_encodings:
            return
        self._buf = cv2.imencode(
            "." + out_format,
            self._im,
            [self._cv2_encodings_to_compression_quality[out_format], quality],
        )[1]
    @overrides(ImageBase)
    def toBytes(self):
        if self._buf is None:
            return None
        return self._buf.tobytes()
    @overrides(ImageBase)
    def fromImgMsg(self, img_msg, output_encoding="passthrough"):
        bridge = CvBridgeCustom()
        self._im = bridge.imgmsg_to_cv2(img_msg, output_encoding)
    @overrides(ImageBase)
    def fromCompressedImgMsg(self, img_msg, output_encoding="passthrough"):
        bridge = CvBridgeCustom()
        self._im = bridge.compressed_imgmsg_to_cv2(img_msg, output_encoding)
    """
    Computes the size of the output image, keeping the aspect ratio.
    """
    def _compute_output_image_size(self, target_size):
        output_width = target_size[0]
        output_height = target_size[1]
        src_height, src_width = self._im.shape[:2]
        # Preserve the aspect ratio of the image by setting its larger
        # dimension to the default values and the other proportional to
        # this scaling.
        if src_width > src_height:
            if src_width > 0:
                output_height = max(1, int(src_height * float(output_width) / src_width))
        else:
            if src_height > 0:
                output_width = max(1, int(src_width * float(output_height) / src_height))
        # TODO (Flor_Grosso): Update custom image config with output width and
        # height to report it. Pay attention to how this config affects
        # images differently according to its mode (landscape or portrait),
        # since it is a general setting for now.
        return output_width, output_height
    def _should_resize_to(self, target_size):
        """
        Checks if the image should be resized to the output width and height,
        depending on the dimensions of the source.
        If the source height and width is equal to the output's, no resizing
        is required.
        """
        src_height, src_width = self._im.shape[:2]
        output_width = target_size[0]
        output_height = target_size[1]
        return not (
            src_width == output_width
            and src_height == output_height
            or output_width > src_width
            or output_height > src_height
        )
    @overrides(ImageBase)
    def fillPolygons(self, base_img, contours, default_cost):
        # Make sure the base image is received as a matrix that can be
        # processed by fillPoly
        image = numpy.ascontiguousarray(base_img, dtype=numpy.uint8)
        # Fill one polygon at a time, since they might have different number of
        # vertices and that breaks the nested array data source.
        for polygon in contours:
            # Round coordinates to integers that match matrix indexes.
            # Note that we use floor here since matrix indexes go from
            # 0...map_width - 1 and 0...map_height - 1
            coords = numpy.array(
                [
                    [(numpy.floor(float(i))).astype(int) for i in vertices]
                    for vertices in polygon.get("data", [])
                ]
            )
            if coords.size == 0:
                continue
            cv2.fillPoly(image, [coords], polygon.get("color", default_cost))
        return image
    @overrides(ImageBase)
    def enhance(self, brightness, contrast):
        """
        Adjust contrast and brightness of the source image by
        scaling and adding an offset to it, respectively:
        out_img = alpha * in_img + beta
        alpha (0-100) controls contrast and beta (1.0-3.0) brightness
        """
        if contrast or brightness != self.DEFAULT_BRIGHTNESS:
            self._im = cv2.convertScaleAbs(self._im, alpha=contrast, beta=brightness)
# Let "Image" class being an alias of one of the Wrappers
if PACKAGE == "PIL":
    Image = PILImageWrapper
elif PACKAGE == "cv2":
    Image = cv2ImageWrapper
else:
    raise ImageError("Neither OpenCv nor PIL found in the system.")
````

## File: util/math_util.py
````python
# Copyright (c) 2018, InOrbit, Inc.
# All rights reserved.
# Useful functions to deal with
#  - 2D and 3D point operations (distance, squared distance, distance from point to segment).
#  - Paths / polylines (distance between paths)
#  - Arrays: downsampling to N elements, at regular intervals.
import math
import numpy as np
def point_distance(p1, p2):
    return math.sqrt(point_sq_distance(p1, p2))
def point_sq_distance(p1, p2):
    """
    Square distance between 2 points
    """
    dx = p1.x - p2.x
    dy = p1.y - p2.y
    dz = getattr(p1, "z", 0) - getattr(p2, "z", 0)
    return dx * dx + dy * dy + dz * dz
def point_to_line_sq_distance(p, p1, p2):
    """
    2D square distance from a point p to a line given by p1-p2
    """
    x = p1.x
    y = p1.y
    dx = p2.x - x
    dy = p2.y - y
    if dx != 0 or dy != 0:
        t = ((p.x - x) * dx + (p.y - y) * dy) / (dx * dx + dy * dy)
        if t > 1:
            x = p2.x
            y = p2.y
        elif t > 0:
            x += dx * t
            y += dy * t
    dx = p.x - x
    dy = p.y - y
    return dx * dx + dy * dy
def path_distance(path1, path2):
    """
    Distance between two paths.
    - Returns inf if any of the paths are invalid,
    they have different size or they differ in more than 2 points.
    - Returns the number of points the paths differ when this is less than
    2.
    NOTE: For paths larger than 1000 points, this method doesn't compare
    paths over their whole length but just on a 20% of them.
    """
    inf = float("inf")
    if path1 is None and path2 is None:
        return 0.0  # Both are None
    if (path1 is None) != (path2 is None):  # (Note: Logical XOR!)
        return inf
    if len(path1) != len(path2):  # Not same number of points
        return inf
    differ = 0  # Number of points they differ
    dist = 0  # Total distance
    # PATHS LONGER THAN 1000 POINTS
    # Avoid comparing point to point when the path has more than 1000
    # points. Instead, compute how many points the 20% of the path's length
    # represent, divide that in 3 (m = 6.6% of total points) and then:
    #   - Compare the first m points
    #   - Compare the mid m points
    #   - Compare the last m points
    if len(path1) > 1000:
        # Just compare over the 20% of the path's size, divided in 3 times
        check_step = int(len(path1) * 0.2 / 3)  # m
        initial_batch = np.arange(check_step)
        mid_batch = np.arange(
            int(len(path1) * 0.5 - check_step / 2), int(len(path1) * 0.5 + check_step / 2)
        )
        final_batch = np.arange(len(path1) - check_step, len(path1))
        # Concatenate all the indexes of path's data that needs to be
        # compared.
        indexes = np.concatenate((initial_batch, mid_batch, final_batch), axis=0)
    else:
        # PATHS SHORTER THAN 500 POINTS
        indexes = range(len(path1))
    # Compare points
    for i in range(len(path1)):
        p1 = path1[i]
        p2 = path2[i]
        d = point_distance(p1, p2)
        dist += d
        if d > 0:
            differ += 1
    if differ > 1:
        # More than two points differ. We rather resend the path.
        return inf
    else:
        # Return 0 (if paths are equal) or the distance in the only point
        # they differ.
        return dist
def downsample_array(arr, maxn):
    """
    Downsamples (any) array to N elements, taking at regular
    intervals. First and last element are always returned (provided N>=2).
    """
    if len(arr) <= maxn or maxn <= 1:
        # It is assumed that _downsample will return a new array,
        # so just save the iterators logic but still return a copy
        return arr[:]
    # Take exactly maxn elements. Select at regular (non-int) intervals
    # maxn-1 of them, and the last one manually.
    return [arr[int(float(i * len(arr)) / (maxn - 1))] for i in range(maxn - 1)] + [arr[-1]]
def get_position_with_offset(pos_x, pos_y):
    """
    Calculates offsets for poses with large coordinates.
    In some occasions pose coordinates are likely very large numbers which may cause losing
    precision when transmitting them as protobuf messages. To avoid this, coordinates are
    separated in a high-order offset component and the low-order higher accuracy part for
    using them on Pose-related messages e.g. PoseMessageData and LocationAndPoseMessage.
    Example:
    get_position_with_offset(173285.4100000000035, 1328333.1799999999348)
    returns:
    {
        "pos_x": 3285.4100000000035,
        "pos_y": -1666.8200000000652,
        "offset_x": 170000,
        "offset_y": 1330000
    }
    """
    POSITION_MAX_VALUE = 10000 # Position will be in interval (-10000.0, 10000.0)
    offset_x = round(pos_x / POSITION_MAX_VALUE) * POSITION_MAX_VALUE
    pos_x = pos_x - offset_x
    offset_y = round(pos_y / POSITION_MAX_VALUE) * POSITION_MAX_VALUE
    pos_y = pos_y - offset_y
    return { 'pos_x': pos_x, 'pos_y': pos_y, 'offset_x': offset_x, 'offset_y': offset_y }
````

## File: util/networking_mixin.py
````python
# Copyright (c) 2022, InOrbit, Inc.
# All rights reserved.
# Utility class superclass to track network usage (bytes transmitted and received)
class NetworkingMixin:
    def __init__(self):
        self._tx_bytes = 0
        self._rx_bytes = 0
    def get_tx_bytes(self):
        """
        Returns total number of bytes transmitted since the object was created
        """
        return self._tx_bytes
    def get_rx_bytes(self):
        """
        Returns total number of bytes received since the object was created
        """
        return self._rx_bytes
    def add_tx_bytes(self, bytes):
        self._tx_bytes += bytes
    def add_rx_bytes(self, bytes):
        self._rx_bytes += bytes
````

## File: util/once_logger.py
````python
# Copyright (c) 2018, InOrbit, Inc.
# All rights reserved.
# Once logger class. Used when tracking exceptions that might occur at a
# high rate, logging them only once. Keeps track of the number of occurrences
# of each exception and allows three log levels: warn, error, exception.
class OnceLogger:
    def __init__(self, log_handle):
        self._log = log_handle
        # Dictionary of number of occurrences of each exception, indexed by
        # event id.
        self._reported_events = {}
    def warn(self, key, msg):
        """
        Logs a warn message if it's a new event, and increments occurrence
        number if it's a known issue.
        TODO (FlorGrosso): rename this method to 'warning' to be consistent
        with inorbit's logger naming convention.
        """
        if self._should_report_event(key):
            self._log.warning(msg + " [reported once]")
            self._reported_events[key] = 1
        else:
            self._reported_events[key] += 1
    def error(self, key, msg):
        """
        Logs an error message if it's a new event, and increments occurrence
        number if it's a known issue.
        """
        if self._should_report_event(key):
            self._log.error(msg + " [reported once]")
            self._reported_events[key] = 1
        else:
            self._reported_events[key] += 1
    def exception(self, key, msg):
        """
        Logs an exception message if it's a new event, and increments occurrence
        number if it's a known issue.
        """
        if self._should_report_event(key):
            self._log.exception(msg + " [reported once]")
            self._reported_events[key] = 1
        else:
            self._reported_events[key] += 1
    def reset_one(self, key):
        """
        Resets event tracking for a particular event.
        """
        if self._reported_events.get(key):
            self._reported_events[key] = 0
    def reset_set(self, keys):
        """
        Resets event tracking for a set of events.
        """
        map(self.reset_one, keys)
    def reset_all(self):
        """
        Resets event tracking for the whole module.
        """
        self._reported_events = {}
    def _should_report_event(self, key):
        """
        Checks if an event should be reported, based on its number of occurrence.
        """
        return self._reported_events.get(key, 0) == 0
````

## File: util/overrides.py
````python
# Copyright (c) 2018, InOrbit, Inc.
# All rights reserved.
"""
Convenience decorator to validate proper method inherittance
Taken from https://stackoverflow.com/a/8313042
"""
def overrides(interface_class):
    def overrider(method):
        assert method.__name__ in dir(interface_class)
        return method
    return overrider
````

## File: util/rate_limiter.py
````python
# Copyright (c) 2021, InOrbit, Inc.
# All rights reserved.
#
# Simple rate limiter (throttler) class. It allows filtering function calls (or incoming messages)
# to a defined maximum rate (in Hz).
#
# Example:
# ```
#    limiter = RateLimiter(0.1) # Will accept only once every 10 seconds
#    if limiter.accepts():
#       print('Throttled message accepted')
#    if limiter.accepts('a topic'):
#       print('Throttled message accepted from a specific topic')
# ```
#
# Trivial example (disabled)
# ```
#    limiter = RateLimiter(None) # accepts() will always return True
#    if limiter.accepts():
#       print('Use None as max_rate_hz to disable the throttler and simplify your code')
# ```
#
# More advanced example:
# ```
#    data = {}
#    limiter = RateLimiter(0.1, dict_data = data, field_name = "ts")
#    if limiter.accepts('topic'):
#        data["topic"]["msg"] = "Hello world"
#    ... later on ...
#    last_msg = data["topic"]["msg"]
#    last_msg_ts = data["topic"]["ts"]
# ```
#
#
# NOTE: This class does NOT hold an internal thread nor will do a last callback after N seconds
# if no more messages were received for a long time. So using it simply as in the example above,
# there is a chance that the last 'accepted' message even after a long time is not really the
# last received message: _messages are simply dropped_. This decision is made to keep the class
# simple; and avoid bringing external dependencies. If this is a concern, use for example
# https://github.com/Exahilosys/throttle
#
# Implementation:
#
# The class keeps an internal (*) dictionary with the last call timestamp for each 'topic'
# (it can be any identifier, used to throttle over different channels using the same RateLimiter).
# Within this dictionary, it keeps objects of the form { 'msg_ts': <timestamp> }
#
# For compatibility with existing agentlets, this dictionary can be external to the RateLimiter,
# owned by the caller: This allows storing other information in each topic, e.g. what was the
# last received message and any other data. For this reason, the field name can also be customized
# when instantiating the object; it defaults to "msg_ts"
#
import time
class RateLimiter:
    def __init__(self, max_rate_hz=1, dict_data=None, field_name="msg_ts"):
        self._last_data = {} if dict_data is None else dict_data
        self._max_rate_hz = max_rate_hz
        self._field_name = field_name
    def accepts(self, from_id="default", current_ts=None):
        if current_ts is None:  # Optional argument, but this dynamic value cannot be part of the
            # function defaults (it would turn out to be static!)
            current_ts = int(time.time() * 1000)
        if from_id not in self._last_data.keys():
            self._last_data[from_id] = {}
        # Calculate time difference between this message and the
        # last one. Discard new messages if the update rate is higher
        # than the maximum configured.
        ts_name = self._field_name
        max_rate_hz = self._max_rate_hz
        if self._last_data.get(from_id, {}).get(ts_name):
            ts_diff = current_ts - self._last_data[from_id][ts_name]
        else:
            ts_diff = None
        # Accept the msg if max_rate_hz is simply not set,
        # or if this is the first message seen, or if it is far enough from last message
        accepts = not max_rate_hz or (ts_diff is None) or (ts_diff > 1000 / max_rate_hz)
        if accepts:
            self._last_data[from_id][ts_name] = current_ts
        return accepts
    def reset(self):
        """
        Resets ts, forcing the Rate limiter to start counting again.
        """
        self._last_data = dict.fromkeys(self._last_data, {})
````

## File: util/robot_script_action.py
````python
# Copyright (c) 2019, InOrbit, Inc.
# All rights reserved.
# RobotScriptAction class. Handles both creating and executing a script
# from a given path. It also keeps track of the file's execution id and
# reports updates with execution status and output using callbacks.
import os
import stat
import subprocess
import threading
import inorbit.logger
# Script execution status
# Keep in sync with client code at common/ActionFeedback.js
# TODO(adamantivm) Consider using protobuf enums to keep in sync
STATUS_ABORTED = "aborted"
STATUS_FINISHED = "finished"
STATUS_INSTALLED = "file created"
STATUS_RUNNING = "running"
STATUS_NOT_INSTALLED = "unable to create file"
STATUS_TO_BE_STARTED = "to be started"
# Timeout for script execution
DEFAULT_TIMEOUT_SEC = 30
class RobotScriptAction:
    def __init__(self, file_name, actions_path, execution_id, exec_args=[], clean_env=True):
        self._file_name = file_name
        self._full_file_path = os.path.join(actions_path, file_name)
        self._exec_args = exec_args
        self._id = execution_id
        self._clean_env = clean_env
        # Subprocess for script execution
        self._subprocess = None
        self._execution_status = None
        self.logger = inorbit.logger.getLog(__name__)
    def from_file_contents(self, content):
        """
        Creates a new text file in self._full_file_path with the content
        provided. It also adds execution permissions to it.
        Returns True if it succeeded, False if not.
        TODO(IO-590): this is creating a new file without checking if it
        already exists. Revisit this before enabling this feature back.
        """
        try:
            with open(self._full_file_path, "w") as f:
                f.write(content)
                f.flush()
                os.fsync(f.fileno())
                # Adding execution permissions to the file.
                # ...st_mode are the current file permissions and
                # stat.S_IXUSR is the user execution permissions flag.
                os.fchmod(f.fileno(), os.fstat(f.fileno()).st_mode | stat.S_IXUSR)
        except Exception as e:
            self.logger.exception(f"Exception when creating the script: {self._full_file_path}")
            # If file creation failed at some point, clean it.
            try:
                # TODO (IO-590): consider being cautious about this
                # when we start supporting a configurable path for
                # user scripts, since this might cause a security issue.
                os.remove(self._full_file_path)
            except Exception as e:
                pass
            return False
        return True
    def run(self, status_cb, timeout=DEFAULT_TIMEOUT_SEC):
        """
        Spawns a thread for script execution.
        """
        if self._execution_status == STATUS_RUNNING:
            self.logger.warning(f"{self._get_name_with_args()} already running")
        # Start script execution thread
        t = threading.Thread(
            target=self._script_runner, name="script_runner_" + self._id, args=(status_cb, timeout)
        )
        t.start()
    def _script_runner(self, status_cb, timeout=DEFAULT_TIMEOUT_SEC):
        """
        Executes a script as a subprocess. Spawns a thread  waiting for it to
        finish, or kills it after timeout seconds. Output is saved in order
        to be sent.
        """
        # Prepare array of filename and args
        path_with_args = (
            (["env", "-i"] if self._clean_env else []) + [self._full_file_path] + self._exec_args
        )
        # Output from file execution
        output = {"stdout": None, "stderr": None, "return_code": None}
        try:
            self._subprocess = subprocess.Popen(
                path_with_args, stderr=subprocess.PIPE, stdout=subprocess.PIPE
            )
        except Exception as e:
            self.logger.exception(f"Exception when running script: {self._file_name}")
            self._execution_status = STATUS_ABORTED
            status_cb(self._file_name, self._id, self._execution_status)
            return
        self._execution_status = STATUS_RUNNING
        status_cb(self._file_name, self._id, self._execution_status)
        # In case of timeout, status is overwritten
        self._execution_status = STATUS_FINISHED
        try:
            timer = threading.Timer(
                timeout, lambda status_cb=status_cb: self._timeout_cb(status_cb)
            )
            timer.start()
            output["stdout"], output["stderr"] = self._subprocess.communicate()
            output["return_code"] = str(self._subprocess.returncode)
            # If the script ends with a non-zero status code, send an aborted status
            # to provide error feedback to the user
            # TODO(adamantivm) Surface status code on the client
            if self._subprocess.returncode != 0:
                self._execution_status = STATUS_ABORTED
        except Exception as e:
            self.logger.exception(
                f"Exception when stopping execution for: {self._get_name_with_args()}"
            )
        finally:
            timer.cancel()
        # Send a status update with the execution output
        status_cb(self._get_name_with_args(), self._id, self._execution_status, True, output)
    def _timeout_cb(self, status_cb):
        """
        Callback used to kill script execution in the case of timeout. Also
        updates status to "aborted".
        """
        # This shouldn't happen, but it's checked anyway.
        if self._subprocess is not None:
            self._subprocess.kill()
        self._execution_status = STATUS_ABORTED
        # Send a status update indicating script execution was aborted
        status_cb(
            self._file_name, self._id, self._execution_status, status_details="execution timed out"
        )
    def get_name(self):
        """
        Returns file name.
        """
        return self._file_name
    def get_args_string(self):
        """
        Returns script arguments as a string (these preserve the order
        in which they are stored in the array).
        """
        return " ".join(self._exec_args)
    def _get_name_with_args(self):
        """
        Returns a string with file name and the concatenated args.
        """
        return self._file_name + self.get_args_string()
````

## File: util/s3_upload_helper.py
````python
# Copyright (c) 2018, InOrbit, Inc.
# All rights reserved.
import os
import boto3
class S3_Client(object):
    """
    Interacts with amazon's S3.
    """
    def __init__(self, config):
        """
        Constructs a S3 helper to upload files.
        The config object contains the accessKey, secretKey, company and bucket properties
        telling where files will be uploaded and under which identity.
        """
        self.session = boto3.Session(
            aws_access_key_id=config["accessKey"], aws_secret_access_key=config["secretKey"]
        )
        self.s3 = self.session.resource("s3")
        self.bucket_name = config["bucket"]
        self.bucket = self.s3.Bucket(self.bucket_name)
        self.company = config["company"]
    def upload_file(self, filename, location=None, callback=None):
        """
        Uploads a file to the class bucket to the specified location.
        filename: The file location string.
        location: A string defining where the file should be stored, if left
        undefined it uses the filename as its filename and the company id as folder.
        Using amazon s3 standards you can store them in folders using slashes
        in its name 'company/robot/test.py'
        """
        optionalLocation = "" if location is None else (location + "/")
        key = self.company + "/" + optionalLocation + os.path.basename(filename)
        # Uploads the given file using a managed uploader, which will split up
        # large files automatically and upload parts in parallel.
        self.bucket.upload_file(filename, key, Callback=callback)
        # Return the object URL. Note this may be a protected object (ie. needs credentials to
        # access it)
        return f"https://s3.amazonaws.com/{self.bucket_name}/{key}"
````

## File: util/simplify.py
````python
# Copyright (c) 2018, InOrbit, Inc.
# All rights reserved.
# The work in this file is based on Simplify.js, a high-performance JS
# polyline simplification library:
# https://github.com/mourner/simplify-js
# Parts of the code were taken and modified. The original license
# is reproduced below.
# Software License Agreement (BSD License)
#
# Copyright (c) 2017, Vladimir Agafonkin
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
from .math_util import point_sq_distance
from .math_util import point_to_line_sq_distance
def simplify(points, tolerance=1, highest_quality=False):
    """
    Simplifies the input points array using radial distance (if highest
    quality is not required) and Douglas-Peuquer algorithm.
    """
    if len(points) <= 2:
        return points
    if tolerance is None:
        tolerance = 1
    if not highest_quality:
        points = simplify_radial_dist(points, tolerance)
    points = simplify_DouglasPeucker(points, tolerance)
    return points
def simplify_radial_dist(points, sq_tolerance):
    """
    Basic distance-based simplification.
    """
    prev_point = points[0]
    new_points = [prev_point]
    for point in points:
        if point_sq_distance(point, prev_point) > sq_tolerance:
            new_points.append(point)
            prev_point = point
    # Make sure the ending point is stored
    if prev_point != point:
        new_points.append(point)
    return new_points
def simplify_DouglasPeucker(points, sq_tolerance):
    """
    Simplification using Ramer-Douglas-Peucker algorithm.
    """
    last = len(points) - 1
    simplified = [points[0]]
    simplify_DP_step(points, 0, last, sq_tolerance, simplified)
    simplified.append(points[last])
    return simplified
def simplify_DP_step(points, first, last, sq_tolerance, simplified):
    """
    Does the core calculation of the Ramer-Douglas-Peucker algorithm and
    decides which points should be preserved from the original array.
    """
    max_sq_dist = sq_tolerance
    for i in range(first + 1, last):
        sq_dist = point_to_line_sq_distance(points[i], points[first], points[last])
        if sq_dist > max_sq_dist:
            index = i
            max_sq_dist = sq_dist
    if max_sq_dist > sq_tolerance:
        if index - first > 1:
            simplify_DP_step(points, first, index, sq_tolerance, simplified)
        simplified.append(points[index])
        if last - index > 1:
            simplify_DP_step(points, index, last, sq_tolerance, simplified)
````

## File: util/suppress_stderr.py
````python
import os
import sys
from contextlib import contextmanager
@contextmanager
def suppress_stderr():
    """
    Helper method to suppress stderr temporarily.
    """
    with open(os.devnull, "w") as devnull:
        old_stderr = sys.stderr
        sys.stderr = devnull
        try:
            yield
        finally:
            sys.stderr = old_stderr
````

## File: util/tf_util.py
````python
"""
Useful functions to deal with 3D poses in the form of
geometry_msgs/Transform messages, 7-DOF poses (translation + quaternion)
and 4 by 4 matrices.
NOTE Importing this module requires knowing ROS module locations. Do not
import this module directly. Instead, use ros.py agentlet and call tf_util()
to obtain a reference.
"""
import numpy as np
import transformations
try:
    from geometry_msgs.msg import Transform, Pose
except Exception as e:
    raise Exception("Error importing ROS modules from tf_util; are ROS paths already set?")
def geommsgpose_to_pose(pose):
    """
    Given a geometry_msgs.Pose,
    Returns pose in the following format
    [p_x, p_y, p_z, q_x, q_y, q_z, q_w]
    """
    return np.array(
        [
            pose.position.x,
            pose.position.y,
            pose.position.z,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ]
    )
def pose_to_geommsgpose(pose):
    """
    Input in the following format
    [p_x, p_y, p_z, q_x, q_y, q_z, q_w]
    Returns a geometry_msgs.Pose object with position and orientation.
    """
    p = Pose()
    (
        p.position.x,
        p.position.y,
        p.position.z,
        p.orientation.x,
        p.orientation.y,
        p.orientation.z,
        p.orientation.w,
    ) = pose
    return p
def pose_to_transform(pose):
    """
    Input pose must have format
    [p_x, p_y, p_z, q_x, q_y, q_z, q_w]
    """
    transform = Transform()
    (
        transform.translation.x,
        transform.translation.y,
        transform.translation.z,
        transform.rotation.x,
        transform.rotation.y,
        transform.rotation.z,
        transform.rotation.w,
    ) = pose
    return transform
def transform_to_pose(transform):
    """
    Returns pose in the following format
    [p_x, p_y, p_z, q_x, q_y, q_z, q_w]
    """
    return np.array(
        [
            transform.translation.x,
            transform.translation.y,
            transform.translation.z,
            transform.rotation.x,
            transform.rotation.y,
            transform.rotation.z,
            transform.rotation.w,
        ]
    )
def pose_to_matrix(p):
    """
    Input pose must have format
    [p_x, p_y, p_z, q_x, q_y, q_z, q_w]
    """
    x, y, z, qx, qy, qz, qw = p
    t = transformations.translation_matrix([x, y, z])
    r = transformations.quaternion_matrix([qx, qy, qz, qw])
    return transformations.concatenate_matrices(t, r)
def matrix_to_pose(m):
    """
    Returns pose in the following format
    [p_x, p_y, p_z, q_x, q_y, q_z, q_w]
    """
    (x, y, z) = transformations.translation_from_matrix(m)
    (qx, qy, qz, qw) = transformations.quaternion_from_matrix(m)
    return [x, y, z, qx, qy, qz, qw]
def transform_to_matrix(transform):
    """
    Returns pose in the following format
    [p_x, p_y, p_z, q_x, q_y, q_z, q_w]
    """
    return pose_to_matrix(transform_to_pose(transform))
def matrix_to_transform(matrix):
    """
    Returns transform with the given matrix
    translation and rotation parts.
    """
    return pose_to_transform(matrix_to_pose(matrix))
def substract_transforms(transform_a, transform_b):
    """
    Returns transform c that goes from b to a (as if
    b were the origin of coordinates).
    """
    matrix_a = transform_to_matrix(transform_a)
    matrix_b = transform_to_matrix(transform_b)
    inv_matrix_b = transformations.inverse_matrix(matrix_b)
    matrix_c = transformations.concatenate_matrices(inv_matrix_b, matrix_a)
    return matrix_to_transform(matrix_c)
````

## File: util/topic_info.py
````python
# Copyright (c) 2018, InOrbit, Inc.
# All rights reserved.
# The work in this file is based on topic_info:
# https://github.com/ros-visualization/rqt_topic/blob/master/src/rqt_topic/topic_info.py
# Parts of the code were taken and modified. The original license
# is reproduced below.
#
# Software License Agreement (BSD License)
#
# Copyright (c) 2011, Dorian Scholz, TU Darmstadt
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#   * Neither the name of the TU Darmstadt nor the names of its
#     contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
from __future__ import division
from __future__ import with_statement
try:
    from cStringIO import StringIO as BufferType
except ImportError:
    from io import BytesIO as BufferType
try:
    import roslib
except Exception as e:
    raise Exception("Failed to load roslib")
try:
    import rospy
except Exception as e:
    raise Exception("Failed to load rospy")
try:
    import rostopic
except Exception as e:
    raise Exception("Failed to load rostopic")
import inorbit.logger
from once_logger import OnceLogger
class TopicInfo(rostopic.ROSTopicHz):
    def __init__(self, topic_name, topic_type=None):
        super(TopicInfo, self).__init__(100)
        self._topic_name = topic_name
        self._topic_type = topic_type
        # Last rate computed for the topic
        self._last_rate = 0
        self.logger = inorbit.logger.getLog(__name__)
        self.once_logger = OnceLogger(self.logger)
        self._subscriber = None
        self.monitoring = False
        self._reset_data()
        self.message_class = None
        # NOTE(Flor_Grosso): both rostopic.get_topic_type() and roslib.message.get_message_class()
        # break if the topic name doesn't start with "/", so we need to add it if the config
        # doesn't have it.
        # TODO(Flor_Grosso): the RosAgentlet does something similar in l.205. Consider moving this
        # functionality to a shared module.
        if self._topic_name is not None and self._topic_name[0] != "/":
            self._topic_name = "/" + self._topic_name
        try:
            self.message_class = roslib.message.get_message_class(self._topic_type)
        except Exception as e:
            self.message_class = None
    def _reset_data(self):
        self.last_message = None
        self.times = []
        self.timestamps = []
        self.sizes = []
        self._last_rate = 0
    def toggle_monitoring(self):
        if self.monitoring:
            self.stop_monitoring()
        else:
            self.start_monitoring()
    def start_monitoring(self):
        # If the message class is defined, then subscribe to the topic
        # providing the specific value.
        if self.message_class:
            self.monitoring = True
            self._subscriber = rospy.Subscriber(
                self._topic_name, self.message_class, self.message_callback
            )
        # If the message class is not known yet, use AnyMsg type and provide
        # a setup callback to find out this value.
        else:
            self.logger.info("Registering subscriber to: " + self._topic_name)
            self._subscriber = rospy.Subscriber(self._topic_name, rospy.AnyMsg, self.setup_callback)
    def stop_monitoring(self):
        self.monitoring = False
        self._reset_data()
        if self._subscriber is not None:
            self._subscriber.unregister()
    def setup_callback(self, message):
        """
        Callback for ros msgs of the type 'rospy.AnyMsg', used to query and set
        message class on the first msg received.
        """
        # Get topic class
        try:
            self._topic_type = rostopic.get_topic_type(self._topic_name)[0]
            self.message_class = roslib.message.get_message_class(self._topic_type)
            # Now reset monitoring to the corresponding message type
            self._subscriber.unregister()
            self.start_monitoring()
        except Exception as e:
            self.message_class = None
            self.once_logger.error(
                self._topic_name, f"Can't get msg class for '{self._topic_name}'."
            )
    def message_callback(self, message):
        """
        Callback for ros msgs with a known msg class.
        """
        rostopic.ROSTopicHz.callback_hz(self, message)
        with self.lock:
            self.timestamps.append(rospy.get_time())
            # FIXME: this only works for message of class AnyMsg
            # self.sizes.append(len(message._buff))
            # time consuming workaround...
            buff = BufferType()
            message.serialize(buff)
            self.sizes.append(len(buff.getvalue()))
            if len(self.timestamps) > self.window_size - 1:
                self.timestamps.pop(0)
                self.sizes.pop(0)
            assert len(self.timestamps) == len(self.sizes)
            self.last_message = message
    def get_bw(self):
        if len(self.timestamps) < 2:
            return None, None, None, None
        current_time = rospy.get_time()
        if current_time <= self.timestamps[0]:
            return None, None, None, None
        with self.lock:
            total = sum(self.sizes)
            bytes_per_s = total / (current_time - self.timestamps[0])
            mean_size = total / len(self.timestamps)
            max_size = max(self.sizes)
            min_size = min(self.sizes)
            return bytes_per_s, mean_size, min_size, max_size
    def get_hz(self):
        # If there's some rate data available, check if those values are valid
        # or have timed out, based on the last saved timestamp on ros callback.
        if self._last_rate > 0 and self.timestamps:
            # Timeout is set to 3 * topic period
            max_timeout = 3 / self._last_rate
            topic_has_timed_out = rospy.get_time() - self.timestamps[-1] > max_timeout
            if topic_has_timed_out:
                self._reset_data()
        if not self.times:
            return None, None, None, None
        with self.lock:
            n = len(self.times)
            mean = sum(self.times) / n
            rate = 1.0 / mean if mean > 0.0 else 0
            min_delta = min(self.times)
            max_delta = max(self.times)
            self._last_rate = rate
        return rate, mean, min_delta, max_delta
````

## File: common_requirements.txt
````
boto3==1.26.37
catkin-pkg==0.5.2
Flask==2.2.2
mock==4.0.3
paho-mqtt==1.6.1
# NOTE(FlorGrosso): protobuf is pinned to 3.5.x for all variants, until we switch to a higher version.
protobuf==3.5.1
psutil==5.9.4
pypng==0.0.20
# Required to send traffic through HTTP proxy servers
PySocks==1.7.1

# TODO(FlorGrosso): Move these packages to a ROS 2 requirements list
transformations==2022.9.26
````

## File: inorbit.py
````python
#!/usr/bin/env python
#
# Copyright (c) 2018, 2020, InOrbit, Inc.
# All rights reserved.
#
# InOrbit agent main entry point.
import ctypes
import json
import os
import random
import signal
import sys
import threading
import time
import inorbit.link
import inorbit.logger  # Import before other modules to set logging format
from inorbit import VERSION
from inorbit.agentlets import agentlet
from inorbit.agentlets.camera import RosImageAgentlet
from inorbit.agentlets.custom_commands import CustomCommandsAgentlet
from inorbit.agentlets.custom_data import CustomDataAgentlet
from inorbit.agentlets.databag import DatabagAgentlet
from inorbit.agentlets.diagnostics import RosDiagnosticsAgentlet
from inorbit.agentlets.events import RobotEventsAgentlet
from inorbit.agentlets.inorbit_pb2 import ModuleStateOptionsMessage
from inorbit.agentlets.inorbit_pb2 import StateOptions
from inorbit.agentlets.localization import RosLocalizationAgentlet
from inorbit.agentlets.map import RosMapAgentlet
from inorbit.agentlets.odometry import RosOdometryAgentlet
from inorbit.agentlets.pose import RosPoseAgentlet
from inorbit.agentlets.ros import RosAgentlet
from inorbit.agentlets.rosbag import RosbagAgentlet
from inorbit.agentlets.state_manager import ModuleStateManager
from inorbit.agentlets.system import SystemAgentlet
from inorbit.agentlets.teleop import RosTeleopAgentlet
from inorbit.agentlets.gps import GPSAgentlet
from inorbit.log_manager import LogManager
from mock import MagicMock
from util.once_logger import OnceLogger
# Attach signals to parent so as to make sure to die if the
# start script dies - necessary to get upstart to work
libc = ctypes.cdll["libc.so.6"]
libc.prctl(1, 15)  # SIGTERM
libc.prctl(1, 9)  # SIGKILL
# Time-related constants
MINUTE_S = 60
HOUR_S = 60 * MINUTE_S
# Time before programmed agent shutdown.
# Default: 24 hours plus a random value between 0 and 4 more hours
# TODO(adamantivm) Make this configurable
REBOOT_TIME_S = 24 * HOUR_S + random.randrange(4 * HOUR_S)
MODULE_STATES_CACHE_LOAD_S = 10
class Agent:
    def __init__(self):
        self.logger = inorbit.logger.getLog(__name__)
        self.once_logger = OnceLogger(self.logger)
        # Hash set of module instances
        self._modules = {}
        # Hash set of module class loading status
        self._module_status = {}
        # Flag that indicates whether the agent is running in a system with or
        # without ros
        self._ros_disabled = False
        # As modules shouldn't be loaded more than once, use a mutex to avoid
        # them being loaded at the same moment by different threads
        # NOTE(diegobatt): This could happened due to a very rare case were the
        # connection is re-established at the same time the modules are being
        # loaded from the cache, and it would be impossible to catch.
        # TODO(diegobatt): Make module's load idempotent as to avoid the need
        # of this mutex
        self._module_states_mutex = threading.Lock()
        # Register the agent's shutdown as the handler for SIGUSR1.
        # NOTE(diegobatt): This user-defined signal is raised in other modules
        # to force restart
        signal.signal(signal.SIGUSR1, self.shutdown)
    def _start_link(self):
        """
        Connects to InOrbit Cloud. If starting the link raises any exception,
        the agent is shutdown.
        """
        try:
            self._link.start()
        except Exception as e:
            self.logger.exception("Exception while initializing the " "connection")
    def start(self):
        # 1. Initialize and establish connection to InOrbit
        self._link = inorbit.link.Link()
        # Note that incoming commands should be guaranteed to arrive
        # (and exactly once, should not repeat!) QoS=2
        self._link.add_listener("in_cmd", self._command_received, 2)
        # Register listener for state updates commands
        # Module states should be guaranteed to arrive and we don't mind if
        # they arrive twice: QoS=1
        self._link.add_listener("modules/set_state", self._state_received, 1)
        self._link.add_listener("modules/get_state_options", self._get_state_options, 1)
        self.logger.info("Initializing MQTT connection...")
        # Start the link in a thread so it doesn't block
        threading.Thread(target=self._start_link).start()
        # 2. Create agentlets
        # TODO(adamantivm) Review and generalize this part
        ros_agentlet = RosAgentlet(self._link)
        self._modules[RosAgentlet.__name__] = ros_agentlet
        odometry_agentlet = RosOdometryAgentlet(self._link, ros_agentlet)
        self._modules[RosOdometryAgentlet.__name__] = odometry_agentlet
        robot_events_agentlet = RobotEventsAgentlet(self._link)
        diagnostics_agentlet = RosDiagnosticsAgentlet(self._link, ros_agentlet)
        self._modules[RosDiagnosticsAgentlet.__name__] = diagnostics_agentlet
        self._modules[RobotEventsAgentlet.__name__] = robot_events_agentlet
        custom_data_agentlet = CustomDataAgentlet(
            self._link, ros_agentlet, diagnostics_agentlet, robot_events_agentlet
        )
        self._modules[CustomDataAgentlet.__name__] = custom_data_agentlet
        system_agentlet = SystemAgentlet(self._link, diagnostics_agentlet)
        self._modules[SystemAgentlet.__name__] = system_agentlet
        # TODO(herchu)/ros2 RosoutAgentlet
        map_agentlet = self._modules[RosMapAgentlet.__name__] = RosMapAgentlet(
            self._link, ros_agentlet, custom_data_agentlet
        )
        pose_agentlet = self._modules[RosPoseAgentlet.__name__] = RosPoseAgentlet(
            self._link, ros_agentlet, map_agentlet
        )
        localization_agentlet = RosLocalizationAgentlet(
            self._link, ros_agentlet, pose_agentlet, map_agentlet
        )
        self._modules[RosLocalizationAgentlet.__name__] = localization_agentlet
        self._modules[RosImageAgentlet.__name__] = RosImageAgentlet(self._link, ros_agentlet)
        self._modules[RosTeleopAgentlet.__name__] = RosTeleopAgentlet(
            self._link, ros_agentlet, odometry_agentlet
        )
        # TODO(herchu)/ros2 AlertManagerAgentlet
        rosbag_agentlet = self._modules[RosbagAgentlet.__name__] = RosbagAgentlet(
            self._link, ros_agentlet
        )
        databag_agentlet = self._modules[DatabagAgentlet.__name__] = DatabagAgentlet(self._link)
        # TODO(Flor_Grosso)/ros2 Enable DatabagAgentlet
        # databag_agentlet = self._modules[DatabagAgentlet.__name__] = DatabagAgentlet(self._link)
        custom_data_agentlet = CustomDataAgentlet(
            self._link, ros_agentlet, diagnostics_agentlet, robot_events_agentlet
        )
        self._modules[CustomDataAgentlet.__name__] = custom_data_agentlet
        custom_commands_agentlet = CustomCommandsAgentlet(self._link)
        self._modules[CustomCommandsAgentlet.__name__] = custom_commands_agentlet
        gps_agentlet = GPSAgentlet(self._link, ros_agentlet)
        self._modules[GPSAgentlet.__name__] = gps_agentlet
        # TODO(herchu)/ros2 RosMonitoringAgentlet
        # TODO(herchu)/ros2 APIAgentlet
        # TODO(herchu)/ros2 SpatialAnnotationsAgentlet
        # Register additional modules that perform networking with SystemAgentlet (in addition
        # to Link which is always tracked).
        # TODO(herchu) In the future, make this configurable
        system_agentlet.register_networking_module(rosbag_agentlet)
        system_agentlet.register_networking_module(databag_agentlet)
        # Load default modules at start
        default_modules = [
            SystemAgentlet.__name__,
            RosAgentlet.__name__,
            # NOTE(adamantivm) The Map agentlet is crucial for
            # ROS based systems so we enable it by default to
            # avoid any cloud config issues
            RosMapAgentlet.__name__,
            # Starting with 'offline' agent variant, the databag agentlet needs to be loaded
            # at all times (to send blackbox rosbags and clean up recorded bags)
            DatabagAgentlet.__name__,
        ]
        loaded = True
        # Initialize module state manager
        # NOTE(Flor_Grosso): the _module_state_manager needs to be up before loading any module
        # coming from the server. As link is already initialized at this point, there is a high
        # chance that load requests from the server starts being received soon and break if
        # the manager doesn't exist yet.
        # TODO(Flor_Grosso): make the agent more robust to state manager. Consider moving this
        # block after the default modules are initialized to avoid blocking the agent on start
        # up (while not breaking for non-default modules).
        self.logger.info("Initializing module state manager")
        self._module_state_manager = ModuleStateManager(self._modules)
        self.logger.info("Module state manager initialized")
        for module_name in default_modules:
            loaded &= self.load_module(
                module_name,
                agentlet.RUNLEVEL_MINIMAL,
                # Avoid saving state to the cache as this is not coming from
                # the server
                save_state=False,
            )
        # Keep track of the modules that require ROS to work properly
        self._ros_dependent_modules = [RosAgentlet.__name__]
        # Check that connection to MQTT could be established
        if self._link.connected:
            # Send a request to the server to re-establish loaded modules if
            # any
            self._resend_modules()
        else:
            # If no connection is available so far, prepare to load modules
            # from cache in a few seconds
            # NOTE(diegobatt): Waiting a few seconds is to avoid unnecessarily
            # going through the cache if the connection was just not
            # established right away
            self.logger.info("Starting timer to load module states from cache")
            self._module_states_cache_timer = threading.Timer(
                MODULE_STATES_CACHE_LOAD_S, self._load_modules_cache
            )
            self._module_states_cache_timer.start()
            # Register the resend_modules command to be published as soon as
            # connection is re-established
            self._link.add_connection_listener(
                self._resend_modules, on="connect", subscriber="modules"
            )
        # 3. Create log file manager, not migrated yet
        # NOTE (Flor_Grosso): this reference is temporary, until we move the
        self.log_manager = LogManager(self._link)
        # 4. Start main loop
        # For sanity reasons, we only keep this running for REBOOT_TIME_S seconds
        # and then proactively reboot to clean-up ROS connections, etc.
        self.logger.info("Agent initialization complete")
        self.logger.info(f"Safety reboot timer set for {REBOOT_TIME_S} seconds")
        time.sleep(REBOOT_TIME_S)
        self.logger.info("Agent finishing to force system restart.")
        self.shutdown()
    def _load_modules_cache(self):
        """
        Load modules from cache under the mutex safety to avoid race conditions
        """
        with self._module_states_mutex:
            self._module_state_manager.load_module_states()
    def _resend_modules(self):
        """
        Clear the module states cache and ask the server to resend them
        NOTE(diegobatt): This should only be called if connection is known to be established
        """
        # If the cache timer exists, cancel it.
        if hasattr(self, "_module_states_cache_timer"):
            self.logger.info("Cancelling timer to load module states from cache")
            self._module_states_cache_timer.cancel()
        # As we will receive the modules from the server again, clear the cache
        with self._module_states_mutex:
            self._module_state_manager.clear_module_states()
        # Ask the server to resend modules
        self._link.publish("out_cmd", "resend_modules", qos=1)
    def shutdown(self, *args):
        """
        Called to shutdown agent by force.
        NOTE(diegobatt): Registered as an exit handler for SIGUSR1 signals, raised by other
        modules to shutdown agent
        """
        # If graceful stopping fails or takes to long, this timer will go off,
        # forcing exit
        threading.Timer(MINUTE_S, os._exit, args=(0,)).start()
        # Stop modules gracefully
        try:
            self._link.stop()
            self._module_state_manager.stop()
        except Exception as e:
            self.logger.warning("Failed to shutdown gracefully: %s", e)
        # Force exit
        # NOTE(diegobatt): This bypasses exit handlers but it is the only way to kill
        # non-daemonic threads.
        # TODO(diegobatt): Most of our threads (to not say all) should be daemonic.
        os._exit(0)
    def load_module(self, module_name, runlevel, save_state=True):
        """
        Loads a module, given its module name.
        NOTE: It should have been instantiated first (currently hardcoded)
        """
        with self._module_states_mutex:
            if module_name not in self._modules:
                self.logger.warning(f"LOAD: Module instance not found: {module_name}.")
                loaded = False
            # If the agent is running on a system with no ROS, avoid spamming the
            # log with failed load messages.
            elif self._ros_disabled and module_name in self._ros_dependent_modules:
                self.once_logger.warn(
                    f"load_{module_name}",
                    f"LOAD: Module '{module_name}' requires ROS. Skipping.",
                )
                loaded = False
            elif self._modules[module_name]._states["loaded"]:
                loaded = True
                if self._modules[module_name]._states["runlevel"] == runlevel:
                    self.logger.warning(
                        f"Module: {module_name} already loaded and at the requested "
                        f"runlevel: {runlevel}."
                    )
                else:
                    self.logger.info(f"Setting module: {module_name} to runlevel: {runlevel}.")
                    self._modules[module_name].set_runlevel(runlevel)
            else:
                loaded = self._modules[module_name].load(runlevel)
                loaded_status = "OK" if loaded else "FAILED"
                self.logger.info(
                    f"Loading module: {module_name} at runlevel: {runlevel} - {loaded_status}."
                )
            # Store module's state in a cache
            if save_state:
                self._module_state_manager.save_module_state(module_name)
        return loaded
    def unload_module(self, module_name, save_state=True):
        with self._module_states_mutex:
            if module_name not in self._modules:
                self.logger.warning(f"UNLOAD: Module instance not found: {module_name}.")
                unloaded = False
            elif not self._modules[module_name]._states["loaded"]:
                # Do not try to unload a ros dependent module
                if self._ros_disabled and module_name in self._ros_dependent_modules:
                    unloaded = True
                else:
                    self.logger.warning(
                        f"Attempt to unload module which wasn't loaded: {module_name}."
                    )
                    unloaded = True
            else:
                unloaded = self._modules[module_name].unload()
                unloaded_status = "OK" if unloaded else "FAILED"
                self.logger.info(f"Unloading module: {module_name} - {unloaded_status}.")
            # Store module's state in a cache
            if save_state:
                self._module_state_manager.save_module_state(module_name)
        return unloaded
    def _command_received(self, payload_bytes):
        """
        Called whenever an agent command is received.
        """
        parts = str(payload_bytes, "utf-8").split("|")
        # It's an update request
        if parts[0] == "update":
            # Create an update request file and die. The re-launch script
            # will perform the update before respawning.
            open(
                "%s/.update" % os.getenv("INORBIT_HOME", os.getenv("HOME") + "/.inorbit"), "w"
            ).close()
            # TODO(adamantivm) Use the main loop to exit cleanly instead of
            # here
            self.shutdown()
        elif parts[0] == "get_state":
            # It's a request to re-send the status update
            self.logger.info("Received request to re-send online status.")
            # Send online status.
            # This method is blocking so do it on a separate thread just in
            # case.
            threading.Thread(target=self._link.send_online_status).start()
        elif parts[0] == "load_module":
            module_name = parts[1]
            runlevel = int(parts[2]) if len(parts) > 2 else agentlet.RUNLEVEL_DEFAULT
            self.load_module(module_name, runlevel)
        elif parts[0] == "unload_module":
            self.unload_module(parts[1])
        elif parts[0] == "restart":
            # TODO(adamantivm) Use the main loop to exit cleanly instead of
            # here
            self.shutdown()
        elif parts[0] == "send_logfiles":
            self.log_manager.send_list_update()
        elif parts[0] == "upload_agent_log":
            self.log_manager.upload_log(parts[1])
        elif parts[0] == "resend_diagnostics_alerts":
            self._modules["AlertManagerAgentlet"]._trigger_alerts_resend()
    def _state_received(self, payload):
        """
        Called whenever an agent state is received.
        """
        self.logger.info(f"Setting state: {payload}")
        state_data = json.loads(payload)
        # Sanity checks
        if "module_name" not in state_data:
            self.logger.warning("Attempt to set a state with no module id. Skipping.")
            return
        if len(state_data) < 2:
            self.logger.warning("Attempt to set empty state. Skipping.")
            return
        module_name = state_data["module_name"]
        # Check if module exists and set state if so
        with self._module_states_mutex:
            if module_name in self._modules:
                self._modules[module_name].set_state(state_data)
                # Store module's state in a cache
                self._module_state_manager.save_module_state(module_name)
    def _get_state_options(self, payload):
        """
        Called by the server to request state options from a given module
        """
        self.logger.info(f"State options requested: {payload}")
        request = json.loads(payload)
        # TODO(adamnativm) Sanity checks
        module_name = request["module_name"]
        state_name = request["state_name"]
        if module_name not in self._modules:
            self.logger.warning(f"Attempt to get state options for unknown module: {module_name}.")
            return
        # TODO(admantivm) Support request for multiple state_names
        options = self._modules[module_name].get_state_options(state_name)
        options_msg = StateOptions()
        options_msg.state_name = state_name
        options_msg.values.extend(options)
        msg = ModuleStateOptionsMessage()
        msg.module_name = module_name
        msg.state_options.extend([options_msg])
        self._link.publish_protobuf("modules/state_options", msg, qos=1)
if __name__ == "__main__":
    print(f"Starting Agent. Version = {VERSION}.")
    agent = Agent()
    try:
        agent.start()
    except KeyboardInterrupt:
        print("Keyboard interrupt captured. Terminating InOrbit Agent.")
        try:
            sys.exit(0)
        except SystemExit:
            agent.shutdown()  # Calls os._exit(0)
````

## File: python3_requirements.txt
````
# Foxy, Humble and Iron agents specific (Python < 3.12)
setuptools==65.6.3; python_version < '3.12'
pyyaml==6.0; python_version < '3.12'
numpy==1.24.0; python_version < '3.12'
opencv-python==4.6.0.66; python_version < '3.12'
Pillow==9.3.0; python_version < '3.12'

# Python >= 3.12
setuptools==75.6.0; python_version >= '3.12'
pyyaml==6.0.2; python_version >= '3.12'
numpy==2.0; python_version >= '3.12'
opencv-python==4.10.0.84; python_version >= '3.12'

# Offline support
pycryptodomex==3.16
python-gnupg==0.5.2
````

## File: RELEASE.txt
````
4.22.0.ros2 :
- Adds support for ROS 2 Jazzy.

4.21.0.ros2:
- Publish key-value pairs on ROS Diagnostics messages.

4.20.0.ros2:
- Adds support for configurable namespaces for frame IDs.
- Adds support for compressed images.
- Adds support for ROS2 diagnostics.

4.19.1.ros2 :
- Adds support for ROS 2 Iron.

4.19.0.ros2:
- Adds new environment variable (INORBIT_ROBOT_NAME) that allows setting the robot name

4.18.3.ros2:
- Handles InvalidHandle exceptions on main ROSAgentlet thread while spinning and makes the agent to restart.

4.18.2.ros2:
- Fixes Waypoint Teleop command by setting the yaw angle specified by the user.

4.18.1.ros2:
- Removes unnecessary GPL libraries from agents.

4.18.0.ros2:
- GPS Agentlet: Adds support for GPSFix message type
- Added offset on pose messages to avoid losing precision on larger x and y coordinates

4.17.4.ros2:
- CustomDataAgentlet: make max bytes for key/values messages (MAX_BYTES_PER_KEY_VAL_MSG) support module_states.

4.17.3.ros2:
- Fixes GPS Agentlet when navsatfix topic is already set

4.17.2.ros2:
- Adds GPS Agentlet set_state missing method

4.17.1.ros2:
- Fixes with module state manager

4.17.0.ros2:
- Add new GPS Agentlet to support outdoor navigation
- Explicit import of Foxy and Humble library path. Fixes rclpy loading on Debian installation

4.16.0.ros2:
- Add support for Databag and Rosbag recording on ROS2.
- Fix RosAgentlet load when tf2 module import cannot be executed

4.15.1.ros2:
+ merged from 4.15.1:
  - Fix compact DeltaInt encoding option for publishing paths.
+ merged from 4.15.0:
  - Add compact DeltaInt encoding option for publishing paths.
+ merged from 4.14.0:
  - Make topic info module compatible for Python 2 and 3.
+ merged from 4.13.0:
  - Include frameId when publishing paths.
+ merged from 4.12.0:
  - Fix synchronization issues when receiving and publishing path data.
+ merged from 4.11.0:
  - Re-sync databags on mqtt reconnection if changes occurred since last full sync; allow disabling minimum full sync threshold via config.
+ merged from 4.10.0:
  - Monitor and update changes on databags properties.
+ merged from 4.9.0:
  - More robust agent initialization to avoid MQTT client deadlocks under certain conditions.
+ merged from 4.8.0:
  - Avoid writing stdout and stderr output to agent log, to prevent accidental disk flooding.
  - Be more defensive with module state manager to avoid blocking initialization.
  - Events agentlet throttles based on 10 second bucket measurements and is configurable now.
+ merged from 4.7.0:
  - Hotfixes to databags protocol; adds parameters to tune agentlet rates and limits.
  - [DISABLED] Adds HTTP traffic from rosbags uploads to reported InOrbit network traffic.
+ merged from 4.6.1:
  - Changes to S3 artifacts upload paths.
+ merged from 4.6.0:
  - Make ROS topic monitoring accept topic names with and without an initial '/'.
+ merged from 4.5.1:
  - Import Flask dependencies in load stage of APIAgentlet to restrict impact of future
    dependency breakages.
+ merged from 4.5.0:
  - CustomDataAgentlet: enable forcing a key for a given custom_field through module_states.
  - CustomDataAgentlet: read text files in tail mode by default.
+ merged from 4.4.0:
  - CustomDataAgentlet: fix crash when values contain non-ascii code. Try to decode it
  as utf-8 first and if the error persists avoid publishing the key/value.
+ merged from 4.3.0:
  - RosMapAgentlet: prevent uploading a map if a different map_topic is configured just before the map image is uploaded.
  This behavior can be configured through the map_prevent_outdated_upload module state.
+ merged from 4.2.0:
  - Allows choosing websockets as the MQTT transport via server configuration.
+ merged from 4.1.0:
  - Fix CustomData agentlet error that caused outages when a value's size exceeded MAX_BYTES_PER_KEY_VAL_MSG.
+ merged from 4.0.0:
  - Add pre-commit hooks to the agent and do a first linter pass on agent-main code.
  - Add more resilience to the state manager agentlet.
  - Add max_files parameter to the databags agentlet.

3.28.0.ros2:
- Add support for ROS 2 Humble.

3.27.0.ros2:
+ merged from master 3.27.0 :
 - Prevent log manager from publishing multiple updates through MQTT for the same log request.
+ merged from master 3.26.0:
  - Clean the environment by default before running an action script. This behavior is configurable
    through module states and can be set back to false if needed (as previous agent versions worked).
    If the environment is not cleaned, the action command  will be run with the environment the agent
    uses, which includes the agent specific python virtualenv and other settings which can interfere
    with the user's expected environment to run scripts.
  - CustomCommandsAgentlet: set script execution status as 'ABORTED' if return code is not zero.
+ merged from master 3.25.0:
  - Support for credentials late-binding
+ merged from master 3.24.0:
  - Support for offline mode and connection state handling
+ merged from master 3.23.0:
  - RosImageAgentlet: Added module states to adjust brightness and contrast per camera.
  - Allow specifying pre-installed credentials via .credentials or the file specified
    in the INORBIT_CONNECTION_CONFIG_FILE environment variable
  - Make RosMapAgentlet loaded by default on start, always
+ merged from master 3.22.0:
  - Added update flag for map messages that act as a map_id/frame_id correction
+ merged from master 3.21.0:
  - Implements a workaround to a publisher unregister rospy bug by avoiding publisher unregister
    on agentlet unloads. Can be overriden with an INORBIT_ROS_UNREGISTER_PUBLISHERS environment
    variable.
+ merged from master 3.20.0:
  - Allow configuring action scripts' path through the env var INORBIT_ACTIONS_PATH
+ merged from master 3.19.0:
  - Wait before sending the map message corrections when map_id or frame_id are coming from a topic
+ merged from master 3.18.0:
  - Added module states to mapAgentlet for multimap support
  - Added map_id concept and configuration logic
+ merged from master 3.17.1:
  - Fix MapAgentlet to set module states when not loaded yet
+ merged from master 3.17.0:
  - Created new MapAgentlet and moved map manipulation from localizationAgentlet to it
+ merged from master 3.15.1:
  - Fixed credentials reading in artifacts util
+ merged from master 3.15.0:
  - Addressed technical debt in logManager. Started using the Artifacts utils
+ merged from master 3.12.1:
  - Fix to skip proxy config if an empty HTTP_PROXY environment variable is present
  - Fix log file upload when INORBIT_HOME is missing

3.12.0.ros2:
- Fix log file util to fetch AWS credentials when the agent is installed on a custom path

3.9.0.ros2:
- Link: Fix on_message callback silent failures

3.3.0.ros2:
+ merged from master 3.8.0:
  - Update agent watchdog to fetch MQTT broker's IP address and port
  - Enable configuring agent's log level
+ merged from master 3.7.0:
  - Send maps in a separate thread to avoid blocking MQTT traffic
  - Add agent watchdog script.
+ merged from master 3.5.0:
  - Added support for agent custom install directory
  - Image module: resize camera images only when necessary
  - Choose explicit python interpreter for virtualenv creation
+ merged from master 3.4.0:
  - Use tls_version PROTOCOL_TLSv1_2

3.2.0.ros2:
+ merged from master 3.6.0:
  - Fix AWS credentials try-except message logging.

3.1.1.ros2:
- Fix logs manager module to be Python 3 compatible.

3.1.0.ros2:
- Fix costmap processing when occupancy grid's values are illegal (above 100).
- Enable logs manager module.
- RosLocalizationAgentlet: add default values for maps' and paths' rate limiting.
- RosLocalizationAgentlet: enable sending localization data even if the map wasn't published.
+ merged from master 3.2.1: 
  - Adds optional throttling to map messages (and fixes throttling for paths)

3.0.0.ros2:
- install.sh: use python3-virtualenv instead of python3-venv
+ merged from master 3.0.0: 
  - Added proxy support (Merged 'proxy' agent variant)

2.20.0.ros2:
- First ROS2 agent

2.19.0:
- RosLocalizationAgentlet: add 'cancel nav goal' simple action

2.18.0:
- adds PoseAgentlet for reporting poses independently from Localization agentlet

2.17.0:
- adds new RobotEventsAgentlet and support for events sampling mode for CustomDataAgentlet

2.16.0:
- start.sh: fetch install script for updates based on robotId

2.15.0:
- start.sh: write actual version and variant to log
- Image module: handle steps on camera data of type sensor_msgs/Image

2.14.0:
- CustomDataAgentlet: support 'regular' and 'diff' sampling modes

2.13.0:
- Force RosLocalizationAgentlet to update the cosmap after changing its runlevel

2.12.0:
- Pins docutils to 0.15.2 to avoid conflict with botocore

2.11.0:
- RosImageAgentlet: add support for CompressedImage messages

2.10.0:
- Fix a breakage with agent installation when running with virtualenv already installed with a predefined custom python interpreter.
- Support reading key-value pairs from a configurable topics
- CustomDataAgentlet: use 'custom_field' as the variable name when referring to data sources ids

2.9.0:
- Fix clear proper laser config name
- Add smart downsampling method for robot paths.

2.8.0:
- CustomCommandsAgentlet: send a status update when a script's execution has timed out.
- Don't send 'no message' if a diagnostics msg is empty, send an empty string instead.

2.7.0:
- RosLocalizationAgentlet: include 'frame_id' on initial pose message

2.6.0:
- Fix for data hash calculation on large maps that were sending an invalid hash when map
  data was included

2.5.0:
- Fix catch exceptions raised when no AWS credentials are configured
- Add support for '8UC1' image enconding
- Fix custom ROS path setting breaking on remote agent update

2.4.0:
- Discarded version, do not reuse this version number

2.3.1:
- Don't select the first available PoseArray subscriber for navigation path messages

2.3.0:
- New interface to send waypoints relative to robot or map

2.2.0:
- Detect failing mqtt connections and force an agent self-restart in
those cases.

2.1.0:
- Fix for cameras default topic setup.

2.0.1:
- Fix for python 2.7 setuptools version.

2.0.0:
- Fix ROS subscriptions for paths and camera images.

1.20.1:
- Don't send path message if there is no path data available

1.20.0:
- Support multiple robot paths

1.19.0:
- RosMonitoringAgentlet loading behavior moved to server config based
- Don't send available nodes, topics and params since they are unused
- New protocol for server-initiated request of module options, implemented for
  RosMonitoringAgentlet
- Add level flag for diagnostics protobuf messages

1.18.0:
- Make safety restart times higher than 24 hours and slightly randomize to avoid
  concurrent agent restarts for robots started at the same time.

1.17.0:
- Publish zero velocity when user stops sending teleop commands
- Fix diagnostics status ts to match publishing time

1.16.1:
- Fixed regression: diagnostics messages not being cleaned up after publishing

1.16.0:
- Subscribe MQTT topics using different QoS settings
- Compute and publish ROS Diagnostics status from RosDiagnosticsAgentlet
- Set QoS = 1 for alert messages
- Add command to force resending alerts

1.15.1:
- Make map data truncation time configurable with a default of 60 seconds and the
  possibility to be disabled by setting it to 0

1.15.0:
- Don't send large maps automatically, instead wait for the server to request it
- Provide topic name and map data hash with the map message
- Truncate map data for large maps to reduce allocated memory
- ROS diagnostics messages migrated to protobuf
- Fix broken installation for non-ROS installations, missing cv2 python module

1.14.0:
- Remove logging for new/cleared alerts

1.13.0:
- Use anonymous mode on ROS init to be more resilient to bad terminations
- Fixes linking of correct cv2 package when installing agent on any CPU architecture
- Updates python boto3 package version

1.12.0:
- Fixed regression: broken custom data agentlet when OpenCV is not available
- Fixed PIL image conversions: not using bgr format

1.11.0:
- Allows installation over a custom ROS distribution by providing ROS_CUSTOM_PATH env var.

1.10.0:
- Fixes CustomDataAgentlet to limit bytes_per_msg to MAX_BYTES_PER_BLOB in case of overflow.

1.9.0:
- Robustness updates to curl-based installer to allow systemd to restart in case of initial failure

1.8.0:
- Script installer automatically picks up custom ROS_MASTER_URI value if it exists when installing

1.7.0:
- Adds explicit dependency to gettext-base to Debian installer
- Fixes RosImageAgentlet to auto-configure properly if no settings provided
- Supports running concurrent script actions
- Updates maximum blob size for custom data text files to 20kB

1.6.0:
- Adds debconf parameter to disable using systemd during installation
- Adds debconf parameters to customize the userid to run agent as
- Enabled configuring read order and blob size for text files in
  CustomDataAgentlet.

1.5.1:
- Fix 1.5.0 regression: image module to encode camera images.

1.5.0:
- Enabled individual config for custom data of type image.

1.4.0:
- Fix alerts manager to clear alerts from previous sessions correctly after an
  agent restart.

1.3.0:
- Reverted selected mqtt version package install. Everyone gets 1.2.3 again
- Fixes restart timer for the case where multiple disconnects are received
- Make action scripts execution timeout configurable through a state

1.2.15:
- Updated python mqtt package installation to also install on 18.04

1.2.14:
- Updated python mqtt package installation depending on python version.
- Updated number of MQTT config retries from 10 to 25 and made it configurable
  through a INORBIT_CONFIG_RETRIES environment variable.
- Created a timer that restarts the agent if it wasn't able to connect
  to MQTT after a configurable number of seconds - defaults to 5 minutes.

1.2.13:
- Fix publishing rate calculation for RosImageAgentlet.

1.2.12:
- Updated binary architecture to 'all'.

1.2.11:
- Enabled camera's individual config.
- Added img resizing through states.

1.2.10:
- Allow binary installation in system location
- Add support for '32FC1' and '16UC1' image encondings.

1.2.9:
- Enabled multiple camera sources.

1.2.8:
- Added python-dev dependency check on installer.

1.2.7:
- Filter rosout messages by verbosity level.
- Updated SpatialAnnotationsAgentlet to receive module states.

1.2.6:
- Fixed pose update on map when reloading LocalizationAgentlet.
- Added SpatialAnnotationsAgentlet.

1.2.5:
- Updated installer script to provide a safe update.

1.2.4:
- Added Agent API.
- Fixed setting topics to record on RosbagAgentlet.
- Added PIL to enable Image Custom Data Source for systems with no ROS.

1.2.3.1:
- Fixed CustomDataAgentlet when handling paths.

1.2.3:
- Prevented agent from spamming when working without ROS.
- Enabled CustomCommandsAgentlet and CustomDataAgentlet to load without ROS.
- Updated script installation task in CustomCommandsAgentlet.
- Allowed configuring diagnostics topic and added support to read raw diagnostics.
- Send list of available keys per name on ROS Diagnostics message.

1.2.2:
- Updated installer for systems with no ROS
- ROS custom commands topic migrated to protobuf
- Add script execution action to custom commands

1.2.1:
- Fix uninstall script
- Throttling for rosout messages
- Created link to cv2 in virtual environment for ROS Melodic
- Log tf frame ids on start up
- Added RAM usage percentage to system stats
- Fix monitor for topics which haven't been published at config time
- Fix monitor updates for topics dropping their rate to 0

1.2.0.1:
- Make robot frame ID in localization configurable by state robot_frame
- Move several repeated logging calls to once_logger

1.2:
- Cap costmap updates to once per second to alleviate server performance
- Camera agentlet fix for Ubuntu 14.04
- Fix double laser support
- Add welcome message to agent installer

1.1.24.1:
- Experimental support for Ubuntu 18.04
- Fix image agentlet publish on default config

1.1.24:
- Support processing different image formats
- Reviewed QoS for agent messages

1.1.23.2:
- Fix 1.1.23 regression: send agent alerts
- Fix reporting of HDD usage to be in MB

1.1.23.1:
- Avoid blocking forever during agent initialization

1.1.23:
- Added ROS monitor agentlet
- Updated optional network interfaces configuration
- Updated installer error messages
- Updated agent log messages
- Enabled loading custom data agentlet without OpenCV

1.1.22:
- Added support to measure and publish per interface network stats
- Fixed measuring of disk usage per volume
- Added custom commands agentlet

1.1.21:
- Fixed ROS thread crash when occupancy grid is not set
- Removed battery information from system messages
- Added support to upload agent logs when requested
- Added support to measure and publish per volume disk usage

1.1.20:
- Fixed rosbag agentlet to perform a clean unload
- Added support for ROS diagnostics custom data type
- Made HDD robot vitals data source configurable
- Added support for dual camera streaming

1.1.19:
- Out-of-range odometry data is filtered out
- Added support for image custom data type
- Dynamic configuration of custom data sources

1.1.18:
- Support receiving agent states in JSON format
- Enabled reading battery information from ROS diagnostics
- Support multiple custom data elements simultaneously

1.1.17:
- Pick-up and use ROS_IP from environment during initial install

1.1.16:
- Enable sending a navigation goal when requested

1.1.15:
- Added support for text file custom data type

1.1.14:
- Added custom data agentlet

1.1.13:
- Map and costmap now encoded in Y+ up format

1.1.12:
- Laser and pose migrated to protobuf
- Laser points data downsampled
- Enable sending the list of logfiles in robot when requested
- Allow waking up any agentlet to switch runlevels faster

1.1.11:
- Camera images are sent in protobuf format
- Set camera topic as camera_id for protobuf messages
- Proper log rotation (1Mb each)

1.1.10:
- Prevent rosbag subprocess from logging stdout
- Fix set_pose configuration to set initialpose as default
- Add checks for ROS master status before updating publishers/subscribers
- Allow user to restart the agent
- Maps are sent in protobuf format

1.1.9.1:
- Prevent rosbag agentlet exception from killing agent
- Fix default path topic selection

1.1.9:
- Obtain AWS keys from server and use for rosbag upload
- Prevent teleoperating when odometry data is outdated
- Added ros navigation paths
- Teleop loaded by the server instead of the agent
- Added ros costmaps

1.1.8:
- Made set_pose topic configurable
- Stability fixes for ROS disconnection and MQTT initial connection
- Enabled camera agentlet to run at a higher runlevel

1.1.7:
- Hotfix for inorbit HDD usage calculation
- Added else scenario to publishing loops in various agentlets
- Compute and publish linear and angular velocities
- Added checks for rosbag manual deletion

1.1.6:
- Support upside-down laser
- Add hdd usage checks while recording rosbags
- Add timestamps to rosbags
- Changed logging initialization, moved to its own module
- Use TLS on Ubuntu 14.04
- Support secondary laser in localization module
- Fix for rosbag uploads
- Publish updates of space used by recorded rosbags

1.1.5:
- Fixed publishing of rosbag states
- Truncate logs on agent start-up
- Fixed camera agentlet import and reduced log spam
- Enable external setting of laser topic

1.1.4:
- Enable external setting of rosbags to record
- Unified state topic to single one
- Hack in start.sh to force find ROS .so find path
- Don't log more than one TF exception for each parent / child combination

1.1.3:
- Added camera topic persistence
- Generalized states publishing

1.1.2.3:
- Hotfix for CPU usage issue

1.1.2.2:
- Added another condition to disable tls on ubuntu trusty.

1.1.2.1:
- Added conditional tls for local testing.

1.1.2:
- Added S3 uploader util
- Added Alert manager
- Added Rosbag Agentlet
- Enabled tls for mqtt

1.1.1:
- Updated diagnostics agentlet to use runlevel 0 by default
- Protobuf serialization for the echo message.
- More detailed logging of state message publishing
- Include agent version and host name in mqtt_config call

1.1.0.1:
- Use QOS=1 for main agent status message

1.1.0:
- Updated installer messages
- Added checks for failed downloads on installer

1.0.10.1:
- Module state fixes for when the agent is restarted
- Fixed teleop agentlet to perform odometry checks

1.0.10:
- Added teleop agentlet
- Added odometry module
- Changed MQTT config request to server to use POST

1.0.9:
- Replaced cpu, hdd and network topics by single 'stats' message
- New protobuf serialization (used in system/stats)
- Echo service update to return agent timestamp in milliseconds
- Exclude localhost from network stats
- Publish storage used by inorbit in MB and percentage
- set_pose uses seq number to allow callbacks

1.0.8:
- Updated camera agentlet to find ROS topic to subscribe to
- Migrated topic finder to ROS agentlet
- Allow localization module to publish from different map topics
- Publish available map ROS topics

1.0.7:
- Fixed discrepancy between laser and robot in localization module
- Fixed camera agentlet to publish resized images
- Updated camera agentlet to publish grayscale images with maximum compression

1.0.6:
- Publish low-resolution, low-bandwidth RGB camera images
- Created link to cv2 in virtual environment for ROS Indigo

1.0.5:
- Publish HDD usage (from all the aggregated storage in the robot)

1.0.4:
- Initial version
````
