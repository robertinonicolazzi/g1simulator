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
