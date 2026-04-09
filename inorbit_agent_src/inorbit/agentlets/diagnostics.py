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
