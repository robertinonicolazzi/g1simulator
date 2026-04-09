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
