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
