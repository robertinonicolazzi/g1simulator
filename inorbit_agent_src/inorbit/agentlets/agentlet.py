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
