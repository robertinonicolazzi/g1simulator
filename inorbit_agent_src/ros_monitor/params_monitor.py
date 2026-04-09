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
