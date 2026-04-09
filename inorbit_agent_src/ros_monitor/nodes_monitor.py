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
