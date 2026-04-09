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
