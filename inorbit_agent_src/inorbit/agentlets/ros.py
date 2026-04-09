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
