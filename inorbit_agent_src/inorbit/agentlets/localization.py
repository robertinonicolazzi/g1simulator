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
