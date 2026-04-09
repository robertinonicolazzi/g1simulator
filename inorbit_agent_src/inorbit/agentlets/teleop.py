# Copyright (c) 2020, InOrbit, Inc.
# All rights reserved.
# Teleop agentlet.
# Depends on the ROS module.
#
# TODO:
# - Serialize messages to cloud using protobuf
import threading
import time

from util.overrides import overrides

from .agentlet import Agentlet
from .inorbit_pb2 import TeleopGoCommand
from .ros import RosPublisher

MQTT_TELEOP_TOPIC_STEP = "ros/teleop/step"
MQTT_TELEOP_TOPIC_GO = "ros/teleop/go"
MQTT_STATE_TOPIC = "ros/teleop/state"
ROS_CMD_VEL_MSG_TYPE = "geometry_msgs/Twist"
ROS_CMD_VEL_TOPIC_DEFAULT = "cmd_vel"

# Direction values allowed
DIRECTION_UP = 0
DIRECTION_DOWN = 2
DIRECTION_LEFT = 1
DIRECTION_RIGHT = -1

# Default angular and linear velocities, in meters and radians per second
DEFAULT_LINEAR_VEL = 0.15
DEFAULT_ANGULAR_VEL = 0.3925  # 1/16th of a turn per second

# Maximum allowed values to be set
MAX_LINEAR_VEL = 0.5
MAX_ANGULAR_VEL = 0.785  # 1/8th of a turn per second

VEL_PUBLISH_RATE_HZ = 10
TELEOP_CMD_TIMEOUT_MS = 2000
MAX_PUBLICATIONS_PER_CMD = VEL_PUBLISH_RATE_HZ * TELEOP_CMD_TIMEOUT_MS / 1000

# Hack (Pisti) read _drive for how this variable is used
ZERO_VEL_THRESHOLD = 5

# Timeout for odometry data to be considered valid
ODOM_DATA_TIMEOUT_MS = 500

# Maximum linear distance traveled allowed per command sent
# (in meters)
MAX_LINEAR_DISTANCE_PER_CMD = 1.0
# Maximum angular distance allowed per command sent (in radians)
MAX_ANGULAR_DISTANCE_PER_CMD = 3.14


class RosTeleopAgentlet(Agentlet):
    def __init__(self, uplink, ros, odometry):
        super(RosTeleopAgentlet, self).__init__(uplink)
        self._ros = ros
        self._odom = odometry

        self._states["cmd_vel_topic"] = None
        self._states["angular_vel"] = DEFAULT_ANGULAR_VEL
        self._states["linear_vel"] = DEFAULT_LINEAR_VEL
        self._states["available_cmd_vel_topics"] = []
        self._states["publish_zero_vel"] = False
        self._states["zero_vel_threshold"] = ZERO_VEL_THRESHOLD

        self._direction = None
        self._publish_zero_vel = False
        # Access to this variable must be guaranteed by _mutex
        self._remaining_commands = 0
        # Access to this variable must be guaranteed by _mutex
        self._current_vel_command = None

        # Odometry data when the a step command is received
        self._initial_odometry = None

        # Mutex used to access _remaining_commands and _current_vel_command
        self._mutex = threading.Lock()

        # Publisher thread running state
        self._vel_publisher_running = False

        # ROS Publisher for velocity commands
        self._ros_cmd_vel_publisher = RosPublisher()

    @overrides(Agentlet)
    def load(self, runlevel):
        self._states["runlevel"] = runlevel

        try:
            import geometry_msgs
            import geometry_msgs.msg

            global geometry_msgs
        except Exception as e:
            self.once_logger.exception("geometry_msgs_load", "Exception loading geometry_msgs.")
            return False

        self._set_initial_teleop_topic()

        self._ros.add_submodule(
            "teleop",
            pubs=(
                (
                    self._states["cmd_vel_topic"],
                    geometry_msgs.msg.Twist,
                    self._ros_cmd_vel_publisher,
                ),
            ),
        )

        # Send a state update to the cloud
        self.publish_state(self.uplink, self._states)

        # Register for upstream incoming teleop STEP commands (step-by-step)
        self.uplink.add_listener(MQTT_TELEOP_TOPIC_STEP, self._set_vel)

        # Register for upstream incoming teleop GO commands (continuous teleop)
        self.uplink.add_listener(MQTT_TELEOP_TOPIC_GO, self._start_go)

        # Start velocity commands publishing thread
        threading.Thread(target=self._drive).start()

        self._states["loaded"] = True
        return True

    @overrides(Agentlet)
    def unload(self):
        # Shutdown publisher thread
        self._vel_publisher_running = False
        # Remove ROS subscribers
        self._ros.remove_submodule("teleop")
        self._states["loaded"] = False
        return True

    @overrides(Agentlet)
    def set_runlevel(self, runlevel):
        self._states["runlevel"] = runlevel

    def set_state(self, state):
        """
        Called whenever a set_module command is received.
        """

        if "cmd_vel_topic" in state.keys():
            if self._states["loaded"]:
                # Reset initial state
                new_pub = (
                    state["cmd_vel_topic"],
                    geometry_msgs.msg.Twist,
                    self._ros_cmd_vel_publisher,
                )

                self._ros.update_publisher_topic("teleop", self._states["cmd_vel_topic"], new_pub)

            self._states["cmd_vel_topic"] = state["cmd_vel_topic"]

        if "publish_zero_vel" in state.keys():
            self._states["publish_zero_vel"] = state["publish_zero_vel"]

        if "zero_vel_threshold" in state.keys():
            self._states["zero_vel_threshold"] = state["zero_vel_threshold"]

        self.set_float_state_value(state, "angular_vel", MAX_ANGULAR_VEL)
        self.set_float_state_value(state, "linear_vel", MAX_LINEAR_VEL)

        # Send a state update
        self.publish_state(self.uplink, self._states)

    def set_float_state_value(self, state, state_key, max_value):
        """
        Aux method to parse and set a float state and cap it to a max value.
        It will only set in case the value exists, can be parsed, is positive
        and less than the maximum.
        """

        if state_key in state.keys():
            value = state[state_key]
            try:
                float_value = float(value)
                if float_value < 0:
                    self.logger.warning(
                        f"Attempt to set state: {state_key} to negative " f"value: {float_value}."
                    )
                elif float_value > max_value:
                    self.logger.warning(
                        f"Attempt to set state: {state_key} to value: "
                        f"{float_value}, higher than the maximum allowed: "
                        f"{max_value}."
                    )
                else:
                    self._states[state_key] = float_value
            except Exception as e:
                self.logger.exception(f"Exception parsing state: {state_key}, value: {value}.")

    def _set_vel(self, payload):
        """
        Called from the server.
        Receives velocity step commands.
        """

        if self._ros_cmd_vel_publisher.pub is None:
            self.logger.warning("ROS publisher not set. Aborting.")
            return

        # Parse payload
        [seq, ts_hint, direction] = payload.split(b"|")
        # Estimated ts in milliseconds of the user's perception
        self._direction = int(direction)

        if not self.is_time_expired(int(ts_hint), TELEOP_CMD_TIMEOUT_MS):
            # Create Twist message
            twist_msg = self._create_empty_twist_msg()

            if self._direction == DIRECTION_UP:
                twist_msg.linear.x = self._states["linear_vel"]
            elif self._direction == DIRECTION_DOWN:
                twist_msg.linear.x = -1 * self._states["linear_vel"]
            elif self._direction == DIRECTION_LEFT:
                twist_msg.angular.z = self._states["angular_vel"]
            elif self._direction == DIRECTION_RIGHT:
                twist_msg.angular.z = -1 * self._states["angular_vel"]

            self._initial_odometry = self._odom.get_odometry()
            self._mutex.acquire()
            self._current_vel_command = twist_msg
            self._remaining_commands = MAX_PUBLICATIONS_PER_CMD
            self._mutex.release()

    def _start_go(self, payload):
        """
        Called from the server.
        Receives velocity GO commands with velocity information
        """

        if self._ros_cmd_vel_publisher.pub is None:
            self.logger.warning("ROS publisher not set. Aborting.")
            return

        # Parse payload
        try:
            message = TeleopGoCommand()
            message.ParseFromString(payload)
            self.linear_vel_command = float(message.linear_velocity)
            self.angular_vel_command = float(message.angular_velocity)
        except Exception as e:
            self.logger.warning("Failed to parse TeleopGoCommand")
            return

        # Estimated ts in millisecs of the user's perception
        if not self.is_time_expired(message.ts_hint, TELEOP_CMD_TIMEOUT_MS):
            # Create Twist message
            twist_msg = self._create_empty_twist_msg()
            twist_msg.linear.x = self.linear_vel_command * self._states["linear_vel"]
            twist_msg.angular.z = self.angular_vel_command * self._states["angular_vel"]

            self._initial_odometry = self._odom.get_odometry()
            self._mutex.acquire()
            self._current_vel_command = twist_msg
            self._remaining_commands = 1
            self._mutex.release()

    def _create_empty_twist_msg(self):
        twist_msg = geometry_msgs.msg.Twist()
        twist_msg.linear.x = 0.0
        twist_msg.linear.y = 0.0
        twist_msg.linear.z = 0.0
        twist_msg.angular.x = 0.0
        twist_msg.angular.y = 0.0
        twist_msg.angular.z = 0.0

        return twist_msg

    def _drive(self):
        """
        Runs on a separate thread. Publishes velocity commands at a fixed rate.
        """

        self._vel_publisher_running = True
        _zero_vel_count = 0

        while self._vel_publisher_running:
            self._mutex.acquire()
            local_command = self._current_vel_command
            local_counter = self._remaining_commands
            send_zero_vel = self._publish_zero_vel
            self._mutex.release()

            if self._states["publish_zero_vel"] is True:
                # HACK (Pisti) There are scenarios in which a robot might not
                # automatically stop moving when we stop publishing velocity commands
                # due to the robot missing a check for velocity command publishing
                # this results in robots being "stuck" with the last velocity message
                # The fix below attempts to prevent the robot being "stuck" with the latest
                # teleop command by publishing a (0,0,0) velocity command
                if local_counter == 0 and _zero_vel_count < self._states["zero_vel_threshold"]:
                    _zero_vel_count += 1
                elif _zero_vel_count == self._states["zero_vel_threshold"] and send_zero_vel:
                    self._publish_vel(self._create_empty_twist_msg())
                    self._publish_zero_vel = self._set_zero_vel_flag(False)

            if local_counter > 0:
                if self._should_halt_movement():
                    self._mutex.acquire()
                    self._remaining_commands = 0
                    self._mutex.release()
                    self.logger.info("Halting current teleoperation.")
                else:
                    try:
                        self._publish_vel(local_command)
                        _zero_vel_count = 0
                        self._mutex.acquire()
                        # Do not decrease remaining commands if a new command
                        # has arrived
                        if local_counter == self._remaining_commands:
                            self._remaining_commands -= 1
                        if self._remaining_commands <= 0:
                            # Change flag if state allows it
                            self._publish_zero_vel = self._set_zero_vel_flag(True)
                        self._mutex.release()
                    except Exception as e:
                        self.logger.exception("Exception publishing data.")

            time.sleep(1.0 / VEL_PUBLISH_RATE_HZ)

        self.logger.info("Publisher thread shutting down.")

    def _publish_vel(self, msg):
        """
        Publishes the latest velocity command if possible.
        """

        self._ros_cmd_vel_publisher.pub.publish(msg)

    def _set_zero_vel_flag(self, bool):
        """
        Shorthand function for checking a state before setting a value.
        """

        if self._states["publish_zero_vel"] is False:
            return False
        else:
            return bool

    def _should_halt_movement(self):
        """
        Checks if the movement should be halted based on odometry feedback.
        """

        # Get initial and last odometry, and check that they both have
        # a valid value
        initial_odometry = self._initial_odometry
        last_odometry = self._odom.get_odometry()

        # Don't allow teleoperations if odometry data is not valid
        if not self._odom.is_odometry_valid(initial_odometry) and not self._odom.is_odometry_valid(
            last_odometry
        ):
            self.logger.warning("Odometry data is not valid.")
            return True

        # Don't allow teleoperations if last odometry data is outdated
        if self._is_odometry_outdated(last_odometry):
            self.logger.warning("Odometry data is outdated.")
            return True

        # If odometry readings are valid, then check if the distance
        # traveled exceeds limits.
        if self._distance_traveled_exceeds_max(initial_odometry, last_odometry):
            self.logger.warning("Robot movement is not as planned.")
            return True

        return False

    def _is_odometry_outdated(self, odom_data):
        """
        Checks if the data received from the odometry module is outdated.
        """

        ts = odom_data[1]
        current_time = self.get_ts()
        # follow the same pattern with a this.newCostmap flag which is updated in
        # componentWillUpdate
        return current_time - ts > ODOM_DATA_TIMEOUT_MS

    def _distance_traveled_exceeds_max(self, initial_odometry, last_odometry):
        """
        Checks if the distance traveled by the robot has exceeded either
        the linear or angular limits.
        """

        # Odometry is a four element array composed of
        # [ts_start, ts, linear_distance, angular_distance]
        initial_linear_distance = initial_odometry[2]
        initial_angular_distance = initial_odometry[3]
        last_linear_distance = last_odometry[2]
        last_angular_distance = last_odometry[3]

        # Check if traversed distances exceed limits
        linear_distance_over_limit = (
            last_linear_distance - initial_linear_distance > MAX_LINEAR_DISTANCE_PER_CMD
        )

        angular_distance_over_limit = (
            last_angular_distance - initial_angular_distance > MAX_ANGULAR_DISTANCE_PER_CMD
        )

        return linear_distance_over_limit or angular_distance_over_limit

    def _get_available_cmd_vel_topics(self):
        """
        Finds topics publishing ROS_CMD_VEL_MSG_TYPE, populate and sort
        available_cmd_vel_topics list.
        """

        cmd_vel_topics = self._ros.get_topics_to_publish(ROS_CMD_VEL_MSG_TYPE)

        # If cmd vel topics were found, then sort the list leaving "cmd_vel"
        # topic above.
        if cmd_vel_topics:
            cmd_vel_topics = sorted(
                cmd_vel_topics, key=lambda x: (x != ROS_CMD_VEL_TOPIC_DEFAULT, x)
            )
        return cmd_vel_topics

    def _set_initial_teleop_topic(self):
        """
        Sets the initial teleop topic for when the agentlet is first loaded.
        """

        self._states["available_cmd_vel_topics"] = self._get_available_cmd_vel_topics()
        if not self._states["cmd_vel_topic"]:
            if not self._states["available_cmd_vel_topics"]:
                self.logger.warning(
                    "No ROS cmd topics available. "
                    f"Setting '{ROS_CMD_VEL_TOPIC_DEFAULT}' as default."
                )
                self._states["cmd_vel_topic"] = ROS_CMD_VEL_TOPIC_DEFAULT
            else:
                # If there are cmd topics, set the first on the list as the
                # current one.
                self._states["cmd_vel_topic"] = self._states["available_cmd_vel_topics"][0]
