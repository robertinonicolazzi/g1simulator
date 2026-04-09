# Copyright (c) 2020, InOrbit, Inc.
# All rights reserved.
# Odometry agentlet.
# Depends on the ROS module.
#
# TODO:
# - Discover the possible frame IDs
# - Review data publishing for minimal runlevel
import math
import threading
import time

import transformations
from util.overrides import overrides

from .agentlet import Agentlet
from .agentlet import RUNLEVEL_DEFAULT
from .agentlet import RUNLEVEL_FULL
from .agentlet import RUNLEVEL_SILENT
from .inorbit_pb2 import OdometryDataMessage

ODOM_FRAME = "odom"
BASE_FRAME = "base_link"

ODOM_QUERY_RATE_HZ = 5

# Odometry publisher's period for different runlevels
# (seconds)
PUBLISHER_PERIOD_FULL_RUNLEVEL = 0.5
PUBLISHER_PERIOD_DEFAULT_RUNLEVEL = 1
PUBLISHER_PERIOD_MINIMAL_RUNLEVEL = 10

# Timeout to compute speed from acquired data (seconds)
SPEED_CALCULATION_TIMEOUT = 3

MQTT_ODOMETRY_TOPIC = "ros/odometry/data"
MQTT_STATE_TOPIC = "ros/odometry/state"

# Max acceptable speed to filter out invalid distances, in m/s
MAX_VALID_SPEED = 3.0


class RosOdometryAgentlet(Agentlet):
    def __init__(self, uplink, ros):
        super(RosOdometryAgentlet, self).__init__(uplink)
        self._ros = ros
        self._states["base_frame"] = self.robot_namespace + BASE_FRAME
        self._states["odom_frame"] = self.robot_namespace + ODOM_FRAME
        # Timestamp, in milliseconds, when the agentlet started
        # accumulating odometry. Access to this variable must be
        # guaranteed by _mutex
        self._ts_start = 0
        # Timestamp, in milliseconds, of the last time odometry
        # accumulator was updated. Access to this variable must
        # be guaranteed by _mutex
        self._ts = 0
        # Accumulated displacement (meters). Access to this
        # variable must be guaranteed by _mutex
        self._linear_distance = 0
        # Accumulated rotation (radians). Access to this
        # variable must be guaranteed by _mutex
        self._angular_distance = 0

        # Flag to compute and publish speed
        self._should_compute_speed = False

        # Odometry data needed to compute speed
        self._last_linear_distance = 0
        self._last_angular_distance = 0
        self._last_ts = 0

        self._initial_pose = None
        self._last_pose = None

        # Mutex used to access _ts_start, _ts, _linear_distance
        # and _angular_distance
        self._mutex = threading.Lock()

        # Odometry listener thread running state
        self._odom_listener_running = False

        # Uplink publisher thread running state
        self._odom_publisher_running = False

        # Condition to handle publisher's sleep/wake up timing
        self._condition = threading.Condition()

    @overrides(Agentlet)
    def load(self, runlevel):
        self._states["runlevel"] = runlevel

        self._ros.add_submodule("odometry")

        # Reset the initial pose
        self._initial_pose = self._get_robot_pose()

        # Save the start time
        self._ts_start = self.get_ts()
        self._last_ts = self._ts_start

        # Define if speed should be computed or not
        self._set_speed_status()

        # Start odometry listener thread
        threading.Thread(target=self._odom_listener).start()

        # Start uplink publishing thread if runlevel is other than silent
        if self.get_runlevel() != RUNLEVEL_SILENT:
            self._launch_publisher_thread()

        self._states["loaded"] = True

        # Send a state update to the cloud
        self.publish_state(self.uplink, self._states)

        return True

    @overrides(Agentlet)
    def unload(self):
        # Shutdown odometry listener thread
        self._odom_listener_running = False
        # Shutdown uplink publisher thread
        self._odom_publisher_running = False
        # Re-initialize exception reporting
        self.once_logger.reset_all()
        # Remove ROS subscribers
        self._ros.remove_submodule("odometry")
        self._states["loaded"] = False
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

        # Define if speed should be computed or not with the new runlevel
        self._set_speed_status()

        # If last runlevel is empty, do nothing
        if last_runlevel is None:
            return

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

    def get_odometry(self):
        """
        Returns odometry data as an array: [ts_start, ts, linear, angular].
        """

        self._mutex.acquire()
        ts_start = self._ts_start
        ts = self._ts
        linear = self._linear_distance
        angular = self._angular_distance
        self._mutex.release()

        return [ts_start, ts, linear, angular]

    def is_odometry_valid(self, odometry_data):
        """
        Checks odometry data as obtained with get_odometry() is valid.
        """

        # A valid odometry is a four-element array with non-empty values

        return (
            odometry_data is not None
            and len(odometry_data) == 4
            and odometry_data[0] is not None
            and odometry_data[1] is not None
            and odometry_data[2] is not None
            and odometry_data[3] is not None
        )

    def _is_distance_valid(self, distance, new_pose, ts_delta_ms):
        """
        Validates a traveled distance appears to be physically plausible. We have
        seen cases with 100s of m/s that we now just filter out.
        """

        # Avoid zero division. 1 ms is short enough to filter almost any
        # non-zero traveled distance if the two snapshots were too close
        if ts_delta_ms <= 0:
            ts_delta_ms = 1
        if distance / (ts_delta_ms * 0.001) > MAX_VALID_SPEED:
            # Print this error to logs for anomaly analysis (at most once)
            error_msg = (
                f"Odometry: Traveled distance is too far: {distance}m at "
                f"{distance / (ts_delta_ms * 0.001)}m/s. Resetting odometry state."
                f"\nPrev pose:\n{self._initial_pose}"
                f"\nNew pose:\n{new_pose}\n"
            )
            self.once_logger.error("odometry_out_of_range", error_msg)
            return False
        return True

    def set_state(self, state):
        """
        Called whenever a set_module command is received.
        """
        if "base_frame" in state:
            self._states["base_frame"] = state["base_frame"]

        if "odom_frame" in state:
            self._states["odom_frame"] = state["odom_frame"]

        # Send a state update
        self.publish_state(self.uplink, self._states)

    def _odom_listener(self):
        """
        Runs on a separate thread. Listens to odometry msgs and updates the
        traversed distance.

        TODO (Flor_Grosso): odom listener and distance computations have been
        taken from teleop module. Consider removing these methods from there.
        """

        self._odom_listener_running = True
        while self._odom_listener_running:
            # If the initial pose was not obtained correctly when loading
            # the module, retry getting it.
            if self._initial_pose is None:
                self._initial_pose = self._get_robot_pose()
                if self._ts_start == 0:  # Start ts was lost due to a reset
                    self._mutex.acquire()
                    self._ts_start = self.get_ts()
                    self._mutex.release()
            else:
                last_pose = self._get_robot_pose()
                if last_pose is not None:
                    self._update_traversed_distance(last_pose)

            time.sleep(1.0 / ODOM_QUERY_RATE_HZ)

    def _get_robot_pose(self, time=None):
        """
        Helper method to get the latest known robot pose.
        Returns None if the pose can't be found.
        """

        odom_T_robot = self._ros.lookup_transform(
            self._states["odom_frame"], self._states["base_frame"]
        )
        return odom_T_robot.transform if odom_T_robot is not None else None

    def _angular_distance_between(self, start_pose, end_pose):
        """
        Computes the angular distance between two poses.
        """

        # Get Yaw from quaternion for the start pose
        start_rotation = start_pose.rotation
        start_quaternion = (start_rotation.x, start_rotation.y, start_rotation.z, start_rotation.w)
        euler_start = transformations.euler_from_quaternion(start_quaternion)
        yaw_start = euler_start[2]

        # Get Yaw from quaternion for the end pose
        end_rotation = end_pose.rotation
        end_quaternion = (end_rotation.x, end_rotation.y, end_rotation.z, end_rotation.w)
        euler_end = transformations.euler_from_quaternion(end_quaternion)
        yaw_end = euler_end[2]

        return yaw_end - yaw_start

    def _linear_distance_squared_between(self, start_pose, end_pose):
        """
        Computes the linear distance (squared) between two poses.
        """

        # Get robot position
        p_start = start_pose.translation
        p_end = end_pose.translation

        delta_x = p_end.x - p_start.x
        delta_y = p_end.y - p_start.y

        return delta_x * delta_x + delta_y * delta_y

    def _linear_distance_between(self, start_pose, end_pose):
        """
        Computes the linear distance between two poses.
        """

        # Get squared distance first
        distance_squared = self._linear_distance_squared_between(start_pose, end_pose)
        return math.sqrt(distance_squared)

    def _update_traversed_distance(self, last_pose):
        """
        Updates the traversed distance with the last odometry data.
        """

        distance = self._linear_distance_between(self._initial_pose, last_pose)
        ts = self.get_ts()

        self._mutex.acquire()
        if self._is_distance_valid(distance, last_pose, ts - self._ts):
            self._ts = ts
            # Compute accumulated linear and angular distances
            self._linear_distance += distance
            # Computing absolute value as the angular distance sign
            # represents direction. Negative is clockwise rotation.
            self._angular_distance += abs(
                self._angular_distance_between(self._initial_pose, last_pose)
            )
            self._initial_pose = last_pose
        else:
            # Poses are too far and distance suggests there is something wrong.
            # Reset the state
            self._ts_start = 0
            self._ts = 0
            self._linear_distance = 0
            self._angular_distance = 0
            self._initial_pose = None

        self._mutex.release()

    def _compute_speed(self):
        """
        Computes the average linear and angular speed between consecutive calls.
        """

        self._mutex.acquire()
        current_linear_distance = self._linear_distance
        current_angular_distance = self._angular_distance
        current_ts = self._ts
        self._mutex.release()

        current_pose = self._get_robot_pose()

        # Compute time difference and convert it to seconds
        delta_ts = float(current_ts - self._last_ts) * 0.001

        # If last data is too old or elapsed time is zero, don't compute speed
        if delta_ts > SPEED_CALCULATION_TIMEOUT or delta_ts == 0:
            speed = None
        else:
            linear_speed = (current_linear_distance - self._last_linear_distance) / delta_ts
            angular_speed = (
                -self._angular_distance_between(self._last_pose, current_pose) / delta_ts
                if self._last_pose
                else 0
            )
            speed = [linear_speed, angular_speed]

        # Store initial values for the next call
        self._last_linear_distance = current_linear_distance
        self._last_angular_distance = current_angular_distance
        self._last_ts = current_ts
        self._last_pose = current_pose

        return speed

    def _set_speed_status(self):
        """
        Sets flag to compute speed only if current runlevel is FULL.
        """

        # If agentlet is loaded in full runlevel, compute and publish speed
        self._should_compute_speed = self.get_runlevel() == RUNLEVEL_FULL

    def _publish_loop(self):
        """
        Runs on a separate thread. Publishes odometry data at a rate dependent
        on the runlevel.
        """

        self._odom_publisher_running = True
        while self._odom_publisher_running:
            try:
                self._publish_odometry_if_available()
            except Exception as e:
                self.once_logger.exception("odometry_publish", "Exception publishing data.")

            self._condition.acquire()
            # Throttle differently depending on the module runlevel
            if self.get_runlevel() == RUNLEVEL_DEFAULT:
                self._condition.wait(PUBLISHER_PERIOD_DEFAULT_RUNLEVEL)
            elif self.get_runlevel() == RUNLEVEL_FULL:
                self._condition.wait(PUBLISHER_PERIOD_FULL_RUNLEVEL)
            # If the runlevel is not recognized, publish at minimum
            # TODO (Flor_Grosso): publish data together with the ROS master
            # status for minimal runlevel
            else:
                self._condition.wait(PUBLISHER_PERIOD_MINIMAL_RUNLEVEL)
            self._condition.release()

        self.logger.info("Publisher thread shutting down.")

    def _publish_odometry_if_available(self):
        """
        Publishes the latest update of the traversed distance if possible.
        """

        odometry = self.get_odometry()
        if odometry is not None:
            odom_data = OdometryDataMessage()
            odom_data.ts_start = odometry[0]
            odom_data.ts = odometry[1]
            odom_data.linear_distance = odometry[2]
            odom_data.angular_distance = odometry[3]
            odom_data.speed_available = False
            if self._should_compute_speed:
                speed = self._compute_speed()
                if speed is not None:
                    odom_data.linear_speed = speed[0]
                    odom_data.angular_speed = speed[1]
                    odom_data.speed_available = True
            self.uplink.publish_protobuf(MQTT_ODOMETRY_TOPIC, odom_data)

    def _launch_publisher_thread(self):
        """
        Starts uplink publishing thread.
        """

        threading.Thread(target=self._publish_loop).start()

    def _shutdown_publisher_thread(self):
        """
        Shuts down publisher thread by setting its state to not running.

        TODO (Flor_Grosso): Consider implementing a way to make sure that thread
        is killed properly.
        """

        self._odom_publisher_running = False
