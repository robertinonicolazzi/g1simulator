# Copyright (c) 2020, InOrbit, Inc.
# All rights reserved.
#
# Pose agentlet: Responsible for calculating poses (transformations between frames).
# It depends on the ROS module, and acts as helper for Localization agentlet.
#
# The Localization agentlet interacts with this agentlet by notifying each
# change in runlevel or loaded/unloaded state. When the LocalizationAgentlet
# is known to be publishing (which includes pose AND lasers, in addition to
# other data like path or map) this agentlet will decide to stop publishing
# so that data is not sent duplicated in 2 channels, with also the chance of
# ending up with slightly unsynchronized poses and lasers
#
#
# TODO(herchu) Support multiple poses / frames. The protobuf format allows it;
#   already; it may require adding one additional field (see .proto file),
#   but the code for now sends only one pose.
#
#
import threading

from util.overrides import overrides
from util.math_util import get_position_with_offset

from .agentlet import Agentlet
from .agentlet import RUNLEVEL_DEFAULT
from .agentlet import RUNLEVEL_FULL
from .agentlet import RUNLEVEL_SILENT
from .inorbit_pb2 import PoseMessage
from .inorbit_pb2 import PoseMessageData

# Default frame names for pose calculation
ROBOT_FRAME = "base_link"

# Publisher's period for different runlevels (seconds)
# Note that DEFAULT and MINIMAL need to meet the values
PUBLISHER_PERIOD_FULL_RUNLEVEL = 1
PUBLISHER_PERIOD_DEFAULT_RUNLEVEL = 10
PUBLISHER_PERIOD_MINIMAL_RUNLEVEL = 60

#
MIN_POSE_REPORT_THRESHOLD_SEC = 60

# MQTT pose topic is not shared with Localization agentlet; it only sends
# pose data. Since both topics' data overlap, they should not be published
# at the same time.
MQTT_POSE_TOPIC = "ros/loc/pose"


class RosPoseAgentlet(Agentlet):
    def __init__(self, uplink, ros, map_agentlet):
        super(RosPoseAgentlet, self).__init__(uplink)
        self._ros = ros
        self._map_agentlet = map_agentlet

        self._states["robot_frame"] = self.robot_namespace + ROBOT_FRAME

        # Condition to handle publisher's sleep/wake up timing
        self._condition = threading.Condition()

        # Mutex for controlling publisher thread variables and create or stop
        # threads. Controlled variables:
        #  - _publisher_thread
        #  - _publisher_period_sec_effective
        self._control_mutex = threading.Lock()

        # Sleep period for the publisher thread. This combines this agentlet
        # runlevel with an "override" from LocalizationAgentlet (its
        # _publisher_period_sec_override) into the _publisher_period_sec_effective.
        # The publisher thread (when it exists) will publish data only if
        # _publisher_period_sec_effective > 0. See _apply_runlevel()
        self._publisher_period_sec_override = 0
        self._publisher_period_sec_effective = 0
        self._publisher_thread = None

    @overrides(Agentlet)
    def load(self, runlevel):
        try:
            import transformations

            global transformations
        except Exception as e:
            self.once_logger.exception(
                "tf_transformations_load", "Exception loading tf.transformations."
            )
            return False

        self._states["runlevel"] = runlevel
        self._states["loaded"] = True

        # Start uplink publishing thread if runlevel is other than silent
        if self.get_runlevel() != RUNLEVEL_SILENT:
            self._apply_runlevel()

        # Send a state update to the cloud
        self.publish_state(self.uplink, self._states)
        return True

    @overrides(Agentlet)
    def unload(self):
        # Re-initialize exception reporting
        self.once_logger.reset_all()
        # Shutdown uplink publisher thread
        self._shutdown_publisher_thread()
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

        # If old and new runlevels match, do nothing
        if runlevel == self.get_runlevel():
            return

        # Update runlevel
        self._states["runlevel"] = runlevel

        # Make this runlevel effective - see comment in that method
        self._apply_runlevel()

    def _apply_runlevel(self):
        """
        Combines the requested runlevel with
        the runlevel obtained by "override modules" (LocalizationAgentlet) to
        come up with the "effective" run level this agentlet should run.
        We do this since if LocalizationAgentlet is already reporting poses
        (loaded and runlevel > 0) then this module should not be repeating the
        same job.
        """
        # Current effective publish rate
        with self._control_mutex:
            last_publisher_period = self._publisher_period_sec_effective

        # Requested publish rate defined by runlevel (not taking into account
        # the LocalizationAgentlet)
        requested_publisher_period = self._runlevel_to_publisher_period(self.get_runlevel())

        if (
            self._publisher_period_sec_override > 0
            and self._publisher_period_sec_override <= requested_publisher_period
        ):
            # Localization module is already reporting data at a greater
            next_publisher_period = 0
        else:
            # else: LocalizationAgentlet is not publishing, or is publishing
            # at a low rate (while this module is requested to publish at
            # higher rate
            next_publisher_period = requested_publisher_period

        # If previously active and no longer publishing, stop thread
        if next_publisher_period == 0 and last_publisher_period > 0:
            self._shutdown_publisher_thread()

        # Conversely, if not publishing earlier and starting to publish
        # now, start a thread
        elif next_publisher_period > 0 and last_publisher_period == 0:
            self._launch_publisher_thread(next_publisher_period)

        # else, if publish period is different (and nonzero), just
        # wake up the thread so it shanges it publish frequency
        elif next_publisher_period != last_publisher_period:
            with self._control_mutex:
                self._publisher_period_sec_effective = next_publisher_period
            self.wake_up_publisher(self._condition)

        # The only remaining case: next_publisher_period == last_publisher_period,
        # there no changes ar eneeded

    def inform_override(self, loaded, period_secs):
        """
        Called from LocalizationAgenlet, to inform its runlevel and loaded state.
        To avoid sending poses through 2 different channels, it may decide to
        stop publishing (done in _apply_runlevel).
        """

        self._publisher_period_sec_override = period_secs if loaded else 0
        self._apply_runlevel()

    def set_state(self, state):
        """
        Called whenever a set_module command is received.
        """

        if "robot_frame" in state:
            self._states["robot_frame"] = state["robot_frame"]

        # Send a state update
        self.publish_state(self.uplink, self._states)

    def _get_robot_pose(self, time=None):
        """
        Helper method to get the latest known robot pose.
        Returns None if the pose can't be found.
        """

        map_frame = self._map_agentlet.map_frame_id
        # TODO(herchu) Cache last pose lookup and return it again if recent
        map_T_robot = self._ros.lookup_transform(map_frame, self._states["robot_frame"])
        ts = self.get_ts()
        return (map_T_robot.transform if map_T_robot is not None else None, ts)

    def _publish_loop(self):
        """
        Runs on a separate thread. Publishes data at a rate depending on runlevel.
        """

        while self._publisher_period_sec_effective > 0:
            try:
                self._publish_pose_if_available()
            except Exception as e:
                self.once_logger.exception("pose_publish", "Exception publishing data.")

            # Throttle differently depending on the module runlevel
            self._condition.acquire()
            self._condition.wait(self._publisher_period_sec_effective)
            self._condition.release()

        self.logger.info("Publisher thread shutting down.")

    def _runlevel_to_publisher_period(self, runlevel):
        """
        Returns the publish frequency (or rather: period, in seconds) associated
        to a runlevel. For now only runlevel DEFAULT is specified; while any
        other would trigger a MINIMAL report rate.
        """

        if runlevel == RUNLEVEL_SILENT:
            # 0: do not publish
            return 0
        if runlevel == RUNLEVEL_DEFAULT:
            return PUBLISHER_PERIOD_DEFAULT_RUNLEVEL
        if runlevel == RUNLEVEL_FULL:
            return PUBLISHER_PERIOD_FULL_RUNLEVEL
        # If the runlevel is not recognized, publish at minimum
        else:
            return PUBLISHER_PERIOD_MINIMAL_RUNLEVEL

    def _publish_pose_if_available(self):
        """
        Publishes the latest pose update.
        """

        (robot_pose, ts_msec) = self._get_robot_pose()
        if robot_pose is not None:
            # Get robot position and orientation
            p = robot_pose.translation
            # Get Yaw from quaternion
            o = robot_pose.rotation
            q = (o.x, o.y, o.z, o.w)
            euler = transformations.euler_from_quaternion(q)
            yaw = euler[2]

            # Create the pose message. For now we are sending ONE pose, but the
            # message has support for sending multiple poses eventually.
            # Note that when this is done, an additional field should identify
            # what each pose represents.
            pose_with_offset = get_position_with_offset(p.x, p.y)
            data = PoseMessageData()
            data.ts = ts_msec
            data.pos_x = pose_with_offset['pos_x']
            data.pos_y = pose_with_offset['pos_y']
            data.offset_x = pose_with_offset['offset_x']
            data.offset_y = pose_with_offset['offset_y']
            data.yaw = yaw
            data.frame_id = self._map_agentlet.inorbit_frame_id
            message = PoseMessage()
            message.poses.extend([data])
            self.uplink.publish_protobuf(MQTT_POSE_TOPIC, message)

    def _launch_publisher_thread(self, publish_period):
        """
        Starts uplink publishing thread.
        """

        with self._control_mutex:
            # in case another concurrent call to _launch_publisher_thread()
            if self._publisher_thread is None:
                self._publisher_period_sec_effective = publish_period
                self._publisher_thread = threading.Thread(
                    target=self._publish_loop, name="RosPose-publisher"
                )
                self._publisher_thread.start()

    def _shutdown_publisher_thread(self):
        """
        Shuts down publisher thread by setting its state to not running.
        """

        with self._control_mutex:
            if self._publisher_thread is not None:
                self._publisher_period_sec_effective = 0
                self.wake_up_publisher(self._condition)
                # The next join() should always succeed immediately as
                # _publisher_period_sec_effective is 0, but give it a timeout
                # of 1 sec anyway
                self._publisher_thread.join(1)
                self._publisher_thread = None
