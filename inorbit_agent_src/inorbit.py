#!/usr/bin/env python
#
# Copyright (c) 2018, 2020, InOrbit, Inc.
# All rights reserved.
#
# InOrbit agent main entry point.
import ctypes
import json
import os
import random
import signal
import sys
import threading
import time

import inorbit.link
import inorbit.logger  # Import before other modules to set logging format
from inorbit import VERSION
from inorbit.agentlets import agentlet
from inorbit.agentlets.camera import RosImageAgentlet
from inorbit.agentlets.custom_commands import CustomCommandsAgentlet
from inorbit.agentlets.custom_data import CustomDataAgentlet
from inorbit.agentlets.databag import DatabagAgentlet
from inorbit.agentlets.diagnostics import RosDiagnosticsAgentlet
from inorbit.agentlets.events import RobotEventsAgentlet
from inorbit.agentlets.inorbit_pb2 import ModuleStateOptionsMessage
from inorbit.agentlets.inorbit_pb2 import StateOptions
from inorbit.agentlets.localization import RosLocalizationAgentlet
from inorbit.agentlets.map import RosMapAgentlet
from inorbit.agentlets.odometry import RosOdometryAgentlet
from inorbit.agentlets.pose import RosPoseAgentlet
from inorbit.agentlets.ros import RosAgentlet
from inorbit.agentlets.rosbag import RosbagAgentlet
from inorbit.agentlets.state_manager import ModuleStateManager
from inorbit.agentlets.system import SystemAgentlet
from inorbit.agentlets.teleop import RosTeleopAgentlet
from inorbit.agentlets.gps import GPSAgentlet
from inorbit.log_manager import LogManager
from mock import MagicMock
from util.once_logger import OnceLogger

# Attach signals to parent so as to make sure to die if the
# start script dies - necessary to get upstart to work
libc = ctypes.cdll["libc.so.6"]
libc.prctl(1, 15)  # SIGTERM
libc.prctl(1, 9)  # SIGKILL

# Time-related constants
MINUTE_S = 60
HOUR_S = 60 * MINUTE_S
# Time before programmed agent shutdown.
# Default: 24 hours plus a random value between 0 and 4 more hours
# TODO(adamantivm) Make this configurable
REBOOT_TIME_S = 24 * HOUR_S + random.randrange(4 * HOUR_S)
MODULE_STATES_CACHE_LOAD_S = 10


class Agent:
    def __init__(self):
        self.logger = inorbit.logger.getLog(__name__)
        self.once_logger = OnceLogger(self.logger)
        # Hash set of module instances
        self._modules = {}
        # Hash set of module class loading status
        self._module_status = {}
        # Flag that indicates whether the agent is running in a system with or
        # without ros
        self._ros_disabled = False
        # As modules shouldn't be loaded more than once, use a mutex to avoid
        # them being loaded at the same moment by different threads
        # NOTE(diegobatt): This could happened due to a very rare case were the
        # connection is re-established at the same time the modules are being
        # loaded from the cache, and it would be impossible to catch.
        # TODO(diegobatt): Make module's load idempotent as to avoid the need
        # of this mutex
        self._module_states_mutex = threading.Lock()
        # Register the agent's shutdown as the handler for SIGUSR1.
        # NOTE(diegobatt): This user-defined signal is raised in other modules
        # to force restart
        signal.signal(signal.SIGUSR1, self.shutdown)

    def _start_link(self):
        """
        Connects to InOrbit Cloud. If starting the link raises any exception,
        the agent is shutdown.
        """

        try:
            self._link.start()
        except Exception as e:
            self.logger.exception("Exception while initializing the " "connection")

    def start(self):
        # 1. Initialize and establish connection to InOrbit
        self._link = inorbit.link.Link()
        # Note that incoming commands should be guaranteed to arrive
        # (and exactly once, should not repeat!) QoS=2
        self._link.add_listener("in_cmd", self._command_received, 2)
        # Register listener for state updates commands
        # Module states should be guaranteed to arrive and we don't mind if
        # they arrive twice: QoS=1
        self._link.add_listener("modules/set_state", self._state_received, 1)
        self._link.add_listener("modules/get_state_options", self._get_state_options, 1)
        self.logger.info("Initializing MQTT connection...")
        # Start the link in a thread so it doesn't block
        threading.Thread(target=self._start_link).start()

        # 2. Create agentlets
        # TODO(adamantivm) Review and generalize this part
        ros_agentlet = RosAgentlet(self._link)
        self._modules[RosAgentlet.__name__] = ros_agentlet
        odometry_agentlet = RosOdometryAgentlet(self._link, ros_agentlet)
        self._modules[RosOdometryAgentlet.__name__] = odometry_agentlet
        robot_events_agentlet = RobotEventsAgentlet(self._link)
        diagnostics_agentlet = RosDiagnosticsAgentlet(self._link, ros_agentlet)
        self._modules[RosDiagnosticsAgentlet.__name__] = diagnostics_agentlet
        self._modules[RobotEventsAgentlet.__name__] = robot_events_agentlet
        custom_data_agentlet = CustomDataAgentlet(
            self._link, ros_agentlet, diagnostics_agentlet, robot_events_agentlet
        )
        self._modules[CustomDataAgentlet.__name__] = custom_data_agentlet

        system_agentlet = SystemAgentlet(self._link, diagnostics_agentlet)
        self._modules[SystemAgentlet.__name__] = system_agentlet
        # TODO(herchu)/ros2 RosoutAgentlet
        map_agentlet = self._modules[RosMapAgentlet.__name__] = RosMapAgentlet(
            self._link, ros_agentlet, custom_data_agentlet
        )
        pose_agentlet = self._modules[RosPoseAgentlet.__name__] = RosPoseAgentlet(
            self._link, ros_agentlet, map_agentlet
        )
        localization_agentlet = RosLocalizationAgentlet(
            self._link, ros_agentlet, pose_agentlet, map_agentlet
        )
        self._modules[RosLocalizationAgentlet.__name__] = localization_agentlet
        self._modules[RosImageAgentlet.__name__] = RosImageAgentlet(self._link, ros_agentlet)
        self._modules[RosTeleopAgentlet.__name__] = RosTeleopAgentlet(
            self._link, ros_agentlet, odometry_agentlet
        )
        # TODO(herchu)/ros2 AlertManagerAgentlet
        rosbag_agentlet = self._modules[RosbagAgentlet.__name__] = RosbagAgentlet(
            self._link, ros_agentlet
        )
        databag_agentlet = self._modules[DatabagAgentlet.__name__] = DatabagAgentlet(self._link)

        # TODO(Flor_Grosso)/ros2 Enable DatabagAgentlet
        # databag_agentlet = self._modules[DatabagAgentlet.__name__] = DatabagAgentlet(self._link)
        custom_data_agentlet = CustomDataAgentlet(
            self._link, ros_agentlet, diagnostics_agentlet, robot_events_agentlet
        )
        self._modules[CustomDataAgentlet.__name__] = custom_data_agentlet
        self._modules[CustomCommandsAgentlet.__name__] = CustomCommandsAgentlet(
            self._link, ros_agentlet
        )
        gps_agentlet = GPSAgentlet(self._link, ros_agentlet)
        self._modules[GPSAgentlet.__name__] = gps_agentlet
        # TODO(herchu)/ros2 RosMonitoringAgentlet
        # TODO(herchu)/ros2 APIAgentlet
        # TODO(herchu)/ros2 SpatialAnnotationsAgentlet

        # Register additional modules that perform networking with SystemAgentlet (in addition
        # to Link which is always tracked).
        # TODO(herchu) In the future, make this configurable
        system_agentlet.register_networking_module(rosbag_agentlet)
        system_agentlet.register_networking_module(databag_agentlet)

        # Load default modules at start
        default_modules = [
            SystemAgentlet.__name__,
            RosAgentlet.__name__,
            # NOTE(adamantivm) The Map agentlet is crucial for
            # ROS based systems so we enable it by default to
            # avoid any cloud config issues
            RosMapAgentlet.__name__,
            # Starting with 'offline' agent variant, the databag agentlet needs to be loaded
            # at all times (to send blackbox rosbags and clean up recorded bags)
            DatabagAgentlet.__name__,
        ]

        loaded = True

        # Initialize module state manager
        # NOTE(Flor_Grosso): the _module_state_manager needs to be up before loading any module
        # coming from the server. As link is already initialized at this point, there is a high
        # chance that load requests from the server starts being received soon and break if
        # the manager doesn't exist yet.
        # TODO(Flor_Grosso): make the agent more robust to state manager. Consider moving this
        # block after the default modules are initialized to avoid blocking the agent on start
        # up (while not breaking for non-default modules).
        self.logger.info("Initializing module state manager")
        self._module_state_manager = ModuleStateManager(self._modules)
        self.logger.info("Module state manager initialized")

        for module_name in default_modules:
            loaded &= self.load_module(
                module_name,
                agentlet.RUNLEVEL_MINIMAL,
                # Avoid saving state to the cache as this is not coming from
                # the server
                save_state=False,
            )

        # Keep track of the modules that require ROS to work properly
        self._ros_dependent_modules = [RosAgentlet.__name__]

        # Check that connection to MQTT could be established
        if self._link.connected:
            # Send a request to the server to re-establish loaded modules if
            # any
            self._resend_modules()
        else:
            # If no connection is available so far, prepare to load modules
            # from cache in a few seconds
            # NOTE(diegobatt): Waiting a few seconds is to avoid unnecessarily
            # going through the cache if the connection was just not
            # established right away
            self.logger.info("Starting timer to load module states from cache")
            self._module_states_cache_timer = threading.Timer(
                MODULE_STATES_CACHE_LOAD_S, self._load_modules_cache
            )
            self._module_states_cache_timer.start()
            # Register the resend_modules command to be published as soon as
            # connection is re-established
            self._link.add_connection_listener(
                self._resend_modules, on="connect", subscriber="modules"
            )

        # 3. Create log file manager, not migrated yet
        # NOTE (Flor_Grosso): this reference is temporary, until we move the
        self.log_manager = LogManager(self._link)

        # 4. Start main loop
        # For sanity reasons, we only keep this running for REBOOT_TIME_S seconds
        # and then proactively reboot to clean-up ROS connections, etc.
        self.logger.info("Agent initialization complete")
        self.logger.info(f"Safety reboot timer set for {REBOOT_TIME_S} seconds")
        time.sleep(REBOOT_TIME_S)
        self.logger.info("Agent finishing to force system restart.")
        self.shutdown()

    def _load_modules_cache(self):
        """
        Load modules from cache under the mutex safety to avoid race conditions
        """

        with self._module_states_mutex:
            self._module_state_manager.load_module_states()

    def _resend_modules(self):
        """
        Clear the module states cache and ask the server to resend them
        NOTE(diegobatt): This should only be called if connection is known to be established
        """

        # If the cache timer exists, cancel it.
        if hasattr(self, "_module_states_cache_timer"):
            self.logger.info("Cancelling timer to load module states from cache")
            self._module_states_cache_timer.cancel()
        # As we will receive the modules from the server again, clear the cache
        with self._module_states_mutex:
            self._module_state_manager.clear_module_states()
        # Ask the server to resend modules
        self._link.publish("out_cmd", "resend_modules", qos=1)

    def shutdown(self, *args):
        """
        Called to shutdown agent by force.
        NOTE(diegobatt): Registered as an exit handler for SIGUSR1 signals, raised by other
        modules to shutdown agent
        """

        # If graceful stopping fails or takes to long, this timer will go off,
        # forcing exit
        threading.Timer(MINUTE_S, os._exit, args=(0,)).start()
        # Stop modules gracefully
        try:
            self._link.stop()
            self._module_state_manager.stop()
        except Exception as e:
            self.logger.warning("Failed to shutdown gracefully: %s", e)
        # Force exit
        # NOTE(diegobatt): This bypasses exit handlers but it is the only way to kill
        # non-daemonic threads.
        # TODO(diegobatt): Most of our threads (to not say all) should be daemonic.
        os._exit(0)

    def load_module(self, module_name, runlevel, save_state=True):
        """
        Loads a module, given its module name.
        NOTE: It should have been instantiated first (currently hardcoded)
        """

        with self._module_states_mutex:
            if module_name not in self._modules:
                self.logger.warning(f"LOAD: Module instance not found: {module_name}.")
                loaded = False
            # If the agent is running on a system with no ROS, avoid spamming the
            # log with failed load messages.
            elif self._ros_disabled and module_name in self._ros_dependent_modules:
                self.once_logger.warn(
                    f"load_{module_name}",
                    f"LOAD: Module '{module_name}' requires ROS. Skipping.",
                )
                loaded = False
            elif self._modules[module_name]._states["loaded"]:
                loaded = True
                if self._modules[module_name]._states["runlevel"] == runlevel:
                    self.logger.warning(
                        f"Module: {module_name} already loaded and at the requested "
                        f"runlevel: {runlevel}."
                    )
                else:
                    self.logger.info(f"Setting module: {module_name} to runlevel: {runlevel}.")
                    self._modules[module_name].set_runlevel(runlevel)
            else:
                loaded = self._modules[module_name].load(runlevel)
                loaded_status = "OK" if loaded else "FAILED"
                self.logger.info(
                    f"Loading module: {module_name} at runlevel: {runlevel} - {loaded_status}."
                )
            # Store module's state in a cache
            if save_state:
                self._module_state_manager.save_module_state(module_name)
        return loaded

    def unload_module(self, module_name, save_state=True):
        with self._module_states_mutex:
            if module_name not in self._modules:
                self.logger.warning(f"UNLOAD: Module instance not found: {module_name}.")
                unloaded = False
            elif not self._modules[module_name]._states["loaded"]:
                # Do not try to unload a ros dependent module
                if self._ros_disabled and module_name in self._ros_dependent_modules:
                    unloaded = True
                else:
                    self.logger.warning(
                        f"Attempt to unload module which wasn't loaded: {module_name}."
                    )
                    unloaded = True
            else:
                unloaded = self._modules[module_name].unload()
                unloaded_status = "OK" if unloaded else "FAILED"
                self.logger.info(f"Unloading module: {module_name} - {unloaded_status}.")
            # Store module's state in a cache
            if save_state:
                self._module_state_manager.save_module_state(module_name)
        return unloaded

    def _command_received(self, payload_bytes):
        """
        Called whenever an agent command is received.
        """

        parts = str(payload_bytes, "utf-8").split("|")
        # It's an update request
        if parts[0] == "update":
            # Create an update request file and die. The re-launch script
            # will perform the update before respawning.
            open(
                "%s/.update" % os.getenv("INORBIT_HOME", os.getenv("HOME") + "/.inorbit"), "w"
            ).close()
            # TODO(adamantivm) Use the main loop to exit cleanly instead of
            # here
            self.shutdown()

        elif parts[0] == "get_state":
            # It's a request to re-send the status update
            self.logger.info("Received request to re-send online status.")
            # Send online status.
            # This method is blocking so do it on a separate thread just in
            # case.
            threading.Thread(target=self._link.send_online_status).start()

        elif parts[0] == "load_module":
            module_name = parts[1]
            runlevel = int(parts[2]) if len(parts) > 2 else agentlet.RUNLEVEL_DEFAULT
            self.load_module(module_name, runlevel)

        elif parts[0] == "unload_module":
            self.unload_module(parts[1])

        elif parts[0] == "restart":
            # TODO(adamantivm) Use the main loop to exit cleanly instead of
            # here
            self.shutdown()

        elif parts[0] == "send_logfiles":
            self.log_manager.send_list_update()

        elif parts[0] == "upload_agent_log":
            self.log_manager.upload_log(parts[1])

        elif parts[0] == "resend_diagnostics_alerts":
            self._modules["AlertManagerAgentlet"]._trigger_alerts_resend()

    def _state_received(self, payload):
        """
        Called whenever an agent state is received.
        """

        self.logger.info(f"Setting state: {payload}")

        state_data = json.loads(payload)
        # Sanity checks
        if "module_name" not in state_data:
            self.logger.warning("Attempt to set a state with no module id. Skipping.")
            return

        if len(state_data) < 2:
            self.logger.warning("Attempt to set empty state. Skipping.")
            return

        module_name = state_data["module_name"]
        # Check if module exists and set state if so
        with self._module_states_mutex:
            if module_name in self._modules:
                self._modules[module_name].set_state(state_data)
                # Store module's state in a cache
                self._module_state_manager.save_module_state(module_name)

    def _get_state_options(self, payload):
        """
        Called by the server to request state options from a given module
        """

        self.logger.info(f"State options requested: {payload}")
        request = json.loads(payload)

        # TODO(adamnativm) Sanity checks
        module_name = request["module_name"]
        state_name = request["state_name"]

        if module_name not in self._modules:
            self.logger.warning(f"Attempt to get state options for unknown module: {module_name}.")
            return

        # TODO(admantivm) Support request for multiple state_names
        options = self._modules[module_name].get_state_options(state_name)

        options_msg = StateOptions()
        options_msg.state_name = state_name
        options_msg.values.extend(options)

        msg = ModuleStateOptionsMessage()
        msg.module_name = module_name
        msg.state_options.extend([options_msg])

        self._link.publish_protobuf("modules/state_options", msg, qos=1)


if __name__ == "__main__":
    print(f"Starting Agent. Version = {VERSION}.")
    agent = Agent()
    try:
        agent.start()
    except KeyboardInterrupt:
        print("Keyboard interrupt captured. Terminating InOrbit Agent.")
        try:
            sys.exit(0)
        except SystemExit:
            agent.shutdown()  # Calls os._exit(0)
