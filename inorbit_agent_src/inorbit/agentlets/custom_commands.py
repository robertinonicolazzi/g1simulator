# Copyright (c) 2018, InOrbit, Inc.
# All rights reserved.
# Custom Commands agentlet. Receives command requests from the cloud, processes
# and redirects them to the robot.
#
# Cloud "publish to ROS topic" actions: the platform sends CustomCommandRosMessage on MQTT
# subtopic custom_command/ros; this agentlet publishes std_msgs/String on ROS_CUSTOM_COMMAND_TOPIC.
# Requirements for that path to work end-to-end:
#   1. std_msgs must import successfully in load(); otherwise _ros_commands_enabled is False and
#      no ROS publisher/subscriber is registered for custom commands.
#   2. The ROS agentlet must be up and have run add_submodule for custom_commands so the publisher
#      exists (_ros_custom_cmd_publisher.pub is not None).
#   3. Some node on the robot must subscribe to ROS_CUSTOM_COMMAND_TOPIC; the agent only publishes
#      (e.g. your stack or `ros2 topic echo` as a consumer).
import os
import threading
from rclpy.node import Node
from std_msgs.msg import String

from inorbit import INORBIT_ACTIONS_PATH_DEFAULT
from util.overrides import overrides
from util.robot_script_action import RobotScriptAction
from util.robot_script_action import STATUS_ABORTED
from util.robot_script_action import STATUS_FINISHED
from util.robot_script_action import STATUS_INSTALLED
from util.robot_script_action import STATUS_NOT_INSTALLED

from .agentlet import Agentlet
from .inorbit_pb2 import CustomCommandRosMessage
from .inorbit_pb2 import CustomScriptCommandMessage
from .inorbit_pb2 import CustomScriptStatusMessage

MQTT_CUSTOM_ROS_COMMAND_TOPIC = "custom_command/ros"
MQTT_CUSTOM_SCRIPTS_TOPIC = "custom_command/script/command"
MQTT_SCRIPT_OUTPUT_TOPIC = "custom_command/script/status"
ROS_CUSTOM_COMMAND_TOPIC = "inorbit/custom_command"
# Timeout for script execution
SCRIPT_TIMEOUT_SEC = 30
# Maximum character len of stdout and stderr in CustomScriptStatusMessage. If
# they are longer, they are truncated to the last SCRIPT_OUTPUT_LEN characters.
SCRIPT_OUTPUT_LEN = 2048

DEFAULT_MAX_PARALLEL_EXECS = 3

# Possible values for concurrent execution (always enabled, disabled,
# disables for the same combination of filename + args)
CONCURRENT_EXEC_ENABLED = "enabled"
CONCURRENT_EXEC_DISABLED = "disabled"
CONCURRENT_EXEC_DISABLED_FOR_SAME_ARGS = "disabled_for_same_args"

# Disable running the same script more than once at the same time.
DEFAULT_CONCURRENT_EXEC = CONCURRENT_EXEC_DISABLED

# Supported options to configure concurrent executions state.
CONCURRENT_EXEC_OPTIONS = [
    CONCURRENT_EXEC_ENABLED,
    CONCURRENT_EXEC_DISABLED,
    CONCURRENT_EXEC_DISABLED_FOR_SAME_ARGS,
]

# ATTENTION!! AGENT VERSION 3.26.
# Flag to indicate whether to run script actions on a clean environment or not.
# Agent versions prior to 3.26 had this set to False by default. However, if
# the environment is not cleaned, the script will be run with the environment
# that the agent is using. This includes the agent's specific python virtualenv
# and other environment variable settings which can interfere with the user's
# expected environment to run scripts (which is most likely the robot's env).
# For this reason, starting on agent version 3.26 we will clean the environment
# before running a user script, unless this is specifically disabled.
CLEAN_ENV = True


class CustomCommandsAgentlet(Agentlet):
    def __init__(self, uplink, ros_agentlet):
        super(CustomCommandsAgentlet, self).__init__(uplink)
        self._available_custom_scripts = []
        self._ros_agentlet = ros_agentlet
        # Set default values for states
        self._states["script_execution_timeout"] = SCRIPT_TIMEOUT_SEC
        self._states["script_max_parallel_execs"] = DEFAULT_MAX_PARALLEL_EXECS
        self._states["script_concurrent_execs"] = DEFAULT_CONCURRENT_EXEC
        self._states["clean_env"] = CLEAN_ENV

         # Flag to indicate if ros commands will be supported or not.
        self._ros_commands_enabled = True

        try:
            from .ros import RosPublisher

            # ROS Publisher for custom commands
            self._ros_custom_cmd_publisher = RosPublisher()
        except Exception:
            self.once_logger.exception(
                "Can't support ROS custom commands. " "Exception loading RosPublisher.",
                "ros_publisher_import",
            )
            self._ros_commands_enabled = False

        # Dictionary execution ids to file info (names and args) with
        # status "running"
        self._current_running_scripts = {}

        # Mutex used to protect shared resources between robot script
        # actions thread (both uplink and _current_running_scripts) and
        # prevent race conditions.
        self._mutex = threading.Lock()

        # Define the path where the custom scripts are located
        if "INORBIT_ACTIONS_PATH" in os.environ:
            user_defined_path = os.environ["INORBIT_ACTIONS_PATH"]
            # The user has configured a path for actions. If it's
            # a valid directory, use this instead of the default.
            self.logger.info(
                f"Found INORBIT_ACTIONS_PATH environment configuration = {user_defined_path}."
            )

            if os.path.isdir(user_defined_path):
                self._configure_actions_path(user_defined_path)
            else:
                self.logger.warning(
                    f"The actions path configured {user_defined_path} does not exist."
                )
                self._configure_actions_path(INORBIT_ACTIONS_PATH_DEFAULT)
        else:
            # Set default path for actions script
            self._configure_actions_path(INORBIT_ACTIONS_PATH_DEFAULT)

    @overrides(Agentlet)
    def load(self, runlevel):
        self._states["runlevel"] = runlevel

        try:
            import std_msgs

            global std_msgs
        except Exception:
            self.once_logger.warn(
                "std_msgs_load",
                "Can't support ROS custom commands. " "Exception loading std_msgs.",
            )
            self._ros_commands_enabled = False

        if self._ros_commands_enabled:
            # A subscription (even no-op) registers an endpoint on this topic in the ROS 2 graph
            # right away. With only a publisher and no outbound traffic yet, some setups omit the
            # topic from discovery / `ros2 topic list` until something subscribes; user nodes that
            # `ros2 topic echo` or wait also need a matching publisher+subscriber graph edge.
            ros_custom_cmd_sub = (
                ROS_CUSTOM_COMMAND_TOPIC,
                std_msgs.msg.String,
                self._ros_custom_command_topic_graph_anchor,
            )
            self._ros_agentlet.add_submodule(
                "custom_commands",
                subs=(ros_custom_cmd_sub,),
                pubs=(
                    (
                        ROS_CUSTOM_COMMAND_TOPIC,
                        std_msgs.msg.String,
                        self._ros_custom_cmd_publisher,
                    ),
                ),
            )

        # Register for upstream incoming commands
        # Note that incoming commands should be guaranteed to arrive (and exactly once, should)
        # not repeat! QoS=2
        self.uplink.add_listener(MQTT_CUSTOM_ROS_COMMAND_TOPIC, self._publish_ros_custom_cmds, 2)
        self.uplink.add_listener(
            MQTT_CUSTOM_SCRIPTS_TOPIC, self._process_custom_scripts_command_message, 2
        )

        try:
            if self._files_path == INORBIT_ACTIONS_PATH_DEFAULT and not os.path.exists(
                self._files_path
            ):
                os.makedirs(self._files_path)
        except Exception as e:
            self.logger.exception("Exception reading scripts from filesystem.")

        self._states["available_custom_scripts"] = self._list_custom_scripts_in_robot()

        self._states["loaded"] = True
        # Send a state update to the clouds
        self.publish_state(self.uplink, self._states)
        return True

    @overrides(Agentlet)
    def unload(self):
        self._ros_agentlet.remove_submodule("custom_commands")
        self._states["loaded"] = False
        return True

    @overrides(Agentlet)
    def set_runlevel(self, runlevel):
        self._states["runlevel"] = runlevel

    def set_state(self, state):
        """
        Called whenever a set_module command is received.
        """

        if "script_execution_timeout" in state.keys():
            self._states["script_execution_timeout"] = state["script_execution_timeout"]

        if "script_max_parallel_execs" in state.keys():
            self._states["script_max_parallel_execs"] = state["script_max_parallel_execs"]

        if "script_concurrent_execs" in state.keys():
            self._store_concurrent_execs_state(state["script_concurrent_execs"])

        if "clean_env" in state.keys():
            self._states["clean_env"] = state["clean_env"]

        # Send a state update to the cloud
        self.publish_state(self.uplink, self._states)

    def _list_custom_scripts_in_robot(self):
        """
        Returns the list of files in directory self._files_path.

        NOTE (Flor_Grosso): This could be updated in order to only list
        files with execution permissions.
        """

        return [
            file
            for file in os.listdir(self._files_path)
            if os.path.isfile(os.path.join(self._files_path, file))
        ]

    def _send_status_base(
        self, file_name, execution_id, status, send_output, output, status_details=None
    ):
        """
        Publishes a CustomScriptStatusMessage in MQTT_SCRIPT_OUTPUT_TOPIC.
        This is the base method used to send status updates for different tasks
        (script installation, execution status, status_details, script output).
        """

        message = CustomScriptStatusMessage()
        message.ts = self.get_ts()
        message.file_name = file_name
        message.execution_id = execution_id
        message.execution_status = status
        # If execution details are available (eg. errors when trying
        # to execute a script), send them.
        if status_details:
            message.execution_status_details = status_details
        if send_output and output:
            message.return_code = output.get("return_code")
            message.stdout = output.get("stdout")[-SCRIPT_OUTPUT_LEN:]
            message.stderr = output.get("stderr")[-SCRIPT_OUTPUT_LEN:]
        self.uplink.publish_protobuf(MQTT_SCRIPT_OUTPUT_TOPIC, message, qos=1)

    def _send_script_execution_status(
        self, file_name, execution_id, status, send_output=False, output={}, status_details=None
    ):
        """
        Publishes execution status of a script.
        """

        # If script execution is over (either aborted or finished
        # correctly), remove it from the dictionary tracking current
        # runs.
        with self._mutex:
            if execution_id in self._current_running_scripts and (
                status == STATUS_FINISHED or status == STATUS_ABORTED
            ):
                self._current_running_scripts[execution_id] = None
                self._current_running_scripts.pop(execution_id)
            self._send_status_base(
                file_name, execution_id, status, send_output, output, status_details
            )

    def _send_script_installation_status(self, file_name, execution_id, status):
        """
        Publishes the installation status of a script.
        Here we aren't using the _last_script and _script_execution_status
        members, in order to be able to install a file while another script
        is running.
        """

        with self._mutex:
            self._send_status_base(file_name, execution_id, status, None, False, {})

    def _ros_custom_command_topic_graph_anchor(self, _msg):
        """
        Intentionally empty. Commands are injected via the publisher; this subscription only
        materializes the topic in the ROS 2 graph (see comment at add_submodule).
        """

        pass

    def _create_custom_cmd_ros_msg(self, payload):
        """
        Returns a ROS message with the custom command received as payload.
        """

        # Parse protobuf message
        try:
            custom_command = CustomCommandRosMessage()
            custom_command.ParseFromString(payload)
        except Exception:
            custom_command = None

        # Only create a ROS message if a valid cmd was received (not empty nor
        # null)
        if not custom_command or not custom_command.cmd:
            self.logger.warn("Invalid custom command received")
            return None

        self.logger.info("Publishing command: {:s}.".format(custom_command.cmd))

        command_msg = std_msgs.msg.String()
        command_msg.data = "{:s}".format(custom_command.cmd)

        return command_msg

    def _publish_ros_custom_cmds(self, in_msg):
        """
        MQTT callback for custom_command/ros. Publishes to ROS_CUSTOM_COMMAND_TOPIC.
        Preconditions: see module docstring (std_msgs, ROS agentlet, downstream subscriber).
        """
        if self._ros_custom_cmd_publisher.pub is None or not self._ros_commands_enabled:
            self.logger.warn("ROS publisher not set. Aborting.")
            return

        # Create ROS message from incoming message
        ros_msg = self._create_custom_cmd_ros_msg(in_msg)

        # If there's nothing to publish, skip it.
        if not ros_msg:
            return

        try:
            self._ros_custom_cmd_publisher.pub.publish(ros_msg)
        except Exception:
            self.logger.exception("Exception publishing command.")

    def _process_custom_scripts_command_message(self, in_msg):
        """
        Called from the server. Processes a custom script command message and
        creates a new script action from it. It triggers the creation of a
        new file and its execution, if requested.
        """

        message = CustomScriptCommandMessage()
        message.ParseFromString(in_msg)

        file_name = message.file_name
        exec_id = message.execution_id
        # Convert the protobuf message to a python list
        args = [arg for arg in message.arg_options] if message.arg_options else []

        # Check if message is well formed
        if message.file_name == "":
            # NOTE(Flor_Grosso): It is very unlikely that the UI would
            #                    send a status with no name field.
            self.logger.warning("Received a script run requirement without" " file_name field")
            return

        # Create a new script action
        script = RobotScriptAction(
            file_name, self._files_path, exec_id, args, self._states["clean_env"]
        )

        # Creating the file if is necessary.
        # In the case that the file already exists, it is overwritten.
        if message.script_contents != "":
            if script.from_file_contents(message.script_contents):
                self._send_script_installation_status(file_name, exec_id, STATUS_INSTALLED)
            else:
                self._send_script_installation_status(file_name, exec_id, STATUS_NOT_INSTALLED)
                return

        max_parallel_execs = self._states.get(
            "script_max_parallel_execs", DEFAULT_MAX_PARALLEL_EXECS
        )

        if message.run:
            # Getting list of keys is atomic, so it's not necessary to
            # use mutex here. Please refer to
            # http://effbot.org/pyfaq/what-kinds-of-global-value-mutation-are-thread-safe.htm
            # for more information.
            if len(self._current_running_scripts.keys()) >= max_parallel_execs:
                self._send_script_execution_status(
                    file_name,
                    exec_id,
                    STATUS_ABORTED,
                    status_details="exceeded parallel script " "executions",
                )
                return
            else:
                # Run the script if requested
                timeout = self._states["script_execution_timeout"]

                concurrent_execs = self._states.get(
                    "script_concurrent_execs", DEFAULT_CONCURRENT_EXEC
                )

            with self._mutex:
                can_run_script = self._can_run_script(file_name, args, concurrent_execs)

            if not can_run_script:
                self._send_script_execution_status(
                    file_name, exec_id, STATUS_ABORTED, status_details="script already running"
                )
                return

            # Add another entry to the current running scripts
            # dictionary
            with self._mutex:
                self._current_running_scripts[exec_id] = script

            script.run(self._send_script_execution_status, timeout)

    def _can_run_script(self, file_name, args, concurrent_execs):
        """
        Checks if a script can be executed based on the concurrent execution
        settings and whether there is a script with the same file_name and
        args already running.
        This method accesses self._current_running_scripts and assumes
        self._mutex is already locked.
        """

        if concurrent_execs == CONCURRENT_EXEC_ENABLED:
            return True

        is_script_running, are_args_equal = self._is_script_running(file_name, args)

        # Script can't be ran if the file is already running and concurrent
        # executions flag is disabled.
        if concurrent_execs == CONCURRENT_EXEC_DISABLED and is_script_running:
            return False

        # Script can't be ran if same file + args is running and concurrent
        # executions with different arguments is enabled
        elif (
            concurrent_execs == CONCURRENT_EXEC_DISABLED_FOR_SAME_ARGS
            and is_script_running
            and are_args_equal
        ):
            return False

        return True

    def _is_script_running(self, file_name, args):
        """
        Returns an array of two booleans, the first one indicated whether
        the given file is being executed and the second one if arguments are
        equal to the ones provided.
        This method accesses self._current_running_scripts and assumes
        self._mutex is already locked.
        """

        active_exec_ids = self._current_running_scripts.keys()

        # Assuming the caller is sending either an array filled with args
        # or an empty array.
        args_string = " ".join(args)

        for exec_id in active_exec_ids:
            is_script_running = self._current_running_scripts[exec_id].get_name() == file_name
            are_args_equal = self._current_running_scripts[exec_id].get_args_string() == args_string

            if is_script_running:
                return [is_script_running, are_args_equal]

        return [False, False]

    def _store_concurrent_execs_state(self, script_concurrent_execs):
        """
        Checks if the script_concurrent_execs option provided is within the
        expected values. If it's not, assumes a default state and warns the
        user.
        """

        if script_concurrent_execs not in CONCURRENT_EXEC_OPTIONS:
            self._states["script_concurrent_execs"] = DEFAULT_CONCURRENT_EXEC
            self.logger.warning(
                f"'script_concurrent_execs' state: '{script_concurrent_execs}' is not valid. "
                f"Assuming '{DEFAULT_CONCURRENT_EXEC}' as default."
            )
        else:
            self._states["script_concurrent_execs"] = script_concurrent_execs

    def _configure_actions_path(self, path):
        """
        Configures the path where the agent will look for the action scripts.
        """

        # Set default path for actions script
        self._files_path = path
        self.logger.info(f"Looking for actions scripts in: {path}.")
