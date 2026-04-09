# Copyright (c) 2019, InOrbit, Inc.
# All rights reserved.
# RobotScriptAction class. Handles both creating and executing a script
# from a given path. It also keeps track of the file's execution id and
# reports updates with execution status and output using callbacks.
import os
import stat
import subprocess
import threading

import inorbit.logger

# Script execution status
# Keep in sync with client code at common/ActionFeedback.js
# TODO(adamantivm) Consider using protobuf enums to keep in sync
STATUS_ABORTED = "aborted"
STATUS_FINISHED = "finished"
STATUS_INSTALLED = "file created"
STATUS_RUNNING = "running"
STATUS_NOT_INSTALLED = "unable to create file"
STATUS_TO_BE_STARTED = "to be started"

# Timeout for script execution
DEFAULT_TIMEOUT_SEC = 30


class RobotScriptAction:
    def __init__(self, file_name, actions_path, execution_id, exec_args=[], clean_env=True):
        self._file_name = file_name
        self._full_file_path = os.path.join(actions_path, file_name)
        self._exec_args = exec_args
        self._id = execution_id
        self._clean_env = clean_env

        # Subprocess for script execution
        self._subprocess = None
        self._execution_status = None

        self.logger = inorbit.logger.getLog(__name__)

    def from_file_contents(self, content):
        """
        Creates a new text file in self._full_file_path with the content
        provided. It also adds execution permissions to it.
        Returns True if it succeeded, False if not.

        TODO(IO-590): this is creating a new file without checking if it
        already exists. Revisit this before enabling this feature back.
        """

        try:
            with open(self._full_file_path, "w") as f:
                f.write(content)
                f.flush()
                os.fsync(f.fileno())
                # Adding execution permissions to the file.
                # ...st_mode are the current file permissions and
                # stat.S_IXUSR is the user execution permissions flag.
                os.fchmod(f.fileno(), os.fstat(f.fileno()).st_mode | stat.S_IXUSR)
        except Exception as e:
            self.logger.exception(f"Exception when creating the script: {self._full_file_path}")
            # If file creation failed at some point, clean it.
            try:
                # TODO (IO-590): consider being cautious about this
                # when we start supporting a configurable path for
                # user scripts, since this might cause a security issue.
                os.remove(self._full_file_path)
            except Exception as e:
                pass
            return False

        return True

    def run(self, status_cb, timeout=DEFAULT_TIMEOUT_SEC):
        """
        Spawns a thread for script execution.
        """

        if self._execution_status == STATUS_RUNNING:
            self.logger.warning(f"{self._get_name_with_args()} already running")
        # Start script execution thread
        t = threading.Thread(
            target=self._script_runner, name="script_runner_" + self._id, args=(status_cb, timeout)
        )
        t.start()

    def _script_runner(self, status_cb, timeout=DEFAULT_TIMEOUT_SEC):
        """
        Executes a script as a subprocess. Spawns a thread  waiting for it to
        finish, or kills it after timeout seconds. Output is saved in order
        to be sent.
        """

        # Prepare array of filename and args
        path_with_args = (
            (["env", "-i"] if self._clean_env else []) + [self._full_file_path] + self._exec_args
        )

        # Output from file execution
        output = {"stdout": None, "stderr": None, "return_code": None}

        try:
            self._subprocess = subprocess.Popen(
                path_with_args, stderr=subprocess.PIPE, stdout=subprocess.PIPE
            )
        except Exception as e:
            self.logger.exception(f"Exception when running script: {self._file_name}")
            self._execution_status = STATUS_ABORTED
            status_cb(self._file_name, self._id, self._execution_status)
            return

        self._execution_status = STATUS_RUNNING
        status_cb(self._file_name, self._id, self._execution_status)

        # In case of timeout, status is overwritten
        self._execution_status = STATUS_FINISHED
        try:
            timer = threading.Timer(
                timeout, lambda status_cb=status_cb: self._timeout_cb(status_cb)
            )
            timer.start()
            output["stdout"], output["stderr"] = self._subprocess.communicate()
            output["return_code"] = str(self._subprocess.returncode)
            # If the script ends with a non-zero status code, send an aborted status
            # to provide error feedback to the user
            # TODO(adamantivm) Surface status code on the client
            if self._subprocess.returncode != 0:
                self._execution_status = STATUS_ABORTED
        except Exception as e:
            self.logger.exception(
                f"Exception when stopping execution for: {self._get_name_with_args()}"
            )
        finally:
            timer.cancel()

        # Send a status update with the execution output
        status_cb(self._get_name_with_args(), self._id, self._execution_status, True, output)

    def _timeout_cb(self, status_cb):
        """
        Callback used to kill script execution in the case of timeout. Also
        updates status to "aborted".
        """

        # This shouldn't happen, but it's checked anyway.
        if self._subprocess is not None:
            self._subprocess.kill()
        self._execution_status = STATUS_ABORTED
        # Send a status update indicating script execution was aborted
        status_cb(
            self._file_name, self._id, self._execution_status, status_details="execution timed out"
        )

    def get_name(self):
        """
        Returns file name.
        """

        return self._file_name

    def get_args_string(self):
        """
        Returns script arguments as a string (these preserve the order
        in which they are stored in the array).
        """

        return " ".join(self._exec_args)

    def _get_name_with_args(self):
        """
        Returns a string with file name and the concatenated args.
        """

        return self._file_name + self.get_args_string()
