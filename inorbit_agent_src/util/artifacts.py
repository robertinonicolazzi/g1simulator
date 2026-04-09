# Copyright (c) 2021, InOrbit, Inc.
# All rights reserved.
# Utility classes representing arbitrary artifacts in the robot.
# An artifact is a file in the robot that we might want to persist at some
# point in time. So an Artifact class is mainly a File system object
# abstraction with uploading capabilities and extra metadata.
# Special kind of Artifacts are Databags (associated with a time frame and
# with extra properties comming from metadata) and Rosbags (Databags that are
# recorded listening to the robot ros topics)
import json
import os
import signal
import subprocess
import sys
import tarfile
import threading
import time

import inorbit.logger
import yaml
from inorbit.rossetup.ros import ros_autodetect
from util.s3_upload_helper import S3_Client
from util.concurrency import Interval

# Some functions from this module (rosbags recording) depend on 'rosbag'
# module from ROS. Try a soft-import
try:
    ros_autodetect()
    import rosbag
except ImportError:
    pass  # Error is ignored. Rosbag class functions can report if they fail


# Using kilobyte = 1024 since it is prevalent in filesystem terminology
KB = 1024
MS = 1000


class Artifact(object):
    """
    Class to handle arbitrary artifacts.

    An Artifact represents an arbitrary file than can be uploaded to the cloud
    to be persisted by InOrbit
    """

    def __init__(self, path, compress=False):
        self.logger = inorbit.logger.getLog(__name__)
        # TODO(herchu) Do not read this information from a file, but receive it
        # from a higher level module instead.
        try:
            self._s3config = self._read_aws_credentials()
        except Exception as e:
            self._s3config = None
            self.logger.warning("Couldn't obtain AWS credentials: %s", str(e))

        # Mutex used to protect uploading/deleting tasks
        self._mutex = threading.Lock()
        # Split the path of the artifact into folder and name
        self._dir, self._name = os.path.split(path)
        self._path = path
        # If the artifact exists in the filesystem, it is in_robot
        self._in_robot = os.path.isfile(path)
        self._uploading = False
        self._compress = compress
        self._url = ""
        # Use try-except with filesystem operations
        try:
            self._ts = int(os.path.getmtime(path) * MS) if self._in_robot else 0
        except Exception:
            self._ts = 0
        try:
            self._size_kb = int(os.path.getsize(path) / KB) if self._in_robot else 0
        except Exception:
            self._size_kb = 0

    def upload(self, subpath=None, callback=None):
        """
        Uploads the artifact to cloud.
        The target location is defined by the S3 config, which contains a company ID as part of the
        bucket name (and credentials provisioned with permissions on this bucket). A sub-path
        within this bucket can be given as optional argument. Finally, the object name will
        be the same as this artifact name in the filesystem

        The (optional) callback argument gets called periodically with number of bytes transferred
        since last call. It must be a function that receives a single numeric argument (bytes). See:
        https://boto3.amazonaws.com/v1/documentation/api/latest/reference/services/s3.html#S3.Bucket.upload_fileobj
        """

        if self._s3config is None:
            raise Exception("Couldn't upload %s because S3 is not configured" % self._path)
        s3_uploader = S3_Client(self._s3config)
        # TODO (Flor_Grosso): check whether the file should be compressed or
        # not, based on the mission status/ resource usage.
        with self._mutex:
            path = self.compress() if self._compress else self._path
            self._url = s3_uploader.upload_file(path, subpath, callback=callback)
        # Delete compressed (temporary) file
        try:
            if self._compress:
                os.remove(path)
        except Exception:
            raise Exception("Could not delete file: %s" % path)

        self._uploading = False

    def delete_from_robot(self):
        """
        Deletes the artifact from the robot.

        TODO (Flor_Grosso): Consider extending the locking to prevent race
        conditions while computing hdd usage in the SystemAgentlet
        TODO(herchu) This method ought to have some validations! Otherwise
        anyone hacking the mqtt channel somehow and sending a command to delete
        a not-rosbag might delete anything in the robot!
        """

        try:
            with self._mutex:
                os.remove(self._path)
            # Mark file as not stored in robot
            self._in_robot = False
        except Exception:
            raise Exception("Could not delete file: %s." % self._path)

    def compress(self):
        """
        Creates a tar file on the same working directory.
        TODO (Flor_Grosso): consider creating this file on a temporary directory,
        rather than using /local.
        """

        compressed_path = self._path + ".tar.gz"
        with tarfile.open(compressed_path, "w:gz") as tar:
            tar.add(self._path, arcname=self._name)
        return compressed_path

    def _read_aws_credentials(self):
        """
        Reads the cached aws credentials file.
        """

        inorbit_home = os.getenv("INORBIT_HOME", os.path.join(os.getenv("HOME"), ".inorbit"))
        aws_cache_filename = os.path.join(inorbit_home, "local", ".cache")
        try:
            aws_file = open(aws_cache_filename, "r")
            cache = aws_file.read()
            keys = json.loads(cache)
            return keys["awsUploadCredentials"]
        except Exception as e:
            raise e

    @property
    def path(self):
        return self._path

    @property
    def name(self):
        return self._name

    @property
    def ts(self):
        return self._ts

    @property
    def in_robot(self):
        return self._in_robot

    @property
    def uploading(self):
        return self._uploading

    @property
    def url(self):
        return self._url

    @property
    def size_kb(self):
        return self._size_kb

    def set_uploading(self, uploading):
        self._uploading = uploading

    def set_in_robot(self, in_robot):
        self._in_robot = in_robot


class Databag(Artifact):
    """
    Class to handle databag artifacts.

    A Databag represents an specific kind of artifact. It contains information
    for a given period of time (between start_ts and end_ts) and could be
    enriched with a properties yaml file
    """

    def __init__(self, path, compress=False):
        super(Databag, self).__init__(path, compress)
        # Default start_ts to last modification time
        try:
            self._start_ts = int(os.path.getmtime(path) * MS) if self._in_robot else 0
        except Exception:
            self._start_ts = 0
        self._end_ts = 0
        self._properties = {}
        self.refresh_properties()

    @property
    def start_ts(self):
        return self._start_ts

    @property
    def end_ts(self):
        return self._end_ts

    @property
    def properties(self):
        return self._properties

    def refresh_properties(self):
        """
        Load databag properties from an associated YAML file. Returns true if properties
        have changed since the last refresh.
        """

        # Replace the file path with a yml extension for the properties
        # HACK: mimics os.path.splitext but works with composed extensions
        # Such as .tar.xz. Will fail if the filename has dots besides extension
        properties_path = os.path.join(self._dir, self._name.split(".")[0] + ".yml")
        prev_properties = self._properties
        if os.path.isfile(properties_path):
            try:
                with open(properties_path, "r") as f:
                    self._properties = yaml.safe_load(f)
            except Exception as e:
                self.logger.warning(
                    "Couldn't parse properties yaml file %s: %s",
                    properties_path,
                    str(e),
                )
        return self._properties != prev_properties


class Rosbag(Databag):
    """
    Class to handle rosbag artifacts.

    A Rosbag is a special kind of databag that is recorded by the robot and has
    all the messages that went through a given set of topics for the recording
    period
    """

    def __init__(self, path, compress=False):
        super(Rosbag, self).__init__(path, compress)
        self._topics = []

        self._subprocess = None
        # Recording state
        self._recording = False
        self._recording_halted = False
        # Recording state
        self._bag_writer = None

        # Thread to stop 'ros2 bag record' subprocess after duration
        self._check_rosbag_recording_time_thread = None

        # Read rosbag metadata file, if present
        self.rosbag_metadata = self.read_metadata_file()

        # Calculate end_ts with the duration from rosbag metadata file
        self._end_ts = self._start_ts + self.get_rosbag_duration_from_metadata()

    def __str__(self) -> str:
        return f"_name: {self._name}, _path: {self._path}, _in_robot: {self._in_robot}"

    def __repr__(self) -> str:
        return self.__str__()

    def start_writer(self):
        if "rosbag" not in sys.modules:  # Depends on rosbag module from ROS
            return False
        try:
            with self._mutex:
                self._bag_writer = rosbag.Bag(self._path, "w")
        except Exception:
            return False
        return True

    def stop_writer(self):
        with self._mutex:
            if self._bag_writer:
                self._bag_writer.close()
                self._bag_writer = None
            else:
                print("error, calling write() without open rosbag")

    def add_topic(self, topic, msg_type):
        # Placeholder. In "rosbags" library we need to create a topic with its
        # message type def; but it is not necessary in ROS "rosbag" library.
        pass

    def read_metadata_file(self):
        """
        Reads ROS2 bag metadata.yaml file.

        Returns dictionary with yaml data if the file exists. If not,
        it returns an empty dict object '{}'
        """
        metadata_path = os.path.join(self._dir, "metadata.yaml")
        if os.path.exists(metadata_path):
            with open(metadata_path, "r") as f:
                return yaml.safe_load(f)
        return {}

    def get_rosbag_duration_from_metadata(self):
        """
        Reads ROS2 bag duration in nanoseconds from rosbag metadata.

        Returns rosbag duration in miliseconds. If there's no metadata
        or there's no duration it returns 0.
        """
        rosbag2_bagfile_information = self.rosbag_metadata.get("rosbag2_bagfile_information", {})
        
        duration_ns =  rosbag2_bagfile_information.get("duration", {}).get("nanoseconds", 0)
        
        return int(duration_ns / 1_000_000)

    def write_message(self, topic, ros_message, ts=None):
        if ts:
            # With the "rosbags" library we can tune the timestamp of the ROS
            # message, which we could ideally set to the original MQTT's
            # message timestamp. That is not possible in ROS' "rosbag" library,
            # so raise an error for now until we resolve it.
            raise Exception("User-defined timestamps are not yet implemented")
        with self._mutex:
            if self._bag_writer:
                self._bag_writer.write(topic, ros_message)
            else:
                print("error, calling write() without open rosbag")
    
    def _check_rosbag_recording_time(self, duration):
        """
        Monitors 'ros2 bag record' subprocess elapsed time and halts
        recording when duration is exceeded.

        It runs periodically when the recording starts, and stopped when
        the recording is halted.
        """
        if self._subprocess:
            elapsed = int(time.time() * MS) - self.start_ts
            if elapsed > duration:
                self.halt_recording()

    def record(self, topics, duration):
        """
        Performs a rosbag record process with the options given as arguments
        and the path defined in the constructor. Waits for completion and checks
        output.
        """

        # TODO check that master is running
        # TODO(adamantivm) Take this path from the ros agentlet instead
        if "INORBIT_ROS_PATH" in os.environ:
            source_ros_env_command = ". $INORBIT_ROS_PATH/setup.sh"
        else:
            source_ros_env_command = ". /opt/ros/$INORBIT_ROS/setup.sh"

        # NOTE: the ros2 bag recording command interface changes from ros1 to ros2.
        # With ros1 the 'rosbag record' command ends when the recording reaches
        # certain duration (specified with the "--duration" parameter).
        # 'ros2 bag record' doesn't allow setting a duration or timecap. Instead,
        # it only allows setting the maximum size in bytes before the bagfile will
        # be split (see "--max-bag-size MAX_BAG_SIZE" flag). The command doesn't
        # stop after reaching said size but it creates multiple rosbags files and
        # split the recording into chunk ending with _0, _1, _2, ...
        #   - rosbag2_2023_08_15-13_39_42/rosbag2_2023_08_15-13_39_42_0.db3
        #   - rosbag2_2023_08_15-13_39_42/rosbag2_2023_08_15-13_39_42_1.db3
        #   - rosbag2_2023_08_15-13_39_42/rosbag2_2023_08_15-13_39_42_2.db3
        # To maintain consistency and to enable recording multiple bags overtime,
        # this implementation assumes 'ros2 bag record' only outputs a single bag
        # file, which is accomplished by NOT setting the MAX_BAG_SIZE parameter.
        # 
        # TODO(@lpineda.io): implement a mechanism to make `ros2 bag record` process
        # stop after X time (using record's method parameter 'duration').
        record_command = (
            f"ros2 bag record --output={self._dir} {' '.join(topics)}"
        )
        self.logger.debug(f"Rosbag: record command '{record_command}'")
        rosbag_record = f"{source_ros_env_command}; {record_command}"

        self._recording = True
        self._topics = topics
        self._start_ts = int(time.time() * MS)

        # Start a thread to monitor 'ros2 bag record' subprocess elapsed time
        # and to kill the subprocess if the elapsed time exceeded duration.
        self._check_rosbag_recording_time_thread = Interval(
            self._check_rosbag_recording_time, 1, duration * MS
        )
        self._check_rosbag_recording_time_thread.start()

        self.logger.debug(f"Subprocess for rosbag record: start_ts {self._start_ts}")
        with open(os.devnull, "w") as devnull:
            self._subprocess = subprocess.Popen(
                rosbag_record,
                shell=True,
                stderr=subprocess.PIPE,
                stdout=devnull,
                preexec_fn=os.setsid,
            )
        self._subprocess.wait()
        self._check_rosbag_recording_time_thread.stop()
        self._check_rosbag_recording_time_thread = None
        self._recording = False
        self._end_ts = int(time.time() * MS)
        self.logger.debug(f"Subprocess for rosbag record: end_ts {self._end_ts}")

        # Raise an exception if process didn't finish correctly, except for
        # those cases when recording is halted.
        if self._subprocess.returncode != 0 and not self._recording_halted:
            output, error = self._subprocess.communicate()
            self._subprocess = None
            raise Exception(error)

        self._subprocess = None
        # Mark bag as stored in robot
        self._in_robot = True

    def halt_recording(self):
        """
        Halts a rosbag recording.
        """
        self.logger.debug(f"Halting rosbag recording")
        if self.recording:
            self._recording_halted = True
            os.killpg(self._subprocess.pid, signal.SIGINT)
        else:
            self.logger.warning("Trying to halt a non-existent recording.")
        self.logger.debug(f"Halting rosbag recording: Done")

    @property
    def topics(self):
        return self._topics

    @property
    def recording(self):
        return self._recording
