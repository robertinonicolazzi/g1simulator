# Copyright (c) 2021, InOrbit, Inc.
# All rights reserved.
#
# Various utility functions to handle high level ROS setup.
import os
import sys

# Flag to mark that ROS autodetection already ran.
ros_autodetect_ran = False

# Keep this list updated with the ROS 2 versions supported.
SUPPORTED_ROS_VERSIONS = ["foxy", "humble", "iron", "jazzy"]

ros_to_python_version = {
    "foxy": "python3.8",
    "humble": "python3.10",
    "iron": "python3.10",
    "jazzy": "python3.12",
}


def ros_autodetect():
    """
    Autodetects and configures the environment to use the existing ROS
    installation, if any. This includes:
      - Patch python path to find ROS libraries.
      - Setup ROS master URI
      - Setup other ROS related environment variables

    This simple implementation just attempts to find ROS in various known
    places from /opt/ros or based on INORBIT_ROS_PATH environment variable.
    It could fail (add wrong paths) if multiple ROS installations exist, as
    in the case of ROS1 + ROS2; see note below.

    Used from ROS agentlet and artifacts (rosbags) utilities.
    """
    global ros_autodetect_ran
    if ros_autodetect_ran:
        return
    ros_autodetect_ran = True

    ros_version = os.environ.get("INORBIT_ROS")
    # NOTE(FlorGrosso): It is unlikely that we get here with an unknown ROS
    # version. The inorbit.py entry script already checks if this variable
    # is set and doesn't load any ROS-dependant module if not.
    if ros_version == "unknown":
        print(
            "INORBIT_ROS env var not set on local/agent.env.sh. "
            "Please set it to use the correct ROS 2 version."
        )

    ros_path = os.getenv("INORBIT_ROS_PATH", None)

    # If the ROS version was set and is one of the supported ones, add the paths to site
    # and dist packages using the correct ROS version + python version combination.
    # If it is not known, then use all the possible combination of ROS and python versions.

    # For any of these cases, use a custom ROS path if provided. Otherwise use the
    # default opt/ros/<version> one.
    if ros_version in ros_to_python_version:
        python_ver = ros_to_python_version[ros_version]
        libs_path = ros_path if ros_path is not None else "/opt/ros/{ros_version}"
        sys.path.append(f"{libs_path}/lib/{python_ver}/site-packages")
        sys.path.append(f"{libs_path}/local/lib/{python_ver}/dist-packages")
        print(sys.path)
    else:
        for ros_ver, python_ver in ros_to_python_version.items():
            libs_path = ros_path if ros_path is not None else "/opt/ros/{ros_ver}"
            sys.path.append(f"{libs_path}/lib/{python_ver}/site-packages")
            sys.path.append(f"{libs_path}/local/lib/{python_ver}/dist-packages")

    if "ROS_PACKAGE_PATH" not in os.environ:
        if ros_path is not None:
            os.environ["ROS_PACKAGE_PATH"] = f"{ros_path}/share"
        else:
            if ros_version in SUPPORTED_ROS_VERSIONS:
                os.environ["ROS_PACKAGE_PATH"] = ":".join(f"/opt/ros/{ros_version}/share")
            else:
                os.environ["ROS_PACKAGE_PATH"] = ":".join(
                    ["/opt/ros/foxy/share", "/opt/ros/humble/share", "/opt/ros/jazzy/share"]
                )
    if "ROS_MASTER_URI" not in os.environ:
        os.environ["ROS_MASTER_URI"] = "http://localhost:11311"
