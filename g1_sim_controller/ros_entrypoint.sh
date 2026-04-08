#!/bin/bash
set -e

# setup ros2 environment
source "/opt/ros/humble/setup.bash"

rm -rf /ros_ws/build
rm -rf /ros_ws/install

colcon build --symlink-install
source "/ros_ws/install/setup.bash"

exec ros2 launch g1_sim_controller bringup.launch.py
