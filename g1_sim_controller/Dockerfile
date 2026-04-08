FROM ros:humble

# Install basic development tools
RUN apt-get update && apt-get install -y \
    python3-pip \
    git \
    build-essential \
    cmake

# Install cyclonedds from source as requested by unitree_sdk2_python FAQ
RUN git clone https://github.com/eclipse-cyclonedds/cyclonedds -b releases/0.10.x /lib/cyclonedds && \
    mkdir -p /lib/cyclonedds/build /lib/cyclonedds/install && \
    cd /lib/cyclonedds/build && \
    cmake .. -DCMAKE_INSTALL_PREFIX=/lib/cyclonedds/install && \
    cmake --build . --target install

# Install unitree_sdk2_python
RUN git clone https://github.com/unitreerobotics/unitree_sdk2_python.git /lib/unitree_sdk2_python && \
    touch /lib/unitree_sdk2_python/unitree_sdk2py/b2/__init__.py && \
    touch /lib/unitree_sdk2_python/unitree_sdk2py/comm/__init__.py && \
    touch /lib/unitree_sdk2_python/unitree_sdk2py/h1/__init__.py && \
    mkdir -p /lib/unitree_sdk2_python/unitree_sdk2py/g1 && \
    touch /lib/unitree_sdk2_python/unitree_sdk2py/g1/__init__.py && \
    touch /lib/unitree_sdk2_python/unitree_sdk2py/g1/loco/__init__.py && \
    touch /lib/unitree_sdk2_python/unitree_sdk2py/g1/arm/__init__.py && \
    touch /lib/unitree_sdk2_python/unitree_sdk2py/g1/audio/__init__.py && \
    cd /lib/unitree_sdk2_python && \
    export CYCLONEDDS_HOME=/lib/cyclonedds/install && \
    pip3 install .

RUN apt-get install -y \
  ros-humble-pointcloud-to-laserscan \
  ros-humble-slam-toolbox \
  ros-humble-nav2-bringup \
  ros-humble-octomap-server \
  ros-humble-tf2-tools

# Set up the workspace
WORKDIR /ros_ws
COPY . /ros_ws

# Source ROS2 Humble
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Set the entrypoint
COPY ros_entrypoint.sh /
RUN chmod +x /ros_entrypoint.sh
CMD ["bash", "-c", "exec /ros_entrypoint.sh"]
