#!/usr/bin/env python3
# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0
"""
DDS-ROS2 Bridge for Unitree G1 Navigation

This bridge enables communication between:
- Unitree DDS (CycloneDDS) used by the robot/simulation
- ROS2 topics used by Nav2 and Isaac ROS

The bridge converts:
- ROS2 /cmd_vel (Twist) → DDS rt/run_command/cmd
- DDS robot state → ROS2 /odom (Odometry)
- DDS robot state → ROS2 /tf (transforms)

This allows Nav2 to control the G1 robot seamlessly, whether in
simulation (Isaac Lab) or on the real robot (Jetson Orin).

Components:
-----------
CmdVelSubscriber:
    Subscribes to /cmd_vel and converts to Unitree velocity command format.
    
OdometryPublisher:
    Reads robot state from DDS/shared memory and publishes to /odom.
    
TfBroadcaster:
    Publishes odom→base_link transform for navigation.

Usage:
------
    # As a standalone script
    python -m navigation.dds_ros2_bridge
    
    # Or import and use in your own node
    from navigation.dds_ros2_bridge import DdsRos2Bridge
    bridge = DdsRos2Bridge()
    bridge.spin()

Requirements:
-------------
- ROS2 Humble with rclpy
- unitree_sdk2py
- geometry_msgs, nav_msgs, tf2_ros
"""

import sys
import time
import threading
import numpy as np
from typing import Optional, Tuple
import math
import json
from multiprocessing import shared_memory

# Check for ROS2 availability
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from geometry_msgs.msg import Twist, TransformStamped, Quaternion
    from nav_msgs.msg import Odometry
    from std_msgs.msg import Header
    from tf2_ros import TransformBroadcaster
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    print("WARNING: ROS2 not available. Install ROS2 Humble for full functionality.")

# Unitree DDS SDK
try:
    from unitree_sdk2py.core.channel import (
        ChannelPublisher, 
        ChannelSubscriber, 
        ChannelFactoryInitialize
    )
    from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_
    DDS_AVAILABLE = True
except ImportError:
    DDS_AVAILABLE = False
    print("WARNING: unitree_sdk2py not available. Install for DDS communication.")


class DdsRos2Bridge:
    """Bidirectional bridge between Unitree DDS and ROS2
    
    This class manages the conversion between Unitree's DDS protocol
    and standard ROS2 messages, enabling Nav2 integration.
    
    Attributes:
    -----------
    node : rclpy.Node
        ROS2 node for publishing/subscribing
    dds_cmd_pub : ChannelPublisher
        DDS publisher for velocity commands
    cmd_vel_sub : Subscription
        ROS2 subscriber for /cmd_vel
    odom_pub : Publisher
        ROS2 publisher for /odom
    tf_broadcaster : TransformBroadcaster
        TF2 broadcaster for odom→base_link
        
    Parameters:
    -----------
    node_name : str
        Name of the ROS2 node (default: 'dds_ros2_bridge')
    default_height : float
        Default robot height for velocity commands (default: 0.8m)
    odom_frame : str
        Frame ID for odometry (default: 'odom')
    base_frame : str
        Frame ID for robot base (default: 'base_link')
    publish_tf : bool
        Whether to publish TF transforms (default: True)
    """
    
    def __init__(
        self,
        node_name: str = 'dds_ros2_bridge',
        default_height: float = 0.8,
        odom_frame: str = 'odom',
        base_frame: str = 'base_link',
        publish_tf: bool = True
    ):
        self.default_height = default_height
        self.odom_frame = odom_frame
        self.base_frame = base_frame
        self.publish_tf = publish_tf
        
        # Robot state tracking for odometry
        self._position = np.array([0.0, 0.0, 0.0])  # x, y, z
        self._prev_position = np.array([0.0, 0.0, 0.0])  # for velocity estimation
        self._orientation = np.array([0.0, 0.0, 0.0, 1.0])  # quaternion x,y,z,w
        self._prev_orientation = np.array([0.0, 0.0, 0.0, 1.0])  # for angular vel estimation
        self._linear_vel = np.array([0.0, 0.0, 0.0])
        self._angular_vel = np.array([0.0, 0.0, 0.0])
        self._last_update_time = time.time()
        self._state_lock = threading.Lock()
        self._first_reading = True  # Skip velocity calc on first reading
        
        # Shared memory for reading robot state from simulation
        self._robot_state_shm = None
        self._shm_lock = threading.Lock()
        self._init_shared_memory()
        
        # Initialize DDS
        if DDS_AVAILABLE:
            self._init_dds()
        
        # Initialize ROS2
        if ROS2_AVAILABLE:
            self._init_ros2(node_name)
    
    def _init_shared_memory(self):
        """Initialize shared memory connection to read robot state from simulation"""
        self._shm_connection_attempts = 0
        self._shm_max_attempts = 10  # Will retry periodically during operation
        self._try_connect_shared_memory()
    
    def _try_connect_shared_memory(self) -> bool:
        """Try to connect to shared memory. Returns True if successful."""
        if self._robot_state_shm is not None:
            return True  # Already connected
        
        try:
            # Try to connect to the shared memory created by Isaac Lab simulation
            self._robot_state_shm = shared_memory.SharedMemory(name="isaac_robot_state")
            print("[DDS-ROS2 Bridge] Connected to shared memory: isaac_robot_state")
            print("  - Will read actual robot pose from simulation")
            return True
        except FileNotFoundError:
            self._shm_connection_attempts += 1
            if self._shm_connection_attempts == 1:
                print("[DDS-ROS2 Bridge] WARNING: Shared memory 'isaac_robot_state' not found")
                print("  - Will retry when simulation creates it...")
                print("  - Using dead reckoning from /cmd_vel in the meantime")
            return False
        except Exception as e:
            self._shm_connection_attempts += 1
            if self._shm_connection_attempts == 1:
                print(f"[DDS-ROS2 Bridge] WARNING: Failed to connect to shared memory: {e}")
                print("  - Will retry periodically...")
            return False
    
    def _read_robot_state_from_shm(self) -> Optional[dict]:
        """Read robot state from shared memory
        
        Returns:
            dict with keys: joint_positions, joint_velocities, joint_torques, imu_data
            imu_data format: [pos_x, pos_y, pos_z, quat_w, quat_x, quat_y, quat_z, acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z]
        """
        # Try to connect if not connected yet (retry periodically)
        if self._robot_state_shm is None:
            # Only retry every ~100 calls (2 seconds at 50Hz) to avoid overhead
            self._shm_connection_attempts += 1
            if self._shm_connection_attempts % 100 == 0:
                self._try_connect_shared_memory()
            return None
        
        try:
            with self._shm_lock:
                # Read timestamp and data length (same format as SharedMemoryManager)
                timestamp = int.from_bytes(self._robot_state_shm.buf[0:4], 'little')
                data_len = int.from_bytes(self._robot_state_shm.buf[4:8], 'little')
                
                if data_len == 0:
                    return None
                
                # Read JSON data
                json_bytes = bytes(self._robot_state_shm.buf[8:8+data_len])
                data = json.loads(json_bytes.decode('utf-8'))
                return data
        except Exception as e:
            # Connection might have been lost - try to reconnect
            try:
                self._robot_state_shm.close()
            except:
                pass
            self._robot_state_shm = None
            return None
    
    def _init_dds(self):
        """Initialize Unitree DDS communication"""
        print("[DDS-ROS2 Bridge] Initializing DDS...")
        ChannelFactoryInitialize(1)
        
        # Publisher for velocity commands to robot
        self.dds_cmd_pub = ChannelPublisher("rt/run_command/cmd", String_)
        self.dds_cmd_pub.Init()
        
        print("[DDS-ROS2 Bridge] DDS initialized - publishing to rt/run_command/cmd")
    
    def _init_ros2(self, node_name: str):
        """Initialize ROS2 node and topics"""
        print(f"[DDS-ROS2 Bridge] Initializing ROS2 node: {node_name}")
        
        if not rclpy.ok():
            rclpy.init()
        
        self.node = rclpy.create_node(node_name)
        
        # QoS profile for navigation
        nav_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscribe to /cmd_vel from Nav2
        self.cmd_vel_sub = self.node.create_subscription(
            Twist,
            '/cmd_vel',
            self._cmd_vel_callback,
            nav_qos
        )
        
        # Publisher for odometry
        self.odom_pub = self.node.create_publisher(
            Odometry,
            '/odom',
            nav_qos
        )
        
        # TF broadcaster
        if self.publish_tf:
            self.tf_broadcaster = TransformBroadcaster(self.node)
        
        # Timer for odometry publishing (50Hz)
        self.odom_timer = self.node.create_timer(0.02, self._publish_odometry)
        
        print("[DDS-ROS2 Bridge] ROS2 initialized")
        print("  - Subscribing to: /cmd_vel")
        print("  - Publishing to: /odom")
        if self.publish_tf:
            print("  - Broadcasting TF: odom → base_link")
    
    def _cmd_vel_callback(self, msg: 'Twist'):
        """Convert ROS2 Twist to Unitree velocity command
        
        Unitree format: [x_vel, y_vel, yaw_vel, height]
        ROS2 Twist:
            - linear.x: forward velocity (m/s)
            - linear.y: lateral velocity (m/s)
            - angular.z: yaw velocity (rad/s)
        
        Note: Unitree SDK expects negative y and yaw for ROS convention
        """
        if not DDS_AVAILABLE:
            return
        
        # Convert Twist to Unitree format
        # Note: Sign conventions may need adjustment based on your robot
        x_vel = float(msg.linear.x)
        y_vel = -float(msg.linear.y)  # Unitree uses opposite sign
        yaw_vel = -float(msg.angular.z)  # Unitree uses opposite sign
        height = self.default_height
        
        # Clamp velocities to safe ranges
        x_vel = np.clip(x_vel, -0.6, 1.0)
        y_vel = np.clip(y_vel, -0.5, 0.5)
        yaw_vel = np.clip(yaw_vel, -1.57, 1.57)
        
        # Create command string
        cmd_list = [x_vel, y_vel, yaw_vel, height]
        cmd_str = str(cmd_list)
        
        # Publish to DDS
        msg_dds = String_(data=cmd_str)
        self.dds_cmd_pub.Write(msg_dds)
        
        # Update velocity state for odometry integration
        with self._state_lock:
            self._linear_vel[0] = msg.linear.x
            self._linear_vel[1] = msg.linear.y
            self._angular_vel[2] = msg.angular.z
    
    def _publish_odometry(self):
        """Publish odometry message and TF transform
        
        Reads actual robot pose from simulation shared memory when available.
        Falls back to dead reckoning if shared memory is not available.
        """
        current_time = time.time()
        
        # Try to read actual robot state from simulation shared memory
        robot_state = self._read_robot_state_from_shm()
        
        with self._state_lock:
            dt = current_time - self._last_update_time
            self._last_update_time = current_time
            
            if robot_state and 'imu_data' in robot_state:
                # Use actual pose from simulation
                # imu_data format: [pos_x, pos_y, pos_z, quat_w, quat_x, quat_y, quat_z, acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z]
                imu_data = robot_state['imu_data']
                if len(imu_data) >= 13:
                    # Store previous position for velocity calculation
                    self._prev_position = self._position.copy()
                    
                    # Extract position (world frame)
                    self._position[0] = imu_data[0]
                    self._position[1] = imu_data[1]
                    self._position[2] = imu_data[2]
                    
                    # Extract orientation (w, x, y, z) -> convert to (x, y, z, w) for ROS
                    # imu_data has (w, x, y, z) format
                    quat_w = imu_data[3]
                    quat_x = imu_data[4]
                    quat_y = imu_data[5]
                    quat_z = imu_data[6]
                    self._orientation = np.array([quat_x, quat_y, quat_z, quat_w])
                    
                    # Extract angular velocity from gyroscope (body frame)
                    self._angular_vel[0] = imu_data[10]
                    self._angular_vel[1] = imu_data[11]
                    self._angular_vel[2] = imu_data[12]
                    
                    # Estimate linear velocity from position change
                    if not self._first_reading and dt > 0.001:
                        # Compute world-frame velocity
                        world_vel = (self._position - self._prev_position) / dt
                        
                        # Transform to body frame (robot's local frame)
                        current_yaw = self._get_yaw_from_quaternion()
                        cos_yaw = np.cos(-current_yaw)
                        sin_yaw = np.sin(-current_yaw)
                        self._linear_vel[0] = world_vel[0] * cos_yaw - world_vel[1] * sin_yaw  # forward
                        self._linear_vel[1] = world_vel[0] * sin_yaw + world_vel[1] * cos_yaw  # lateral
                        self._linear_vel[2] = world_vel[2]
                    
                    self._first_reading = False
            else:
                # Fallback to dead reckoning (integrate velocities from /cmd_vel)
                yaw = self._get_yaw_from_quaternion()
                
                # Update position based on velocity
                self._position[0] += self._linear_vel[0] * np.cos(yaw) * dt
                self._position[0] -= self._linear_vel[1] * np.sin(yaw) * dt
                self._position[1] += self._linear_vel[0] * np.sin(yaw) * dt
                self._position[1] += self._linear_vel[1] * np.cos(yaw) * dt
                
                # Update orientation
                yaw += self._angular_vel[2] * dt
                self._orientation = self._quaternion_from_yaw(yaw)
            
            # Create copies for thread safety
            pos = self._position.copy()
            ori = self._orientation.copy()
            lin_vel = self._linear_vel.copy()
            ang_vel = self._angular_vel.copy()
        
        # Get timestamp
        stamp = self.node.get_clock().now().to_msg()
        
        # Create and publish odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = stamp
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.child_frame_id = self.base_frame
        
        # Position
        odom_msg.pose.pose.position.x = pos[0]
        odom_msg.pose.pose.position.y = pos[1]
        odom_msg.pose.pose.position.z = pos[2]
        
        # Orientation
        odom_msg.pose.pose.orientation.x = ori[0]
        odom_msg.pose.pose.orientation.y = ori[1]
        odom_msg.pose.pose.orientation.z = ori[2]
        odom_msg.pose.pose.orientation.w = ori[3]
        
        # Velocity
        odom_msg.twist.twist.linear.x = lin_vel[0]
        odom_msg.twist.twist.linear.y = lin_vel[1]
        odom_msg.twist.twist.linear.z = lin_vel[2]
        odom_msg.twist.twist.angular.x = ang_vel[0]
        odom_msg.twist.twist.angular.y = ang_vel[1]
        odom_msg.twist.twist.angular.z = ang_vel[2]
        
        # Covariance (simplified - use actual values in production)
        odom_msg.pose.covariance[0] = 0.01  # x
        odom_msg.pose.covariance[7] = 0.01  # y
        odom_msg.pose.covariance[35] = 0.01  # yaw
        
        self.odom_pub.publish(odom_msg)
        
        # Publish TF transform
        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            
            t.transform.translation.x = pos[0]
            t.transform.translation.y = pos[1]
            t.transform.translation.z = pos[2]
            
            t.transform.rotation.x = ori[0]
            t.transform.rotation.y = ori[1]
            t.transform.rotation.z = ori[2]
            t.transform.rotation.w = ori[3]
            
            self.tf_broadcaster.sendTransform(t)
    
    def _get_yaw_from_quaternion(self) -> float:
        """Extract yaw angle from quaternion"""
        x, y, z, w = self._orientation
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)
    
    def _quaternion_from_yaw(self, yaw: float) -> np.ndarray:
        """Create quaternion from yaw angle"""
        return np.array([0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)])
    
    def update_robot_state(
        self,
        position: Tuple[float, float, float],
        orientation: Tuple[float, float, float, float],
        linear_vel: Tuple[float, float, float],
        angular_vel: Tuple[float, float, float]
    ):
        """Update robot state from external source (e.g., DDS callback)
        
        Call this method when you receive actual robot state from DDS
        to improve odometry accuracy over dead reckoning.
        
        Args:
            position: (x, y, z) in meters
            orientation: quaternion (x, y, z, w)
            linear_vel: (vx, vy, vz) in m/s
            angular_vel: (wx, wy, wz) in rad/s
        """
        with self._state_lock:
            self._position = np.array(position)
            self._orientation = np.array(orientation)
            self._linear_vel = np.array(linear_vel)
            self._angular_vel = np.array(angular_vel)
            self._last_update_time = time.time()
    
    def spin(self):
        """Run the ROS2 node"""
        if ROS2_AVAILABLE:
            print("[DDS-ROS2 Bridge] Spinning...")
            rclpy.spin(self.node)
    
    def shutdown(self):
        """Clean shutdown"""
        print("[DDS-ROS2 Bridge] Shutting down...")
        
        # Close shared memory connection
        if self._robot_state_shm is not None:
            try:
                self._robot_state_shm.close()
                print("[DDS-ROS2 Bridge] Closed shared memory connection")
            except Exception as e:
                print(f"[DDS-ROS2 Bridge] Warning: Error closing shared memory: {e}")
        
        if ROS2_AVAILABLE and rclpy.ok():
            self.node.destroy_node()
            rclpy.shutdown()


def main():
    """Main entry point for the DDS-ROS2 bridge"""
    print("=" * 60)
    print("Unitree G1 DDS-ROS2 Navigation Bridge")
    print("=" * 60)
    print()
    print("This bridge connects:")
    print("  - Unitree DDS (rt/run_command/cmd)")
    print("  - ROS2 Nav2 (/cmd_vel, /odom)")
    print()
    
    if not ROS2_AVAILABLE:
        print("ERROR: ROS2 not available. Please install ROS2 Humble:")
        print("  https://docs.ros.org/en/humble/Installation.html")
        sys.exit(1)
    
    if not DDS_AVAILABLE:
        print("ERROR: unitree_sdk2py not available. Please install:")
        print("  pip install unitree_sdk2py")
        sys.exit(1)
    
    try:
        bridge = DdsRos2Bridge()
        bridge.spin()
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    finally:
        if 'bridge' in locals():
            bridge.shutdown()


if __name__ == "__main__":
    main()
