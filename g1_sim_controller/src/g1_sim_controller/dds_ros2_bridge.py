#!/usr/bin/env python3
# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0
"""
DDS-ROS2 Bridge Node for Unitree G1 Navigation

Bridges:
  - ROS2 /cmd_vel (Twist) → DDS rt/run_command/cmd
  - Shared-memory robot state → ROS2 /odom (Odometry)
  - Shared-memory robot state → ROS2 /tf (odom → base_link)
"""

import time
import threading
import math
import json
import numpy as np
from typing import Optional, Tuple
from multiprocessing import shared_memory

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster

# Unitree DDS SDK
try:
    from unitree_sdk2py.core.channel import (
        ChannelPublisher,
        ChannelSubscriber,
        ChannelFactoryInitialize,
    )
    from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_
    DDS_AVAILABLE = True
except ImportError:
    DDS_AVAILABLE = False
    print("WARNING: unitree_sdk2py not available. Install for DDS communication.")


class DdsRos2Bridge(Node):
    """ROS 2 node that bridges /cmd_vel → DDS and shared-memory odom → /odom + /tf."""

    def __init__(self):
        super().__init__('dds_ros2_bridge')

        # ── Parameters ──────────────────────────────────────────────
        self.declare_parameter('network_interface', 'ens5')
        self.declare_parameter('default_height', 0.8)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('velocity_scale', 1.0)

        network_interface = self.get_parameter('network_interface').get_parameter_value().string_value
        self.default_height = self.get_parameter('default_height').get_parameter_value().double_value
        self.odom_frame = self.get_parameter('odom_frame').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.publish_tf_flag = self.get_parameter('publish_tf').get_parameter_value().bool_value
        self.velocity_scale = self.get_parameter('velocity_scale').get_parameter_value().double_value

        # ── Robot state tracking ────────────────────────────────────
        self._position = np.array([0.0, 0.0, 0.0])
        self._prev_position = np.array([0.0, 0.0, 0.0])
        self._orientation = np.array([0.0, 0.0, 0.0, 1.0])  # x, y, z, w
        self._prev_orientation = np.array([0.0, 0.0, 0.0, 1.0])
        self._linear_vel = np.array([0.0, 0.0, 0.0])
        self._angular_vel = np.array([0.0, 0.0, 0.0])
        self._last_update_time = time.time()
        self._state_lock = threading.Lock()
        self._first_reading = True

        # ── Shared memory (simulation robot state) ──────────────────
        self._robot_state_shm: Optional[shared_memory.SharedMemory] = None
        self._shm_lock = threading.Lock()
        self._shm_connection_attempts = 0
        self._try_connect_shared_memory()

        # ── DDS initialisation ──────────────────────────────────────
        if DDS_AVAILABLE:
            self.get_logger().info(f"Initializing DDS on interface: {network_interface}")
            ChannelFactoryInitialize(0, network_interface)
            self.dds_cmd_pub = ChannelPublisher("rt/run_command/cmd", String_)
            self.dds_cmd_pub.Init()
            self.get_logger().info("DDS publisher ready on 'rt/run_command/cmd'")
        else:
            self.dds_cmd_pub = None

        # ── ROS 2 pub / sub ─────────────────────────────────────────
        nav_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self._cmd_vel_callback, 10
        )
        self.odom_pub = self.create_publisher(Odometry, '/odom', nav_qos)

        if self.publish_tf_flag:
            self.tf_broadcaster = TransformBroadcaster(self)

        # Odometry publishing timer (50 Hz)
        self.create_timer(0.02, self._publish_odometry)

        if self.velocity_scale != 1.0:
            self.get_logger().info(f"Velocity scale: {self.velocity_scale}x")
        self.get_logger().info("dds_ros2_bridge node started")
        self.get_logger().info("  Subscribing: /cmd_vel")
        self.get_logger().info("  Publishing:  /odom")
        if self.publish_tf_flag:
            self.get_logger().info("  Broadcasting TF: odom → base_link")

    # ── Shared memory helpers ───────────────────────────────────────

    def _try_connect_shared_memory(self) -> bool:
        if self._robot_state_shm is not None:
            return True
        try:
            self._robot_state_shm = shared_memory.SharedMemory(name="isaac_robot_state")
            self.get_logger().info("Connected to shared memory: isaac_robot_state")
            return True
        except FileNotFoundError:
            self._shm_connection_attempts += 1
            if self._shm_connection_attempts == 1:
                self.get_logger().warn(
                    "Shared memory 'isaac_robot_state' not found – will retry. "
                    "Using dead-reckoning from /cmd_vel in the meantime."
                )
            return False
        except Exception as e:
            self._shm_connection_attempts += 1
            if self._shm_connection_attempts == 1:
                self.get_logger().warn(f"Failed to connect to shared memory: {e}")
            return False

    def _read_robot_state_from_shm(self) -> Optional[dict]:
        if self._robot_state_shm is None:
            self._shm_connection_attempts += 1
            if self._shm_connection_attempts % 100 == 0:
                self._try_connect_shared_memory()
            return None
        try:
            with self._shm_lock:
                timestamp = int.from_bytes(self._robot_state_shm.buf[0:4], 'little')
                data_len = int.from_bytes(self._robot_state_shm.buf[4:8], 'little')
                if data_len == 0:
                    return None
                json_bytes = bytes(self._robot_state_shm.buf[8:8 + data_len])
                return json.loads(json_bytes.decode('utf-8'))
        except Exception:
            try:
                self._robot_state_shm.close()
            except Exception:
                pass
            self._robot_state_shm = None
            return None

    # ── /cmd_vel → DDS ──────────────────────────────────────────────

    def _cmd_vel_callback(self, msg: Twist):
        if self.dds_cmd_pub is None:
            self.get_logger().warn("DDS publisher not available")
            return

        x_vel = float(msg.linear.x) * self.velocity_scale
        y_vel = float(msg.linear.y) * self.velocity_scale
        yaw_vel = float(msg.angular.z) * self.velocity_scale

        cmd_str = str([float(x_vel), -float(y_vel), -float(yaw_vel), float(self.default_height)])
        self.get_logger().info(f"Forwarding - cmd_vel: x={x_vel:.2f}, y={y_vel:.2f}, w={yaw_vel:.2f}")
        self.dds_cmd_pub.Write(String_(data=cmd_str))
        self.get_logger().info(f"Forwarded - cmd_vel: x={x_vel:.2f}, y={y_vel:.2f}, w={yaw_vel:.2f}")

        with self._state_lock:
            self._linear_vel[0] = msg.linear.x
            self._linear_vel[1] = msg.linear.y
            self._angular_vel[2] = msg.angular.z

    # ── Odometry + TF ──────────────────────────────────────────────

    def _publish_odometry(self):
        current_time = time.time()
        robot_state = self._read_robot_state_from_shm()

        with self._state_lock:
            dt = current_time - self._last_update_time
            self._last_update_time = current_time

            if robot_state and 'imu_data' in robot_state:
                imu = robot_state['imu_data']
                if len(imu) >= 13:
                    self._prev_position = self._position.copy()

                    self._position[0] = imu[0]
                    self._position[1] = imu[1]
                    self._position[2] = imu[2]

                    qw, qx, qy, qz = imu[3], imu[4], imu[5], imu[6]
                    self._orientation = np.array([qx, qy, qz, qw])

                    self._angular_vel[0] = imu[10]
                    self._angular_vel[1] = imu[11]
                    self._angular_vel[2] = imu[12]

                    if not self._first_reading and dt > 0.001:
                        world_vel = (self._position - self._prev_position) / dt
                        yaw = self._get_yaw_from_quaternion()
                        c, s = np.cos(-yaw), np.sin(-yaw)
                        self._linear_vel[0] = world_vel[0] * c - world_vel[1] * s
                        self._linear_vel[1] = world_vel[0] * s + world_vel[1] * c
                        self._linear_vel[2] = world_vel[2]

                    self._first_reading = False
            else:
                # Dead-reckoning fallback
                yaw = self._get_yaw_from_quaternion()
                self._position[0] += self._linear_vel[0] * np.cos(yaw) * dt
                self._position[0] -= self._linear_vel[1] * np.sin(yaw) * dt
                self._position[1] += self._linear_vel[0] * np.sin(yaw) * dt
                self._position[1] += self._linear_vel[1] * np.cos(yaw) * dt
                yaw += self._angular_vel[2] * dt
                self._orientation = self._quaternion_from_yaw(yaw)

            pos = self._position.copy()
            ori = self._orientation.copy()
            lin_vel = self._linear_vel.copy()
            ang_vel = self._angular_vel.copy()

        stamp = self.get_clock().now().to_msg()

        # Odometry message
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = pos[0]
        odom.pose.pose.position.y = pos[1]
        odom.pose.pose.position.z = pos[2]
        odom.pose.pose.orientation.x = ori[0]
        odom.pose.pose.orientation.y = ori[1]
        odom.pose.pose.orientation.z = ori[2]
        odom.pose.pose.orientation.w = ori[3]

        odom.twist.twist.linear.x = lin_vel[0]
        odom.twist.twist.linear.y = lin_vel[1]
        odom.twist.twist.linear.z = lin_vel[2]
        odom.twist.twist.angular.x = ang_vel[0]
        odom.twist.twist.angular.y = ang_vel[1]
        odom.twist.twist.angular.z = ang_vel[2]

        odom.pose.covariance[0] = 0.01
        odom.pose.covariance[7] = 0.01
        odom.pose.covariance[35] = 0.01

        self.odom_pub.publish(odom)

        # TF broadcast
        if self.publish_tf_flag:
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

    # ── Math helpers ────────────────────────────────────────────────

    def _get_yaw_from_quaternion(self) -> float:
        x, y, z, w = self._orientation
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def _quaternion_from_yaw(self, yaw: float) -> np.ndarray:
        return np.array([0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)])

    # ── Cleanup ─────────────────────────────────────────────────────

    def destroy_node(self):
        if self._robot_state_shm is not None:
            try:
                self._robot_state_shm.close()
                self.get_logger().info("Closed shared memory connection")
            except Exception as e:
                self.get_logger().warn(f"Error closing shared memory: {e}")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DdsRos2Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
