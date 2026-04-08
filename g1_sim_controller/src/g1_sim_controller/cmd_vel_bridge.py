#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import json

# Import Unitree SDK components (Assumed available in Docker)
from unitree_sdk2py.core.channel import ChannelPublisher, ChannelFactoryInitialize
from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_

class CmdVelBridge(Node):
    def __init__(self):
        super().__init__('cmd_vel_bridge')

        # Declare parameters for network interface and height
        self.declare_parameter('network_interface', 'ens5')
        self.declare_parameter('default_height', 0.8)

        network_interface = self.get_parameter('network_interface').get_parameter_value().string_value
        self.default_height = self.get_parameter('default_height').get_parameter_value().double_value

        self.get_logger().info(f"Initializing Unitree SDK on interface: {network_interface}")
        
        # Initialize SDK
        try:
            ChannelFactoryInitialize(0, network_interface)
            self.cmd_pub = ChannelPublisher("rt/run_command/cmd", String_)
            self.cmd_pub.Init()
            self.get_logger().info("Unitree SDK Publisher initialized on 'rt/run_command/cmd'")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize Unitree SDK: {e}")
            raise e

        # Subscriber for cmd_vel
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        self.get_logger().info("cmd_vel_bridge node started, subscribing to /cmd_vel")

    def cmd_vel_callback(self, msg):
        """
        Receives geometry_msgs/msg/Twist and sends formatted string to simulation via DDS.
        """
        x = msg.linear.x
        y = msg.linear.y
        omega = msg.angular.z

        # Format: [vx, vy, w, height]
        # Same format used in sim_loco_service.py
        cmd_list = [float(x), float(y), float(omega), self.default_height]
        cmd_str = str(cmd_list)

        # Publish to DDS
        dds_msg = String_(data=cmd_str)
        try:
            self.get_logger().info(f"Forwarding - cmd_vel: x={x:.2f}, y={y:.2f}, w={omega:.2f}")
            self.cmd_pub.Write(dds_msg)
            self.get_logger().info(f"Forwarded - cmd_vel: x={x:.2f}, y={y:.2f}, w={omega:.2f}")
        except Exception as e:
            self.get_logger().error(f"Failed to write to DDS: {e}")

def main(args=None):
    rclpy.init(args=args)
    try:
        node = CmdVelBridge()
        rclpy.spin(node)
    except Exception as e:
        print(f"Error in cmd_vel_bridge: {e}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
