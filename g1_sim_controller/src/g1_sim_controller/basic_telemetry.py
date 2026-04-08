#!/usr/bin/env python3
import sys
import json
import time
import random
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class BasicTelemetry(Node):
    # FSM ID to Status Mapping
    FSM_ID_MAP = {
        0: "ZeroTorque",
        1: "Damp",
        2: "StandUp",
        3: "PositionControlSitDown",
        4: "LockStanding",
        200: "Start",
        702: "LieDown_StandUp",
        706: "BalanceSquat_SquatStand",
        500: "WalkMotion",
        501: "WalkMotion_3DofWaist",
        801: "Run",
    }

    def __init__(self):
        super().__init__('basic_telemetry')

        self.publisher_ = self.create_publisher(String, '/inorbit/custom_data', 10)

        # -- Simulated States --
        self.battery_percent = 100
        self.sim_integer = 100
        self.fsm_id = 1  # Starting with 'Damp' state

        # -- Rates/Intervals --
        self.status_rate = 1.0  # Seconds (1 Hz) for status/FSM
        self.battery_rate = 10.0 # Seconds for battery updates
        self.sim_rate = 10.0     # Seconds for sim_integer updates
        self.fsm_cycle_rate = 30.0 # Seconds to change simulated FSM

        # -- Last Update Trackers --
        now = self.get_clock().now().nanoseconds / 1e9
        self.last_battery_time = now
        self.last_sim_time = now
        self.last_fsm_change_time = now

        # Timer for polling status/fsm
        self.timer = self.create_timer(self.status_rate, self.timer_callback)

        self.get_logger().info("Simulated Basic Telemetry Node Started (No hardware used)")
        self.get_logger().info(f"Rates: Battery={self.battery_rate}s, Sim={self.sim_rate}s, FSM Cycle={self.fsm_cycle_rate}s")

    def publish_kv(self, key, value):
        msg = String()
        msg.data = f"{key}={value}"
        self.publisher_.publish(msg)

    def get_robot_status(self, fsm_id):
        busy_states = [3, 500, 501, 702, 706, 801]
        charging_states = []
        idle_states = [0, 1, 2, 4, 200]
        
        if fsm_id in busy_states:
            return "BUSY"
        elif fsm_id in charging_states:
            return "CHARGING"
        elif fsm_id in idle_states:
            return "IDLE"
        return "IDLE"

    def timer_callback(self):
        now = self.get_clock().now().nanoseconds / 1e9

        # --- Simulate Battery Discharge ---
        if now - self.last_battery_time >= self.battery_rate:
            self.battery_percent = max(0, self.battery_percent - 1)
            self.publish_kv("battery_percent", str(self.battery_percent))
            self.last_battery_time = now

        # --- Simulate Sim Integer Decrease ---
        if now - self.last_sim_time >= self.sim_rate:
            self.sim_integer -= 1
            self.publish_kv("sim_integer", str(self.sim_integer))
            self.last_sim_time = now

        # --- Simulate FSM State Changes ---
        if now - self.last_fsm_change_time >= self.fsm_cycle_rate:
            # Randomly switch to a new state for simulation
            self.fsm_id = random.choice(list(self.FSM_ID_MAP.keys()))
            self.get_logger().info(f"Simulated FSM Change to: {self.FSM_ID_MAP[self.fsm_id]} ({self.fsm_id})")
            self.last_fsm_change_time = now

        # --- Publish Core Status ---
        try:
            self.publish_kv("loco_fsm_id", str(self.fsm_id))
            status = self.get_robot_status(self.fsm_id)
            self.publish_kv("status", status)
        except Exception as e:
            self.get_logger().error(f"Error in timer callback: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = BasicTelemetry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
