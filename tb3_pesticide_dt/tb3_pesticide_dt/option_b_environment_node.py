#!/usr/bin/env python3
"""Publishes Gazebo scan events as the Option B physical-stand-in environment state."""

import json
import math
import time
from typing import List, Optional

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


class OptionBEnvironmentNode(Node):
    """Turns Gazebo /scan into a DT-visible environment event stream."""

    def __init__(self):
        super().__init__("option_b_environment_node")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("environment_state_topic", "/dt/physical/environment_state")
        self.declare_parameter("source_entity", "gazebo_physical_standin")
        self.declare_parameter("publish_period_s", 0.25)
        self.declare_parameter("stop_distance_m", 0.45)
        self.declare_parameter("front_angle_deg", 30.0)
        self.declare_parameter("scan_timeout_s", 1.0)

        self.latest_scan_at: Optional[float] = None
        self.min_front_m: Optional[float] = None
        self.front_obstacle = False

        self.create_subscription(
            LaserScan,
            str(self.get_parameter("scan_topic").value),
            self.on_scan,
            qos_profile_sensor_data,
        )
        self.pub_state = self.create_publisher(
            String,
            str(self.get_parameter("environment_state_topic").value),
            10,
        )
        self.create_timer(float(self.get_parameter("publish_period_s").value), self.publish_state)
        self.get_logger().info(
            "OptionBEnvironmentNode started. "
            f"scan={self.get_parameter('scan_topic').value} "
            f"state={self.get_parameter('environment_state_topic').value}"
        )

    def on_scan(self, msg: LaserScan):
        ranges = self.front_arc_ranges(msg, float(self.get_parameter("front_angle_deg").value))
        valid = [r for r in ranges if math.isfinite(r) and msg.range_min < r < msg.range_max]
        self.latest_scan_at = time.monotonic()
        if not valid:
            self.min_front_m = None
            self.front_obstacle = False
            return
        self.min_front_m = min(valid)
        self.front_obstacle = self.min_front_m < float(self.get_parameter("stop_distance_m").value)

    @staticmethod
    def front_arc_ranges(scan_msg: LaserScan, front_angle_deg: float) -> List[float]:
        front_angle_rad = math.radians(front_angle_deg)
        selected = []
        for i, distance in enumerate(scan_msg.ranges):
            angle = scan_msg.angle_min + i * scan_msg.angle_increment
            angle = math.atan2(math.sin(angle), math.cos(angle))
            if abs(angle) <= front_angle_rad:
                selected.append(distance)
        return selected

    def publish_state(self):
        now = time.monotonic()
        timeout = float(self.get_parameter("scan_timeout_s").value)
        stale = self.latest_scan_at is None or (now - self.latest_scan_at) > timeout
        front_obstacle = bool(self.front_obstacle and not stale)
        payload = {
            "event": "ENVIRONMENT_STATE",
            "entity": str(self.get_parameter("source_entity").value),
            "scan_topic": str(self.get_parameter("scan_topic").value),
            "front_angle_deg": float(self.get_parameter("front_angle_deg").value),
            "stop_distance_m": float(self.get_parameter("stop_distance_m").value),
            "min_front_m": None if self.min_front_m is None else round(float(self.min_front_m), 3),
            "front_obstacle": front_obstacle,
            "scan_stale": bool(stale),
            "environment_mode": "OBSTACLE_AHEAD" if front_obstacle else ("SCAN_STALE" if stale else "CLEAR"),
        }
        self.pub_state.publish(String(data=json.dumps(payload, sort_keys=True)))


def main(args=None):
    rclpy.init(args=args)
    node = OptionBEnvironmentNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
