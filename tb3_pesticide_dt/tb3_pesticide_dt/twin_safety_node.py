#!/usr/bin/env python3
"""Safety bridge that keeps the real robot and Gazebo twin in the same command stream."""

import json
import math
import time
from copy import deepcopy
from typing import Dict, List, Tuple

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


class TwinSafetyNode(Node):
    def __init__(self):
        super().__init__("twin_safety_node")
        self._declare_parameters()

        self.real = self._new_scan_state()
        self.sim = self._new_scan_state()
        self.last_blocked = None

        if bool(self.get_parameter("use_real_scan").value):
            self.create_subscription(
                LaserScan,
                str(self.get_parameter("real_scan_topic").value),
                lambda msg: self.on_scan("real", msg),
                qos_profile_sensor_data,
            )

        if bool(self.get_parameter("use_sim_scan").value):
            self.create_subscription(
                LaserScan,
                str(self.get_parameter("sim_scan_topic").value),
                lambda msg: self.on_scan("sim", msg),
                qos_profile_sensor_data,
            )

        self.create_subscription(
            TwistStamped,
            str(self.get_parameter("input_cmd_topic").value),
            self.on_cmd,
            10,
        )

        self.pub_real_cmd = self.create_publisher(
            TwistStamped,
            str(self.get_parameter("real_cmd_topic").value),
            10,
        )
        self.pub_sim_cmd = self.create_publisher(
            TwistStamped,
            str(self.get_parameter("sim_cmd_topic").value),
            10,
        )
        self.pub_state = self.create_publisher(
            String,
            str(self.get_parameter("safety_state_topic").value),
            10,
        )

        self.get_logger().info(
            "TwinSafetyNode started. "
            f"raw={self.get_parameter('input_cmd_topic').value} "
            f"real={self.get_parameter('real_cmd_topic').value} "
            f"sim={self.get_parameter('sim_cmd_topic').value}"
        )

    def _declare_parameters(self):
        self.declare_parameter("real_scan_topic", "/scan")
        self.declare_parameter("sim_scan_topic", "/sim/scan")
        self.declare_parameter("input_cmd_topic", "/cmd_vel_raw")
        self.declare_parameter("real_cmd_topic", "/cmd_vel")
        self.declare_parameter("sim_cmd_topic", "/sim/cmd_vel")
        self.declare_parameter("safety_state_topic", "/dt/safety_state")
        self.declare_parameter("use_real_scan", True)
        self.declare_parameter("use_sim_scan", True)
        self.declare_parameter("publish_to_real", True)
        self.declare_parameter("publish_to_sim", True)
        self.declare_parameter("stop_distance", 0.45)
        self.declare_parameter("front_angle_deg", 30.0)
        self.declare_parameter("scan_timeout_s", 1.0)
        self.declare_parameter("fail_safe_on_stale_scan", False)

    @staticmethod
    def _new_scan_state() -> Dict:
        return {
            "min_front_m": float("inf"),
            "blocked": False,
            "last_scan_time": None,
            "stale": True,
        }

    def on_scan(self, source: str, msg: LaserScan):
        state = self.real if source == "real" else self.sim
        min_front, blocked = self.evaluate_front_obstacle(msg)
        state["min_front_m"] = min_front
        state["blocked"] = blocked
        state["last_scan_time"] = time.monotonic()
        state["stale"] = False

    def evaluate_front_obstacle(self, msg: LaserScan) -> Tuple[float, bool]:
        front_ranges = self.front_arc_ranges(msg, float(self.get_parameter("front_angle_deg").value))
        valid = [
            r
            for r in front_ranges
            if math.isfinite(r) and msg.range_min < r < msg.range_max
        ]
        if not valid:
            return float("inf"), False
        min_distance = min(valid)
        return min_distance, min_distance < float(self.get_parameter("stop_distance").value)

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

    def on_cmd(self, msg: TwistStamped):
        blocked, reasons = self.compute_blocked()
        forward_requested = msg.twist.linear.x > 0.0

        out = TwistStamped()
        out.header = msg.header
        out.twist = deepcopy(msg.twist)

        if blocked and forward_requested:
            out.twist.linear.x = 0.0
            out.twist.linear.y = 0.0
            out.twist.linear.z = 0.0
            out.twist.angular.x = 0.0
            out.twist.angular.y = 0.0
            # Turning in place is allowed so the robot can rotate away from the obstacle.
            out.twist.angular.z = msg.twist.angular.z

        if bool(self.get_parameter("publish_to_real").value):
            self.pub_real_cmd.publish(out)
        if bool(self.get_parameter("publish_to_sim").value):
            self.pub_sim_cmd.publish(out)

        self.publish_state(blocked, reasons, msg, out)

        if self.last_blocked is None or self.last_blocked != blocked:
            text = f"safety_blocked={blocked} reasons={','.join(reasons) or 'clear'}"
            if blocked:
                self.get_logger().warn(text)
            else:
                self.get_logger().info(text)
            self.last_blocked = blocked

    def compute_blocked(self) -> Tuple[bool, List[str]]:
        reasons = []
        now = time.monotonic()
        timeout = float(self.get_parameter("scan_timeout_s").value)
        fail_safe = bool(self.get_parameter("fail_safe_on_stale_scan").value)

        checks = []
        if bool(self.get_parameter("use_real_scan").value):
            checks.append(("real", self.real))
        if bool(self.get_parameter("use_sim_scan").value):
            checks.append(("sim", self.sim))

        for name, state in checks:
            last_scan_time = state["last_scan_time"]
            stale = last_scan_time is None or (now - last_scan_time) > timeout
            state["stale"] = stale
            if stale and fail_safe:
                reasons.append(f"{name}_scan_stale")
                continue
            if state["blocked"]:
                reasons.append(f"{name}_front_obstacle")

        return bool(reasons), reasons

    def publish_state(self, blocked: bool, reasons: List[str], raw: TwistStamped, safe: TwistStamped):
        payload = {
            "entity": "twin_safety_node",
            "blocked": blocked,
            "reasons": reasons,
            "stop_distance_m": float(self.get_parameter("stop_distance").value),
            "real": self.format_scan_state(self.real),
            "sim": self.format_scan_state(self.sim),
            "raw_cmd": {
                "linear_x": raw.twist.linear.x,
                "angular_z": raw.twist.angular.z,
            },
            "safe_cmd": {
                "linear_x": safe.twist.linear.x,
                "angular_z": safe.twist.angular.z,
            },
        }
        self.pub_state.publish(String(data=json.dumps(payload, sort_keys=True)))

    @staticmethod
    def format_scan_state(state: Dict) -> Dict:
        min_front = state["min_front_m"]
        return {
            "min_front_m": None if math.isinf(min_front) else round(float(min_front), 3),
            "blocked": bool(state["blocked"]),
            "stale": bool(state["stale"]),
        }


def main(args=None):
    rclpy.init(args=args)
    node = TwinSafetyNode()
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
