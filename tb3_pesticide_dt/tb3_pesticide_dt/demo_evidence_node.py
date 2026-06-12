#!/usr/bin/env python3
"""Aggregates the DT rubric evidence into one easy-to-echo topic."""

import json
import time
from typing import Dict, Optional

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String


class DemoEvidenceNode(Node):
    """Publishes a compact live summary of bidirectional, state, and environment evidence."""

    def __init__(self):
        super().__init__("demo_evidence_node")
        self._declare_parameters()

        self.latest: Dict[str, Dict] = {}
        self.last_seen: Dict[str, float] = {}
        self.counts: Dict[str, int] = {}
        self.evidence_seen = {
            "physical_to_digital": False,
            "digital_to_physical": False,
            "mission_or_pose_sync": False,
            "internal_state_sync": False,
            "environment_state": False,
            "environment_change": False,
            "front_obstacle": False,
            "inspection_log": False,
        }
        self.environment_min_front_seen = None
        self.environment_max_front_seen = None

        self._subscribe("physical_state", "physical_state_topic")
        self._subscribe("digital_state", "digital_state_topic")
        self._subscribe("inspection_request", "inspection_request_topic")
        self._subscribe("inspection_result", "inspection_result_topic")
        self._subscribe("inspection_log", "inspection_log_topic")
        self._subscribe("environment_state", "environment_state_topic")
        self._subscribe("digital_control", "digital_control_topic")
        self._subscribe("digital_dashboard", "digital_dashboard_topic")
        self._subscribe("digital_dashboard_summary", "digital_dashboard_summary_topic")

        self.pub_evidence = self.create_publisher(
            String,
            str(self.get_parameter("evidence_topic").value),
            10,
        )
        self.create_timer(float(self.get_parameter("publish_period_s").value), self.publish_evidence)
        self.get_logger().info(
            f"DemoEvidenceNode started. Echo {self.get_parameter('evidence_topic').value} "
            "during the final demo."
        )

    def _declare_parameters(self):
        self.declare_parameter("physical_state_topic", "/dt/physical/mission_state")
        self.declare_parameter("digital_state_topic", "/dt/digital/mission_state")
        self.declare_parameter("inspection_request_topic", "/dt/physical/inspection_request")
        self.declare_parameter("inspection_result_topic", "/dt/digital/inspection_result")
        self.declare_parameter("inspection_log_topic", "/dt/physical/inspection_log")
        self.declare_parameter("environment_state_topic", "/dt/physical/environment_state")
        self.declare_parameter("digital_control_topic", "/dt/digital/control")
        self.declare_parameter("digital_dashboard_topic", "/dt/digital/dashboard")
        self.declare_parameter("digital_dashboard_summary_topic", "/dt/digital/dashboard_summary")
        self.declare_parameter("evidence_topic", "/dt/demo_evidence")
        self.declare_parameter("publish_period_s", 1.0)
        self.declare_parameter("stale_after_s", 5.0)

    def _subscribe(self, key: str, parameter_name: str):
        topic = str(self.get_parameter(parameter_name).value)
        self.counts[key] = 0
        self.create_subscription(String, topic, lambda msg, name=key: self.on_json(name, msg), 10)

    def on_json(self, key: str, msg: String):
        try:
            self.latest[key] = json.loads(msg.data)
        except json.JSONDecodeError:
            self.latest[key] = {"raw": msg.data}
        self.counts[key] = self.counts.get(key, 0) + 1
        self.last_seen[key] = time.monotonic()

    def publish_evidence(self):
        now = time.monotonic()
        physical = self.latest.get("physical_state", {})
        digital = self.latest.get("digital_state", {})
        environment = self.latest.get("environment_state", {})
        inspection_log = self.latest.get("inspection_log", {})

        physical_to_digital_current = self.fresh("physical_state", now) and self.fresh("digital_state", now) and (
            digital.get("mirrored_zone_id") == physical.get("current_zone_id")
            or digital.get("mirrored_pose") is not None
            or digital.get("mode") == physical.get("mode")
        )
        digital_to_physical_current = self.fresh("inspection_result", now) or (
            self.fresh("digital_state", now)
            and str(physical.get("digital_camera_health", "unknown")).lower() != "unknown"
        )

        camera_health = str(
            digital.get("camera_health", physical.get("digital_camera_health", "unknown"))
        ).lower()
        mission_state_sync_current = physical_to_digital_current
        internal_state_sync_current = camera_health not in {"", "unknown", "none"}
        environment_current = self.fresh("environment_state", now)
        min_front = self.safe_float(environment.get("min_front_m"))
        if min_front is not None:
            if self.environment_min_front_seen is None or min_front < self.environment_min_front_seen:
                self.environment_min_front_seen = min_front
            if self.environment_max_front_seen is None or min_front > self.environment_max_front_seen:
                self.environment_max_front_seen = min_front
            if (
                self.environment_min_front_seen is not None
                and self.environment_max_front_seen is not None
                and (self.environment_max_front_seen - self.environment_min_front_seen) >= 0.10
            ):
                self.evidence_seen["environment_change"] = True

        self.evidence_seen["physical_to_digital"] |= bool(physical_to_digital_current)
        self.evidence_seen["digital_to_physical"] |= bool(digital_to_physical_current)
        self.evidence_seen["mission_or_pose_sync"] |= bool(mission_state_sync_current)
        self.evidence_seen["internal_state_sync"] |= bool(internal_state_sync_current)
        self.evidence_seen["environment_state"] |= bool(self.counts.get("environment_state", 0) > 0)
        self.evidence_seen["front_obstacle"] |= bool(environment.get("front_obstacle"))
        self.evidence_seen["inspection_log"] |= bool(self.counts.get("inspection_log", 0) > 0)

        bidirectional_ready = (
            self.evidence_seen["physical_to_digital"]
            and self.evidence_seen["digital_to_physical"]
            and self.counts.get("inspection_request", 0) > 0
            and self.counts.get("inspection_result", 0) > 0
        )
        state_ready = (
            self.evidence_seen["mission_or_pose_sync"]
            and self.evidence_seen["internal_state_sync"]
        )
        environment_ready = (
            self.evidence_seen["environment_state"]
            and (
                self.evidence_seen["environment_change"]
                or self.evidence_seen["front_obstacle"]
            )
        )

        payload = {
            "event": "DEMO_EVIDENCE",
            "rubric_ready": {
                "bidirectional_pubsub": bool(bidirectional_ready),
                "state_synchronization": bool(state_ready),
                "environmental_interaction": bool(environment_ready),
            },
            "bidirectional_pubsub": {
                "physical_to_digital_current": bool(physical_to_digital_current),
                "physical_to_digital_seen": bool(self.evidence_seen["physical_to_digital"]),
                "digital_to_physical_current": bool(digital_to_physical_current),
                "digital_to_physical_seen": bool(self.evidence_seen["digital_to_physical"]),
                "physical_topics": [
                    "/dt/physical/mission_state",
                    "/dt/physical/inspection_request",
                    "/dt/physical/environment_state",
                ],
                "digital_topics": [
                    "/dt/digital/mission_state",
                    "/dt/digital/inspection_result",
                    "/dt/digital/control",
                    "/dt/digital/dashboard",
                    "/dt/digital/dashboard_summary",
                ],
                "message_counts": {
                    "physical_state": self.counts.get("physical_state", 0),
                    "inspection_request": self.counts.get("inspection_request", 0),
                    "environment_state": self.counts.get("environment_state", 0),
                    "digital_state": self.counts.get("digital_state", 0),
                    "inspection_result": self.counts.get("inspection_result", 0),
                    "digital_control": self.counts.get("digital_control", 0),
                    "digital_dashboard": self.counts.get("digital_dashboard", 0),
                    "digital_dashboard_summary": self.counts.get("digital_dashboard_summary", 0),
                    "inspection_log": self.counts.get("inspection_log", 0),
                },
            },
            "state_synchronization": {
                "mission_or_pose_mirrored_current": bool(mission_state_sync_current),
                "mission_or_pose_mirrored_seen": bool(self.evidence_seen["mission_or_pose_sync"]),
                "camera_health": camera_health,
                "internal_state_current": bool(internal_state_sync_current),
                "internal_state_seen": bool(self.evidence_seen["internal_state_sync"]),
                "physical_mode": physical.get("mode"),
                "digital_mode": digital.get("mode"),
                "physical_zone": physical.get("current_zone_id"),
                "digital_mirrored_zone": digital.get("mirrored_zone_id"),
                "mirrored_pose": digital.get("mirrored_pose"),
            },
            "environmental_interaction": {
                "environment_state_current": bool(environment_current),
                "environment_state_seen": bool(self.evidence_seen["environment_state"]),
                "environment_change_seen": bool(self.evidence_seen["environment_change"]),
                "front_obstacle_seen": bool(self.evidence_seen["front_obstacle"]),
                "front_obstacle": environment.get("front_obstacle"),
                "min_front_m": environment.get("min_front_m"),
                "min_front_seen_m": self.environment_min_front_seen,
                "max_front_seen_m": self.environment_max_front_seen,
                "environment_mode": environment.get("environment_mode"),
                "scan_stale": environment.get("scan_stale"),
            },
            "latest_inspection": self.latest_result(inspection_log, self.latest.get("inspection_result")),
            "fresh_topics": {
                key: self.fresh(key, now)
                for key in [
                    "physical_state",
                    "digital_state",
                    "inspection_request",
                    "inspection_result",
                    "inspection_log",
                    "environment_state",
                    "digital_control",
                    "digital_dashboard",
                    "digital_dashboard_summary",
                ]
            },
        }
        self.pub_evidence.publish(String(data=json.dumps(payload, sort_keys=True)))

    @staticmethod
    def safe_float(value) -> Optional[float]:
        try:
            if value is None:
                return None
            return float(value)
        except (TypeError, ValueError):
            return None

    def fresh(self, key: str, now: Optional[float] = None) -> bool:
        if key not in self.last_seen:
            return False
        current = time.monotonic() if now is None else now
        return (current - self.last_seen[key]) <= float(self.get_parameter("stale_after_s").value)

    @staticmethod
    def latest_result(log_msg: Dict, result_msg: Optional[Dict]) -> Optional[Dict]:
        if log_msg.get("event") == "INSPECTION_LOG":
            return {
                "source": "inspection_log",
                "zone_id": log_msg.get("zone_id"),
                "status": log_msg.get("status"),
                "recommendation": log_msg.get("recommendation"),
                "camera_health": log_msg.get("camera_health"),
            }
        if result_msg:
            return {
                "source": "inspection_result",
                "zone_id": result_msg.get("zone_id"),
                "status": result_msg.get("status"),
                "recommendation": result_msg.get("recommendation"),
                "camera_health": result_msg.get("camera_health"),
            }
        return None


def main(args=None):
    rclpy.init(args=args)
    node = DemoEvidenceNode()
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
