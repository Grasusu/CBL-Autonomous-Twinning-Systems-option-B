#!/usr/bin/env python3
"""Records Option B demo evidence topics to a JSONL file while publishing a status topic."""

import json
import os
import time
from pathlib import Path
from typing import Dict

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String


class EvidenceRecorderNode(Node):
    """Subscribes to the raw proof topics and writes every message to disk."""

    def __init__(self):
        super().__init__("evidence_recorder_node")
        self._declare_parameters()

        output_path = Path(str(self.get_parameter("output_path").value)).expanduser()
        output_path.parent.mkdir(parents=True, exist_ok=True)
        self.output_path = output_path
        self.file_handle = output_path.open("a", encoding="utf-8")

        self.counts: Dict[str, int] = {}
        self.latest: Dict[str, Dict] = {}
        self.started_at = time.time()

        self._subscribe("physical_mission_state", "/dt/physical/mission_state")
        self._subscribe("physical_inspection_request", "/dt/physical/inspection_request")
        self._subscribe("physical_environment_state", "/dt/physical/environment_state")
        self._subscribe("digital_mission_state", "/dt/digital/mission_state")
        self._subscribe("digital_inspection_result", "/dt/digital/inspection_result")
        self._subscribe("digital_control", "/dt/digital/control")
        self._subscribe("digital_dashboard", "/dt/digital/dashboard")
        self._subscribe("physical_inspection_log", "/dt/physical/inspection_log")
        self._subscribe("demo_evidence", "/dt/demo_evidence")

        self.pub_status = self.create_publisher(String, "/dt/evidence_recording", 10)
        self.create_timer(1.0, self.publish_status)
        self.get_logger().info(f"Recording demo evidence to {self.output_path}")

    def _declare_parameters(self):
        self.declare_parameter("output_path", "/tmp/tb3_option_b_demo_evidence.jsonl")

    def _subscribe(self, label: str, topic: str):
        self.counts[label] = 0
        self.create_subscription(String, topic, lambda msg, name=label, topic_name=topic: self.on_msg(name, topic_name, msg), 10)

    def on_msg(self, label: str, topic: str, msg: String):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            payload = {"raw": msg.data}

        self.counts[label] += 1
        self.latest[label] = payload
        record = {
            "received_wall_time_s": round(time.time(), 3),
            "label": label,
            "topic": topic,
            "data": payload,
        }
        self.file_handle.write(json.dumps(record, sort_keys=True) + "\n")
        self.file_handle.flush()

    def publish_status(self):
        demo = self.latest.get("demo_evidence", {})
        latest_log = self.latest.get("physical_inspection_log", {})
        environment = self.latest.get("physical_environment_state", {})
        payload = {
            "event": "EVIDENCE_RECORDING",
            "output_path": str(self.output_path),
            "uptime_s": round(time.time() - self.started_at, 1),
            "message_counts": self.counts,
            "rubric_ready": demo.get("rubric_ready", {}),
            "latest_environment": {
                "environment_mode": environment.get("environment_mode"),
                "front_obstacle": environment.get("front_obstacle"),
                "min_front_m": environment.get("min_front_m"),
            },
            "latest_log_event": {
                "event": latest_log.get("event"),
                "zone_id": latest_log.get("zone_id"),
                "status": latest_log.get("status"),
                "recommendation": latest_log.get("recommendation"),
            },
        }
        self.pub_status.publish(String(data=json.dumps(payload, sort_keys=True)))

    def destroy_node(self):
        try:
            self.file_handle.close()
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = EvidenceRecorderNode()
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
