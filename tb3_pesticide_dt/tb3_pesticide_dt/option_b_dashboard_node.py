#!/usr/bin/env python3
"""Digital control-panel entity for the Option B demo.

This node replaces the fragile second Gazebo robot in the final Option B
presentation.  It is the digital entity users can show directly: it mirrors
mission/environment/inspection state, publishes a compact dashboard topic, and
sends a real digital control command into the twin system.
"""

import json
import time
from typing import Dict, Optional

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String


class OptionBDashboardNode(Node):
    """A ROS-native dashboard/control panel for the digital twin side."""

    def __init__(self):
        super().__init__("option_b_dashboard_node")
        self._declare_parameters()

        self.latest: Dict[str, Dict] = {}
        self.counts: Dict[str, int] = {}
        self.last_seen: Dict[str, float] = {}
        self.started_at = time.monotonic()
        self.last_log_at = 0.0
        self.startup_control_sent = False
        self.last_control_command: Optional[Dict] = None
        self.inspection_history = []

        self._subscribe("physical_state", "physical_state_topic")
        self._subscribe("digital_state", "digital_state_topic")
        self._subscribe("inspection_request", "inspection_request_topic")
        self._subscribe("inspection_result", "inspection_result_topic")
        self._subscribe("inspection_log", "inspection_log_topic")
        self._subscribe("environment_state", "environment_state_topic")
        self._subscribe("demo_evidence", "demo_evidence_topic")

        self.pub_dashboard = self.create_publisher(
            String,
            str(self.get_parameter("dashboard_topic").value),
            10,
        )
        self.pub_dashboard_summary = self.create_publisher(
            String,
            str(self.get_parameter("dashboard_summary_topic").value),
            10,
        )
        self.pub_control = self.create_publisher(
            String,
            str(self.get_parameter("digital_control_topic").value),
            10,
        )

        period = float(self.get_parameter("publish_period_s").value)
        self.create_timer(period, self.tick)
        self.get_logger().info(
            "Option B digital control panel started. "
            "Run `ros2 run tb3_pesticide_dt option_b_dashboard_viewer` for the readable dashboard."
        )

    def _declare_parameters(self):
        self.declare_parameter("physical_state_topic", "/dt/physical/mission_state")
        self.declare_parameter("digital_state_topic", "/dt/digital/mission_state")
        self.declare_parameter("inspection_request_topic", "/dt/physical/inspection_request")
        self.declare_parameter("inspection_result_topic", "/dt/digital/inspection_result")
        self.declare_parameter("inspection_log_topic", "/dt/physical/inspection_log")
        self.declare_parameter("environment_state_topic", "/dt/physical/environment_state")
        self.declare_parameter("demo_evidence_topic", "/dt/demo_evidence")
        self.declare_parameter("digital_control_topic", "/dt/digital/control")
        self.declare_parameter("dashboard_topic", "/dt/digital/dashboard")
        self.declare_parameter("dashboard_summary_topic", "/dt/digital/dashboard_summary")
        self.declare_parameter("publish_period_s", 1.0)
        self.declare_parameter("log_period_s", 12.0)
        self.declare_parameter("publish_startup_control", True)
        self.declare_parameter("startup_camera_health", "healthy")

    def _subscribe(self, key: str, parameter_name: str):
        topic = str(self.get_parameter(parameter_name).value)
        self.counts[key] = 0
        self.create_subscription(String, topic, lambda msg, name=key: self.on_json(name, msg), 10)

    def on_json(self, key: str, msg: String):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            data = {"raw": msg.data}
        self.latest[key] = data
        self.counts[key] = self.counts.get(key, 0) + 1
        self.last_seen[key] = time.monotonic()
        if key == "inspection_log":
            self.update_inspection_history(data)

    def update_inspection_history(self, data: Dict):
        event = data.get("event")
        if event == "MISSION_SUMMARY" and isinstance(data.get("inspections"), list):
            self.inspection_history = [self.compact_inspection(item) for item in data["inspections"]]
            if data.get("final_status"):
                self.upsert_inspection(
                    {
                        "event": "RETURN_HOME_LOG",
                        "zone_id": "plant_home",
                        "zone_name": "Home / Start",
                        "status": data.get("final_status"),
                        "recommendation": "MISSION_COMPLETE",
                    }
                )
            return

        if event in {"INSPECTION_LOG", "RETURN_HOME_LOG"}:
            self.upsert_inspection(data)

    def upsert_inspection(self, data: Dict):
        entry = self.compact_inspection(data)
        zone_id = entry.get("zone_id")
        if not zone_id:
            return
        for i, existing in enumerate(self.inspection_history):
            if existing.get("zone_id") == zone_id:
                self.inspection_history[i] = entry
                break
        else:
            self.inspection_history.append(entry)
        self.inspection_history = self.inspection_history[-8:]

    @staticmethod
    def compact_inspection(data: Dict) -> Dict:
        return {
            "zone_id": data.get("zone_id"),
            "zone_name": data.get("zone_name"),
            "status": data.get("status"),
            "plant_stress_index": data.get("plant_stress_index"),
            "disease_level": data.get("disease_level"),
            "recommendation": data.get("recommendation"),
            "confidence": data.get("confidence"),
            "camera_health": data.get("camera_health"),
        }

    def tick(self):
        if not self.startup_control_sent and bool(self.get_parameter("publish_startup_control").value):
            self.publish_camera_health_command(str(self.get_parameter("startup_camera_health").value))
            self.startup_control_sent = True

        payload = self.build_dashboard_payload()
        self.pub_dashboard.publish(String(data=json.dumps(payload, sort_keys=True)))
        self.pub_dashboard_summary.publish(String(data=self.build_dashboard_summary(payload)))
        self.log_dashboard(payload)

    def publish_camera_health_command(self, camera_health: str):
        command = {
            "event": "DIGITAL_CONTROL",
            "source_entity": "digital_control_panel",
            "camera_health": camera_health.lower(),
            "reason": "initialize digital twin sensor state",
            "sent_at_uptime_s": round(time.monotonic() - self.started_at, 2),
        }
        self.pub_control.publish(String(data=json.dumps(command, sort_keys=True)))
        self.last_control_command = command
        self.get_logger().info(
            f"Published digital control command on {self.get_parameter('digital_control_topic').value}: "
            f"camera_health={command['camera_health']}"
        )

    def build_dashboard_payload(self) -> Dict:
        physical = self.latest.get("physical_state", {})
        digital = self.latest.get("digital_state", {})
        inspection = self.latest.get("inspection_log", {})
        environment = self.latest.get("environment_state", {})
        evidence = self.latest.get("demo_evidence", {})

        latest_result = digital.get("latest_result") or {}
        if inspection.get("event") in {"INSPECTION_LOG", "RETURN_HOME_LOG", "MISSION_SUMMARY"}:
            latest_log = inspection
        else:
            latest_log = {}

        return {
            "event": "DIGITAL_DASHBOARD",
            "entity": "digital_control_panel",
            "uptime_s": round(time.monotonic() - self.started_at, 1),
            "physical_standin": {
                "mode": physical.get("mode"),
                "zone": physical.get("current_zone_id"),
                "pose": physical.get("pose"),
                "nav_feedback": physical.get("nav_feedback"),
                "completed_inspections": physical.get("completed_inspections"),
                "final_status": physical.get("final_status"),
                # Camera health set on the digital side, mirrored onto the physical
                # mission_state -- the cross-entity reflection for the R2 demo.
                "digital_camera_health": physical.get("digital_camera_health"),
            },
            "digital_twin": {
                "mode": digital.get("mode"),
                "mirrored_zone": digital.get("mirrored_zone_id"),
                "mirrored_pose": digital.get("mirrored_pose"),
                "camera_health": digital.get("camera_health"),
                "latest_result": latest_result,
            },
            "environment": {
                "mode": environment.get("environment_mode"),
                "front_obstacle": environment.get("front_obstacle"),
                "min_front_m": environment.get("min_front_m"),
                "scan_stale": environment.get("scan_stale"),
            },
            "latest_log": {
                "event": latest_log.get("event"),
                "zone_id": latest_log.get("zone_id"),
                "status": latest_log.get("status"),
                "recommendation": latest_log.get("recommendation"),
                "final_status": latest_log.get("final_status"),
            },
            "inspection_history": list(self.inspection_history),
            "rubric_ready": evidence.get("rubric_ready", {}),
            "message_counts": dict(self.counts),
            "last_control_command": self.last_control_command,
            "operator_topics": {
                "dashboard": str(self.get_parameter("dashboard_topic").value),
                "dashboard_summary": str(self.get_parameter("dashboard_summary_topic").value),
                "control": str(self.get_parameter("digital_control_topic").value),
            },
        }

    @staticmethod
    def short_status(status) -> str:
        if status == "TREATMENT_NEEDED":
            return "TREAT"
        if status in {"RETURNED_HOME", "RETURN_SUCCEEDED"}:
            return "HOME"
        if status is None:
            return "-"
        return str(status)

    @staticmethod
    def ready_flag(value) -> str:
        return "OK" if bool(value) else "WAIT"

    def build_dashboard_summary(self, payload: Dict) -> str:
        physical = payload["physical_standin"]
        digital = payload["digital_twin"]
        env = payload["environment"]
        rubric = payload["rubric_ready"]
        history = payload.get("inspection_history", [])
        plant_history = [item for item in history if item.get("zone_id") != "plant_home"]
        last = plant_history[-1] if plant_history else {}
        nav = physical.get("nav_feedback") or {}

        rubric_text = (
            f"B={self.ready_flag(rubric.get('bidirectional_pubsub'))} "
            f"S={self.ready_flag(rubric.get('state_synchronization'))} "
            f"E={self.ready_flag(rubric.get('environmental_interaction'))}"
        )
        return (
            "SMARTLE "
            f"mode={physical.get('mode') or '-'} "
            f"zone={physical.get('zone') or '-'} "
            f"progress={len(plant_history)}/6 "
            f"last={last.get('zone_id') or '-'}:{self.short_status(last.get('status'))} "
            f"stress={last.get('plant_stress_index', '-')} "
            f"action={last.get('recommendation') or '-'} "
            f"mirror={digital.get('mirrored_zone') or '-'} "
            f"camera={digital.get('camera_health') or '-'} "
            f"env={env.get('mode') or '-'} "
            f"front={env.get('min_front_m', '-')}m "
            f"nav_left={nav.get('distance_remaining', '-')}m "
            f"rubric[{rubric_text}]"
        )

    def log_dashboard(self, payload: Dict):
        log_period = float(self.get_parameter("log_period_s").value)
        now = time.monotonic()
        if log_period <= 0.0 or (now - self.last_log_at) < log_period:
            return
        self.last_log_at = now
        physical = payload["physical_standin"]
        digital = payload["digital_twin"]
        env = payload["environment"]
        rubric = payload["rubric_ready"]
        history = payload.get("inspection_history", [])
        last = history[-1] if history else {}
        rubric_text = (
            f"B={bool(rubric.get('bidirectional_pubsub'))} "
            f"S={bool(rubric.get('state_synchronization'))} "
            f"E={bool(rubric.get('environmental_interaction'))}"
        )
        self.get_logger().info(
            "Dashboard | "
            f"mode={physical.get('mode')} zone={physical.get('zone')} "
            f"inspections={len([item for item in history if item.get('zone_id') != 'plant_home'])}/6 "
            f"last={last.get('zone_id')}:{last.get('status')} "
            f"camera={digital.get('camera_health')} env={env.get('mode')} "
            f"rubric={rubric_text}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = OptionBDashboardNode()
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
