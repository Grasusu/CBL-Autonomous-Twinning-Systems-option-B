#!/usr/bin/env python3
"""Readable terminal viewer for the Option B digital dashboard."""

import json
import math
import sys
import time
from typing import Dict, Optional

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String


def fmt_value(value, default="-"):
    if value is None:
        return default
    return str(value)


def fmt_float(value, digits: int = 2, default: str = "-"):
    try:
        if value is None:
            return default
        if isinstance(value, float) and not math.isfinite(value):
            return default
        return f"{float(value):.{digits}f}"
    except (TypeError, ValueError):
        return default


def flag(value) -> str:
    return "OK" if bool(value) else "WAIT"


def short_status(value) -> str:
    status = fmt_value(value)
    if status == "TREATMENT_NEEDED":
        return "TREAT"
    if status in {"RETURN_SUCCEEDED", "RETURNED_HOME"}:
        return "HOME"
    if status == "SENSOR_TIMEOUT":
        return "TIMEOUT"
    return status


def short_action(value) -> str:
    action = fmt_value(value)
    if action == "APPLY_TARGETED_TREATMENT":
        return "APPLY"
    if action == "NO_TREATMENT":
        return "NONE"
    if action == "MISSION_COMPLETE":
        return "DONE"
    return action


class OptionBDashboardViewer(Node):
    """Subscribes to /dt/digital/dashboard and prints a compact live panel."""

    def __init__(self):
        super().__init__("option_b_dashboard_viewer")
        self.declare_parameter("dashboard_topic", "/dt/digital/dashboard")
        self.declare_parameter("refresh_period_s", 0.5)
        self.declare_parameter("clear_screen", True)

        self.latest: Optional[Dict] = None
        self.last_message_at: Optional[float] = None
        self.last_render_at = 0.0

        topic = str(self.get_parameter("dashboard_topic").value)
        self.create_subscription(String, topic, self.on_dashboard, 10)
        self.create_timer(float(self.get_parameter("refresh_period_s").value), self.render)
        self.get_logger().info(f"Viewing {topic}. Keep this terminal open during the demo.")

    def on_dashboard(self, msg: String):
        try:
            self.latest = json.loads(msg.data)
            self.last_message_at = time.monotonic()
        except json.JSONDecodeError:
            self.latest = {"raw": msg.data}
            self.last_message_at = time.monotonic()

    def render(self):
        now = time.monotonic()
        if self.latest is None:
            self.print_panel(
                [
                    "SMARTLE Option B Digital Twin Dashboard",
                    "",
                    "Waiting for /dt/digital/dashboard ...",
                    "Start the mission launch if this stays empty.",
                ]
            )
            return

        data = self.latest
        physical = data.get("physical_standin", {})
        digital = data.get("digital_twin", {})
        env = data.get("environment", {})
        latest_log = data.get("latest_log", {})
        rubric = data.get("rubric_ready", {})
        counts = data.get("message_counts", {})
        nav = physical.get("nav_feedback") or {}
        pose = physical.get("pose") or {}
        latest_result = digital.get("latest_result") or {}
        control = data.get("last_control_command") or {}
        history = data.get("inspection_history") or []
        age = "-" if self.last_message_at is None else fmt_float(now - self.last_message_at, 1)

        result_zone = latest_log.get("zone_id") or latest_result.get("zone_id")
        result_status = latest_log.get("status") or latest_result.get("status")
        recommendation = latest_log.get("recommendation") or latest_result.get("recommendation")
        completed = len([item for item in history if item.get("zone_id") != "plant_home"])
        control_health = control.get("camera_health")

        lines = [
            "SMARTLE OPTION B DIGITAL TWIN",
            f"updated={age}s | uptime={fmt_value(data.get('uptime_s'))}s | progress={completed}/6",
            "",
            f"physical : mode={fmt_value(physical.get('mode'))} zone={fmt_value(physical.get('zone'))} final={fmt_value(physical.get('final_status'))}",
            f"pose     : x={fmt_float(pose.get('x'))} y={fmt_float(pose.get('y'))} yaw={fmt_float(pose.get('yaw'))} nav_left={fmt_float(nav.get('distance_remaining'))}m recov={fmt_value(nav.get('number_of_recoveries'), '0')}",
            f"digital  : mirrored={fmt_value(digital.get('mirrored_zone'))} camera={fmt_value(digital.get('camera_health'))} control_camera={fmt_value(control_health)}",
            f"latest   : {fmt_value(result_zone)} -> {short_status(result_status)} action={short_action(recommendation)}",
            f"env      : mode={fmt_value(env.get('mode'))} obstacle={fmt_value(env.get('front_obstacle'))} min_front={fmt_float(env.get('min_front_m'))}m",
            "",
            "INSPECTION RESULTS",
            "  plant     result   stress   action",
        ]

        if history:
            for item in history:
                zone_id = fmt_value(item.get("zone_id"))
                status = short_status(item.get("status"))
                stress = fmt_float(item.get("plant_stress_index") or item.get("disease_level"))
                action = short_action(item.get("recommendation"))
                if zone_id == "plant_home":
                    lines.append(f"  {zone_id:<9} {status:<8} {'-':<8} {action}")
                else:
                    lines.append(f"  {zone_id:<9} {status:<8} {stress:<8} {action}")
        else:
            lines.append("  waiting for first plant result")

        lines.extend(
            [
                "",
                "RUBRIC EVIDENCE",
                f"  bidirectional={flag(rubric.get('bidirectional_pubsub'))} | state_sync={flag(rubric.get('state_synchronization'))} | environment={flag(rubric.get('environmental_interaction'))}",
                f"  traffic: phys={fmt_value(counts.get('physical_state'), '0')} dig={fmt_value(counts.get('digital_state'), '0')} req={fmt_value(counts.get('inspection_request'), '0')} res={fmt_value(counts.get('inspection_result'), '0')} env={fmt_value(counts.get('environment_state'), '0')}",
                "",
                "Ctrl+C closes this viewer.",
            ]
        )
        self.print_panel(lines)

    def print_panel(self, lines):
        if bool(self.get_parameter("clear_screen").value):
            sys.stdout.write("\033[2J\033[H")
        sys.stdout.write("\n".join(lines) + "\n")
        sys.stdout.flush()


def main(args=None):
    rclpy.init(args=args)
    node = OptionBDashboardViewer()
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
