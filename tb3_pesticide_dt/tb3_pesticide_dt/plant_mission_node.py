#!/usr/bin/env python3
"""Autonomous route node for the plant-health inspection demo.

The node drives through predefined plant zones using odometry feedback, waits at
each zone to simulate inspection, asks the digital twin for a hyperspectral
classification, then logs the result before moving to the next zone.
"""

import json
import math
import time
from typing import Dict, Optional

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TwistStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

from tb3_pesticide_dt.pesticide_logic import (
    build_zones,
    clamp,
    is_treatment_needed_status,
    normalize_angle,
    quaternion_to_yaw,
    recommendation_for_status,
)


DEFAULT_ZONE_IDS = [
    "plant_a",
    "plant_b",
    "plant_c",
    "plant_d",
    "plant_e",
    "plant_f",
    "plant_g",
    "plant_h",
]
DEFAULT_ZONE_NAMES = [
    "Start bed",
    "Upper east bed",
    "East middle bed",
    "Lower east bed",
    "South middle bed",
    "South center bed",
    "West lower bed",
    "West return bed",
]
DEFAULT_ZONE_X = [0.25, 0.75, 0.85, 0.85, 0.45, -0.25, -0.65, -0.60]
DEFAULT_ZONE_Y = [-0.20, -0.45, -0.95, -1.45, -2.05, -2.10, -2.05, -1.35]
DEFAULT_ZONE_YAW = [0.0, -0.30, -1.20, -1.57, -2.20, 2.90, 2.70, 1.57]
DEFAULT_RESIDUES = [0.18, 0.74, 0.31, 0.56, 0.22, 0.81, 0.44, 0.63]
DEFAULT_STATUSES = [
    "OK",
    "TREATMENT_NEEDED",
    "OK",
    "TREATMENT_NEEDED",
    "OK",
    "TREATMENT_NEEDED",
    "OK",
    "TREATMENT_NEEDED",
]


def yaw_to_quaternion(yaw: float):
    half = yaw * 0.5
    return {
        "x": 0.0,
        "y": 0.0,
        "z": math.sin(half),
        "w": math.cos(half),
    }


class PlantMissionNode(Node):
    def __init__(self):
        super().__init__("plant_mission_node")

        self._declare_parameters()
        self.zones = self._load_zones()

        self.odom_topic = str(self.get_parameter("odom_topic").value)
        self.cmd_topic = str(self.get_parameter("cmd_topic").value)
        self.state_topic = str(self.get_parameter("state_topic").value)
        self.request_topic = str(self.get_parameter("inspection_request_topic").value)
        self.result_topic = str(self.get_parameter("inspection_result_topic").value)
        self.digital_state_topic = str(self.get_parameter("digital_state_topic").value)
        self.safety_state_topic = str(self.get_parameter("safety_state_topic").value)
        self.log_topic = str(self.get_parameter("inspection_log_topic").value)
        self.return_strategy = str(self.get_parameter("return_strategy").value).lower()
        self.nav_action_name = str(self.get_parameter("nav_action_name").value)
        self.nav2_cmd_topic = str(self.get_parameter("nav2_cmd_topic").value)
        self.nav2_return_cmd_topic = str(self.get_parameter("nav2_return_cmd_topic").value)

        self.pose = None
        self.start_pose = None
        self.mode = "WAITING_FOR_ODOM" if bool(self.get_parameter("start_automatically").value) else "IDLE"
        self.current_index = 0
        self.current_result: Optional[Dict] = None
        self.last_result: Optional[Dict] = None
        self.request_id = 0
        self.request_sent = False
        self.inspection_started_at: Optional[float] = None
        self.hold_until: Optional[float] = None
        self.mission_started_at = time.monotonic()
        self.summary = []
        self.digital_camera_health = "unknown"
        self.digital_mode = "unknown"
        self.last_state_publish_at = 0.0
        self.last_summary_publish_at = 0.0
        self.safety_blocked = False
        self.safety_reasons = []
        self.safety_blocked_since: Optional[float] = None
        self.recovery_until: Optional[float] = None
        self.recovery_attempts = 0
        self.recovery_direction = 1.0
        self.recovery_resume_mode = "NAVIGATING"
        self.returned_home = False
        self.return_status = "PENDING"
        self.return_targets = []
        self.return_target_index = 0
        self.nav_return_started_at: Optional[float] = None
        self.nav_goal_started_at: Optional[float] = None
        self.nav_goal_handle = None
        self.nav_goal_in_flight = False
        self.nav_return_pose_published_at: Optional[float] = None
        self.nav_feedback = {}
        self.nav2_initial_pose_published = False
        self.nav2_recovery_attempts = 0
        self.nav2_recovery_phase = None
        self.nav2_recovery_until: Optional[float] = None
        self.nav2_recovery_reason = None
        self.nav2_last_home_distance: Optional[float] = None
        self.nav2_last_progress_at: Optional[float] = None
        self.nav2_cancel_results_to_ignore = 0

        self.create_subscription(Odometry, self.odom_topic, self.on_odom, 10)
        self.create_subscription(String, self.result_topic, self.on_inspection_result, 10)
        self.create_subscription(String, self.digital_state_topic, self.on_digital_state, 10)
        self.create_subscription(String, self.safety_state_topic, self.on_safety_state, 10)
        self.create_subscription(TwistStamped, self.nav2_cmd_topic, self.on_nav2_cmd, 10)

        self.pub_cmd = self.create_publisher(TwistStamped, self.cmd_topic, 10)
        self.pub_nav2_return_cmd = self.create_publisher(TwistStamped, self.nav2_return_cmd_topic, 10)
        self.pub_state = self.create_publisher(String, self.state_topic, 10)
        self.pub_request = self.create_publisher(String, self.request_topic, 10)
        self.pub_log = self.create_publisher(String, self.log_topic, 10)
        self.pub_initial_pose = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
        self.nav_client = ActionClient(self, NavigateToPose, self.nav_action_name)

        period = float(self.get_parameter("control_period_s").value)
        self.create_timer(period, self.tick)

        self.get_logger().info(
            f"PlantMissionNode started with {len(self.zones)} zones. "
            f"Commands -> {self.cmd_topic}; inspection requests -> {self.request_topic}"
        )

    def _declare_parameters(self):
        self.declare_parameter("start_automatically", True)
        self.declare_parameter("source_entity", "physical_robot")

        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("cmd_topic", "/cmd_vel_raw")
        self.declare_parameter("state_topic", "/dt/physical/mission_state")
        self.declare_parameter("inspection_request_topic", "/dt/physical/inspection_request")
        self.declare_parameter("inspection_result_topic", "/dt/digital/inspection_result")
        self.declare_parameter("digital_state_topic", "/dt/digital/mission_state")
        self.declare_parameter("safety_state_topic", "/dt/safety_state")
        self.declare_parameter("inspection_log_topic", "/dt/physical/inspection_log")

        self.declare_parameter("zone_ids", DEFAULT_ZONE_IDS)
        self.declare_parameter("zone_names", DEFAULT_ZONE_NAMES)
        self.declare_parameter("zone_x", DEFAULT_ZONE_X)
        self.declare_parameter("zone_y", DEFAULT_ZONE_Y)
        self.declare_parameter("zone_yaw", DEFAULT_ZONE_YAW)
        self.declare_parameter("zone_plant_stress_indices", DEFAULT_RESIDUES)
        self.declare_parameter("zone_residue_indices", DEFAULT_RESIDUES)
        self.declare_parameter("zone_expected_statuses", DEFAULT_STATUSES)

        self.declare_parameter("control_period_s", 0.10)
        self.declare_parameter("arrival_tolerance_m", 0.12)
        self.declare_parameter("yaw_tolerance_rad", 0.25)
        self.declare_parameter("max_linear_speed", 0.16)
        self.declare_parameter("max_angular_speed", 0.85)
        self.declare_parameter("linear_gain", 0.65)
        self.declare_parameter("angular_gain", 1.80)
        self.declare_parameter("heading_slowdown_rad", 0.75)
        self.declare_parameter("inspection_duration_s", 3.0)
        self.declare_parameter("inspection_result_timeout_s", 8.0)
        self.declare_parameter("treatment_alert_hold_s", 2.0)
        self.declare_parameter("treatment_next_speed_scale", 0.70)
        self.declare_parameter("degraded_camera_speed_scale", 0.75)
        self.declare_parameter("summary_republish_period_s", 5.0)
        self.declare_parameter("return_to_start_after_route", True)
        self.declare_parameter("return_strategy", "odom_retrace")
        self.declare_parameter("return_via_visited_zones", True)
        self.declare_parameter("return_arrival_tolerance_m", 0.12)
        self.declare_parameter("return_yaw_tolerance_rad", 0.25)
        self.declare_parameter("nav_action_name", "navigate_to_pose")
        self.declare_parameter("nav2_cmd_topic", "/nav2/cmd_vel")
        self.declare_parameter("nav2_return_cmd_topic", "/cmd_vel")
        self.declare_parameter("nav2_frame_id", "map")
        self.declare_parameter("nav2_home_x", 0.0)
        self.declare_parameter("nav2_home_y", 0.0)
        self.declare_parameter("nav2_home_yaw", 0.0)
        self.declare_parameter("publish_nav2_initial_pose", True)
        self.declare_parameter("publish_nav2_return_initial_pose", False)
        self.declare_parameter("initial_pose_x", 0.0)
        self.declare_parameter("initial_pose_y", 0.0)
        self.declare_parameter("initial_pose_yaw", 0.0)
        self.declare_parameter("nav2_return_wait_timeout_s", 30.0)
        self.declare_parameter("nav2_return_settle_s", 2.0)
        self.declare_parameter("nav2_return_goal_timeout_s", 120.0)
        self.declare_parameter("nav2_return_success_tolerance_m", 0.25)
        self.declare_parameter("nav2_return_fallback_strategy", "odom_retrace")
        self.declare_parameter("nav2_progress_timeout_s", 8.0)
        self.declare_parameter("nav2_progress_min_delta_m", 0.05)
        self.declare_parameter("nav2_safety_block_timeout_s", 2.0)
        self.declare_parameter("nav2_recovery_backoff_s", 1.5)
        self.declare_parameter("nav2_recovery_backoff_speed", -0.07)
        self.declare_parameter("nav2_recovery_rotate_s", 3.0)
        self.declare_parameter("nav2_recovery_angular_speed", 0.85)
        self.declare_parameter("nav2_recovery_max_attempts", 4)
        self.declare_parameter("safety_recovery_delay_s", 1.0)
        self.declare_parameter("recovery_rotate_s", 3.0)
        self.declare_parameter("recovery_angular_speed", 0.85)
        self.declare_parameter("max_recovery_attempts_per_zone", 4)
        self.declare_parameter("frame_id", "base_link")

    def _load_zones(self):
        zone_ids = self.get_parameter("zone_ids").value
        plant_stress_indices = self.get_parameter("zone_plant_stress_indices").value
        legacy_indices = self.get_parameter("zone_residue_indices").value
        if len(plant_stress_indices) != len(zone_ids) and len(legacy_indices) == len(zone_ids):
            plant_stress_indices = legacy_indices
        return build_zones(
            zone_ids,
            self.get_parameter("zone_names").value,
            self.get_parameter("zone_x").value,
            self.get_parameter("zone_y").value,
            self.get_parameter("zone_yaw").value,
            plant_stress_indices,
            self.get_parameter("zone_expected_statuses").value,
        )

    def on_odom(self, msg: Odometry):
        q = msg.pose.pose.orientation
        self.pose = {
            "x": float(msg.pose.pose.position.x),
            "y": float(msg.pose.pose.position.y),
            "yaw": quaternion_to_yaw(q.x, q.y, q.z, q.w),
        }
        if self.start_pose is None:
            self.start_pose = dict(self.pose)
            if self.should_use_nav2_return() and bool(self.get_parameter("publish_nav2_initial_pose").value):
                self.publish_nav2_initial_pose()

    def on_inspection_result(self, msg: String):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn(f"Ignoring malformed inspection result: {msg.data}")
            return

        if self.current_index >= len(self.zones):
            return
        zone = self.zones[self.current_index]
        if data.get("zone_id") != zone.zone_id:
            return
        self.current_result = data
        plant_stress = data.get("plant_stress_index", data.get("residue_index"))
        self.get_logger().info(
            f"Digital twin inspection result for {zone.zone_id}: "
            f"{data.get('status')} plant_stress={plant_stress} "
            f"recommendation={data.get('recommendation')}"
        )

    def on_digital_state(self, msg: String):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        self.digital_camera_health = str(data.get("camera_health", self.digital_camera_health))
        self.digital_mode = str(data.get("mode", self.digital_mode))

    def on_safety_state(self, msg: String):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        blocked = bool(data.get("blocked", False))
        self.safety_reasons = list(data.get("reasons", []))

        if blocked and not self.safety_blocked:
            self.safety_blocked_since = time.monotonic()
        if not blocked:
            self.safety_blocked_since = None

        self.safety_blocked = blocked

    def on_nav2_cmd(self, msg: TwistStamped):
        if self.mode != "RETURNING_HOME_NAV2":
            return
        self.pub_nav2_return_cmd.publish(msg)

    def tick(self):
        now = time.monotonic()

        if self.mode == "IDLE":
            self.publish_state(force=False)
            self.publish_stop()
            return

        if self.mode == "WAITING_FOR_ODOM":
            self.publish_stop()
            if self.pose is not None:
                self.mode = "NAVIGATING"
                self.get_logger().info(f"Starting route toward {self.zones[0].zone_id}")
            self.publish_state(force=True)
            return

        if self.mode == "NAVIGATING":
            self.handle_navigation()
            self.publish_state(force=False)
            return

        if self.mode == "RECOVERING":
            self.handle_recovery(now)
            self.publish_state(force=False)
            return

        if self.mode == "RETURNING_HOME":
            self.handle_return_home()
            self.publish_state(force=False)
            return

        if self.mode == "WAITING_FOR_NAV2_RETURN":
            self.handle_nav2_return_wait(now)
            self.publish_state(force=False)
            return

        if self.mode == "RETURNING_HOME_NAV2":
            self.monitor_nav2_return(now)
            self.publish_state(force=False)
            return

        if self.mode == "NAV2_RETURN_RECOVERY":
            self.handle_nav2_return_recovery(now)
            self.publish_state(force=False)
            return

        if self.mode == "INSPECTING":
            self.handle_inspection(now)
            self.publish_state(force=False)
            return

        if self.mode == "HOLDING_ALERT":
            self.publish_stop()
            if self.hold_until is not None and now >= self.hold_until:
                self.advance_zone()
            self.publish_state(force=False)
            return

        if self.mode == "COMPLETE":
            self.publish_stop()
            self.republish_summary_if_due(now)
            self.publish_state(force=False)
            return

    def handle_navigation(self):
        if self.pose is None:
            self.publish_stop()
            self.mode = "WAITING_FOR_ODOM"
            return

        zone = self.zones[self.current_index]
        dx = zone.x - self.pose["x"]
        dy = zone.y - self.pose["y"]
        distance = math.hypot(dx, dy)

        if distance <= float(self.get_parameter("arrival_tolerance_m").value):
            yaw_error = normalize_angle(zone.yaw - self.pose["yaw"])
            if abs(yaw_error) <= float(self.get_parameter("yaw_tolerance_rad").value):
                self.begin_inspection()
                return
            self.publish_cmd(0.0, self.angular_control(yaw_error))
            return

        target_heading = math.atan2(dy, dx)
        heading_error = normalize_angle(target_heading - self.pose["yaw"])
        heading_limit = float(self.get_parameter("heading_slowdown_rad").value)
        heading_scale = clamp(1.0 - abs(heading_error) / max(heading_limit, 0.01), 0.0, 1.0)

        if self.safety_blocked:
            if heading_scale <= 0.05:
                self.publish_cmd(0.0, self.angular_control(heading_error))
                return
            blocked_for = time.monotonic() - (self.safety_blocked_since or time.monotonic())
            if blocked_for >= float(self.get_parameter("safety_recovery_delay_s").value):
                self.begin_recovery()
                return

        max_linear = float(self.get_parameter("max_linear_speed").value) * self.behavior_speed_scale()
        linear = clamp(
            float(self.get_parameter("linear_gain").value) * distance * heading_scale,
            0.0,
            max_linear,
        )
        angular = self.angular_control(heading_error)
        self.publish_cmd(linear, angular)

    def begin_recovery(self):
        zone = self.zones[self.current_index] if self.current_index < len(self.zones) else None
        target_label = self.active_target_label(zone)
        max_attempts = int(self.get_parameter("max_recovery_attempts_per_zone").value)

        if self.recovery_attempts >= max_attempts:
            if self.mode == "RETURNING_HOME":
                self.get_logger().warn(
                    f"Return to start blocked after {max_attempts} safety recoveries. "
                    "Completing the mission at the current pose."
                )
                self.complete_route(returned_home=False, return_status="RETURN_BLOCKED")
                return

            self.log_skipped_zone(zone, "SKIPPED_BLOCKED")
            self.get_logger().warn(
                f"{target_label} could not be reached after {max_attempts} safety recoveries. "
                "Skipping to keep the demo autonomous."
            )
            self.advance_zone()
            return

        self.recovery_attempts += 1
        self.recovery_direction *= -1.0
        self.recovery_until = time.monotonic() + float(self.get_parameter("recovery_rotate_s").value)
        self.recovery_resume_mode = self.mode
        self.mode = "RECOVERING"
        self.get_logger().warn(
            f"Safety blocked near {target_label}; recovery turn "
            f"{self.recovery_attempts}/{max_attempts}. reasons={self.safety_reasons}"
        )

    def handle_recovery(self, now: float):
        if self.recovery_until is not None and now < self.recovery_until:
            angular = self.recovery_direction * float(self.get_parameter("recovery_angular_speed").value)
            self.publish_cmd(0.0, angular)
            return

        self.publish_stop()
        self.recovery_until = None
        self.mode = self.recovery_resume_mode

    def active_target_label(self, zone=None) -> str:
        if self.mode in ("WAITING_FOR_NAV2_RETURN", "RETURNING_HOME_NAV2", "NAV2_RETURN_RECOVERY"):
            return "Nav2 home pose"
        if self.mode == "RETURNING_HOME" or self.recovery_resume_mode == "RETURNING_HOME":
            target = self.current_return_target()
            if target is not None:
                return str(target["label"])
            return "start pose"
        if zone is not None:
            return zone.zone_id
        return "target"

    def should_use_nav2_return(self) -> bool:
        return self.return_strategy in ("nav2", "nav2_planned_path", "nav2_shortest_path")

    def start_return(self):
        if self.should_use_nav2_return():
            self.start_nav2_return()
        else:
            self.start_return_home()

    def start_nav2_return(self):
        self.publish_stop()
        self.return_status = "WAITING_FOR_NAV2_RETURN"
        self.nav_return_started_at = time.monotonic()
        self.nav_goal_started_at = None
        self.nav_goal_handle = None
        self.nav_goal_in_flight = False
        self.nav_return_pose_published_at = None
        self.nav_feedback = {}
        self.nav2_recovery_attempts = 0
        self.nav2_recovery_phase = None
        self.nav2_recovery_until = None
        self.nav2_recovery_reason = None
        self.nav2_last_home_distance = None
        self.nav2_last_progress_at = None
        self.nav2_cancel_results_to_ignore = 0
        if bool(self.get_parameter("publish_nav2_return_initial_pose").value):
            self.nav_return_pose_published_at = time.monotonic()
            self.publish_nav2_initial_pose(self.pose)
        self.mode = "WAITING_FOR_NAV2_RETURN"
        self.get_logger().info(
            "Plant inspections complete; waiting for Nav2 to plan the fast return to start"
        )

    def handle_nav2_return_wait(self, now: float):
        settle_s = float(self.get_parameter("nav2_return_settle_s").value)
        pose_published_at = self.nav_return_pose_published_at or self.nav_return_started_at or now
        if settle_s > 0.0 and (now - pose_published_at) < settle_s:
            return

        if self.nav_client.wait_for_server(timeout_sec=0.0):
            self.send_nav2_return_goal()
            return

        started_at = self.nav_return_started_at or now
        timeout = float(self.get_parameter("nav2_return_wait_timeout_s").value)
        if timeout > 0.0 and (now - started_at) >= timeout:
            self.handle_nav2_return_failure("NAV2_RETURN_SERVER_TIMEOUT")

    def publish_nav2_initial_pose(self, pose: Optional[Dict] = None):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = str(self.get_parameter("nav2_frame_id").value)
        pose = pose or {
            "x": float(self.get_parameter("initial_pose_x").value),
            "y": float(self.get_parameter("initial_pose_y").value),
            "yaw": float(self.get_parameter("initial_pose_yaw").value),
        }
        msg.pose.pose.position.x = float(pose["x"])
        msg.pose.pose.position.y = float(pose["y"])
        q = yaw_to_quaternion(float(pose["yaw"]))
        msg.pose.pose.orientation.x = q["x"]
        msg.pose.pose.orientation.y = q["y"]
        msg.pose.pose.orientation.z = q["z"]
        msg.pose.pose.orientation.w = q["w"]
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.0685
        self.pub_initial_pose.publish(msg)
        if not self.nav2_initial_pose_published:
            self.get_logger().info("Published initial pose for Nav2 return")
        else:
            self.get_logger().info(f"Published current pose for Nav2 return: {pose}")
        self.nav2_initial_pose_published = True

    def send_nav2_return_goal(self):
        # Prefer the actual captured start pose over hardcoded params so the
        # goal exactly matches where nav2_return_pose_is_home() checks against.
        if self.start_pose is not None:
            pose = {
                "x": self.start_pose["x"],
                "y": self.start_pose["y"],
                "yaw": self.start_pose["yaw"],
            }
        else:
            pose = {
                "x": float(self.get_parameter("nav2_home_x").value),
                "y": float(self.get_parameter("nav2_home_y").value),
                "yaw": float(self.get_parameter("nav2_home_yaw").value),
            }
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.make_nav2_pose_stamped(pose)
        self.nav_goal_in_flight = True
        self.nav_goal_started_at = time.monotonic()
        self.nav2_last_home_distance = self.distance_to_nav2_home()
        self.nav2_last_progress_at = self.nav_goal_started_at
        self.return_status = "NAV2_RETURNING_HOME"
        self.mode = "RETURNING_HOME_NAV2"
        future = self.nav_client.send_goal_async(goal_msg, feedback_callback=self.on_nav2_feedback)
        future.add_done_callback(self.on_nav2_goal_response)
        self.get_logger().info(
            f"Sent Nav2 return goal to start_pose {pose}; Nav2 will plan the shortest collision-free path"
        )

    def make_nav2_pose_stamped(self, pose: Dict) -> PoseStamped:
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = str(self.get_parameter("nav2_frame_id").value)
        msg.pose.position.x = float(pose["x"])
        msg.pose.position.y = float(pose["y"])
        msg.pose.position.z = 0.0
        q = yaw_to_quaternion(float(pose["yaw"]))
        msg.pose.orientation.x = q["x"]
        msg.pose.orientation.y = q["y"]
        msg.pose.orientation.z = q["z"]
        msg.pose.orientation.w = q["w"]
        return msg

    def on_nav2_goal_response(self, future):
        self.nav_goal_handle = future.result()
        if not self.nav_goal_handle.accepted:
            self.nav_goal_in_flight = False
            self.nav_goal_handle = None
            self.handle_nav2_return_failure("NAV2_RETURN_REJECTED")
            return

        result_future = self.nav_goal_handle.get_result_async()
        result_future.add_done_callback(self.on_nav2_result)
        self.get_logger().info("Nav2 accepted return goal home")

    def on_nav2_feedback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.nav_feedback = {
            "distance_remaining": round(float(feedback.distance_remaining), 3),
            "number_of_recoveries": int(feedback.number_of_recoveries),
            "navigation_time_s": int(feedback.navigation_time.sec),
        }

    def on_nav2_result(self, future):
        result = future.result()
        if result.status == GoalStatus.STATUS_CANCELED and self.nav2_cancel_results_to_ignore > 0:
            self.nav2_cancel_results_to_ignore -= 1
            self.get_logger().info("Nav2 return goal canceled for supervised recovery")
            return

        self.nav_goal_in_flight = False
        self.nav_goal_handle = None
        self.nav_goal_started_at = None

        if result.status == GoalStatus.STATUS_SUCCEEDED:
            if not self.nav2_return_pose_is_home():
                self.handle_nav2_return_failure("NAV2_RETURN_POSE_NOT_HOME")
                return
            self.complete_route(returned_home=True, return_status="RETURNED_HOME_NAV2")
            self.get_logger().info("Nav2 returned to start; plant inspection route complete")
            return

        self.handle_nav2_return_failure(f"NAV2_RETURN_{self.goal_status_name(result.status)}")

    def nav2_return_pose_is_home(self) -> bool:
        if self.pose is None or self.start_pose is None:
            return False
        distance = math.hypot(self.pose["x"] - self.start_pose["x"], self.pose["y"] - self.start_pose["y"])
        tolerance = float(self.get_parameter("nav2_return_success_tolerance_m").value)
        if distance > tolerance:
            self.get_logger().warn(
                f"Nav2 reported return success, but odom is still {distance:.2f} m from start"
            )
            return False
        return True

    def distance_to_nav2_home(self) -> Optional[float]:
        if self.pose is None:
            return None
        home_x = float(self.get_parameter("nav2_home_x").value)
        home_y = float(self.get_parameter("nav2_home_y").value)
        return math.hypot(self.pose["x"] - home_x, self.pose["y"] - home_y)

    def monitor_nav2_return(self, now: float):
        if not self.nav_goal_in_flight or self.nav_goal_started_at is None:
            return

        distance = self.distance_to_nav2_home()
        if distance is not None:
            min_delta = float(self.get_parameter("nav2_progress_min_delta_m").value)
            if self.nav2_last_home_distance is None or distance < (self.nav2_last_home_distance - min_delta):
                self.nav2_last_home_distance = distance
                self.nav2_last_progress_at = now

        block_timeout = float(self.get_parameter("nav2_safety_block_timeout_s").value)
        if (
            block_timeout > 0.0
            and self.safety_blocked
            and self.safety_blocked_since is not None
            and (now - self.safety_blocked_since) >= block_timeout
        ):
            self.begin_nav2_return_recovery("SAFETY_BLOCKED")
            return

        progress_timeout = float(self.get_parameter("nav2_progress_timeout_s").value)
        last_progress = self.nav2_last_progress_at or self.nav_goal_started_at
        if progress_timeout > 0.0 and (now - last_progress) >= progress_timeout:
            self.begin_nav2_return_recovery("NO_PROGRESS")
            return

        timeout = float(self.get_parameter("nav2_return_goal_timeout_s").value)
        if timeout <= 0.0 or (now - self.nav_goal_started_at) < timeout:
            return
        self.cancel_nav2_goal()
        self.handle_nav2_return_failure("NAV2_RETURN_TIMEOUT")

    def cancel_nav2_goal(self, ignore_result: bool = False):
        if self.nav_goal_handle is not None:
            if ignore_result:
                self.nav2_cancel_results_to_ignore += 1
            self.nav_goal_handle.cancel_goal_async()
        self.nav_goal_in_flight = False
        self.nav_goal_handle = None
        self.nav_goal_started_at = None

    def begin_nav2_return_recovery(self, reason: str):
        max_attempts = int(self.get_parameter("nav2_recovery_max_attempts").value)
        if self.nav2_recovery_attempts >= max_attempts:
            self.cancel_nav2_goal()
            self.handle_nav2_return_failure(f"NAV2_RETURN_{reason}_AFTER_RECOVERY")
            return

        self.nav2_recovery_attempts += 1
        self.nav2_recovery_reason = reason
        self.return_status = f"NAV2_RETURN_RECOVERY_{reason}"
        self.cancel_nav2_goal(ignore_result=True)
        self.publish_nav2_return_stop()
        self.nav2_recovery_phase = "BACKOFF"
        self.nav2_recovery_until = time.monotonic() + float(self.get_parameter("nav2_recovery_backoff_s").value)
        self.mode = "NAV2_RETURN_RECOVERY"
        self.get_logger().warn(
            f"Nav2 return stuck ({reason}); supervised recovery "
            f"{self.nav2_recovery_attempts}/{max_attempts}: back off, rotate, then retry Nav2"
        )

    def handle_nav2_return_recovery(self, now: float):
        if self.nav2_recovery_phase == "BACKOFF":
            if self.nav2_recovery_until is not None and now < self.nav2_recovery_until:
                self.publish_nav2_return_cmd(
                    float(self.get_parameter("nav2_recovery_backoff_speed").value),
                    0.0,
                )
                return
            self.nav2_recovery_phase = "ROTATE"
            self.nav2_recovery_until = now + float(self.get_parameter("nav2_recovery_rotate_s").value)

        if self.nav2_recovery_phase == "ROTATE":
            if self.nav2_recovery_until is not None and now < self.nav2_recovery_until:
                direction = -1.0 if self.nav2_recovery_attempts % 2 else 1.0
                self.publish_nav2_return_cmd(
                    0.0,
                    direction * float(self.get_parameter("nav2_recovery_angular_speed").value),
                )
                return
            self.publish_nav2_return_stop()
            self.nav2_recovery_phase = None
            self.nav2_recovery_until = None
            self.nav2_last_home_distance = self.distance_to_nav2_home()
            self.nav2_last_progress_at = now
            self.send_nav2_return_goal()

    @staticmethod
    def goal_status_name(status: int) -> str:
        names = {
            GoalStatus.STATUS_ABORTED: "ABORTED",
            GoalStatus.STATUS_CANCELED: "CANCELED",
            GoalStatus.STATUS_UNKNOWN: "UNKNOWN",
        }
        return names.get(status, f"STATUS_{status}")

    def handle_nav2_return_failure(self, status: str):
        fallback = str(self.get_parameter("nav2_return_fallback_strategy").value).lower()
        self.get_logger().warn(f"Nav2 return failed with {status}; fallback={fallback}")
        if fallback == "odom_retrace":
            self.return_strategy = "odom_retrace_after_nav2_failure"
            self.start_return_home()
        else:
            self.complete_route(returned_home=False, return_status=status)

    def start_return_home(self):
        self.return_targets = []
        if bool(self.get_parameter("return_via_visited_zones").value):
            for zone in reversed(self.zones[:-1]):
                self.return_targets.append(
                    {
                        "label": f"return waypoint {zone.zone_id}",
                        "x": zone.x,
                        "y": zone.y,
                        "yaw": zone.yaw,
                        "require_yaw": False,
                    }
                )

        self.return_targets.append(
            {
                "label": "start pose",
                "x": self.start_pose["x"],
                "y": self.start_pose["y"],
                "yaw": self.start_pose["yaw"],
                "require_yaw": True,
            }
        )
        self.return_target_index = 0
        self.recovery_attempts = 0
        self.mode = "RETURNING_HOME"
        self.get_logger().info(
            f"Plant inspections complete; returning to start via {len(self.return_targets)} waypoint(s)"
        )

    def current_return_target(self):
        if self.return_target_index >= len(self.return_targets):
            return None
        return self.return_targets[self.return_target_index]

    def advance_return_target(self):
        self.return_target_index += 1
        self.recovery_attempts = 0
        self.safety_blocked_since = None
        if self.return_target_index >= len(self.return_targets):
            self.get_logger().info("Returned to start; plant inspection route complete")
            self.complete_route(returned_home=True, return_status="RETURNED_HOME")
            return

        target = self.current_return_target()
        self.get_logger().info(f"Continuing return toward {target['label']}")

    def handle_return_home(self):
        if self.pose is None or self.start_pose is None:
            self.publish_stop()
            return

        if not self.return_targets:
            self.start_return_home()

        target = self.current_return_target()
        if target is None:
            self.complete_route(returned_home=True, return_status="RETURNED_HOME")
            return

        dx = float(target["x"]) - self.pose["x"]
        dy = float(target["y"]) - self.pose["y"]
        distance = math.hypot(dx, dy)

        if distance <= float(self.get_parameter("return_arrival_tolerance_m").value):
            yaw_error = normalize_angle(float(target["yaw"]) - self.pose["yaw"])
            if not target["require_yaw"]:
                self.advance_return_target()
                return
            if abs(yaw_error) <= float(self.get_parameter("return_yaw_tolerance_rad").value):
                self.advance_return_target()
                return
            self.publish_cmd(0.0, self.angular_control(yaw_error))
            return

        target_heading = math.atan2(dy, dx)
        heading_error = normalize_angle(target_heading - self.pose["yaw"])
        heading_limit = float(self.get_parameter("heading_slowdown_rad").value)
        heading_scale = clamp(1.0 - abs(heading_error) / max(heading_limit, 0.01), 0.0, 1.0)

        if self.safety_blocked:
            if heading_scale <= 0.05:
                self.publish_cmd(0.0, self.angular_control(heading_error))
                return
            blocked_for = time.monotonic() - (self.safety_blocked_since or time.monotonic())
            if blocked_for >= float(self.get_parameter("safety_recovery_delay_s").value):
                self.begin_recovery()
                return

        max_linear = float(self.get_parameter("max_linear_speed").value) * self.behavior_speed_scale()
        linear = clamp(
            float(self.get_parameter("linear_gain").value) * distance * heading_scale,
            0.0,
            max_linear,
        )
        self.publish_cmd(linear, self.angular_control(heading_error))

    def log_skipped_zone(self, zone, status: str):
        entry = {
            "event": "INSPECTION_LOG",
            "zone_id": zone.zone_id,
            "zone_name": zone.name,
            "status": status,
            "plant_stress_index": None,
            "disease_level": None,
            "recommendation": recommendation_for_status(status),
            "confidence": 0.0,
            "camera_health": self.digital_camera_health,
            "mission_index": self.current_index,
            "safety_reasons": self.safety_reasons,
        }
        self.summary.append(entry)
        self.last_result = entry
        self.pub_log.publish(String(data=json.dumps(entry, sort_keys=True)))

    def angular_control(self, error: float) -> float:
        max_angular = float(self.get_parameter("max_angular_speed").value)
        angular = float(self.get_parameter("angular_gain").value) * error
        return clamp(angular, -max_angular, max_angular)

    def behavior_speed_scale(self) -> float:
        scale = 1.0
        if self.last_result and is_treatment_needed_status(self.last_result.get("status")):
            scale *= float(self.get_parameter("treatment_next_speed_scale").value)
        if self.digital_camera_health.lower() == "degraded":
            scale *= float(self.get_parameter("degraded_camera_speed_scale").value)
        if self.digital_camera_health.lower() == "failed":
            scale *= 0.5
        return scale

    def begin_inspection(self):
        zone = self.zones[self.current_index]
        self.publish_stop()
        self.mode = "INSPECTING"
        self.current_result = None
        self.inspection_started_at = time.monotonic()
        self.request_sent = False
        self.get_logger().info(f"Arrived at {zone.zone_id}; starting simulated inspection")

    def handle_inspection(self, now: float):
        self.publish_stop()
        zone = self.zones[self.current_index]

        if not self.request_sent:
            self.request_id += 1
            payload = {
                "event": "INSPECT_PLANT",
                "request_id": self.request_id,
                "zone_id": zone.zone_id,
                "zone_name": zone.name,
                "source_entity": str(self.get_parameter("source_entity").value),
                "simulated_sensor": "hyperspectral_camera",
                "pose": {"x": zone.x, "y": zone.y, "yaw": zone.yaw},
                "sent_at_monotonic_s": now,
            }
            self.pub_request.publish(String(data=json.dumps(payload, sort_keys=True)))
            self.request_sent = True
            self.get_logger().info(f"Inspection request sent for {zone.zone_id}")

        elapsed = now - (self.inspection_started_at or now)
        min_wait = float(self.get_parameter("inspection_duration_s").value)
        timeout = float(self.get_parameter("inspection_result_timeout_s").value)

        if self.current_result is not None and elapsed >= min_wait:
            self.finish_inspection(self.current_result)
            return

        if elapsed >= timeout:
            timeout_result = {
                "zone_id": zone.zone_id,
                "zone_name": zone.name,
                "status": "SENSOR_TIMEOUT",
                "plant_stress_index": None,
                "disease_level": None,
                "recommendation": "RETRY_INSPECTION",
                "confidence": 0.0,
                "camera_health": self.digital_camera_health,
            }
            self.finish_inspection(timeout_result)

    def finish_inspection(self, result: Dict):
        zone = self.zones[self.current_index]
        self.last_result = result
        plant_stress = result.get("plant_stress_index", result.get("residue_index"))
        status = result.get("status", "UNKNOWN")
        entry = {
            "event": "INSPECTION_LOG",
            "zone_id": zone.zone_id,
            "zone_name": zone.name,
            "status": status,
            "plant_stress_index": plant_stress,
            "disease_level": result.get("disease_level", plant_stress),
            "recommendation": result.get("recommendation", recommendation_for_status(status)),
            "confidence": result.get("confidence"),
            "camera_health": result.get("camera_health", self.digital_camera_health),
            "mission_index": self.current_index,
        }
        self.summary.append(entry)
        self.pub_log.publish(String(data=json.dumps(entry, sort_keys=True)))

        status = entry["status"]
        if is_treatment_needed_status(status):
            hold_s = float(self.get_parameter("treatment_alert_hold_s").value)
            self.hold_until = time.monotonic() + hold_s
            self.mode = "HOLDING_ALERT"
            self.get_logger().warn(
                f"{zone.zone_id} plant stress/disease level is high. "
                f"Recommendation: APPLY_PESTICIDE. Holding {hold_s:.1f}s before next zone."
            )
            return

        self.get_logger().info(f"{zone.zone_id} inspection complete: {status}")
        self.advance_zone()

    def advance_zone(self):
        self.current_index += 1
        self.current_result = None
        self.request_sent = False
        self.inspection_started_at = None
        self.hold_until = None
        self.safety_blocked_since = None
        self.recovery_until = None
        self.recovery_attempts = 0

        if self.current_index >= len(self.zones):
            if bool(self.get_parameter("return_to_start_after_route").value) and self.start_pose is not None:
                self.start_return()
            else:
                status = "RETURN_DISABLED" if self.start_pose is not None else "NO_START_POSE"
                self.complete_route(returned_home=False, return_status=status)
        else:
            zone = self.zones[self.current_index]
            self.mode = "NAVIGATING"
            self.get_logger().info(f"Continuing route toward {zone.zone_id}")

    def complete_route(self, returned_home: bool, return_status: str):
        self.mode = "COMPLETE"
        self.returned_home = returned_home
        self.return_status = return_status
        self.publish_nav2_return_stop()
        self.publish_summary()
        if not returned_home:
            self.get_logger().info("Plant inspection route complete")

    def publish_cmd(self, linear_x: float, angular_z: float):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = str(self.get_parameter("frame_id").value)
        msg.twist.linear.x = float(linear_x)
        msg.twist.angular.z = float(angular_z)
        self.pub_cmd.publish(msg)

    def publish_stop(self):
        self.publish_cmd(0.0, 0.0)

    def publish_nav2_return_stop(self):
        self.publish_nav2_return_cmd(0.0, 0.0)

    def publish_nav2_return_cmd(self, linear_x: float, angular_z: float):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = str(self.get_parameter("frame_id").value)
        msg.twist.linear.x = float(linear_x)
        msg.twist.angular.z = float(angular_z)
        self.pub_nav2_return_cmd.publish(msg)

    def publish_state(self, force: bool):
        now = time.monotonic()
        if not force and (now - self.last_state_publish_at) < 0.5:
            return
        self.last_state_publish_at = now

        zone = self.zones[self.current_index] if self.current_index < len(self.zones) else None
        payload = {
            "entity": str(self.get_parameter("source_entity").value),
            "mode": self.mode,
            "current_zone_id": zone.zone_id if zone else None,
            "current_zone_name": zone.name if zone else None,
            "mission_index": self.current_index,
            "zone_count": len(self.zones),
            "pose": self.pose,
            "start_pose": self.start_pose,
            "returned_home": self.returned_home,
            "return_status": self.return_status,
            "return_strategy": self.return_strategy,
            "return_target_index": self.return_target_index,
            "return_target_count": len(self.return_targets),
            "nav_goal_in_flight": self.nav_goal_in_flight,
            "nav_feedback": self.nav_feedback,
            "nav2_recovery_attempts": self.nav2_recovery_attempts,
            "nav2_recovery_phase": self.nav2_recovery_phase,
            "nav2_recovery_reason": self.nav2_recovery_reason,
            "digital_camera_health": self.digital_camera_health,
            "digital_mode": self.digital_mode,
            "safety_blocked": self.safety_blocked,
            "safety_reasons": self.safety_reasons,
            "recovery_attempts": self.recovery_attempts,
            "behavior_speed_scale": self.behavior_speed_scale(),
            "last_result": self.last_result,
            "completed_inspections": len(self.summary),
            "uptime_s": round(now - self.mission_started_at, 2),
        }
        self.pub_state.publish(String(data=json.dumps(payload, sort_keys=True)))

    def publish_summary(self):
        payload = {
            "event": "MISSION_SUMMARY",
            "total_zones": len(self.zones),
            "returned_home": self.returned_home,
            "return_status": self.return_status,
            "return_strategy": self.return_strategy,
            "nav_feedback": self.nav_feedback,
            "nav2_recovery_attempts": self.nav2_recovery_attempts,
            "nav2_recovery_reason": self.nav2_recovery_reason,
            "final_pose": self.pose,
            "start_pose": self.start_pose,
            "inspections": self.summary,
        }
        self.pub_log.publish(String(data=json.dumps(payload, sort_keys=True)))

    def republish_summary_if_due(self, now: float):
        period = float(self.get_parameter("summary_republish_period_s").value)
        if period <= 0.0:
            return
        if (now - self.last_summary_publish_at) >= period:
            self.last_summary_publish_at = now
            self.publish_summary()


def main(args=None):
    rclpy.init(args=args)
    node = PlantMissionNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.publish_stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
