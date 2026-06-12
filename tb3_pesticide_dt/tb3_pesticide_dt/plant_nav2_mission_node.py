#!/usr/bin/env python3
"""Nav2 action-based plant-health inspection mission for the DT demo."""

import json
import math
import time
from typing import Dict, Optional

import rclpy
import rclpy.duration
import rclpy.time
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

from tb3_pesticide_dt.pesticide_logic import (
    build_zones,
    is_treatment_needed_status,
    recommendation_for_status,
)
from tb3_pesticide_dt.plant_mission_node import (
    DEFAULT_RESIDUES,
    DEFAULT_STATUSES,
    DEFAULT_ZONE_IDS,
    DEFAULT_ZONE_NAMES,
    DEFAULT_ZONE_X,
    DEFAULT_ZONE_Y,
    DEFAULT_ZONE_YAW,
)


def yaw_to_quaternion(yaw: float):
    half = yaw * 0.5
    return {
        "x": 0.0,
        "y": 0.0,
        "z": math.sin(half),
        "w": math.cos(half),
    }


class PlantNav2MissionNode(Node):
    """Sends NavigateToPose goals, then runs the same inspection/logging workflow."""

    def __init__(self):
        super().__init__("plant_nav2_mission_node")
        self._declare_parameters()
        self.zones = self._load_zones()
        self.home_pose = {
            "x": float(self.get_parameter("home_x").value),
            "y": float(self.get_parameter("home_y").value),
            "yaw": float(self.get_parameter("home_yaw").value),
        }

        self.state_topic = str(self.get_parameter("state_topic").value)
        self.request_topic = str(self.get_parameter("inspection_request_topic").value)
        self.result_topic = str(self.get_parameter("inspection_result_topic").value)
        self.digital_state_topic = str(self.get_parameter("digital_state_topic").value)
        self.log_topic = str(self.get_parameter("inspection_log_topic").value)
        self.nav_action_name = str(self.get_parameter("nav_action_name").value)

        self.mode = "WAITING_FOR_NAV2" if bool(self.get_parameter("start_automatically").value) else "IDLE"
        self.current_index = 0
        self.current_result: Optional[Dict] = None
        self.last_result: Optional[Dict] = None
        self.request_id = 0
        self.request_sent = False
        self.inspection_started_at: Optional[float] = None
        self.hold_until: Optional[float] = None
        self.goal_started_at: Optional[float] = None
        self.goal_handle = None
        self.goal_in_flight = False
        self.active_goal_id: Optional[str] = None
        self.retry_goal_after: Optional[float] = None
        self.goal_retry_counts: Dict[str, int] = {}
        self.return_goal_sent = False
        self.near_goal_since: Optional[float] = None
        self.current_pose: Optional[Dict] = None
        self.localization_started_at: Optional[float] = None
        self.amcl_home_pose: Optional[Dict] = None  # captured from AMCL at startup
        self._last_initial_pose_pub: Optional[float] = None
        self.summary = []
        self.final_status = "RUNNING"
        self.digital_camera_health = "unknown"
        self.digital_mode = "unknown"
        self.latest_feedback = {}
        self.last_state_publish_at = 0.0
        self.last_summary_publish_at = 0.0
        self.mission_started_at = time.monotonic()

        self.nav_client = ActionClient(self, NavigateToPose, self.nav_action_name)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(String, self.result_topic, self.on_inspection_result, 10)
        self.create_subscription(String, self.digital_state_topic, self.on_digital_state, 10)
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self.on_amcl_pose, 10)

        self.pub_state = self.create_publisher(String, self.state_topic, 10)
        self.pub_request = self.create_publisher(String, self.request_topic, 10)
        self.pub_log = self.create_publisher(String, self.log_topic, 10)
        self.pub_initial_pose = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)

        self.create_timer(float(self.get_parameter("control_period_s").value), self.tick)
        self.get_logger().info(
            f"PlantNav2MissionNode started with {len(self.zones)} zones. "
            f"Nav action={self.nav_action_name}"
        )

    def _declare_parameters(self):
        self.declare_parameter("start_automatically", True)
        self.declare_parameter("source_entity", "physical_robot_nav2")
        self.declare_parameter("nav_action_name", "navigate_to_pose")
        self.declare_parameter("state_topic", "/dt/physical/mission_state")
        self.declare_parameter("inspection_request_topic", "/dt/physical/inspection_request")
        self.declare_parameter("inspection_result_topic", "/dt/digital/inspection_result")
        self.declare_parameter("digital_state_topic", "/dt/digital/mission_state")
        self.declare_parameter("inspection_log_topic", "/dt/physical/inspection_log")

        self.declare_parameter("zone_ids", DEFAULT_ZONE_IDS)
        self.declare_parameter("zone_names", DEFAULT_ZONE_NAMES)
        self.declare_parameter("zone_x", DEFAULT_ZONE_X)
        self.declare_parameter("zone_y", DEFAULT_ZONE_Y)
        self.declare_parameter("zone_yaw", DEFAULT_ZONE_YAW)
        self.declare_parameter("zone_plant_stress_indices", DEFAULT_RESIDUES)
        self.declare_parameter("zone_residue_indices", DEFAULT_RESIDUES)
        self.declare_parameter("zone_expected_statuses", DEFAULT_STATUSES)

        self.declare_parameter("frame_id", "map")
        self.declare_parameter("control_period_s", 0.20)
        self.declare_parameter("wait_for_nav2_timeout_s", 60.0)
        self.declare_parameter("goal_timeout_s", 90.0)
        self.declare_parameter("accept_near_goal_distance_m", 0.30)
        self.declare_parameter("accept_near_home_distance_m", 0.25)
        self.declare_parameter("accept_near_goal_after_s", 4.0)
        self.declare_parameter("goal_retry_delay_s", 5.0)
        self.declare_parameter("max_goal_retries", 5)
        self.declare_parameter("retry_fast_failure_window_s", 12.0)
        self.declare_parameter("inspection_duration_s", 3.0)
        self.declare_parameter("inspection_result_timeout_s", 8.0)
        self.declare_parameter("treatment_alert_hold_s", 2.0)
        self.declare_parameter("summary_republish_period_s", 5.0)
        self.declare_parameter("return_to_start", True)
        self.declare_parameter("home_x", 0.0)
        self.declare_parameter("home_y", 0.0)
        self.declare_parameter("home_yaw", 0.0)
        self.declare_parameter("use_captured_home_pose", False)
        self.declare_parameter("publish_initial_pose", True)
        self.declare_parameter("initial_pose_x", 0.0)
        self.declare_parameter("initial_pose_y", 0.0)
        self.declare_parameter("initial_pose_yaw", 0.0)
        self.declare_parameter("initial_pose_settle_s", 3.0)

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

    def _capture_home_from_tf(self):
        """Look up where the odom origin sits in the map frame.

        The robot starts at odom (0, 0, 0).  The TF transform map←odom tells
        us exactly where that point is in map coordinates — this is the true
        map-frame home position, independent of AMCL drift or particle jitter.
        """
        frame_id = str(self.get_parameter("frame_id").value)
        try:
            tf = self.tf_buffer.lookup_transform(
                frame_id,          # target: map
                "odom",            # source: odom
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=2.0),
            )
            t = tf.transform.translation
            r = tf.transform.rotation
            yaw = math.atan2(
                2.0 * (r.w * r.z + r.x * r.y),
                1.0 - 2.0 * (r.y * r.y + r.z * r.z),
            )
            self.amcl_home_pose = {"x": t.x, "y": t.y, "yaw": yaw}
            self.get_logger().info(
                f"TF home captured ({frame_id}←odom): "
                f"x={t.x:.3f} y={t.y:.3f} yaw={yaw:.3f}"
            )
        except Exception as exc:
            self.get_logger().warn(f"TF home lookup failed: {exc}")

    def on_amcl_pose(self, msg: PoseWithCovarianceStamped):
        """Track AMCL's estimate of the robot position.

        During the settle phase, this also captures the home pose.  During the
        mission it provides a live state value for the digital twin evidence.
        """
        q = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        self.current_pose = {
            "x": msg.pose.pose.position.x,
            "y": msg.pose.pose.position.y,
            "yaw": yaw,
        }
        if self.mode not in ("WAITING_FOR_NAV2", "LOCALIZING"):
            return  # navigation already underway — do not change home pose
        self.amcl_home_pose = dict(self.current_pose)
        self.get_logger().info(
            f"AMCL pose update (settling): "
            f"x={self.amcl_home_pose['x']:.3f} "
            f"y={self.amcl_home_pose['y']:.3f} "
            f"yaw={self.amcl_home_pose['yaw']:.3f}"
        )

    def tick(self):
        now = time.monotonic()

        if self.mode == "IDLE":
            self.publish_state(False)
            return

        if self.mode == "WAITING_FOR_NAV2":
            if self.nav_client.wait_for_server(timeout_sec=0.0):
                if bool(self.get_parameter("publish_initial_pose").value):
                    self.publish_initial_pose()
                self.localization_started_at = now
                self.mode = "LOCALIZING"
                settle_s = float(self.get_parameter("initial_pose_settle_s").value)
                self.get_logger().info(
                    f"Nav2 action server is ready; waiting {settle_s:.1f}s for AMCL to settle"
                )
            elif (now - self.mission_started_at) > float(self.get_parameter("wait_for_nav2_timeout_s").value):
                self.mode = "COMPLETE"
                self.publish_summary()
                self.get_logger().error("Timed out waiting for Nav2 navigate_to_pose action server")
            self.publish_state(True)
            return

        if self.mode == "LOCALIZING":
            settle_s = float(self.get_parameter("initial_pose_settle_s").value)
            # Republish initial pose every 2 s until AMCL acknowledges (covers the case where
            # AMCL wasn't subscribed yet when we first published).
            if self.amcl_home_pose is None and bool(self.get_parameter("publish_initial_pose").value):
                last_pub = self._last_initial_pose_pub if self._last_initial_pose_pub is not None \
                    else (self.localization_started_at or now)
                if (now - last_pub) >= 2.0:
                    self.publish_initial_pose()
                    self._last_initial_pose_pub = now
            if self.localization_started_at is None or (now - self.localization_started_at) >= settle_s:
                # Primary: read where odom-origin is in the map frame via TF.
                # This is the exact map-frame coordinate of the physical starting
                # position regardless of how AMCL's particle filter initialised.
                self._capture_home_from_tf()
                # Fallback: /amcl_pose subscription (may already be set above)
                if self.amcl_home_pose is not None:
                    self.get_logger().info(
                        f"Home pose captured in map frame: "
                        f"x={self.amcl_home_pose['x']:.3f} "
                        f"y={self.amcl_home_pose['y']:.3f} "
                        f"yaw={self.amcl_home_pose['yaw']:.3f}"
                    )
                else:
                    self.get_logger().warn(
                        "Could not capture home pose from TF or AMCL. "
                        "Will use YAML home (0,0) — return may be imprecise."
                    )
                self.mode = "NAVIGATING"
                self.get_logger().info("Starting plant route after localization settle")
            self.publish_state(False)
            return

        if self.mode == "NAVIGATING":
            if self.retry_goal_after is not None:
                if now < self.retry_goal_after:
                    self.publish_state(False)
                    return
                self.retry_goal_after = None
            if not self.goal_in_flight:
                self.send_current_zone_goal()
            self.check_goal_close_enough(now)
            self.check_goal_timeout(now)
            self.publish_state(False)
            return

        if self.mode == "INSPECTING":
            self.handle_inspection(now)
            self.publish_state(False)
            return

        if self.mode == "HOLDING_ALERT":
            if self.hold_until is not None and now >= self.hold_until:
                self.advance_zone()
            self.publish_state(False)
            return

        if self.mode == "RETURNING_HOME":
            if self.retry_goal_after is not None:
                if now < self.retry_goal_after:
                    self.publish_state(False)
                    return
                self.retry_goal_after = None
            if not self.goal_in_flight and not self.return_goal_sent:
                home = self.home_goal_pose()
                if bool(self.get_parameter("use_captured_home_pose").value) and self.amcl_home_pose is None:
                    self.get_logger().warn(
                        "amcl_home_pose not captured; returning to configured home_pose. "
                        "Return position may be inaccurate if AMCL drifted."
                    )
                self.send_pose_goal("home", "Return to start", home, returning_home=True)
            self.check_goal_close_enough(now)
            self.check_goal_timeout(now)
            self.publish_state(False)
            return

        if self.mode == "COMPLETE":
            self.republish_summary_if_due(now)
            self.publish_state(False)

    def publish_initial_pose(self):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = str(self.get_parameter("frame_id").value)
        msg.pose.pose.position.x = float(self.get_parameter("initial_pose_x").value)
        msg.pose.pose.position.y = float(self.get_parameter("initial_pose_y").value)
        q = yaw_to_quaternion(float(self.get_parameter("initial_pose_yaw").value))
        msg.pose.pose.orientation.x = q["x"]
        msg.pose.pose.orientation.y = q["y"]
        msg.pose.pose.orientation.z = q["z"]
        msg.pose.pose.orientation.w = q["w"]
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.0685
        self.pub_initial_pose.publish(msg)
        self.get_logger().info("Published initial pose for AMCL")

    def send_current_zone_goal(self):
        zone = self.zones[self.current_index]
        if zone.zone_id == "plant_home":
            home = self.home_goal_pose()
            self.get_logger().info(
                f"Route waypoint {self.current_index + 1} is plant_home; sending the final "
                "Nav2 goal back to the start instead of inspecting it."
            )
            self.send_pose_goal(zone.zone_id, zone.name, home, returning_home=True)
            return

        pose = {"x": zone.x, "y": zone.y, "yaw": zone.yaw}
        self.send_pose_goal(zone.zone_id, zone.name, pose, returning_home=False)

    def home_goal_pose(self) -> Dict:
        if bool(self.get_parameter("use_captured_home_pose").value) and self.amcl_home_pose is not None:
            return self.amcl_home_pose
        return self.home_pose

    def send_pose_goal(self, goal_id: str, goal_name: str, pose: Dict, returning_home: bool):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.make_pose_stamped(pose)
        self.goal_in_flight = True
        self.goal_started_at = time.monotonic()
        self.latest_feedback = {"goal_id": goal_id, "goal_name": goal_name}
        self.active_goal_id = goal_id
        self.near_goal_since = None
        self.return_goal_sent = returning_home

        future = self.nav_client.send_goal_async(goal_msg, feedback_callback=self.on_nav_feedback)
        future.add_done_callback(lambda fut: self.on_goal_response(fut, goal_id, goal_name, returning_home))
        if returning_home:
            self.get_logger().info(
                f"Sent Nav2 return goal {goal_id}: {goal_name} at {pose}; "
                "Nav2 will plan the shortest collision-free path on the map"
            )
        else:
            self.get_logger().info(f"Sent Nav2 goal {goal_id}: {goal_name} at {pose}")

    def make_pose_stamped(self, pose: Dict) -> PoseStamped:
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = str(self.get_parameter("frame_id").value)
        msg.pose.position.x = float(pose["x"])
        msg.pose.position.y = float(pose["y"])
        msg.pose.position.z = 0.0
        q = yaw_to_quaternion(float(pose["yaw"]))
        msg.pose.orientation.x = q["x"]
        msg.pose.orientation.y = q["y"]
        msg.pose.orientation.z = q["z"]
        msg.pose.orientation.w = q["w"]
        return msg

    def on_goal_response(self, future, goal_id: str, goal_name: str, returning_home: bool):
        if goal_id != self.active_goal_id:
            self.get_logger().info(f"Ignoring stale Nav2 goal response for {goal_id}")
            return
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.goal_in_flight = False
            self.goal_handle = None
            self.active_goal_id = None
            self.near_goal_since = None
            if self.schedule_goal_retry(goal_id, "GOAL_REJECTED", returning_home):
                return
            if returning_home:
                self.publish_return_home_log("RETURN_REJECTED", goal_id, goal_name)
                self.finish_mission("RETURN_REJECTED")
            else:
                self.log_skipped_zone("NAV_GOAL_REJECTED", f"Nav2 rejected goal {goal_id}")
                self.advance_zone()
            return

        result_future = self.goal_handle.get_result_async()
        result_future.add_done_callback(lambda fut: self.on_nav_result(fut, goal_id, goal_name, returning_home))
        self.get_logger().info(f"Nav2 accepted goal {goal_id}: {goal_name}")

    def on_nav_feedback(self, feedback_msg):
        feedback = feedback_msg.feedback
        distance_remaining = float(feedback.distance_remaining)
        self.latest_feedback = {
            "distance_remaining": round(distance_remaining, 3),
            "number_of_recoveries": int(feedback.number_of_recoveries),
            "navigation_time_s": int(feedback.navigation_time.sec),
        }
        near_m = float(self.get_parameter("accept_near_goal_distance_m").value)
        if self.return_goal_sent:
            near_m = float(self.get_parameter("accept_near_home_distance_m").value)
        if distance_remaining <= near_m:
            if self.near_goal_since is None:
                self.near_goal_since = time.monotonic()
        else:
            self.near_goal_since = None

    def on_nav_result(self, future, goal_id: str, goal_name: str, returning_home: bool):
        if goal_id != self.active_goal_id:
            self.get_logger().info(f"Ignoring stale Nav2 result for {goal_id}")
            return
        result = future.result()
        elapsed = time.monotonic() - (self.goal_started_at or time.monotonic())
        self.goal_in_flight = False
        self.goal_handle = None
        self.goal_started_at = None
        self.active_goal_id = None
        self.near_goal_since = None

        if result.status == GoalStatus.STATUS_SUCCEEDED:
            if returning_home:
                self.publish_return_home_log("RETURNED_HOME", goal_id, goal_name)
                self.finish_mission("RETURNED_HOME")
            else:
                self.get_logger().info(f"Arrived at {goal_id}: {goal_name}; starting inspection")
                self.begin_inspection()
            return

        status_text = self.goal_status_name(result.status)
        error_msg = getattr(result.result, "error_msg", "")
        fast_window = float(self.get_parameter("retry_fast_failure_window_s").value)
        if elapsed <= fast_window and status_text in {"ABORTED", "CANCELED", "UNKNOWN"}:
            if self.schedule_goal_retry(goal_id, status_text, returning_home):
                return
        if returning_home:
            final_status = f"RETURN_{status_text}"
            self.publish_return_home_log(final_status, goal_id, goal_name, error_msg)
            self.finish_mission(final_status)
        else:
            self.log_skipped_zone(f"NAV_{status_text}", error_msg or f"Nav2 status {status_text}")
            self.advance_zone()

    def schedule_goal_retry(self, goal_id: str, reason: str, returning_home: bool) -> bool:
        max_retries = int(self.get_parameter("max_goal_retries").value)
        retries = self.goal_retry_counts.get(goal_id, 0)
        if retries >= max_retries:
            return False
        self.goal_retry_counts[goal_id] = retries + 1
        delay = float(self.get_parameter("goal_retry_delay_s").value)
        self.retry_goal_after = time.monotonic() + delay
        self.goal_in_flight = False
        self.goal_handle = None
        self.goal_started_at = None
        self.active_goal_id = None
        self.near_goal_since = None
        if returning_home:
            self.return_goal_sent = False
        self.latest_feedback = {
            "goal_id": goal_id,
            "retry_reason": reason,
            "retry_count": retries + 1,
            "retry_delay_s": delay,
        }
        self.get_logger().warn(
            f"{goal_id} {reason}; retry {retries + 1}/{max_retries} in {delay:.1f}s"
        )
        return True

    def check_goal_close_enough(self, now: float):
        if (
            not self.goal_in_flight
            or self.goal_handle is None
            or self.near_goal_since is None
        ):
            return
        accept_after = float(self.get_parameter("accept_near_goal_after_s").value)
        if accept_after <= 0.0 or (now - self.near_goal_since) < accept_after:
            return
        goal_id = self.active_goal_id or "unknown_goal"
        feedback = self.latest_feedback.get("distance_remaining", "unknown")
        if self.return_goal_sent:
            self.get_logger().info(
                f"{goal_id} is close enough to home "
                f"(distance_remaining={feedback}); accepting return"
            )
        else:
            self.get_logger().info(
                f"{goal_id} is close enough for inspection "
                f"(distance_remaining={feedback}); accepting waypoint"
            )
        self.goal_handle.cancel_goal_async()
        self.goal_in_flight = False
        self.goal_handle = None
        self.goal_started_at = None
        self.active_goal_id = None
        self.near_goal_since = None
        if self.return_goal_sent:
            self.publish_return_home_log("RETURNED_HOME", goal_id, "Home / Start")
            self.finish_mission("RETURNED_HOME")
            return
        self.begin_inspection()

    def check_goal_timeout(self, now: float):
        if not self.goal_in_flight or self.goal_started_at is None:
            return
        timeout = float(self.get_parameter("goal_timeout_s").value)
        if timeout <= 0.0 or (now - self.goal_started_at) < timeout:
            return

        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
        self.goal_in_flight = False
        self.goal_handle = None
        self.goal_started_at = None
        self.active_goal_id = None
        self.near_goal_since = None
        if self.mode == "RETURNING_HOME" or self.return_goal_sent:
            self.publish_return_home_log("RETURN_TIMEOUT")
            self.finish_mission("RETURN_TIMEOUT")
        else:
            self.log_skipped_zone("NAV_TIMEOUT", "Nav2 goal timed out")
            self.advance_zone()

    @staticmethod
    def goal_status_name(status: int) -> str:
        names = {
            GoalStatus.STATUS_ABORTED: "ABORTED",
            GoalStatus.STATUS_CANCELED: "CANCELED",
            GoalStatus.STATUS_UNKNOWN: "UNKNOWN",
        }
        return names.get(status, f"STATUS_{status}")

    def begin_inspection(self):
        self.mode = "INSPECTING"
        self.current_result = None
        self.inspection_started_at = time.monotonic()
        self.request_sent = False

    def handle_inspection(self, now: float):
        zone = self.zones[self.current_index]
        if not self.request_sent:
            self.request_id += 1
            payload = {
                "event": "INSPECT_PLANT",
                "request_id": self.request_id,
                "zone_id": zone.zone_id,
                "zone_name": zone.name,
                "source_entity": str(self.get_parameter("source_entity").value),
                "navigation": "nav2",
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
        elif elapsed >= timeout:
            self.finish_inspection(
                {
                    "zone_id": zone.zone_id,
                    "zone_name": zone.name,
                    "status": "SENSOR_TIMEOUT",
                    "plant_stress_index": None,
                    "disease_level": None,
                    "recommendation": "RETRY_INSPECTION",
                    "confidence": 0.0,
                    "camera_health": self.digital_camera_health,
                }
            )

    def finish_inspection(self, result: Dict):
        zone = self.zones[self.current_index]
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
            "navigation": "nav2",
        }
        self.summary.append(entry)
        self.last_result = entry
        self.pub_log.publish(String(data=json.dumps(entry, sort_keys=True)))

        if is_treatment_needed_status(entry["status"]):
            hold_s = float(self.get_parameter("treatment_alert_hold_s").value)
            self.hold_until = time.monotonic() + hold_s
            self.mode = "HOLDING_ALERT"
            self.get_logger().warn(
                f"{zone.zone_id} plant stress/disease level is high. "
                f"Recommendation: APPLY_PESTICIDE. Holding {hold_s:.1f}s before next goal."
            )
        else:
            self.get_logger().info(f"{zone.zone_id} inspection complete: {entry['status']}")
            self.advance_zone()

    def log_skipped_zone(self, status: str, reason: str):
        zone = self.zones[self.current_index]
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
            "navigation": "nav2",
            "reason": reason,
        }
        self.summary.append(entry)
        self.last_result = entry
        self.pub_log.publish(String(data=json.dumps(entry, sort_keys=True)))
        self.get_logger().warn(f"{zone.zone_id} skipped: {status} ({reason})")

    def publish_return_home_log(
        self,
        status: str,
        goal_id: str = "plant_home",
        goal_name: str = "Home / Start",
        reason: str = "",
    ):
        entry = {
            "event": "RETURN_HOME_LOG",
            "zone_id": goal_id,
            "zone_name": goal_name,
            "status": status,
            "mission_index": self.current_index,
            "route_waypoint_count": len(self.zones),
            "navigation": "nav2",
            "pose": self.home_goal_pose(),
        }
        if reason:
            entry["reason"] = reason
        self.last_result = entry
        self.pub_log.publish(String(data=json.dumps(entry, sort_keys=True)))
        self.get_logger().info(f"Return home log published: {status} at {entry['pose']}")

    def advance_zone(self):
        self.current_index += 1
        self.current_result = None
        self.request_sent = False
        self.inspection_started_at = None
        self.hold_until = None
        self.latest_feedback = {}
        self.near_goal_since = None
        if self.current_index >= len(self.zones):
            if bool(self.get_parameter("return_to_start").value):
                self.mode = "RETURNING_HOME"
                self.return_goal_sent = False
                self.get_logger().info("Plant route complete; returning to start")
            else:
                self.finish_mission("COMPLETE")
        else:
            zone = self.zones[self.current_index]
            self.mode = "NAVIGATING"
            self.get_logger().info(f"Continuing Nav2 route toward {zone.zone_id}")

    def finish_mission(self, final_status: str):
        self.mode = "COMPLETE"
        self.return_goal_sent = False
        self.final_status = final_status
        self.last_result = {"status": final_status}
        self.publish_summary(final_status)
        self.get_logger().info(f"Plant inspection route complete: {final_status}")

    def publish_state(self, force: bool):
        now = time.monotonic()
        if not force and (now - self.last_state_publish_at) < 0.5:
            return
        self.last_state_publish_at = now
        zone = self.zones[self.current_index] if self.current_index < len(self.zones) else None
        payload = {
            "entity": str(self.get_parameter("source_entity").value),
            "navigation": "nav2",
            "mode": self.mode,
            "current_zone_id": zone.zone_id if zone else None,
            "current_zone_name": zone.name if zone else None,
            "pose": self.current_pose,
            "mission_index": self.current_index,
            "zone_count": len(self.zones),
            "goal_in_flight": self.goal_in_flight,
            "nav_feedback": self.latest_feedback,
            "digital_camera_health": self.digital_camera_health,
            "digital_mode": self.digital_mode,
            "last_result": self.last_result,
            "completed_inspections": len(self.summary),
            "return_to_start": bool(self.get_parameter("return_to_start").value),
            "return_strategy": "nav2_planned_path",
            "final_status": self.final_status,
            "uptime_s": round(now - self.mission_started_at, 2),
        }
        self.pub_state.publish(String(data=json.dumps(payload, sort_keys=True)))

    def publish_summary(self, final_status: str = "COMPLETE"):
        actual_home = self.home_goal_pose()
        payload = {
            "event": "MISSION_SUMMARY",
            "navigation": "nav2",
            "final_status": final_status,
            "return_strategy": "nav2_planned_path",
            "home_pose_configured": self.home_pose,
            "home_pose_used": actual_home,
            "amcl_home_captured": self.amcl_home_pose is not None,
            "total_route_waypoints": len(self.zones),
            "total_inspection_zones": len([zone for zone in self.zones if zone.zone_id != "plant_home"]),
            "inspections": self.summary,
        }
        self.pub_log.publish(String(data=json.dumps(payload, sort_keys=True)))

    def republish_summary_if_due(self, now: float):
        period = float(self.get_parameter("summary_republish_period_s").value)
        if period <= 0.0:
            return
        if (now - self.last_summary_publish_at) >= period:
            self.last_summary_publish_at = now
            status = self.last_result.get("status", "COMPLETE") if self.last_result else "COMPLETE"
            self.publish_summary(status)


def main(args=None):
    rclpy.init(args=args)
    node = PlantNav2MissionNode()
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
