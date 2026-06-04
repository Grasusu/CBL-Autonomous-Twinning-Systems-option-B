#!/usr/bin/env python3
"""Mirror a real robot pose into a Gazebo Sim model.

This is for the physical-robot demo: Nav2 drives the real TurtleBot3, while the
Gazebo robot is only a visual digital twin.  The node listens to the real robot
pose and periodically teleports the Gazebo model to the same x/y/yaw.
"""

import math
import subprocess
import time
from typing import Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


Pose2D = Tuple[float, float, float]


def yaw_from_quaternion(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class GazeboPoseMirrorNode(Node):
    """Copies /odom or /amcl_pose into Gazebo's /world/<name>/set_pose service."""

    def __init__(self):
        super().__init__("gazebo_pose_mirror_node")
        self.declare_parameter("source_topic", "/odom")
        self.declare_parameter("source_type", "odom")
        self.declare_parameter("world_name", "default")
        self.declare_parameter("model_name", "burger")
        self.declare_parameter("mirror_period_s", 0.5)
        self.declare_parameter("service_timeout_ms", 1000)
        self.declare_parameter("x_scale", 1.0)
        self.declare_parameter("y_scale", 1.0)
        self.declare_parameter("x_offset", 0.0)
        self.declare_parameter("y_offset", 0.0)
        self.declare_parameter("yaw_offset", 0.0)
        self.declare_parameter("z", 0.01)
        self.declare_parameter("min_translation_delta_m", 0.01)
        self.declare_parameter("min_yaw_delta_rad", 0.02)

        self.source_topic = str(self.get_parameter("source_topic").value)
        self.source_type = str(self.get_parameter("source_type").value).lower()
        self.world_name = str(self.get_parameter("world_name").value)
        self.model_name = str(self.get_parameter("model_name").value)
        self.latest_pose: Optional[Pose2D] = None
        self.last_sent_pose: Optional[Pose2D] = None
        self.last_source_stamp = time.monotonic()
        self.last_warn_at = 0.0
        self.sent_count = 0

        if self.source_type == "amcl_pose":
            self.create_subscription(
                PoseWithCovarianceStamped,
                self.source_topic,
                self.on_amcl_pose,
                10,
            )
        elif self.source_type == "odom":
            self.create_subscription(Odometry, self.source_topic, self.on_odom, 10)
        else:
            raise ValueError("source_type must be 'odom' or 'amcl_pose'")

        period = float(self.get_parameter("mirror_period_s").value)
        self.create_timer(period, self.mirror_latest_pose)
        self.get_logger().info(
            f"Mirroring {self.source_topic} ({self.source_type}) to Gazebo model "
            f"{self.model_name} in world {self.world_name} every {period:.2f}s"
        )

    def on_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        self.latest_pose = (p.x, p.y, yaw)
        self.last_source_stamp = time.monotonic()

    def on_amcl_pose(self, msg: PoseWithCovarianceStamped):
        p = msg.pose.pose.position
        yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        self.latest_pose = (p.x, p.y, yaw)
        self.last_source_stamp = time.monotonic()

    def transformed_pose(self, pose: Pose2D) -> Pose2D:
        x, y, yaw = pose
        return (
            x * float(self.get_parameter("x_scale").value)
            + float(self.get_parameter("x_offset").value),
            y * float(self.get_parameter("y_scale").value)
            + float(self.get_parameter("y_offset").value),
            normalize_angle(yaw + float(self.get_parameter("yaw_offset").value)),
        )

    def pose_changed_enough(self, pose: Pose2D) -> bool:
        if self.last_sent_pose is None:
            return True
        dx = pose[0] - self.last_sent_pose[0]
        dy = pose[1] - self.last_sent_pose[1]
        dyaw = normalize_angle(pose[2] - self.last_sent_pose[2])
        min_xy = float(self.get_parameter("min_translation_delta_m").value)
        min_yaw = float(self.get_parameter("min_yaw_delta_rad").value)
        return math.hypot(dx, dy) >= min_xy or abs(dyaw) >= min_yaw

    def mirror_latest_pose(self):
        if self.latest_pose is None:
            self.warn_throttled(f"Waiting for pose messages on {self.source_topic}")
            return

        pose = self.transformed_pose(self.latest_pose)
        if not self.pose_changed_enough(pose):
            return

        ok, detail = self.call_gazebo_set_pose(pose)
        if ok:
            self.last_sent_pose = pose
            self.sent_count += 1
            if self.sent_count == 1:
                self.get_logger().info(
                    f"Gazebo pose mirror active at x={pose[0]:.2f}, y={pose[1]:.2f}, yaw={pose[2]:.2f}"
                )
        else:
            self.warn_throttled(detail)

    def call_gazebo_set_pose(self, pose: Pose2D) -> Tuple[bool, str]:
        x, y, yaw = pose
        half_yaw = yaw * 0.5
        z = float(self.get_parameter("z").value)
        request = (
            f'name: "{self.model_name}"\n'
            f"position {{ x: {x:.6f} y: {y:.6f} z: {z:.6f} }}\n"
            f"orientation {{ x: 0.0 y: 0.0 z: {math.sin(half_yaw):.6f} "
            f"w: {math.cos(half_yaw):.6f} }}"
        )
        timeout_ms = int(self.get_parameter("service_timeout_ms").value)
        command = [
            "gz",
            "service",
            "-s",
            f"/world/{self.world_name}/set_pose",
            "--reqtype",
            "gz.msgs.Pose",
            "--reptype",
            "gz.msgs.Boolean",
            "--timeout",
            str(timeout_ms),
            "--req",
            request,
        ]

        try:
            result = subprocess.run(
                command,
                capture_output=True,
                text=True,
                timeout=max(1.0, timeout_ms / 1000.0 + 0.5),
                check=False,
            )
        except FileNotFoundError:
            return False, "Could not find 'gz'. Start this node inside the ROS/Gazebo environment."
        except subprocess.TimeoutExpired:
            return False, "Timed out calling Gazebo set_pose service. Is Gazebo running?"

        output = f"{result.stdout}\n{result.stderr}".strip()
        output_lower = output.lower()
        if result.returncode != 0:
            return False, f"Gazebo set_pose failed: {output}"
        if "data: false" in output_lower:
            return False, f"Gazebo set_pose returned false: {output}"
        return True, output

    def warn_throttled(self, message: str, period_s: float = 5.0):
        now = time.monotonic()
        if now - self.last_warn_at >= period_s:
            self.last_warn_at = now
            self.get_logger().warn(message)


def main(args=None):
    rclpy.init(args=args)
    node = GazeboPoseMirrorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
