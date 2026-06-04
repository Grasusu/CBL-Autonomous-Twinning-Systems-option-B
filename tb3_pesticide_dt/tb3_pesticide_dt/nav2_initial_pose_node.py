#!/usr/bin/env python3
"""Publish AMCL's initial pose long enough for Nav2 to activate reliably."""

import math
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


def yaw_to_quaternion(yaw: float):
    half = yaw * 0.5
    return {
        "x": 0.0,
        "y": 0.0,
        "z": math.sin(half),
        "w": math.cos(half),
    }


class Nav2InitialPoseNode(Node):
    def __init__(self):
        super().__init__("nav2_initial_pose_node")
        self.declare_parameter("initial_pose_topic", "/initialpose")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("x", 0.0)
        self.declare_parameter("y", 0.0)
        self.declare_parameter("yaw", 0.0)
        self.declare_parameter("covariance_x", 0.25)
        self.declare_parameter("covariance_y", 0.25)
        self.declare_parameter("covariance_yaw", 0.0685)
        self.declare_parameter("publish_period_s", 0.5)
        self.declare_parameter("duration_s", 10.0)

        topic = str(self.get_parameter("initial_pose_topic").value)
        self.publisher = self.create_publisher(PoseWithCovarianceStamped, topic, 10)
        self.started_at = time.monotonic()
        self.publish_count = 0
        self.timer = self.create_timer(
            float(self.get_parameter("publish_period_s").value),
            self.publish_initial_pose,
        )

        self.get_logger().info(
            f"Publishing Nav2 initial pose on {topic} so AMCL can activate before the return goal"
        )
        self.publish_initial_pose()

    def publish_initial_pose(self):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = str(self.get_parameter("frame_id").value)
        msg.pose.pose.position.x = float(self.get_parameter("x").value)
        msg.pose.pose.position.y = float(self.get_parameter("y").value)
        q = yaw_to_quaternion(float(self.get_parameter("yaw").value))
        msg.pose.pose.orientation.x = q["x"]
        msg.pose.pose.orientation.y = q["y"]
        msg.pose.pose.orientation.z = q["z"]
        msg.pose.pose.orientation.w = q["w"]
        msg.pose.covariance[0] = float(self.get_parameter("covariance_x").value)
        msg.pose.covariance[7] = float(self.get_parameter("covariance_y").value)
        msg.pose.covariance[35] = float(self.get_parameter("covariance_yaw").value)
        self.publisher.publish(msg)
        self.publish_count += 1

        duration_s = float(self.get_parameter("duration_s").value)
        if duration_s > 0.0 and (time.monotonic() - self.started_at) >= duration_s:
            self.get_logger().info(f"Finished publishing Nav2 initial pose ({self.publish_count} samples)")
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = Nav2InitialPoseNode()
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
