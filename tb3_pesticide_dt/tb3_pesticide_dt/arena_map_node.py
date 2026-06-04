#!/usr/bin/env python3
"""Publish a fixed RViz marker map of the Gazebo arena."""

import math
import os
import xml.etree.ElementTree as ET

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray


def parse_floats(text, count):
    values = [float(value) for value in text.split()] if text else []
    while len(values) < count:
        values.append(0.0)
    return values[:count]


def yaw_to_quaternion(yaw):
    half = yaw * 0.5
    return {
        "x": 0.0,
        "y": 0.0,
        "z": math.sin(half),
        "w": math.cos(half),
    }


class ArenaMapNode(Node):
    def __init__(self):
        super().__init__("arena_map_node")
        self.declare_parameter("marker_topic", "/dt/digital/arena_map")
        self.declare_parameter("frame_id", "odom")
        self.declare_parameter("world_path", "")
        self.declare_parameter("publish_period_s", 1.0)
        self.declare_parameter("min_box_size_m", 0.20)

        self.marker_topic = str(self.get_parameter("marker_topic").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.boxes = self.load_world_boxes()

        self.pub_markers = self.create_publisher(MarkerArray, self.marker_topic, 10)
        self.create_timer(float(self.get_parameter("publish_period_s").value), self.publish_map)
        self.get_logger().info(
            f"ArenaMapNode publishing {len(self.boxes)} arena boxes on {self.marker_topic}"
        )

    def load_world_boxes(self):
        world_path = str(self.get_parameter("world_path").value)
        if not world_path:
            world_path = os.path.join(
                get_package_share_directory("my_tb3_world"),
                "worlds",
                "new_world.world",
            )

        root = ET.parse(world_path).getroot()
        boxes = []
        min_box_size = float(self.get_parameter("min_box_size_m").value)

        for model in root.findall(".//model"):
            size_text = model.findtext(".//collision/geometry/box/size")
            if size_text is None:
                continue
            size = parse_floats(size_text, 3)
            if max(size[0], size[1]) < min_box_size:
                continue
            pose = parse_floats(model.findtext("pose"), 6)
            boxes.append(
                {
                    "name": model.attrib.get("name", "arena_box"),
                    "x": pose[0],
                    "y": pose[1],
                    "z": pose[2],
                    "yaw": pose[5],
                    "size_x": size[0],
                    "size_y": size[1],
                    "size_z": size[2],
                }
            )
        return boxes

    def publish_map(self):
        markers = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        for i, box in enumerate(self.boxes):
            marker = Marker()
            marker.header.frame_id = self.frame_id
            marker.header.stamp = stamp
            marker.ns = "arena_walls"
            marker.id = i
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = box["x"]
            marker.pose.position.y = box["y"]
            marker.pose.position.z = max(0.04, box["z"])
            q = yaw_to_quaternion(box["yaw"])
            marker.pose.orientation.x = q["x"]
            marker.pose.orientation.y = q["y"]
            marker.pose.orientation.z = q["z"]
            marker.pose.orientation.w = q["w"]
            marker.scale.x = max(box["size_x"], 0.03)
            marker.scale.y = max(box["size_y"], 0.03)
            marker.scale.z = max(min(box["size_z"], 0.12), 0.06)
            marker.color.r = 0.52
            marker.color.g = 0.55
            marker.color.b = 0.58
            marker.color.a = 0.78
            markers.markers.append(marker)

        self.pub_markers.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = ArenaMapNode()
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
