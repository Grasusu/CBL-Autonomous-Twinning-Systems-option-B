#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("tb3_pesticide_dt")
    model_path = os.path.join(pkg_share, "models", "visual_twin_burger", "model.sdf")

    model_name = LaunchConfiguration("model_name")
    source_topic = LaunchConfiguration("source_topic")
    world_name = LaunchConfiguration("world_name")
    mirror_period_s = LaunchConfiguration("mirror_period_s")
    x_offset = LaunchConfiguration("x_offset")
    y_offset = LaunchConfiguration("y_offset")
    yaw_offset = LaunchConfiguration("yaw_offset")
    z = LaunchConfiguration("z")

    return LaunchDescription(
        [
            DeclareLaunchArgument("model_name", default_value="digital_burger"),
            DeclareLaunchArgument("source_topic", default_value="/odom"),
            DeclareLaunchArgument("world_name", default_value="default"),
            DeclareLaunchArgument("mirror_period_s", default_value="0.25"),
            DeclareLaunchArgument("x_offset", default_value="0.0"),
            DeclareLaunchArgument("y_offset", default_value="0.35"),
            DeclareLaunchArgument("yaw_offset", default_value="0.0"),
            DeclareLaunchArgument("z", default_value="0.02"),
            Node(
                package="ros_gz_sim",
                executable="create",
                name="spawn_option_b_visual_twin",
                output="screen",
                arguments=[
                    "-name",
                    model_name,
                    "-file",
                    model_path,
                    "-x",
                    x_offset,
                    "-y",
                    y_offset,
                    "-z",
                    z,
                ],
            ),
            Node(
                package="tb3_pesticide_dt",
                executable="gazebo_pose_mirror_node",
                name="option_b_visual_twin_mirror",
                output="screen",
                parameters=[
                    {
                        "source_topic": source_topic,
                        "source_type": "odom",
                        "world_name": world_name,
                        "model_name": model_name,
                        "mirror_period_s": mirror_period_s,
                        "x_offset": x_offset,
                        "y_offset": y_offset,
                        "yaw_offset": yaw_offset,
                        "z": z,
                        "min_translation_delta_m": 0.005,
                        "min_yaw_delta_rad": 0.01,
                    }
                ],
            ),
        ]
    )
