#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    source_topic = LaunchConfiguration("source_topic")
    source_type = LaunchConfiguration("source_type")
    world_name = LaunchConfiguration("world_name")
    model_name = LaunchConfiguration("model_name")
    mirror_period_s = LaunchConfiguration("mirror_period_s")
    x_offset = LaunchConfiguration("x_offset")
    y_offset = LaunchConfiguration("y_offset")
    yaw_offset = LaunchConfiguration("yaw_offset")

    return LaunchDescription(
        [
            DeclareLaunchArgument("source_topic", default_value="/odom"),
            DeclareLaunchArgument("source_type", default_value="odom"),
            DeclareLaunchArgument("world_name", default_value="default"),
            DeclareLaunchArgument("model_name", default_value="burger"),
            DeclareLaunchArgument("mirror_period_s", default_value="0.5"),
            DeclareLaunchArgument("x_offset", default_value="0.0"),
            DeclareLaunchArgument("y_offset", default_value="0.0"),
            DeclareLaunchArgument("yaw_offset", default_value="0.0"),
            Node(
                package="tb3_pesticide_dt",
                executable="gazebo_pose_mirror_node",
                name="gazebo_pose_mirror_node",
                output="screen",
                parameters=[
                    {
                        "source_topic": source_topic,
                        "source_type": source_type,
                        "world_name": world_name,
                        "model_name": model_name,
                        "mirror_period_s": mirror_period_s,
                        "x_offset": x_offset,
                        "y_offset": y_offset,
                        "yaw_offset": yaw_offset,
                    }
                ],
            ),
        ]
    )
