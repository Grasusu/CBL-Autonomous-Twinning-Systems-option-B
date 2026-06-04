#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("tb3_pesticide_dt")
    default_params = os.path.join(pkg_share, "config", "plant_zones.yaml")

    params_file = LaunchConfiguration("params_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument("params_file", default_value=default_params),
            Node(
                package="tb3_pesticide_dt",
                executable="twin_safety_node",
                name="twin_safety_node",
                output="screen",
                parameters=[params_file],
            ),
        ]
    )

