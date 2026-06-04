#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("tb3_pesticide_dt")
    default_params = os.path.join(pkg_share, "config", "nav2_plant_zones.yaml")

    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription(
        [
            DeclareLaunchArgument("params_file", default_value=default_params),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            Node(
                package="tb3_pesticide_dt",
                executable="inspection_twin_node",
                name="inspection_twin_node",
                output="screen",
                parameters=[params_file, {"use_sim_time": use_sim_time}],
            ),
            Node(
                package="tb3_pesticide_dt",
                executable="arena_map_node",
                name="arena_map_node",
                output="screen",
                parameters=[params_file, {"use_sim_time": use_sim_time}],
            ),
            Node(
                package="tb3_pesticide_dt",
                executable="option_b_environment_node",
                name="option_b_environment_node",
                output="screen",
                parameters=[params_file, {"use_sim_time": use_sim_time}],
            ),
            Node(
                package="tb3_pesticide_dt",
                executable="plant_nav2_mission_node",
                name="plant_nav2_mission_node",
                output="screen",
                parameters=[params_file, {"use_sim_time": use_sim_time}],
            ),
        ]
    )
