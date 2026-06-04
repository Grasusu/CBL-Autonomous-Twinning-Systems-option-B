#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetRemap


def generate_launch_description():
    nav2_bringup_share = get_package_share_directory("nav2_bringup")
    pkg_share = get_package_share_directory("tb3_pesticide_dt")

    default_params = os.path.join(pkg_share, "config", "nav2_odom_return.yaml")
    navigation_launch = os.path.join(nav2_bringup_share, "launch", "navigation_launch.py")

    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic")

    return LaunchDescription(
        [
            DeclareLaunchArgument("params_file", default_value=default_params),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("autostart", default_value="true"),
            DeclareLaunchArgument("cmd_vel_topic", default_value="/nav2/cmd_vel"),
            SetRemap(src="/cmd_vel", dst=cmd_vel_topic),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(navigation_launch),
                launch_arguments={
                    "params_file": params_file,
                    "use_sim_time": use_sim_time,
                    "autostart": autostart,
                    "use_composition": "False",
                }.items(),
            ),
        ]
    )
