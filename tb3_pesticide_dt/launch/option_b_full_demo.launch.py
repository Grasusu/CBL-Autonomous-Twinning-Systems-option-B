#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("tb3_pesticide_dt")

    use_sim_time = LaunchConfiguration("use_sim_time")
    gui = LaunchConfiguration("gui")
    map_file = LaunchConfiguration("map")
    mission_params_file = LaunchConfiguration("mission_params_file")
    nav2_params_file = LaunchConfiguration("nav2_params_file")

    world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, "launch", "pesticide_world.launch.py")
        ),
        launch_arguments={
            "gui": gui,
            "use_sim_time": use_sim_time,
        }.items(),
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, "launch", "option_b_navigation2.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "map": map_file,
            "params_file": nav2_params_file,
            "autostart": "false",
        }.items(),
    )

    mission_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, "launch", "pesticide_nav2_dt.launch.py")
        ),
        launch_arguments={
            "params_file": mission_params_file,
            "use_sim_time": use_sim_time,
        }.items(),
    )

    initial_pose_node = Node(
        package="tb3_pesticide_dt",
        executable="nav2_initial_pose_node",
        name="nav2_initial_pose_node",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "x": -0.80,
                "y": -0.07,
                "yaw": 0.0,
                "duration_s": 20.0,
                "publish_period_s": 0.5,
            }
        ],
    )

    startup_localization = ExecuteProcess(
        cmd=[
            "ros2",
            "service",
            "call",
            "/lifecycle_manager_localization/manage_nodes",
            "nav2_msgs/srv/ManageLifecycleNodes",
            "{command: 0}",
        ],
        output="screen",
    )

    startup_navigation = ExecuteProcess(
        cmd=[
            "ros2",
            "service",
            "call",
            "/lifecycle_manager_navigation/manage_nodes",
            "nav2_msgs/srv/ManageLifecycleNodes",
            "{command: 0}",
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("map", default_value=os.path.join(pkg_share, "maps", "map.yaml")),
            DeclareLaunchArgument(
                "mission_params_file",
                default_value=os.path.join(pkg_share, "config", "nav2_plant_zones.yaml"),
            ),
            DeclareLaunchArgument(
                "nav2_params_file",
                default_value=os.path.join(pkg_share, "config", "nav2_burger_option_b.yaml"),
            ),
            world_launch,
            TimerAction(period=8.0, actions=[nav2_launch]),
            TimerAction(period=18.0, actions=[startup_localization]),
            TimerAction(period=25.0, actions=[initial_pose_node]),
            TimerAction(period=48.0, actions=[startup_navigation]),
            TimerAction(period=65.0, actions=[mission_launch]),
        ]
    )
