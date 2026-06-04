#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    turtlebot3_gazebo_share = get_package_share_directory("turtlebot3_gazebo")
    ros_gz_sim_share = get_package_share_directory("ros_gz_sim")
    world_share = get_package_share_directory("my_tb3_world")
    turtlebot3_model = os.environ.get("TURTLEBOT3_MODEL", "burger")

    gui = LaunchConfiguration("gui")
    model_name = LaunchConfiguration("model_name")
    x_pose = LaunchConfiguration("x_pose")
    y_pose = LaunchConfiguration("y_pose")
    z_pose = LaunchConfiguration("z_pose")

    world_path = os.path.join(world_share, "worlds", "new_world.world")
    model_path = os.path.join(
        turtlebot3_gazebo_share,
        "models",
        f"turtlebot3_{turtlebot3_model}",
        "model.sdf",
    )

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": f"-r -s -v2 {world_path}",
            "on_exit_shutdown": "true",
        }.items(),
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": "-g -v2",
            "on_exit_shutdown": "true",
        }.items(),
        condition=IfCondition(gui),
    )

    spawn_visual_model_cmd = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_visual_twin",
        output="screen",
        arguments=[
            "-name",
            model_name,
            "-file",
            model_path,
            "-x",
            x_pose,
            "-y",
            y_pose,
            "-z",
            z_pose,
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("model_name", default_value="burger"),
            DeclareLaunchArgument("x_pose", default_value="0.0"),
            DeclareLaunchArgument("y_pose", default_value="0.0"),
            DeclareLaunchArgument("z_pose", default_value="0.01"),
            AppendEnvironmentVariable(
                "GZ_SIM_RESOURCE_PATH",
                os.path.dirname(turtlebot3_gazebo_share),
            ),
            AppendEnvironmentVariable(
                "GZ_SIM_RESOURCE_PATH",
                os.path.join(turtlebot3_gazebo_share, "models"),
            ),
            gzserver_cmd,
            gzclient_cmd,
            spawn_visual_model_cmd,
        ]
    )
