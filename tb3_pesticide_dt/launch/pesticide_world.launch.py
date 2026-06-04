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

    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    x_pose = LaunchConfiguration("x_pose", default="0.0")
    y_pose = LaunchConfiguration("y_pose", default="0.0")
    gui = LaunchConfiguration("gui", default="true")

    world_path = os.path.join(world_share, "worlds", "new_world.world")
    urdf_path = os.path.join(
        turtlebot3_gazebo_share,
        "urdf",
        f"turtlebot3_{turtlebot3_model}.urdf",
    )
    with open(urdf_path, "r") as urdf_file:
        robot_description = urdf_file.read()

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

    robot_state_publisher_cmd = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "robot_description": robot_description,
            }
        ],
    )

    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                turtlebot3_gazebo_share,
                "launch",
                "spawn_turtlebot3.launch.py",
            )
        ),
        launch_arguments={"x_pose": x_pose, "y_pose": y_pose}.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("x_pose", default_value="0.0"),
            DeclareLaunchArgument("y_pose", default_value="0.0"),
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
            robot_state_publisher_cmd,
            spawn_turtlebot_cmd,
        ]
    )
