#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    launch_file_dir = os.path.join(get_package_share_directory("turtlebot3_gazebo"), "launch")
    ros_gz_sim_share = get_package_share_directory("ros_gz_sim")
    my_pkg_share = get_package_share_directory("my_tb3_world")
    turtlebot3_model = os.environ.get("TURTLEBOT3_MODEL", "burger")

    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    x_pose = LaunchConfiguration("x_pose", default="0.0")
    y_pose = LaunchConfiguration("y_pose", default="0.0")
    world = os.path.join(my_pkg_share, "worlds", "new_world.world")
    urdf_path = os.path.join(
        get_package_share_directory("turtlebot3_gazebo"),
        "urdf",
        f"turtlebot3_{turtlebot3_model}.urdf",
    )
    with open(urdf_path, "r") as infp:
        robot_desc = infp.read()

    set_env_vars_resources = AppendEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH",
        os.path.join(get_package_share_directory("turtlebot3_gazebo"), "models"),
    )

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ros_gz_sim_share, "launch", "gz_sim.launch.py")),
        launch_arguments={
            "gz_args": f"-r -s -v2 {world}",
            "on_exit_shutdown": "true",
        }.items(),
    )

    robot_state_publisher_cmd = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "robot_description": robot_desc,
            }
        ],
    )

    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_file_dir, "spawn_turtlebot3.launch.py")),
        launch_arguments={"x_pose": x_pose, "y_pose": y_pose}.items(),
    )

    return LaunchDescription(
        [
            set_env_vars_resources,
            gzserver_cmd,
            robot_state_publisher_cmd,
            spawn_turtlebot_cmd,
        ]
    )
