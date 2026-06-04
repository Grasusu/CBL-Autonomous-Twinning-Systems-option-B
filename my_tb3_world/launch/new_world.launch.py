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
    # Get paths to needed packages
    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')
    my_pkg_share     = get_package_share_directory('my_tb3_world')
    turtlebot3_model = os.environ.get('TURTLEBOT3_MODEL', 'burger')

    # Set launch arguments. Set initial position of robot
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    gui = LaunchConfiguration('gui', default='true')

    # path to the 3d model world file
    world = os.path.join(my_pkg_share, 'worlds', 'new_world.world')
    urdf_file_name = 'turtlebot3_' + turtlebot3_model + '.urdf'
    urdf_path = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'urdf',
        urdf_file_name)
    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    turtlebot3_share = get_package_share_directory('turtlebot3_gazebo')

    # Add TurtleBot3 model paths before Gazebo starts.
    set_env_vars_share = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.dirname(turtlebot3_share))
    set_env_vars_models = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(turtlebot3_share, 'models'))

    # Launch Gazebo server with custom world
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': f'-r -s -v2 {world}',
            'on_exit_shutdown': 'true'
        }.items()
    )

    # Launch Gazebo client
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': '-g -v2',
            'on_exit_shutdown': 'true'
        }.items(),
        condition=IfCondition(gui)
    )

    robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_desc,
        }],
    )

    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_file_dir, 'spawn_turtlebot3.launch.py')),
        launch_arguments={'x_pose': x_pose, 'y_pose': y_pose}.items()
    )

    ld = LaunchDescription()

    # Add environment/resources before Gazebo and spawn actions.
    ld.add_action(DeclareLaunchArgument('gui', default_value='true'))
    ld.add_action(set_env_vars_share)
    ld.add_action(set_env_vars_models)
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(spawn_turtlebot_cmd)

    return ld
