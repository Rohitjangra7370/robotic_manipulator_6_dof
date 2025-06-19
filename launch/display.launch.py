#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Package paths
    pkg_name = 'robotic_manipulator_6_dof'
    pkg_moveit = 'move_it_config'
    pkg_gz = 'ros_gz_sim'
    pkg_path = get_package_share_directory(pkg_name)
    launch_dir = os.path.join(get_package_share_directory(pkg_moveit), 'launch')
    xacro_file = os.path.join(pkg_path, 'urdf', 'robot.urdf.xacro')
    controllers_yaml = os.path.join(pkg_path, 'config', 'ros2_controllers.yaml')

    # Launch arguments
    declare_gazebo = DeclareLaunchArgument(
        'use_gazebo',
        default_value='true',
        description='Whether to launch Gazebo Fortress'
    )
    declare_description = DeclareLaunchArgument(
        'robot_description',
        default_value=Command([
            FindExecutable(name='xacro'), ' ',
            xacro_file
        ]),
        description='URDF description of the robot'
    )

    # Launch description
    return LaunchDescription([
        declare_gazebo,
        declare_description,

        # Robot state publisher with simulated time
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'robot_description': LaunchConfiguration('robot_description')
            }]
        ),

        # Gazebo Fortress launcher
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(
                    get_package_share_directory(pkg_gz),
                    'launch', 'gz_sim.launch.py'
                )
            ]),
            condition=IfCondition(LaunchConfiguration('use_gazebo')),
            launch_arguments={'gz_args':'-r empty.sdf'}.items()
        ),

        # Spawn the robot into Gazebo
        Node(
            package='ros_gz_sim',
            executable='create',
            name='spawn_robot',
            output='screen',
            arguments=[
                '-entity', 'robot_arm',
                '-topic', 'robot_description',
                '-x', '0', '-y', '0', '-z', '0.1',
                '-R', '0', '-P', '0', '-Y', '0'
            ]
        ),

        # ros2_control manager node
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            name='ros2_control_node',
            output='screen',
            parameters=[
                controllers_yaml,
                {'use_sim_time': True}
            ]
        ),

        # Spawn controllers
        Node(
            package='controller_manager',
            executable='spawner',
            name='spawn_controllers',
            arguments=[
                'joint_state_controller',
                'manipulator_controller',
                'gripper_controller'
            ]
        ),

        # Include MoveIt demo launch (RViz, planning interface)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(launch_dir, 'demo.launch.py')
            ]),
            launch_arguments={
                'use_sim_time': 'true',  # Critical for sync
                'moveit_controller_manager': 'ros2_control'  # Explicit manager
            }.items()
        )
    ])
