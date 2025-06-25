#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.actions import SetEnvironmentVariable

def generate_launch_description():
    # Package paths
    pkg_main = 'robotic_manipulator_6_dof'
    pkg_moveit = 'move_it_config_testing_june_25'
    pkg_gz = 'ros_gz_sim'
    pkg_path = get_package_share_directory(pkg_main)
    launch_dir_moveit = os.path.join(get_package_share_directory(pkg_moveit), 'launch')
    launch_dir_main = os.path.join(get_package_share_directory(pkg_main), 'launch')
    xacro_file = os.path.join(pkg_path, 'urdf', 'robot.urdf.xacro')
    controllers_yaml = os.path.join(pkg_path, 'config', 'ros2_controllers.yaml')

    set_gz_version = SetEnvironmentVariable(
        name='GZ_VERSION',
        value='garden'
    )
    
    # srdf_file = os.path.join(pkg_moveit, 'config', 'arm.srdf')
    # with open(srdf_file, 'r') as f:
    #     robot_description_semantic = f.read()
    # kinematics_yaml = os.path.join(pkg_moveit, 'config', 'kinematics.yaml')

    # Launch description
    return LaunchDescription([
        set_gz_version,
        

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(launch_dir_main, 'robot_state_publisher_launch.py')
        ])
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(launch_dir_main, 'gazebo_launch.py')
        ])
        ),

        # Robot state publisher with simulated time


        # Gazebo Fortress launcher
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([
        #         os.path.join(
        #             get_package_share_directory(pkg_gz),
        #             'launch', 'gz_sim.launch.py'
        #         )
        #     ]),
        #     condition=IfCondition(LaunchConfiguration('use_gazebo')),
        #     launch_arguments={'gz_args':'-r empty.sdf'}.items()
        # ),

        # Spawn the robot into Gazebo
        # Node(
        #     package='ros_gz_sim',
        #     executable='create',
        #     name='spawn_robot',
        #     output='screen',
        #     arguments=[
        #         '-entity', 'robot_arm',
        #         '-topic', 'robot_description',
        #         '-x', '0', '-y', '0', '-z', '0.1',
        #         '-R', '0', '-P', '0', '-Y', '0'
        #     ]
        # ),

        # ros2_control manager node
        # Node(
        #     package='controller_manager',
        #     executable='ros2_control_node',
        #     name='ros2_control_node',
        #     output='screen',
        #     parameters=[
        #         controllers_yaml,
        #         {'use_sim_time': True}
        #     ]
        # ),

        # Spawn controllers
        # Node(
        #     package='controller_manager',
        #     executable='spawner',
        #     name='spawn_controllers',
        #     parameters=[{'use_sim_time': True}],  # Added sim time sync
        #     arguments=[
        #         'joint_state_broadcaster',    # Essential for MoveIt2
        #         'manipulator_controller',
        #         'gripper_controller',
        #         '--controller-manager', '/controller_manager',
        #         '--controller-manager-timeout', '20'  # Allow time for Gazebo to initialize
        #     ]
        # ),

        # Include MoveIt demo launch (RViz, planning interface)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join((launch_dir_moveit), 'demo.launch.py')
            ]),
            # launch_arguments={
            #     'use_sim_time': 'true',  # Critical for sync
            #     'moveit_controller_manager': 'ros2_control'  # Explicit manager
            # }.items()
        )
    ])
