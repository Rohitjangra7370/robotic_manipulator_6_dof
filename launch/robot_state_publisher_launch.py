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
    pkg_moveit = 'move_it_config_testing_june_25'
    pkg_gz = 'ros_gz_sim'
    pkg_path = get_package_share_directory(pkg_name)
    launch_dir = os.path.join(get_package_share_directory(pkg_moveit), 'launch')
    xacro_file = os.path.join(pkg_path, 'urdf', 'robot.urdf.xacro')
    controllers_yaml = os.path.join(pkg_path, 'config', 'ros2_controllers.yaml')



    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    using_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_gazebo = DeclareLaunchArgument(
        'use_gazebo',
        default_value='true',
        description='Whether to launch Gazebo Fortress'
    )

    declare_robot_description=DeclareLaunchArgument(
        'robot_description',
        default_value=Command([
            FindExecutable(name='xacro'), ' ',
            xacro_file
        ]),
        description='URDF description of the robot'
    )

    robot_state_publisher=Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': LaunchConfiguration('robot_description')
        }]
    )


    return LaunchDescription([
        using_sim_time,
        declare_gazebo,
        declare_robot_description,
        robot_state_publisher

    ])
