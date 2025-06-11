from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
import os
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, Command

def generate_launch_description():
    pkg_path = get_package_share_directory('robotic_manipulator_6_dof')
    pkg_path1 = get_package_share_directory('move_it_config')
    launch_dir = os.path.join(pkg_path1, 'launch')
    urdf_file = os.path.join(pkg_path,'urdf','robot.urdf.xacro')
    return LaunchDescription([
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description":Command(["xacro ", urdf_file])}]
        ),
        IncludeLaunchDescription(
            PathJoinSubstitution([launch_dir, 'demo.launch.py'])
        )

    ])