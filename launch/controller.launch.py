from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
import os
from launch.conditions import IfCondition
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource

declare_gazebo_arg = DeclareLaunchArgument(
    'use_gazebo', default_value='true',
    description='Whether to launch Gazebo simulation'
)

def generate_launch_description():
    pkg_path = get_package_share_directory('robotic_manipulator_6_dof')
    pkg_path1 = get_package_share_directory('move_it_config_testing_june_25')
    launch_dir = os.path.join(pkg_path1, 'launch')
    urdf_file = os.path.join(pkg_path,'urdf','robot.urdf.xacro')
    gazebo_pkg = get_package_share_directory('gazebo_ros')
    use_gazebo = LaunchConfiguration('use_gazebo', default='true')


    
    return LaunchDescription([


        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[
              os.path.join(pkg_path, 'config', 'ros2_controllers.yaml'),
              {'use_sim_time': True}
            ],
            output='screen'
        ),





    ])