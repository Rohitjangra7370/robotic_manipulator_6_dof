from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
import os
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_path = get_package_share_directory('robotic_manipulator_6_dof')
    pkg_path1 = get_package_share_directory('move_it_config')
    launch_dir = os.path.join(pkg_path1, 'launch')
    urdf_file = os.path.join(pkg_path,'urdf','robot.urdf.xacro')
    config_path = os.path.join(pkg_path ,'config', 'kinematics.yaml')
    use_gazebo = LaunchConfiguration('use_gazebo', default='true')
    gazebo_pkg = get_package_share_directory('gazebo_ros')
    return LaunchDescription([
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description":Command(["xacro ", urdf_file])}]
        ),
        # Node(
        #     package="joint_state_publisher_gui",
        #     executable="joint_state_publisher_gui",
        #     name="joint_state_publisher_gui",
        #     output="screen"
        # ),
        # Node(
        #     package='robotic_manipulator_6_dof',
        #     executable='pose_goal_commander',
        #     name='pose_goal_commander',
        #     output='screen',
        #     parameters=[config_path]
        # ),
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-topic', 'robot_description',
                '-entity', 'maze_bot',
                '-x', '9.464182',
                '-y', '13.545395',
                '-z', '0.034947',
                '-robot_namespace', '/'
            ],
            output='screen'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(gazebo_pkg, 'launch', 'gazebo.launch.py')])
        ),




        IncludeLaunchDescription(
            PathJoinSubstitution([launch_dir, 'demo.launch.py'])
            
        )

    ])