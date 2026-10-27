"""View the TurtleBot 4 model in RViz (no simulation): ros2 launch tb4_description display.launch.py"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('tb4_description')
    xacro_file = os.path.join(share, 'urdf', 'tb4.urdf.xacro')
    robot_description = ParameterValue(
        Command([FindExecutable(name='xacro'), ' ', xacro_file]), value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description}]),
        Node(package='joint_state_publisher', executable='joint_state_publisher'),
        Node(package='rviz2', executable='rviz2', output='screen',
             arguments=['-d', os.path.join(share, 'rviz', 'model.rviz')]),
    ])
