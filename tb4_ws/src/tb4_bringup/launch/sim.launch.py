"""Gazebo Harmonic + TurtleBot 4 with drive, lidar and stereo camera.

  ros2 launch tb4_bringup sim.launch.py
  ros2 launch tb4_bringup sim.launch.py world:=arena x:=1.0 y:=0.5 namespace:=runner
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, OpaqueFunction)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _bool_arg(name, default, doc):
    return DeclareLaunchArgument(name, default_value=default, choices=['true', 'false'],
                                 description=doc)


def _launch_robot(context):
    """Everything that depends on the namespace string is built here, once it is known."""
    ns = LaunchConfiguration('namespace').perform(context).strip('/')
    tp = f'/{ns}' if ns else ''          # prefix for absolute topic names
    robot_name = ns if ns else 'tb4'
    use_sim_time = {'use_sim_time': True}

    desc_share = get_package_share_directory('tb4_description')
    xacro_file = os.path.join(desc_share, 'urdf', 'tb4.urdf.xacro')
    xacro_cmd = [
        FindExecutable(name='xacro'), ' ', xacro_file,
        ' namespace:=', ns,
        ' lidar_samples:=', LaunchConfiguration('lidar_samples'),
        ' lidar_rate:=', LaunchConfiguration('lidar_rate'),
        ' stereo_width:=', LaunchConfiguration('stereo_width'),
        ' stereo_height:=', LaunchConfiguration('stereo_height'),
        ' stereo_fps:=', LaunchConfiguration('stereo_fps'),
    ]
    robot_description = ParameterValue(Command(xacro_cmd), value_type=str)
    tf_remaps = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    actions = []

    # --- robot_state_publisher (namespaced topics, un-prefixed frames) ---
    actions.append(Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        namespace=ns, output='screen', remappings=tf_remaps,
        parameters=[{'robot_description': robot_description}, use_sim_time]))

    # --- spawn from the robot_description topic ---
    actions.append(Node(
        package='ros_gz_sim', executable='create', namespace=ns, output='screen',
        arguments=['-name', robot_name, '-topic', 'robot_description',
                   '-x', LaunchConfiguration('x'), '-y', LaunchConfiguration('y'),
                   '-z', LaunchConfiguration('z'), '-Y', LaunchConfiguration('yaw')]))

    # --- ROS <-> Gazebo bridge ---
    stereo = [f'{tp}/stereo/{eye}/{kind}' for eye in ('left', 'right')
              for kind in ('image_raw', 'camera_info')]
    bridge_args = [
        '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        f'{tp}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
        f'{tp}/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
        f'{tp}/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
        f'{tp}/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
        f'{tp}/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
        f'{tp}/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
    ]
    for t in stereo:
        msg = 'Image' if t.endswith('image_raw') else 'CameraInfo'
        bridge_args.append(f'{t}@sensor_msgs/msg/{msg}[gz.msgs.{msg}')
    actions.append(Node(
        package='ros_gz_bridge', executable='parameter_bridge', name='tb4_bridge',
        output='screen', arguments=bridge_args, parameters=[use_sim_time]))

    # --- velocity gate: cmd_vel_raw -> (limits, accel ramp, lidar safety) -> cmd_vel ---
    actions.append(Node(
        package='tb4_drive', executable='velocity_gate', namespace=ns, output='screen',
        parameters=[use_sim_time], condition=IfCondition(LaunchConfiguration('drive'))))

    # --- stereo depth: disparity + point cloud ---
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [get_package_share_directory('tb4_bringup'), 'launch', 'stereo.launch.py'])),
        launch_arguments={'namespace': ns}.items(),
        condition=IfCondition(LaunchConfiguration('stereo_proc'))))

    # --- RViz (relative topic names in the config + node namespace) ---
    rviz_cfg = os.path.join(get_package_share_directory('tb4_bringup'), 'rviz', 'tb4.rviz')
    rviz_args = ['-d', rviz_cfg]
    actions.append(Node(
        package='rviz2', executable='rviz2', namespace=ns, output='log',
        arguments=rviz_args, remappings=tf_remaps, parameters=[use_sim_time],
        condition=IfCondition(LaunchConfiguration('rviz'))))
    return actions


def generate_launch_description():
    bringup = get_package_share_directory('tb4_bringup')
    desc_parent = os.path.dirname(get_package_share_directory('tb4_description'))

    world_file = PathJoinSubstitution([bringup, 'worlds', [LaunchConfiguration('world'), '.sdf']])

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='empty',
                              description='World in tb4_bringup/worlds (empty, arena)'),
        DeclareLaunchArgument('namespace', default_value='',
                              description='Robot namespace (topics become /<ns>/...)'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.05'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        _bool_arg('gui', 'true', 'Show the Gazebo GUI (false = headless server only)'),
        _bool_arg('rviz', 'true', 'Start RViz'),
        _bool_arg('drive', 'true', 'Start the velocity gate (cmd_vel_raw -> cmd_vel)'),
        _bool_arg('stereo_proc', 'true', 'Compute stereo disparity + point cloud'),
        DeclareLaunchArgument('lidar_samples', default_value='720'),
        DeclareLaunchArgument('lidar_rate', default_value='10'),
        DeclareLaunchArgument('stereo_width', default_value='640'),
        DeclareLaunchArgument('stereo_height', default_value='400'),
        DeclareLaunchArgument('stereo_fps', default_value='30'),

        # lets Gazebo resolve package://tb4_description/meshes/...
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', desc_parent),

        ExecuteProcess(cmd=['gz', 'sim', '-r', world_file], output='screen',
                       condition=IfCondition(LaunchConfiguration('gui'))),
        ExecuteProcess(cmd=['gz', 'sim', '-r', '-s', '--headless-rendering', world_file],
                       output='screen', condition=UnlessCondition(LaunchConfiguration('gui'))),

        OpaqueFunction(function=_launch_robot),
    ])
