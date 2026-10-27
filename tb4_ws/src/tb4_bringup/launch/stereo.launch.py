"""Stereo depth from the simulated OAK-D pair (disparity + XYZRGB point cloud).

Inputs : stereo/{left,right}/{image_raw,camera_info}
Outputs: stereo/disparity, stereo/points2
The simulated images are already rectified, so no rectification node is needed.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    params = [{'approximate_sync': True, 'use_sim_time': True,
               'queue_size': 10, 'use_system_default_qos': False}]
    container = ComposableNodeContainer(
        name='stereo_container', namespace=LaunchConfiguration('namespace'),
        package='rclcpp_components', executable='component_container', output='screen',
        composable_node_descriptions=[
            ComposableNode(
                package='stereo_image_proc', plugin='stereo_image_proc::DisparityNode',
                name='disparity_node', parameters=params,
                remappings=[('left/image_rect', 'stereo/left/image_raw'),
                            ('left/camera_info', 'stereo/left/camera_info'),
                            ('right/image_rect', 'stereo/right/image_raw'),
                            ('right/camera_info', 'stereo/right/camera_info'),
                            ('disparity', 'stereo/disparity')]),
            ComposableNode(
                package='stereo_image_proc', plugin='stereo_image_proc::PointCloudNode',
                name='point_cloud_node', parameters=params,
                remappings=[('left/image_rect_color', 'stereo/left/image_raw'),
                            ('left/camera_info', 'stereo/left/camera_info'),
                            ('right/camera_info', 'stereo/right/camera_info'),
                            ('disparity', 'stereo/disparity'),
                            ('points2', 'stereo/points2')]),
        ])
    return LaunchDescription([DeclareLaunchArgument('namespace', default_value=''), container])
