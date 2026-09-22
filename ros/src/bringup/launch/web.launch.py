"""Launch the robot web-control stack."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import (
    FrontendLaunchDescriptionSource,
    PythonLaunchDescriptionSource,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory('bringup')
    slam_params = os.path.join(
        bringup_share, 'config', 'slam.yaml')
    navigation_config = LaunchConfiguration('navigation_config')
    camera_params = os.path.join(
        get_package_share_directory('usb_cam'), 'config', 'params_1.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_camera', default_value='true',
            description='Start USB camera'),
        DeclareLaunchArgument(
            'navigation_config',
            default_value='/workspace/src/bringup/config/navigation.yaml',
            description='Navigation parameter file'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('imu_complementary_filter'),
                'launch', 'complementary_filter.launch.py'))),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('robot_localization'),
                'launch', 'ekf.launch.py'))),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_footprint_to_base_link',
            arguments=[
                '0', '0', '0.05', '0.0', '0.0', '0.0',
                'base_footprint', 'base_link',
            ],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_base_imu',
            arguments=[
                '-0.002999', '-0.0030001', '0.031701', '0', '0', '0',
                'base_link', 'imu_frame',
            ],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_laser',
            arguments=[
                '--x', '-0.0046412', '--y', '0', '--z', '0.094079',
                '--roll', '0', '--pitch', '0', '--yaw', '0',
                '--frame-id', 'base_link', '--child-frame-id', 'laser_frame',
            ],
            output='screen',
        ),
        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            parameters=[slam_params],
            output='screen',
        ),
        Node(
            package='bringup',
            executable='web_bridge.py',
            name='web_bridge',
            output='screen',
        ),
        Node(
            package='bringup',
            executable='cmd_vel_arbiter.py',
            name='cmd_vel_arbiter',
            parameters=[{'default_mode': 'manual'}],
            output='screen',
        ),
        Node(
            package='bringup',
            executable='navigation_node.py',
            name='navigation',
            output='screen',
            parameters=[navigation_config],
        ),
        IncludeLaunchDescription(
            FrontendLaunchDescriptionSource(os.path.join(
                get_package_share_directory('rosbridge_server'),
                'launch', 'rosbridge_websocket_launch.xml'))),
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='camera1',
            parameters=[
                camera_params,
                {'framerate': 8.0, 'image_raw.jpeg_quality': 45},
            ],
            remappings=[
                ('image_raw/compressed', 'usb_cam/image_raw/compressed'),
            ],
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_camera')),
        ),
        Node(
            package='bringup',
            executable='camera_mjpeg.py',
            name='camera_mjpeg',
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_camera')),
        ),
    ])
