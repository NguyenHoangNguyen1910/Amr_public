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
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory('bringup')
    slam_params = os.path.join(
        bringup_share, 'config', 'slam.yaml')
    navigation_config = LaunchConfiguration('navigation_config')
    camera_enabled = IfCondition(PythonExpression([
        "'",
        LaunchConfiguration('use_camera'), "'.lower() in ('true', '1', 'yes')",
    ]))
    camera_params = os.path.join(
        get_package_share_directory('usb_cam'), 'config', 'params_1.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_camera', default_value='false',
            description='Start legacy USB camera'),
        DeclareLaunchArgument(
            'use_astra', default_value='false',
            description='Start the legacy in-process Astra bridge'),
        DeclareLaunchArgument(
            'use_depth_websocket', default_value='true',
            description='Serve the ROS depth topic through WebSocket'),
        DeclareLaunchArgument(
            'depth_websocket_topic', default_value='/camera/depth/image_raw',
            description='Depth topic served by the WebSocket preview'),
        DeclareLaunchArgument(
            'depth_websocket_port', default_value='8082',
            description='TCP port for the depth WebSocket preview'),
        DeclareLaunchArgument(
            'use_slam_toolbox', default_value='false',
            description='Start LiDAR SLAM Toolbox alongside RTAB-Map'),
        DeclareLaunchArgument(
            'map_topic', default_value='/rtabmap/map',
            description='OccupancyGrid topic used by web and navigation'),
        DeclareLaunchArgument('camera_mount_x', default_value='0.0'),
        DeclareLaunchArgument('camera_mount_y', default_value='0.0'),
        DeclareLaunchArgument('camera_mount_z', default_value='0.0'),
        DeclareLaunchArgument('camera_mount_roll', default_value='0.0'),
        DeclareLaunchArgument('camera_mount_pitch', default_value='0.0'),
        DeclareLaunchArgument('camera_mount_yaw', default_value='0.0'),
        DeclareLaunchArgument(
            'slam_mode', default_value='mapping',
            description='slam_toolbox mode: mapping or localization',
            choices=['mapping', 'localization']),
        DeclareLaunchArgument(
            'slam_posegraph', default_value='',
            description='Serialized pose graph basename used for localization'),
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
            package='bringup',
            executable='imu_covariance_adapter.py',
            name='imu_covariance_adapter',
            output='screen',
        ),
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
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_camera',
            arguments=[
                LaunchConfiguration('camera_mount_x'),
                LaunchConfiguration('camera_mount_y'),
                LaunchConfiguration('camera_mount_z'),
                LaunchConfiguration('camera_mount_roll'),
                LaunchConfiguration('camera_mount_pitch'),
                LaunchConfiguration('camera_mount_yaw'),
                'base_link', 'camera_link',
            ],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_to_color_optical',
            arguments=[
                '0', '0', '0', '-1.57079632679', '0', '-1.57079632679',
                'camera_link', 'camera_color_optical_frame',
            ],
            condition=IfCondition(LaunchConfiguration('use_astra')),
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_to_depth_optical',
            arguments=[
                '0', '0', '0', '-1.57079632679', '0', '-1.57079632679',
                'camera_link', 'camera_depth_optical_frame',
            ],
            condition=IfCondition(LaunchConfiguration('use_astra')),
        ),
        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            parameters=[
                slam_params,
                {
                    'mode': LaunchConfiguration('slam_mode'),
                    'map_file_name': LaunchConfiguration('slam_posegraph'),
                    'map_start_at_dock': True,
                },
            ],
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_slam_toolbox')),
        ),
        Node(
            package='bringup',
            executable='web_bridge.py',
            name='web_bridge',
            parameters=[{'transform_timeout': 0.30}],
            remappings=[
                ('/map', LaunchConfiguration('map_topic')),
            ],
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
            remappings=[
                ('/map', LaunchConfiguration('map_topic')),
            ],
            parameters=[
                navigation_config,
                {'map_topic': LaunchConfiguration('map_topic')},
            ],
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
            condition=camera_enabled,
        ),
        Node(
            package='bringup',
            executable='camera_mjpeg.py',
            name='camera_mjpeg',
            output='screen',
            condition=camera_enabled,
        ),
        Node(
            package='bringup',
            executable='depth_websocket.py',
            name='depth_websocket',
            parameters=[{
                'topic': LaunchConfiguration('depth_websocket_topic'),
                'port': LaunchConfiguration('depth_websocket_port'),
            }],
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_depth_websocket')),
        ),
        Node(
            package='bringup',
            executable='astra_bridge.py',
            name='astra_bridge',
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_astra')),
        ),
    ])
