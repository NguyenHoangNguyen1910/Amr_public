"""Launch the robot web-control stack."""

import os
import math
import yaml

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
    with open(os.path.join(bringup_share, 'config', 'geometry.yaml')) as stream:
        geometry = yaml.safe_load(stream)
    for key, value in geometry.items():
        if key != 'calibration_confirmed' and not math.isfinite(float(value)):
            raise ValueError('geometry must contain finite measurements')
    perception_config = os.path.join(bringup_share, 'config', 'perception.yaml')
    front = float(geometry['robot_front_extent_m'])
    rear = float(geometry['robot_rear_extent_m'])
    half = float(geometry['robot_half_width_m'])
    measured_footprint = ({
        'robot_front_extent_m': front, 'robot_half_width_m': half,
        'robot_radius_m': math.hypot(max(front, rear), half),
    } if min(front, rear, half) > 0 else {})
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_depth_websocket', default_value='true',
            description='Serve the Astra depth topic through WebSocket'),
        DeclareLaunchArgument(
            'depth_websocket_topic', default_value='/camera/depth/image_raw',
            description='Depth topic served by the WebSocket preview'),
        DeclareLaunchArgument(
            'depth_websocket_port', default_value='8082',
            description='TCP port for the depth WebSocket preview'),
        DeclareLaunchArgument(
            'depth_websocket_hz', default_value='5.0',
            description='Maximum depth preview publish rate'),
        DeclareLaunchArgument(
            'use_slam_toolbox', default_value='false',
            description='Start LiDAR SLAM Toolbox alongside RTAB-Map'),
        DeclareLaunchArgument(
            'use_navigation', default_value='true',
            description='Start navigation using the projected depth scan'),
        DeclareLaunchArgument(
            'map_topic', default_value='/rtabmap/map',
            description='OccupancyGrid topic used by web and navigation'),
        *[DeclareLaunchArgument(key, default_value=str(geometry[key])) for key in (
            'camera_mount_x', 'camera_mount_y', 'camera_mount_z',
            'camera_mount_roll', 'camera_mount_pitch', 'camera_mount_yaw')],
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
        Node(
            package='imu_complementary_filter', executable='complementary_filter_node',
            name='complementary_filter_gain_node', output='screen',
            remappings=[('imu/data_raw', '/imu')],
            parameters=[{'do_bias_estimation': True, 'do_adaptive_gain': True,
                         'use_mag': False, 'publish_tf': False,
                         'publish_debug_topics': True, 'bias_alpha': 0.01,
                         'gain_acc': 0.01}],
        ),
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
        Node(package='bringup', executable='wheel_covariance_adapter.py',
             name='wheel_covariance_adapter', output='screen'),
        Node(package='bringup', executable='depth_obstacle_adapter.py',
             name='depth_obstacle_adapter', output='screen',
             parameters=[perception_config, {
                 'calibration_confirmed': bool(geometry['calibration_confirmed'])}]),
        # Advisory only, as requested: arbiter remains the sole /cmd_vel publisher.
        Node(package='bringup', executable='motion_monitor.py',
             name='motion_monitor', output='screen',
             parameters=[perception_config, {
                 'calibrated': bool(geometry['calibration_confirmed']),
                 'front': front, 'rear': rear, 'half_width': half}]),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_footprint_to_base_link',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0.05',
                '--roll', '0', '--pitch', '0', '--yaw', '0',
                '--frame-id', 'base_footprint', '--child-frame-id', 'base_link',
            ],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_base_imu',
            arguments=[
                '--x', '-0.002999', '--y', '-0.0030001', '--z', '0.031701',
                '--roll', '0', '--pitch', '0', '--yaw', '0',
                '--frame-id', 'base_link', '--child-frame-id', 'imu_frame',
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
                '--x', LaunchConfiguration('camera_mount_x'),
                '--y', LaunchConfiguration('camera_mount_y'),
                '--z', LaunchConfiguration('camera_mount_z'),
                '--roll', LaunchConfiguration('camera_mount_roll'),
                '--pitch', LaunchConfiguration('camera_mount_pitch'),
                '--yaw', LaunchConfiguration('camera_mount_yaw'),
                '--frame-id', 'base_link', '--child-frame-id', 'camera_link',
            ],
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
            parameters=[{
                'transform_timeout': 0.05,
                'use_scan': LaunchConfiguration('use_navigation'),
            }],
            remappings=[
                ('/map', LaunchConfiguration('map_topic')),
            ],
            output='screen',
        ),
        Node(
            package='bringup',
            executable='cmd_vel_arbiter.py',
            name='cmd_vel_arbiter',
            parameters=[{
                'default_mode': 'manual',
                'manual_max_linear_x': 0.15,
                'manual_max_angular_z': 0.30,
            }],
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
                measured_footprint,
            ],
            condition=IfCondition(LaunchConfiguration('use_navigation')),
        ),
        IncludeLaunchDescription(
            FrontendLaunchDescriptionSource(os.path.join(
                get_package_share_directory('rosbridge_server'),
                'launch', 'rosbridge_websocket_launch.xml'))),
        Node(
            package='bringup',
            executable='depth_websocket.py',
            name='depth_websocket',
            parameters=[{
                'topic': LaunchConfiguration('depth_websocket_topic'),
                'port': LaunchConfiguration('depth_websocket_port'),
                'stream_hz': LaunchConfiguration('depth_websocket_hz'),
            }],
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_depth_websocket')),
        ),
    ])
