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
    robot_backend = LaunchConfiguration('robot_backend')
    hardware_backend = IfCondition(PythonExpression([
        "'", robot_backend, "' == 'hardware'",
    ]))
    simulation_backend = IfCondition(PythonExpression([
        "'", robot_backend, "' == 'simulation'",
    ]))
    hardware_camera = IfCondition(PythonExpression([
        "'", robot_backend, "' == 'hardware' and '",
        LaunchConfiguration('use_camera'), "'.lower() in ('true', '1', 'yes')",
    ]))
    camera_stream = IfCondition(PythonExpression([
        "'", robot_backend, "' == 'hardware' and (('",
        LaunchConfiguration('use_camera'),
        "'.lower() in ('true', '1', 'yes')) or ('",
        LaunchConfiguration('use_depth_camera'),
        "'.lower() in ('true', '1', 'yes')))",
    ]))
    depth_adapter = IfCondition(PythonExpression([
        "'", robot_backend, "' == 'hardware' and '",
        LaunchConfiguration('use_rtabmap_depth_adapter'),
        "'.lower() in ('true', '1', 'yes')",
    ]))
    lidar_slam = IfCondition(PythonExpression([
        "'", robot_backend, "' == 'hardware' and '",
        LaunchConfiguration('use_lidar_slam'),
        "'.lower() in ('true', '1', 'yes')",
    ]))
    camera_params = os.path.join(
        get_package_share_directory('usb_cam'), 'config', 'params_1.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_camera', default_value='false',
            description='Start USB camera when available'),
        DeclareLaunchArgument(
            'use_depth_camera', default_value='true',
            description='Serve the Astra depth camera in the web UI'),
        DeclareLaunchArgument(
            'camera_stream_topic', default_value='/camera/depth/image_raw',
            description='ROS image topic served by camera_mjpeg'),
        DeclareLaunchArgument(
            'camera_stream_mode', default_value='depth',
            description='camera_mjpeg input mode: depth or compressed'),
        DeclareLaunchArgument(
            'use_rtabmap_depth_adapter', default_value='false',
            description='Optionally publish canonical 32FC1 depth for RTAB-Map'),
        DeclareLaunchArgument(
            'rtabmap_depth_input_topic', default_value='/camera/depth/image_raw',
            description='Raw depth topic consumed by the RTAB-Map adapter'),
        DeclareLaunchArgument(
            'rtabmap_depth_topic', default_value='/camera/depth/rtabmap',
            description='Canonical 32FC1 depth topic for RTAB-Map'),
        DeclareLaunchArgument(
            'map_topic', default_value='/rtabmap/map',
            description='Map topic forwarded to the web UI'),
        DeclareLaunchArgument(
            'camera_mount_x', default_value='0.0',
            description='Camera_link x offset from base_link in metres'),
        DeclareLaunchArgument(
            'camera_mount_y', default_value='0.0',
            description='Camera_link y offset from base_link in metres'),
        DeclareLaunchArgument(
            'camera_mount_z', default_value='0.0',
            description='Camera_link z offset from base_link in metres'),
        DeclareLaunchArgument(
            'camera_mount_roll', default_value='0.0',
            description='Camera_link roll from base_link in radians'),
        DeclareLaunchArgument(
            'camera_mount_pitch', default_value='0.0',
            description='Camera_link pitch from base_link in radians'),
        DeclareLaunchArgument(
            'camera_mount_yaw', default_value='0.0',
            description='Camera_link yaw from base_link in radians'),
        DeclareLaunchArgument(
            'use_lidar_slam', default_value='false',
            description='Start slam_toolbox from LiDAR /scan'),
        DeclareLaunchArgument(
            'robot_backend', default_value='hardware',
            description='Robot adapter: hardware or simulation',
            choices=['hardware', 'simulation']),
        DeclareLaunchArgument(
            'slam_mode', default_value='mapping',
            description='slam_toolbox mode for hardware: mapping or localization',
            choices=['mapping', 'localization']),
        DeclareLaunchArgument(
            'slam_posegraph', default_value='',
            description='Serialized pose graph basename used for localization'),
        DeclareLaunchArgument(
            'sim_map',
            default_value=os.path.join(bringup_share, 'maps', 'amr_world.json'),
            description='Simulator .json world or ROS .yaml/.pgm map pair'),
        DeclareLaunchArgument(
            'sim_initial_x', default_value='-2.0',
            description='Virtual robot initial map-frame x coordinate'),
        DeclareLaunchArgument(
            'sim_initial_y', default_value='-0.5',
            description='Virtual robot initial map-frame y coordinate'),
        DeclareLaunchArgument(
            'sim_initial_yaw', default_value='0.0',
            description='Virtual robot initial yaw in radians'),
        DeclareLaunchArgument(
            'navigation_config',
            default_value='/workspace/src/bringup/config/navigation.yaml',
            description='Navigation parameter file'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('imu_complementary_filter'),
                'launch', 'complementary_filter.launch.py')),
            condition=hardware_backend),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('robot_localization'),
                'launch', 'ekf.launch.py')),
            condition=hardware_backend),
        Node(
            package='bringup',
            executable='imu_covariance_adapter.py',
            name='imu_covariance_adapter',
            output='screen',
            condition=hardware_backend,
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
            output='screen',
            condition=hardware_backend,
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
            condition=lidar_slam,
        ),
        Node(
            package='bringup',
            executable='simulation_adapter.py',
            name='simulation_adapter',
            parameters=[{
                'map_path': LaunchConfiguration('sim_map'),
                'navigation_config_path': navigation_config,
                'initial_x': LaunchConfiguration('sim_initial_x'),
                'initial_y': LaunchConfiguration('sim_initial_y'),
                'initial_yaw': LaunchConfiguration('sim_initial_yaw'),
            }],
            output='screen',
            condition=simulation_backend,
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
            condition=hardware_camera,
        ),
        Node(
            package='bringup',
            executable='camera_mjpeg.py',
            name='camera_mjpeg',
            parameters=[{
                'topic': LaunchConfiguration('camera_stream_topic'),
                'mode': LaunchConfiguration('camera_stream_mode'),
            }],
            output='screen',
            condition=camera_stream,
        ),
        Node(
            package='bringup',
            executable='depth_rtabmap_adapter.py',
            name='depth_rtabmap_adapter',
            parameters=[{
                'input_topic': LaunchConfiguration('rtabmap_depth_input_topic'),
                'output_topic': LaunchConfiguration('rtabmap_depth_topic'),
            }],
            output='screen',
            condition=depth_adapter,
        ),
    ])
