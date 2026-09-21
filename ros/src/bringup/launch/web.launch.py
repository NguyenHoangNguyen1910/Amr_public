"""Launch the robot web-control stack."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import (
    FrontendLaunchDescriptionSource,
    PythonLaunchDescriptionSource,
)
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory('bringup')
    nav2_params = os.path.join(
        bringup_share, 'config', 'nav2.yaml')
    slam_params = os.path.join(
        bringup_share, 'config', 'slam.yaml')
    custom_navigation_params = os.path.join(
        bringup_share, 'config', 'custom_navigation.yaml')

    navigation_backend = LaunchConfiguration('navigation_backend')
    custom_condition = IfCondition(PythonExpression([
        "'", navigation_backend, "' == \"custom\""]))
    nav2_condition = IfCondition(PythonExpression([
        "'", navigation_backend, "' == \"nav2\""]))

    nav2_common = {
        'output': 'screen',
        'parameters': [nav2_params],
    }
    tf_remaps = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    lifecycle_nodes = [
        'controller_server',
        'smoother_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
        'velocity_smoother',
    ]

    def validate_backend(context):
        backend = navigation_backend.perform(context).strip().lower()
        if backend not in ('custom', 'nav2'):
            raise RuntimeError(
                "navigation_backend must be either 'custom' or 'nav2', "
                f"got {backend!r}")
        return []

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_camera', default_value='true',
            description='Start USB camera'),
        DeclareLaunchArgument(
            'navigation_backend', default_value='nav2',
            description='Navigation backend: nav2 or custom'),
        OpaqueFunction(function=validate_backend),
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
            package='nav2_controller',
            executable='controller_server',
            remappings=tf_remaps + [('cmd_vel', '/cmd_vel_nav_raw')],
            condition=nav2_condition,
            **nav2_common,
        ),
        Node(
            package='nav2_smoother',
            executable='smoother_server',
            name='smoother_server',
            remappings=tf_remaps,
            condition=nav2_condition,
            **nav2_common,
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            remappings=tf_remaps,
            condition=nav2_condition,
            **nav2_common,
        ),
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            remappings=tf_remaps + [('cmd_vel', '/cmd_vel_nav_raw')],
            condition=nav2_condition,
            **nav2_common,
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            remappings=tf_remaps,
            condition=nav2_condition,
            **nav2_common,
        ),
        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            remappings=tf_remaps,
            condition=nav2_condition,
            **nav2_common,
        ),
        Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            name='velocity_smoother',
            remappings=tf_remaps + [
                ('cmd_vel', '/cmd_vel_nav_raw'),
                ('cmd_vel_smoothed', '/cmd_vel_nav'),
            ],
            condition=nav2_condition,
            **nav2_common,
        ),
        Node(
            package='bringup',
            executable='custom_navigation_node.py',
            name='custom_navigation',
            output='screen',
            parameters=[custom_navigation_params],
            condition=custom_condition,
        ),
        # Delay lifecycle transitions while Nav2 and SLAM settle on the Pi.
        TimerAction(
            period=8.0,
            actions=[Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                parameters=[
                    {'use_sim_time': False},
                    {'autostart': True},
                    {'node_names': lifecycle_nodes},
                ],
                condition=nav2_condition,
            )],
        ),
        IncludeLaunchDescription(
            FrontendLaunchDescriptionSource(os.path.join(
                get_package_share_directory('rosbridge_server'),
                'launch', 'rosbridge_websocket_launch.xml'))),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('usb_cam'),
                'launch', 'camera.launch.py')),
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
