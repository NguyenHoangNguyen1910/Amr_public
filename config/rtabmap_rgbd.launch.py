"""Shared registered RGB-D, wheel/IMU-driven mapper and optional independent VO."""
import shlex

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def mapping_nodes(context):
    queue = int(LaunchConfiguration('sync_queue_size').perform(context))
    topic_queue = int(LaunchConfiguration('topic_queue_size').perform(context))
    return [
        Node(package='rtabmap_sync', executable='rgbd_sync', name='rgbd_sync',
             output='screen', parameters=[{
                 'approx_sync': True, 'approx_sync_max_interval': 0.03,
                 'topic_queue_size': topic_queue, 'sync_queue_size': queue,
                 'qos': 2, 'qos_camera_info': 2,
             }], remappings=[
                 ('rgb/image', '/camera/color/image_raw'),
                 ('depth/image', '/camera/depth/image_raw'),
                 ('rgb/camera_info', '/camera/color/camera_info'),
                 ('rgbd_image', '/camera/rgbd_image')]),
        Node(package='rtabmap_slam', executable='rtabmap', name='rtabmap',
             namespace='rtabmap', output='screen', parameters=[{
                 'frame_id': 'base_footprint', 'map_frame_id': 'map',
                 'odom_frame_id': '', 'subscribe_depth': False,
                 'subscribe_rgbd': True, 'subscribe_odom_info': False,
                 'publish_tf': True, 'approx_sync': True,
                 'approx_sync_max_interval': 0.05, 'odom_sensor_sync': True,
                 'topic_queue_size': topic_queue, 'sync_queue_size': queue,
                 'qos_image': 2, 'qos_odom': 1, 'wait_for_transform': 0.1,
                 'database_path': LaunchConfiguration('database_path').perform(context),
                 'Rtabmap/DetectionRate': '1.0', 'Grid/Sensor': '1', 'Grid/RangeMax': '4.0',
             }], remappings=[('rgbd_image', '/camera/rgbd_image'), ('odom', '/odom'),
                             ('map', '/rtabmap/map')],
             arguments=shlex.split(LaunchConfiguration('rtabmap_args').perform(context))),
        Node(package='rtabmap_odom', executable='rgbd_odometry', name='rgbd_odometry',
             namespace='vo', output='screen', condition=IfCondition(LaunchConfiguration('enable_vo')),
             parameters=[{
                 'frame_id': 'base_footprint', 'odom_frame_id': 'vo_odom',
                 'publish_tf': False, 'subscribe_rgbd': True, 'guess_frame_id': '',
                 'wait_imu_to_init': False, 'publish_null_when_lost': True,
                 'qos': 2, 'topic_queue_size': 2, 'sync_queue_size': 2,
                 'wait_for_transform': 0.1, 'max_update_rate': 5.0,
                 'always_process_most_recent_frame': True,
                 'Odom/ImageDecimation': '2', 'Vis/MaxFeatures': '600',
             }], remappings=[('rgbd_image', '/camera/rgbd_image'),
                             ('odom', '/vo/odom_raw'), ('odom_info', '/vo/odom_info')]),
        ExecuteProcess(cmd=['python3', '/opt/amr/vo_gate.py', '--ros-args',
                            '--params-file', '/opt/amr/perception.yaml'],
                       output='screen', condition=IfCondition(LaunchConfiguration('enable_vo'))),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('enable_vo', default_value='false'),
        DeclareLaunchArgument('rtabmap_args', default_value=''),
        DeclareLaunchArgument('database_path', default_value='/root/.ros/rtabmap.db'),
        DeclareLaunchArgument('topic_queue_size', default_value='3'),
        DeclareLaunchArgument('sync_queue_size', default_value='3'),
        OpaqueFunction(function=mapping_nodes),
    ])
