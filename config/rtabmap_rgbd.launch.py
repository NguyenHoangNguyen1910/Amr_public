from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    official_launch = (
        '/opt/ros/humble/share/rtabmap_launch/launch/rtabmap.launch.py')

    return LaunchDescription([
        DeclareLaunchArgument('rtabmap_args', default_value=''),
        DeclareLaunchArgument(
            'database_path', default_value='/root/.ros/rtabmap.db'),
        DeclareLaunchArgument('topic_queue_size', default_value='10'),
        DeclareLaunchArgument('sync_queue_size', default_value='10'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(official_launch),
            launch_arguments={
                'frame_id': 'base_link',
                'rgb_topic': '/camera/color/image_raw',
                'depth_topic': '/camera/depth/image_raw',
                'camera_info_topic': '/camera/color/camera_info',
                'visual_odometry': 'false',
                'odom_topic': '/odom',
                'rgbd_sync': 'false',
                'subscribe_rgbd': 'false',
                'approx_sync': 'true',
                'approx_sync_max_interval': '0.1',
                'qos': '2',
                'topic_queue_size': LaunchConfiguration('topic_queue_size'),
                'sync_queue_size': LaunchConfiguration('sync_queue_size'),
                'wait_for_transform': '2.0',
                'args': LaunchConfiguration('rtabmap_args'),
                'database_path': LaunchConfiguration('database_path'),
                'rtabmap_viz': 'false',
                'rviz': 'false',
            }.items()),
    ])
