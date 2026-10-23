"""Exercise installed RTAB-Map launch and optional gate without camera hardware."""
import json
import os
import signal
import subprocess
import time
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped, TwistWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from nav_msgs.srv import GetMap
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.srv import GetParameters
from rtabmap_msgs.msg import OdomInfo, RGBDImage
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Empty, String
from tf2_msgs.msg import TFMessage
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster


def main():
    assert os.environ.get('ROS_DOMAIN_ID') == '84'
    enabled = os.environ['VERIFY_VO'] == 'true'
    rclpy.init()
    probe = Node('rtabmap_verification')

    def spin(seconds):
        deadline = time.monotonic()+seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(probe, timeout_sec=0.01)

    def parameter(node, name):
        client = probe.create_client(GetParameters, node+'/get_parameters')
        assert client.wait_for_service(timeout_sec=5), node
        future = client.call_async(GetParameters.Request(names=[name]))
        rclpy.spin_until_future_complete(probe, future, timeout_sec=5)
        return future.result().values[0]

    label = 'vo-on' if enabled else 'vo-off'
    with open('/verify/rtabmap-'+label+'.log', 'w') as log:
        process = subprocess.Popen([
            'ros2', 'launch', '/opt/amr/rtabmap_rgbd.launch.py',
            'enable_vo:='+str(enabled).lower(), 'database_path:=/tmp/verify.db',
            'map_archive_dir:=/verify/maps-'+label],
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            expected = {'rgbd_sync', 'rtabmap', 'map_lifecycle'} | ({'rgbd_odometry', 'vo_gate'} if enabled else set())
            deadline = time.monotonic()+20
            while time.monotonic() < deadline:
                spin(0.1)
                if expected.issubset(set(probe.get_node_names())):
                    break
            assert process.poll() is None, 'RTAB-Map launch exited'
            assert expected.issubset(set(probe.get_node_names())), probe.get_node_names()
            assert parameter('/rtabmap/rtabmap', 'subscribe_rgbd').bool_value
            assert not parameter('/rtabmap/rtabmap', 'subscribe_odom_info').bool_value
            subscriptions = probe.get_subscriber_names_and_types_by_node('rtabmap', '/rtabmap')
            assert '/odom' in [topic for topic, _ in subscriptions]
            assert '/camera/rgbd_image' in [topic for topic, _ in subscriptions]
            assert not any(topic.startswith('/vo/') for topic, _ in subscriptions)
            if not enabled:
                assert 'rgbd_odometry' not in probe.get_node_names()
                assert 'vo_gate' not in probe.get_node_names()
                # Exercise actual RGB-D synchronization and mapping without VO.
                rgb = probe.create_publisher(Image, '/camera/color/image_raw', 1)
                depth = probe.create_publisher(Image, '/camera/depth/image_raw', 1)
                camera = probe.create_publisher(CameraInfo, '/camera/color/camera_info', 1)
                wheel = probe.create_publisher(Odometry, '/odom', 1)
                synced, maps = [], []
                probe.create_subscription(RGBDImage, '/camera/rgbd_image', synced.append,
                                          qos_profile_sensor_data)
                probe.create_subscription(OccupancyGrid, '/rtabmap/map', maps.append, 10)
                static = StaticTransformBroadcaster(probe)
                dynamic = TransformBroadcaster(probe)
                transform = TransformStamped()
                transform.header.stamp = probe.get_clock().now().to_msg()
                transform.header.frame_id, transform.child_frame_id = 'base_footprint', 'verify_camera_optical'
                transform.transform.translation.z = 0.3
                q = transform.transform.rotation
                q.x, q.y, q.z, q.w = -0.5, 0.5, -0.5, 0.5
                static.sendTransform(transform)
                pixels = np.random.default_rng(7).integers(0, 255, (120, 160, 3), dtype=np.uint8)
                spin(0.5)
                for index in range(60):
                    stamp = probe.get_clock().now().to_msg()
                    color, distance, info, odom = Image(), Image(), CameraInfo(), Odometry()
                    for message in (color, distance, info):
                        message.header.stamp, message.header.frame_id = stamp, 'verify_camera_optical'
                        message.width, message.height = 160, 120
                    color.encoding, color.step, color.data = 'rgb8', 480, pixels.tobytes()
                    distance.encoding, distance.step = '16UC1', 320
                    distance.data = np.full((120, 160), 1000, dtype='<u2').tobytes()
                    info.k = [140., 0., 80., 0., 140., 60., 0., 0., 1.]
                    info.p = [140., 0., 80., 0., 0., 140., 60., 0., 0., 0., 1., 0.]
                    info.r = [1., 0., 0., 0., 1., 0., 0., 0., 1.]
                    info.distortion_model, info.d = 'plumb_bob', [0.]*5
                    odom.header.stamp, odom.header.frame_id = stamp, 'odom'
                    odom.child_frame_id, odom.pose.pose.orientation.w = 'base_footprint', 1.0
                    odom.pose.pose.position.x = index*0.01
                    odom.twist.twist.linear.x = 0.1
                    for axis in range(6):
                        odom.pose.covariance[axis*7] = odom.twist.covariance[axis*7] = 0.001
                    base = TransformStamped()
                    base.header, base.child_frame_id = odom.header, 'base_footprint'
                    base.transform.translation.x = odom.pose.pose.position.x
                    base.transform.rotation.w = 1.0
                    dynamic.sendTransform(base)
                    wheel.publish(odom)
                    camera.publish(info)
                    rgb.publish(color)
                    depth.publish(distance)
                    spin(0.1)
                spin(0.5)
                assert len(synced) > 10, 'shared RGB-D sync did not process frames'
                assert any(m.info.width > 0 and m.info.height > 0 for m in maps), 'mapper produced no map without VO'
                statuses = []
                freeze = probe.create_publisher(Empty, '/web/freeze_map', 10)
                probe.create_subscription(String, '/web/map_mode', statuses.append, 10)
                spin(0.3)
                for _ in range(4):
                    stationary = Odometry()
                    stationary.header.stamp = probe.get_clock().now().to_msg()
                    stationary.header.frame_id = 'odom'
                    stationary.child_frame_id = 'base_footprint'
                    stationary.pose.pose.orientation.w = 1.0
                    wheel.publish(stationary)
                    spin(0.1)
                freeze.publish(Empty())
                deadline = time.monotonic()+40
                while time.monotonic() < deadline:
                    spin(0.1)
                    if statuses and json.loads(statuses[-1].data)['state'] == 'LOCALIZING':
                        break
                assert statuses and json.loads(statuses[-1].data)['state'] == 'LOCALIZING', (
                    'map freeze failed: '+str([x.data for x in statuses[-3:]]))
                archive_dir = Path('/verify/maps-'+label)
                name = (archive_dir/'active_map').read_text().strip()
                archived_database = archive_dir/(name+'.db')
                assert archived_database.stat().st_size > 4096
                assert parameter('/rtabmap/rtabmap', 'Mem/IncrementalMemory').string_value.lower() == 'false'
                process.send_signal(signal.SIGINT)
                assert process.wait(timeout=15) == 0
                process = subprocess.Popen([
                    'ros2', 'launch', '/opt/amr/rtabmap_rgbd.launch.py',
                    'enable_vo:=false', 'database_path:='+str(archived_database),
                    'map_archive_dir:='+str(archive_dir), 'map_mode:=localizing',
                    'map_name:='+name, 'rtabmap_args:=--Mem/IncrementalMemory false'],
                    stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
                map_client = probe.create_client(GetMap, '/rtabmap/rtabmap/get_map')
                assert map_client.wait_for_service(timeout_sec=15), 'saved map did not reload'
                assert parameter('/rtabmap/rtabmap', 'Mem/IncrementalMemory').string_value.lower() == 'false'
                future = map_client.call_async(GetMap.Request())
                rclpy.spin_until_future_complete(probe, future, timeout_sec=10)
                assert future.result() is not None and future.result().map.info.width > 0, (
                    'saved database did not restore the occupancy map')
                print('Synthetic RGB-D + external odometry produced a map with VO absent')
            else:
                assert not parameter('/vo/rgbd_odometry', 'publish_tf').bool_value
                assert parameter('/vo/rgbd_odometry', 'max_update_rate').double_value == 5.0
                odom_pub = probe.create_publisher(Odometry, '/vo/odom_raw', 10)
                info_pub = probe.create_publisher(OdomInfo, '/vo/odom_info', 10)
                outputs, statuses, transforms = [], [], []
                probe.create_subscription(TwistWithCovarianceStamped, '/vo/twist', outputs.append, 10)
                probe.create_subscription(String, '/vo/status', statuses.append, 10)
                probe.create_subscription(TFMessage, '/tf', transforms.append, 10)
                spin(0.4)

                def sample(index, lost=False):
                    odom, info = Odometry(), OdomInfo()
                    odom.header.stamp = probe.get_clock().now().to_msg()
                    odom.header.frame_id, odom.child_frame_id = 'vo_odom', 'base_footprint'
                    odom.pose.pose.orientation.w = 1.0
                    odom.pose.pose.position.x = index*0.02
                    odom.twist.twist.linear.x = 0.1
                    for axis in range(6):
                        odom.twist.covariance[axis*7] = 0.001
                    info.header, info.lost = odom.header, lost
                    info.inliers, info.matches, info.interval = 50, 80, 0.2
                    info.distance_travelled = index*0.02
                    odom_pub.publish(odom)
                    info_pub.publish(info)
                    spin(0.2)

                for index in range(7):
                    sample(index)
                assert outputs, 'good VO did not pass gate'
                assert all(x.header.frame_id == 'base_footprint' for x in outputs)
                assert all(abs(x.twist.covariance[0]-0.04) < 1e-9 for x in outputs)
                assert all(x.twist.twist.angular.z == 0.0 for x in outputs)
                count = len(outputs)
                sample(7, lost=True)
                spin(0.5)
                assert len(outputs) == count, 'lost VO must not publish fake zero twists'
                assert json.loads(statuses[-1].data)['state'] == 'LOST'
                for index in range(8, 12):
                    sample(index)
                assert len(outputs) == count, 'recovery admitted too early'
                sample(12)
                assert len(outputs) > count, 'VO did not recover'
                # RTAB-Map creates its broadcaster endpoint unconditionally;
                # publish_tf=false prevents sends, not endpoint creation.
                assert not any(t.header.frame_id == 'vo_odom' for msg in transforms
                               for t in msg.transforms)
            print('RTAB-Map '+label+' passed: shared RGB-D, mapper independent of VO'
                  + (', quality/loss/recovery gate verified' if enabled else ''))
        finally:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=5)
            probe.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
