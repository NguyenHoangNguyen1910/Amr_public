#!/usr/bin/env python3
"""Archive a completed RTAB-Map session and keep localizing against it."""

import argparse
import json
import os
import shutil
import sqlite3
import time
from datetime import datetime, timezone
from pathlib import Path

import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Empty as EmptyMessage, String
from std_srvs.srv import Empty as EmptyService


def checked_database(path):
    """Reject an empty or corrupt archive before selecting it for future runs."""
    if path.stat().st_size < 4096:
        raise ValueError("RTAB-Map database is empty")
    connection = sqlite3.connect(f"file:{path}?mode=ro", uri=True)
    try:
        result = connection.execute("PRAGMA quick_check").fetchone()
        if result != ("ok",):
            raise ValueError(f"RTAB-Map database integrity check failed: {result}")
    finally:
        connection.close()


def archive_database(source, archive_dir):
    """Copy a completed RTAB-Map backup and atomically publish the archive."""
    archive_dir.mkdir(parents=True, exist_ok=True)
    name = datetime.now(timezone.utc).strftime("map-%Y%m%d-%H%M%S-%f")
    target = archive_dir / f"{name}.db"
    temporary = archive_dir / f".{name}.db.tmp"
    try:
        with source.open("rb") as input_file, temporary.open("xb") as output_file:
            shutil.copyfileobj(input_file, output_file, 1024 * 1024)
            output_file.flush()
            os.fsync(output_file.fileno())
        checked_database(temporary)
        os.replace(temporary, target)
        directory_fd = os.open(archive_dir, os.O_DIRECTORY)
        try:
            os.fsync(directory_fd)
        finally:
            os.close(directory_fd)
    finally:
        temporary.unlink(missing_ok=True)
    return name, target


def select_active_map(archive_dir, name):
    temporary = archive_dir / ".active_map.tmp"
    with temporary.open("w", encoding="ascii") as output_file:
        output_file.write(name + "\n")
        output_file.flush()
        os.fsync(output_file.fileno())
    os.replace(temporary, archive_dir / "active_map")
    directory_fd = os.open(archive_dir, os.O_DIRECTORY)
    try:
        os.fsync(directory_fd)
    finally:
        os.close(directory_fd)


class MapLifecycle(Node):
    def __init__(self, database_path, archive_dir, initial_mode, map_name):
        super().__init__("map_lifecycle")
        self.database_path = Path(database_path)
        self.archive_dir = Path(archive_dir)
        self.mode = initial_mode
        self.busy = False
        self.latest_map = None
        self.latest_odom = None
        self.latest_odom_time = 0.0
        self.archive_name = map_name or None
        self.backup_started = 0
        self.status_message = "Map đang được xây dựng" if initial_mode == "mapping" else "Đang định vị trên map đã lưu"

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_publisher = self.create_publisher(String, "/web/map_mode", qos)
        self.create_subscription(OccupancyGrid, "/rtabmap/map", self.map_callback, qos)
        self.create_subscription(Odometry, "/odom", self.odom_callback, 10)
        self.create_subscription(EmptyMessage, "/web/freeze_map", self.freeze_callback, 10)
        self.backup_client = self.create_client(EmptyService, "/rtabmap/rtabmap/backup")
        self.localization_client = self.create_client(
            EmptyService, "/rtabmap/rtabmap/set_mode_localization")
        self.create_timer(1.0, self.publish_status)
        self.publish_status()

    def publish_status(self):
        message = String()
        message.data = json.dumps({"state": "BUSY" if self.busy else self.mode.upper(),
                                   "message": self.status_message,
                                   "map_name": self.archive_name}, ensure_ascii=False)
        self.status_publisher.publish(message)

    def map_callback(self, message):
        if message.info.width and message.info.height and any(value >= 0 for value in message.data):
            self.latest_map = message

    def odom_callback(self, message):
        self.latest_odom = message
        self.latest_odom_time = time.monotonic()

    def fail(self, reason):
        self.busy = False
        self.status_message = reason
        self.get_logger().error(reason)
        self.publish_status()

    def freeze_callback(self, _request):
        if self.busy or self.mode == "localizing":
            return
        if self.latest_map is None:
            self.fail("Chưa có map hợp lệ; tiếp tục quét rồi thử lại")
            return
        if self.latest_odom is None or time.monotonic() - self.latest_odom_time > 0.6:
            self.fail("Odometry chưa sẵn sàng để chốt map")
            return
        velocity = self.latest_odom.twist.twist
        if abs(velocity.linear.x) > 0.02 or abs(velocity.angular.z) > 0.03:
            self.fail("Robot phải đứng yên trước khi chốt map")
            return
        if not self.backup_client.service_is_ready() or not self.localization_client.service_is_ready():
            self.fail("Dịch vụ RTAB-Map chưa sẵn sàng")
            return
        self.busy = True
        self.status_message = "Đang lưu map vào bộ nhớ bền vững"
        self.publish_status()
        self.backup_started = time.time_ns()
        future = self.backup_client.call_async(EmptyService.Request())
        future.add_done_callback(self.backup_done)

    def backup_done(self, future):
        try:
            future.result()
            backup = Path(str(self.database_path) + ".back")
            if not backup.exists() or backup.stat().st_mtime_ns < self.backup_started:
                raise RuntimeError("RTAB-Map không tạo bản sao database mới")
            name, target = archive_database(backup, self.archive_dir)
            backup.unlink()
            self.archive_name = name
            self.get_logger().info(f"Map archived at {target}")
            self.status_message = "Đã lưu map; đang chuyển sang định vị"
            self.publish_status()
            mode_future = self.localization_client.call_async(EmptyService.Request())
            mode_future.add_done_callback(self.localization_done)
        except Exception as error:
            self.fail(f"Lưu map thất bại: {error}")

    def localization_done(self, future):
        try:
            future.result()
            select_active_map(self.archive_dir, self.archive_name)
            self.mode = "localizing"
            self.busy = False
            self.status_message = "Đã dừng xây dựng map; localization và Nav vẫn hoạt động"
            self.get_logger().info(self.status_message)
            self.publish_status()
        except Exception as error:
            # The archive remains available even if the active-map pointer fails.
            self.fail(f"Chuyển sang localization chưa hoàn tất: {error}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--database", required=True)
    parser.add_argument("--archive-dir", required=True)
    parser.add_argument("--mode", choices=("mapping", "localizing"), default="mapping")
    parser.add_argument("--map-name", default="")
    args = parser.parse_args()
    rclpy.init()
    node = MapLifecycle(args.database, args.archive_dir, args.mode, args.map_name)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
