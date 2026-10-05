import unittest
import threading
from types import SimpleNamespace
from unittest.mock import Mock

from nav_msgs.msg import OccupancyGrid

from navigation.config import DEFAULT_CONFIG
from navigation.ros_adapter import NavigationNode, occupancy_grid_from_ros


class NavigationMapTest(unittest.TestCase):
    def make_map(self, values, width=3, height=2):
        message = OccupancyGrid()
        message.header.frame_id = "map"
        message.info.width = width
        message.info.height = height
        message.info.resolution = 0.05
        message.info.origin.position.x = -1.0
        message.info.origin.position.y = 2.0
        message.info.origin.orientation.w = 1.0
        message.data = values
        return message

    def test_unknown_and_high_probability_cells_are_blocked(self):
        grid = occupancy_grid_from_ros(
            self.make_map([-1, 0, 64, 65, 100, 20]),
            DEFAULT_CONFIG,
            occupied_threshold=65,
        )

        self.assertEqual(grid.raw.shape, (2, 3))
        self.assertTrue(bool(grid.raw[0, 0]))
        self.assertFalse(bool(grid.raw[0, 2]))
        self.assertTrue(bool(grid.raw[1, 0]))
        self.assertEqual(grid.min_x, -1.0)
        self.assertEqual(grid.min_y, 2.0)

    def test_invalid_map_length_is_rejected(self):
        with self.assertRaises(ValueError):
            occupancy_grid_from_ros(
                self.make_map([0, 0, 0]),
                DEFAULT_CONFIG,
                occupied_threshold=65,
            )

    def test_rotated_origin_is_rejected(self):
        message = self.make_map([0, 0, 0, 0, 0, 0])
        message.info.origin.orientation.z = 0.70710678
        message.info.origin.orientation.w = 0.70710678
        with self.assertRaises(ValueError):
            occupancy_grid_from_ros(message, DEFAULT_CONFIG, occupied_threshold=65)

    def test_identical_map_republication_keeps_snapshot_but_changes_replace_it(self):
        node = SimpleNamespace(_mission_lock=threading.Lock(), _latest_map=None,
                               map_frame="map", get_logger=Mock())
        original = self.make_map([0] * 6)
        NavigationNode._map_callback(node, original)
        repeated = self.make_map([0] * 6)
        repeated.header.stamp.sec = 2
        NavigationNode._map_callback(node, repeated)
        self.assertIs(node._latest_map, original)
        changed = self.make_map([0, 0, 100, 0, 0, 0])
        NavigationNode._map_callback(node, changed)
        self.assertIs(node._latest_map, changed)

    def test_wrong_map_frame_does_not_replace_valid_map(self):
        original = self.make_map([0] * 6)
        node = SimpleNamespace(_mission_lock=threading.Lock(), _latest_map=original,
                               map_frame="map", get_logger=Mock())
        invalid = self.make_map([0] * 6)
        invalid.header.frame_id = "odom"
        NavigationNode._map_callback(node, invalid)
        self.assertIs(node._latest_map, original)


if __name__ == "__main__":
    unittest.main()
