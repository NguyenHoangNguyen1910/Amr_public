import unittest

from nav_msgs.msg import OccupancyGrid

from custom_navigation.config import DEFAULT_CONFIG
from custom_navigation.ros_adapter import occupancy_grid_from_ros


class CustomNavigationMapTest(unittest.TestCase):
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


if __name__ == "__main__":
    unittest.main()
