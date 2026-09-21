import math
import unittest

import rclpy

from custom_navigation.ros_adapter import CustomNavigationNode


class CustomNavigationSafetyTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        if rclpy.ok():
            rclpy.shutdown()

    def test_velocity_is_finite_and_clamped_to_robot_limits(self):
        node = CustomNavigationNode()
        try:
            command = node.publish_velocity(math.inf, -100.0)
            self.assertEqual(command["linear"], 0.0)
            self.assertEqual(command["angular"], -0.55)
        finally:
            node.destroy_node()


if __name__ == "__main__":
    unittest.main()
