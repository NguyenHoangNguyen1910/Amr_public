import unittest

import rclpy

from navigation.ros_adapter import NavigationNode


class NavigationSafetyTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        if rclpy.ok():
            rclpy.shutdown()

    def test_velocity_is_finite_and_clamped_to_robot_limits(self):
        node = NavigationNode()
        try:
            command = node.publish_velocity(-100.0, -100.0)
            self.assertEqual(command["linear"], -0.35)
            self.assertEqual(command["angular"], -0.40)
        finally:
            node.destroy_node()


if __name__ == "__main__":
    unittest.main()
