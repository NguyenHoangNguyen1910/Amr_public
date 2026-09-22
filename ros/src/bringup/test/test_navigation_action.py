import unittest

import rclpy
from bringup.action import NavigateToPose

from navigation.ros_adapter import NavigationNode


class NavigationActionTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        if rclpy.ok():
            rclpy.shutdown()

    def test_node_exposes_existing_navigate_to_pose_contract(self):
        node = NavigationNode()
        try:
            self.assertEqual(node.get_name(), "navigation")
            self.assertEqual(
                NavigateToPose.Goal.get_fields_and_field_types()["pose"],
                "geometry_msgs/PoseStamped",
            )
            self.assertIsNotNone(node._action_server)
        finally:
            node.destroy_node()


if __name__ == "__main__":
    unittest.main()
