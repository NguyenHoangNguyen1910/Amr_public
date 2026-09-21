import unittest

import rclpy
from nav2_msgs.action import NavigateToPose

from custom_navigation.ros_adapter import CustomNavigationNode


class CustomNavigationActionTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        if rclpy.ok():
            rclpy.shutdown()

    def test_node_exposes_existing_navigate_to_pose_contract(self):
        node = CustomNavigationNode()
        try:
            self.assertEqual(node.get_name(), "custom_navigation")
            self.assertEqual(
                NavigateToPose.Goal.get_fields_and_field_types()["pose"],
                "geometry_msgs/PoseStamped",
            )
            self.assertIsNotNone(node._action_server)
        finally:
            node.destroy_node()


if __name__ == "__main__":
    unittest.main()
