import math
from pathlib import Path
import tempfile
import unittest

from simulation.core import DifferentialDriveSimulator, GridMap, Pose2D, load_map


class SimulationCoreTest(unittest.TestCase):
    def test_robot_moves_and_rotates_in_free_space(self):
        grid = GridMap(20, 20, 0.1, -1.0, -1.0, 0.0, (0,) * 400)
        simulator = DifferentialDriveSimulator(grid, Pose2D(0.0, 0.0, 0.0), 0.05)

        pose = simulator.step(0.2, math.pi / 2.0, 1.0)

        self.assertAlmostEqual(pose.x, math.sqrt(0.02), places=5)
        self.assertAlmostEqual(pose.y, math.sqrt(0.02), places=5)
        self.assertAlmostEqual(pose.yaw, math.pi / 2.0, places=5)

    def test_collision_stops_translation_but_allows_rotation(self):
        data = [0] * 100
        data[6 * 10 + 6] = 100
        grid = GridMap(10, 10, 0.1, 0.0, 0.0, 0.0, tuple(data))
        simulator = DifferentialDriveSimulator(grid, Pose2D(0.45, 0.55, 0.0), 0.04)

        pose = simulator.step(0.2, 1.0, 1.0)

        self.assertEqual((pose.x, pose.y), (0.45, 0.55))
        self.assertAlmostEqual(pose.yaw, 1.0)
        self.assertEqual(simulator.linear_velocity, 0.0)

    def test_raycast_hits_occupied_cell(self):
        data = [0] * 100
        data[5 * 10 + 7] = 100
        grid = GridMap(10, 10, 0.1, 0.0, 0.0, 0.0, tuple(data))

        distance = grid.raycast(0.25, 0.55, 0.0, 0.0, 1.0)

        self.assertAlmostEqual(distance, 0.45, delta=0.03)

    def test_loads_ros_pgm_map_and_flips_image_rows(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "map.pgm").write_text("P2\n2 2\n255\n0 255\n255 0\n")
            (root / "map.yaml").write_text(
                "image: map.pgm\nresolution: 0.5\norigin: [-1, -2, 0]\n"
                "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.2\n"
            )

            grid = load_map(root / "map.yaml")

        self.assertEqual((grid.width, grid.height), (2, 2))
        self.assertEqual(grid.data, (0, 100, 100, 0))
        self.assertEqual(grid.world_to_grid(-0.75, -1.75), (0, 0))

    def test_loads_binary_pgm_map(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "map.pgm").write_bytes(b"P5\n# map saver\n2 1\n255\n" + bytes((0, 255)))
            (root / "map.yaml").write_text(
                "image: map.pgm\nresolution: 1.0\norigin: [0, 0, 0]\n"
                "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.2\n"
            )

            grid = load_map(root / "map.yaml")

        self.assertEqual(grid.data, (100, 0))


if __name__ == "__main__":
    unittest.main()
