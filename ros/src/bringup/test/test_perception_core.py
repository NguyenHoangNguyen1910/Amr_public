import math
import unittest

import numpy as np

from amr_perception.core import (
    DepthConfig, MotionConfig, VOGate, camera_intrinsics, decode_depth, depth_scan, fresh, supervise_motion,
)


# Optical x right, y down, z forward -> base x forward, y left, z up.
OPTICAL_TO_BASE = (-0.5, 0.5, -0.5, 0.5)


class DepthTests(unittest.TestCase):
    def test_astra_registered_depth_nan_k_uses_valid_projection(self):
        k = [float('nan')]*9
        p = [570., 0., 319.5, 0., 0., 570., 239.5, 0., 0., 0., 1., 0.]
        self.assertEqual(camera_intrinsics(k, p), (570., 570., 319.5, 239.5))

    def test_invalid_camera_models_not_guessed(self):
        with self.assertRaises(ValueError):
            camera_intrinsics([float('nan')]*9, [0.]*12)
        k = [80., 0., 40., 0., 80., 40., 0., 0., 1.]
        self.assertEqual(camera_intrinsics(k, [0.]*12), (80., 80., 40., 40.))

    def test_uint16_padding_big_endian_and_float_metres(self):
        raw = b'\x03\xe8\x07\xd0xx\x0b\xb8\x0f\xa0xx'
        np.testing.assert_array_equal(decode_depth(raw, 2, 2, 6, '16UC1', True),
                                      [[1, 2], [3, 4]])
        image = np.array([[1.25, float('nan')]], dtype='<f4')
        result = decode_depth(image.tobytes(), 2, 1, 8, '32FC1')
        self.assertEqual(result[0, 0], 1.25)
        self.assertTrue(math.isnan(result[0, 1]))

    def test_malformed_buffer_and_encoding_rejected(self):
        for args in ((b'xx', 2, 1, 2, '16UC1'), (b'xx', 1, 1, 2, 'rgb8')):
            with self.assertRaises(ValueError):
                decode_depth(*args)

    def project(self, image, height=0.3, config=None):
        return depth_scan(image, (20, 20, 10, 10), (0, 0, height),
                          OPTICAL_TO_BASE, config or DepthConfig(stride=1, min_samples=1))

    def test_wall_projects_front_not_rear_and_unknown_is_nan(self):
        obstacles, clear, fraction = self.project(np.ones((21, 21)))
        self.assertEqual(fraction, 1.0)
        self.assertAlmostEqual(obstacles[90], 1.0)
        self.assertTrue(math.isnan(obstacles[0]))
        self.assertTrue(math.isnan(clear[0]))

    def test_empty_depth_never_becomes_free_space(self):
        obstacles, clear, fraction = self.project(np.zeros((21, 21)))
        self.assertEqual(fraction, 0.0)
        self.assertTrue(np.isnan(obstacles).all())
        self.assertTrue(np.isnan(clear).all())

    def test_floor_removed_but_free_distance_is_bounded(self):
        image = np.full((21, 21), np.nan)
        image[16:, :] = 1.0  # These rays hit at/below the floor.
        obstacles, clear, _ = self.project(image)
        self.assertFalse(np.isfinite(obstacles).any())
        # Sparse floor-only returns cannot certify an otherwise missing column.
        self.assertTrue(np.isnan(clear).all())

    def test_sparse_obstacle_preserved_without_certifying_free_space(self):
        image = np.full((21, 21), np.nan)
        image[10, 10] = 0.5
        obstacles, clear, _ = self.project(image)
        self.assertAlmostEqual(obstacles[90], 0.5)
        self.assertTrue(math.isnan(clear[90]))

    def test_observed_floor_is_not_unlimited_free_space(self):
        obstacles, clear, _ = depth_scan(
            np.ones((1, 21)), (20, 20, 10, 0), (0, 0, 0), OPTICAL_TO_BASE,
            DepthConfig(stride=1, min_samples=1))
        self.assertTrue(math.isinf(obstacles[90]))
        self.assertAlmostEqual(clear[90], 1.0)

    def test_full_rotation_and_translation_are_applied(self):
        image = np.ones((21, 21))
        _, first, _ = self.project(image)
        _, translated, _ = depth_scan(image, (20, 20, 10, 10), (0.5, 0, 0.3),
                                     OPTICAL_TO_BASE, DepthConfig(stride=1, min_samples=1))
        self.assertAlmostEqual(first[90], 1.0)
        self.assertAlmostEqual(translated[90], 1.5)

    def test_invalid_intrinsics_and_rotation_rejected(self):
        with self.assertRaises(ValueError):
            depth_scan(np.ones((2, 2)), (0, 1, 0, 0), (0, 0, 0), (0, 0, 0, 1))
        with self.assertRaises(ValueError):
            depth_scan(np.ones((2, 2)), (1, 1, 0, 0), (0, 0, 0), (0, 0, 0, 0))


class VOTests(unittest.TestCase):
    def setUp(self):
        self.gate = VOGate(recovery_samples=3)

    def sample(self, stamp, **overrides):
        values = dict(stamp=stamp, now=stamp+0.01, velocity=(0.1, 0, 0),
                      covariance=(np.eye(6)*0.001).ravel().tolist(),
                      position=(stamp*0.1, 0, 0), yaw=0.0, lost=False,
                      inliers=50, matches=80, interval=0.2, distance_travelled=stamp*0.1)
        values.update(overrides)
        return self.gate.admit(**values)

    def test_requires_consecutive_good_samples_and_floors_covariance(self):
        self.assertIsNone(self.sample(10))
        self.assertIsNone(self.sample(10.2))
        result = self.sample(10.4)
        self.assertEqual(self.gate.state, 'VALID')
        self.assertAlmostEqual(result[0], 0.04)
        self.assertEqual(result[35], 1e6)

    def test_loss_publishes_no_fake_zero_and_requires_recovery(self):
        for t in (10, 10.2, 10.4):
            self.sample(t)
        self.assertIsNone(self.sample(10.6, lost=True))
        self.assertEqual(self.gate.state, 'LOST')
        self.assertIsNone(self.sample(10.8))
        self.assertIsNone(self.sample(11.0))
        self.assertIsNotNone(self.sample(11.2))

    def test_stale_future_and_bad_quality_rejected(self):
        for override in (dict(now=12), dict(now=9), dict(inliers=2), dict(matches=500)):
            self.assertIsNone(self.sample(10, **override))
            self.assertEqual(self.gate.state, 'LOST')

    def test_reset_jump_and_distance_counter_reset_rejected(self):
        self.sample(10)
        self.assertIsNone(self.sample(10.2, position=(0, 0, 0)))
        self.assertEqual(self.gate.state, 'LOST')
        self.sample(11)
        self.assertIsNone(self.sample(11.2, distance_travelled=0))
        self.assertEqual(self.gate.state, 'LOST')

    def test_timestamp_duplicate_and_gap_rejected(self):
        self.sample(10)
        self.sample(10)
        self.assertEqual(self.gate.state, 'LOST')
        self.sample(11)
        self.sample(12)
        self.assertEqual(self.gate.state, 'LOST')

    def test_invalid_covariance_and_speed_rejected(self):
        for covariance in ([0.0]*36, [float('nan')]*36, (np.eye(6)*10).ravel().tolist()):
            self.sample(10, covariance=covariance)
            self.assertEqual(self.gate.state, 'LOST')
        self.sample(10, velocity=(4, 0, 0))
        self.assertEqual(self.gate.state, 'LOST')


class MonitorMathTests(unittest.TestCase):
    def setUp(self):
        self.config = MotionConfig(front=0.15, rear=0.15, half_width=0.1,
                                   margin=0.02, calibrated=True)
        self.obstacles = np.full(361, np.inf)
        self.clearance = np.full(361, 3.0)

    def check(self, v=0.1, w=0, **kwargs):
        args = dict(linear=v, angular=w, measured_linear=0.0, measured_angular=0.0,
                    obstacles=self.obstacles, clearance=self.clearance,
                    angle_min=-math.pi, angle_increment=math.pi/180,
                    range_min=0.01, config=self.config)
        args.update(kwargs)
        return supervise_motion(**args)

    def test_observed_corridor_passes_and_obstacle_warns(self):
        self.assertEqual(self.check()[2], 'clear')
        self.obstacles[180] = 0.20
        self.assertEqual(self.check()[2], 'obstacle in stopping envelope')

    def test_unknown_and_blind_zone_are_reported(self):
        self.clearance[:] = np.nan
        self.assertIn('unknown', self.check()[2])
        self.clearance[:] = 3
        self.assertIn('blind zone', self.check(range_min=0.3)[2])

    def test_rear_unobserved_rotation_reported(self):
        self.clearance[:150] = np.nan
        self.clearance[210:] = np.nan
        self.assertNotEqual(self.check(v=0, w=0.2)[2], 'clear')
        self.assertIn('reverse', self.check(v=-0.1)[2])

    def test_nonfinite_and_uncalibrated_are_reported(self):
        self.assertIn('non-finite', self.check(v=float('nan'))[2])
        self.assertIn('calibration', self.check(config=MotionConfig())[2])

    def test_receipt_time_cannot_hide_old_source_timestamp(self):
        self.assertFalse(fresh(10, 20, 0.6))
        self.assertFalse(fresh(0, 20, 0.6))
        self.assertFalse(fresh(21, 20, 0.6))
        self.assertTrue(fresh(19.8, 20, 0.6))


if __name__ == '__main__':
    unittest.main()
