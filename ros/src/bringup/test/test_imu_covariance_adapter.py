import unittest

from imu_covariance_adapter import covariance_or_default


class ImuCovarianceTest(unittest.TestCase):
    def test_unknown_covariance_gets_explicit_variance(self):
        self.assertEqual(covariance_or_default([0.0] * 9, 0.02),
                         [0.0004, 0, 0, 0, 0.0004, 0, 0, 0, 0.0004])

    def test_known_covariance_and_unavailable_sentinel_are_preserved(self):
        for covariance in ([0.01, 0.001, 0, 0.001, 0.02, 0, 0, 0, 0.03], [-1.0] + [0.0] * 8):
            self.assertIs(covariance_or_default(covariance, 0.02), covariance)

    def test_invalid_uncertainty_is_rejected(self):
        for value in (0.0, -1.0, float('nan'), float('inf')):
            with self.assertRaises(ValueError):
                covariance_or_default([0.0] * 9, value)
