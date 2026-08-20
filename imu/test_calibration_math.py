#!/usr/bin/env python3

import math
import unittest

from calibration_math import (
    StationaryCalibrator,
    quaternion_conjugate,
    quaternion_multiply,
    rotate_vector,
    shortest_arc_quaternion,
)


class CalibrationMathTests(unittest.TestCase):
    def test_vertical_minus_y_mount_maps_gravity_to_plus_z(self):
        rotation = shortest_arc_quaternion((0.0, -9.80665, 0.0), (0.0, 0.0, 1.0))
        corrected = rotate_vector(rotation, (0.0, -9.80665, 0.0))
        self.assertAlmostEqual(corrected[0], 0.0, places=6)
        self.assertAlmostEqual(corrected[1], 0.0, places=6)
        self.assertAlmostEqual(corrected[2], 9.80665, places=6)

    def test_mount_orientation_becomes_level_base_orientation(self):
        mount = shortest_arc_quaternion((0.0, -1.0, 0.0), (0.0, 0.0, 1.0))
        base_orientation = quaternion_multiply(mount, quaternion_conjugate(mount))
        self.assertAlmostEqual(base_orientation[0], 0.0, places=6)
        self.assertAlmostEqual(base_orientation[1], 0.0, places=6)
        self.assertAlmostEqual(base_orientation[2], 0.0, places=6)
        self.assertAlmostEqual(base_orientation[3], 1.0, places=6)

    def test_stationary_calibration_estimates_bias_and_scale(self):
        calibrator = StationaryCalibrator(
            sample_count=4,
            gravity_mps2=9.80665,
            yaw_rad=0.0,
            max_gyro_rad_s=0.2,
            accel_tolerance_mps2=1.0,
            max_gyro_stddev=0.02,
            max_accel_stddev=0.2,
        )
        for index in range(4):
            status = calibrator.add_sample(
                (0.1, -9.8 + index * 0.001, -0.2),
                (0.01, -0.02, 0.005),
            )
        self.assertEqual(status, "complete")
        self.assertIsNotNone(calibrator.result)
        assert calibrator.result is not None
        self.assertAlmostEqual(calibrator.result.gyro_bias_rad_s[0], 0.01)
        corrected = rotate_vector(
            calibrator.result.base_from_sensor_xyzw,
            tuple(
                component * calibrator.result.accel_scale
                for component in calibrator.result.accel_mean_mps2
            ),
        )
        self.assertAlmostEqual(corrected[0], 0.0, places=6)
        self.assertAlmostEqual(corrected[1], 0.0, places=6)
        self.assertAlmostEqual(corrected[2], 9.80665, places=6)

    def test_motion_resets_collection(self):
        calibrator = StationaryCalibrator(3, 9.80665, 0.0, 0.2, 1.0, 0.02, 0.2)
        self.assertEqual(
            calibrator.add_sample((0.0, -9.80665, 0.0), (0.0, 0.0, 0.0)),
            "collecting",
        )
        self.assertEqual(
            calibrator.add_sample((0.0, -9.80665, 0.0), (0.3, 0.0, 0.0)),
            "rejected",
        )
        self.assertEqual(calibrator.progress, 0)


if __name__ == "__main__":
    unittest.main()
