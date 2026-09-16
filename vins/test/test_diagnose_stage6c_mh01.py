#!/usr/bin/env python3

import importlib.util
import math
import sys
import unittest
from pathlib import Path

import numpy as np


SCRIPT = (Path(__file__).resolve().parents[1] / "scripts" /
          "diagnose_stage6c_mh01.py")
sys.path.insert(0, str(SCRIPT.parent))
SPEC = importlib.util.spec_from_file_location("diagnose_stage6c_mh01", SCRIPT)
DIAGNOSE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(DIAGNOSE)


class DiagnoseStage6cMh01Test(unittest.TestCase):

    def test_pitch_error_sign_uses_established_rpy_convention(self):
        angle = math.radians(10.0)
        rotation = np.asarray([[math.cos(angle), 0.0, math.sin(angle)],
                               [0.0, 1.0, 0.0],
                               [-math.sin(angle), 0.0, math.cos(angle)]])
        rpy = DIAGNOSE.rotation_error_rpy(rotation)
        self.assertAlmostEqual(math.degrees(rpy[1]), 10.0)

    def test_trailing_rmse_and_sustained_onset_use_inclusive_boundaries(self):
        times = np.asarray([0.0, 0.5, 1.0, 1.5, 2.0, 2.5])
        values = np.asarray([1.0, 1.0, 1.0, 2.0, 2.0, 2.0])
        result = DIAGNOSE.trailing_rmse(times, values, 1.0)
        self.assertTrue(math.isnan(result[1]))
        self.assertAlmostEqual(result[2], 1.0)

        condition = np.asarray([False, False, True, True, True, False])
        self.assertEqual(
            DIAGNOSE.find_sustained_onset(times, condition, 1.0), 2)
        condition[4] = False
        self.assertIsNone(
            DIAGNOSE.find_sustained_onset(times, condition, 1.0))

    def test_gate_segments_count_frames_and_duration(self):
        times = np.arange(6) * 0.05
        segments = DIAGNOSE.gate_segments(
            times, [True, True, False, True, True, True])
        self.assertEqual(len(segments), 2)
        self.assertEqual(segments[0]["frames"], 2)
        self.assertAlmostEqual(segments[0]["duration_s"], 0.1)
        self.assertEqual(segments[1]["frames"], 3)
        self.assertAlmostEqual(segments[1]["duration_s"], 0.15)

    def test_post_optimization_velocity_residual_uses_body_frame(self):
        rotation = np.asarray([[0.0, -1.0, 0.0],
                               [1.0, 0.0, 0.0],
                               [0.0, 0.0, 1.0]])
        residual = DIAGNOSE.body_velocity_residual(
            rotation, np.asarray([0.0, 1.0, 0.0]),
            np.asarray([0.75, 0.0, 0.0]))
        np.testing.assert_allclose(residual, [0.25, 0.0, 0.0])

    def test_imu_aggregation_uses_each_camera_interval(self):
        camera = np.asarray([100, 200], dtype=np.int64)
        imu = np.asarray([50, 100, 150, 200], dtype=np.int64)
        acceleration = np.asarray([[0.0, 0.0, 9.81],
                                   [0.0, 0.0, 9.81],
                                   [0.0, 0.0, 10.81],
                                   [0.0, 0.0, 10.81]])
        angular_velocity = np.asarray([[1.0, 0.0, 0.0],
                                       [1.0, 0.0, 0.0],
                                       [2.0, 0.0, 0.0],
                                       [2.0, 0.0, 0.0]])
        result = DIAGNOSE.aggregate_imu(
            camera, imu, acceleration, angular_velocity)
        self.assertEqual(result[0]["imu_sample_count"], 2)
        self.assertEqual(result[1]["imu_sample_count"], 2)
        self.assertAlmostEqual(result[0]["omega_norm_rms"], 1.0)
        self.assertAlmostEqual(result[1]["omega_norm_rms"], 2.0)
        self.assertAlmostEqual(result[1]["accel_dynamic_mean"], 1.0)

    def test_imu_aggregation_fails_closed_on_empty_interval(self):
        with self.assertRaisesRegex(RuntimeError, "no IMU samples"):
            DIAGNOSE.aggregate_imu(
                np.asarray([100, 200]), np.asarray([100]),
                np.asarray([[0.0, 0.0, 9.81]]),
                np.asarray([[0.0, 0.0, 0.0]]))


if __name__ == "__main__":
    unittest.main()
