#!/usr/bin/env python3

import importlib.util
import os
import sys
import unittest

import numpy as np


SCRIPTS = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                       "scripts")
sys.path.insert(0, SCRIPTS)
PATH = os.path.join(SCRIPTS, "audit_uzhfpv_time_offsets.py")
SPEC = importlib.util.spec_from_file_location("audit_uzhfpv_time_offsets", PATH)
AUDIT = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(AUDIT)


class AuditUzhFpvTimeOffsetsTest(unittest.TestCase):

    def test_family_direct_decision(self):
        status, applied, median, mad = AUDIT.decide_family(
            [0.001, -0.002, 0.003, 0.0])
        self.assertEqual(status, "direct")
        self.assertEqual(applied, 0.0)
        self.assertLess(abs(median), 0.002)
        self.assertLess(mad, 0.005)

    def test_family_uniform_indoor_compensation(self):
        status, applied, _, _ = AUDIT.decide_family(
            [-0.099, -0.097, -0.096, -0.098])
        self.assertEqual(status, "uniform_indoor_compensation")
        self.assertAlmostEqual(applied, -0.0975)

    def test_family_rejects_ambiguous_offset(self):
        status, applied, _, _ = AUDIT.decide_family(
            [-0.030, -0.032, -0.029])
        self.assertEqual(status, "rejected")
        self.assertTrue(np.isnan(applied))

    def test_cross_correlation_sign(self):
        times = np.arange(0.0, 20.0, 0.002)
        signal = np.sin(1.7 * times) + 0.4 * np.sin(4.3 * times)
        # IMU(t) carries the GT motion from t - offset, so GT(t) matches
        # IMU(t + offset).
        offset = -0.097
        imu_signal = np.sin(1.7 * (times - offset)) + 0.4 * np.sin(
            4.3 * (times - offset))
        gt_gyro = np.column_stack((signal, 0.5 * signal, 0.2 * signal))
        imu_gyro = np.column_stack((imu_signal, 0.5 * imu_signal,
                                    0.2 * imu_signal))
        estimate = AUDIT.estimate_offset(
            times, gt_gyro, times, imu_gyro, "norm")
        self.assertAlmostEqual(estimate["offset_s"], offset, delta=0.001)
        self.assertGreater(estimate["correlation"], 0.99)


if __name__ == "__main__":
    unittest.main()
