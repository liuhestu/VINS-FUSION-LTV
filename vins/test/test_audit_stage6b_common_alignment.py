#!/usr/bin/env python3

import importlib.util
import math
import sys
import unittest
from pathlib import Path

import numpy as np


SCRIPT = (Path(__file__).resolve().parents[1] / "scripts" /
          "audit_stage6b_common_alignment.py")
sys.path.insert(0, str(SCRIPT.parent))
SPEC = importlib.util.spec_from_file_location(
    "audit_stage6b_common_alignment", SCRIPT)
AUDIT = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(AUDIT)


def rotation_z(degrees):
    angle = math.radians(degrees)
    return np.asarray([[math.cos(angle), -math.sin(angle), 0.0],
                       [math.sin(angle), math.cos(angle), 0.0],
                       [0.0, 0.0, 1.0]])


class AuditStage6bCommonAlignmentTest(unittest.TestCase):

    def test_orientation_metrics_apply_supplied_common_alignment(self):
        rotations = np.tile(rotation_z(-10.0), (3, 1, 1))
        ground_truth = np.tile(np.eye(3), (3, 1, 1))
        indices = np.arange(3)
        metrics = AUDIT.orientation_metrics(
            rotations, indices, ground_truth, indices, rotation_z(10.0))
        self.assertAlmostEqual(metrics["rotation_rmse_deg"], 0.0, places=7)
        self.assertAlmostEqual(metrics["roll_rmse_deg"], 0.0, places=7)
        self.assertAlmostEqual(metrics["pitch_rmse_deg"], 0.0, places=7)

    def test_common_alignment_exposes_candidate_gauge_difference(self):
        ground_truth = np.tile(np.eye(3), (4, 1, 1))
        baseline = np.tile(np.eye(3), (4, 1, 1))
        candidate = np.tile(rotation_z(5.0), (4, 1, 1))
        indices = np.arange(4)
        baseline_metrics = AUDIT.orientation_metrics(
            baseline, indices, ground_truth, indices, np.eye(3))
        candidate_metrics = AUDIT.orientation_metrics(
            candidate, indices, ground_truth, indices, np.eye(3))
        self.assertAlmostEqual(baseline_metrics["rotation_rmse_deg"], 0.0,
                               places=7)
        self.assertAlmostEqual(candidate_metrics["rotation_rmse_deg"], 5.0,
                               places=7)

    def test_alignment_delta_reports_angle_and_rpy(self):
        delta = AUDIT.alignment_delta(np.eye(3), rotation_z(3.0))
        self.assertAlmostEqual(delta["angle_deg"], 3.0, places=7)
        np.testing.assert_allclose(delta["rpy_deg"], [0.0, 0.0, 3.0],
                                   atol=1e-12)

    def test_relative_change_fails_closed_for_zero_or_nonfinite_input(self):
        self.assertAlmostEqual(AUDIT.relative_change_percent(2.0, 2.1), 5.0)
        with self.assertRaises(ValueError):
            AUDIT.relative_change_percent(0.0, 1.0)
        with self.assertRaises(ValueError):
            AUDIT.relative_change_percent(1.0, math.nan)


if __name__ == "__main__":
    unittest.main()
