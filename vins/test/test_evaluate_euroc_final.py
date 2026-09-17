#!/usr/bin/env python3

import importlib.util
import math
import sys
import unittest
from pathlib import Path

import numpy as np


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))
SCRIPT = SCRIPTS / "evaluate_euroc_final.py"
SPEC = importlib.util.spec_from_file_location("evaluate_euroc_final", SCRIPT)
FINAL = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(FINAL)


def rotation_z(degrees):
    angle = math.radians(degrees)
    return np.asarray([[math.cos(angle), -math.sin(angle), 0.0],
                       [math.sin(angle), math.cos(angle), 0.0],
                       [0.0, 0.0, 1.0]])


class EvaluateEurocFinalTest(unittest.TestCase):

    def trajectories_with_times(self, times_by_mode):
        return {
            mode: {"timestamps": np.asarray(times_by_mode[mode])}
            for mode in FINAL.MODES
        }

    def test_one_to_one_matching_is_inclusive_and_does_not_reuse(self):
        indices, errors = FINAL.match_one_to_one(
            np.asarray([1.0, 1.0015]), np.asarray([1.001]), 0.001)
        np.testing.assert_array_equal(indices, [0, -1])
        self.assertAlmostEqual(errors[0], 0.001)
        self.assertTrue(math.isnan(errors[1]))

    def test_common_support_accepts_exactly_99_percent(self):
        times = np.arange(100, dtype=float) * 0.05
        times_by_mode = {mode: times.copy() for mode in FINAL.MODES}
        times_by_mode["joint"] = np.delete(times, 50)
        support = FINAL.common_time_support(
            self.trajectories_with_times(times_by_mode),
            {"timestamps": times.copy()})
        self.assertEqual(support["common_count"], 99)
        self.assertAlmostEqual(support["coverage"]["baseline"], 0.99)

    def test_common_support_rejects_below_99_percent(self):
        times = np.arange(100, dtype=float) * 0.05
        times_by_mode = {mode: times.copy() for mode in FINAL.MODES}
        times_by_mode["joint"] = np.delete(times, [40, 50])
        with self.assertRaisesRegex(RuntimeError, "below 0.99"):
            FINAL.common_time_support(
                self.trajectories_with_times(times_by_mode),
                {"timestamps": times.copy()})

    def test_metrics_use_independent_ate_and_common_baseline_gauge(self):
        gt_positions = np.asarray([
            [0.0, 0.0, 0.0], [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])
        gt_rotations = np.tile(np.eye(3), (4, 1, 1))
        gt_velocities = np.tile([1.0, 0.5, -0.25], (4, 1))
        baseline_gauge = rotation_z(10.0)
        baseline_positions = (baseline_gauge.T @ gt_positions.T).T
        baseline_rotations = np.tile(baseline_gauge.T, (4, 1, 1))
        baseline_velocities = (
            baseline_gauge.T @ gt_velocities.T).T
        trajectories = {}
        for mode in FINAL.MODES:
            positions = baseline_positions.copy()
            if mode != "baseline":
                candidate_gauge = rotation_z(-20.0)
                positions = (candidate_gauge.T @ gt_positions.T).T
            trajectories[mode] = {
                "positions": positions,
                "rotations": baseline_rotations,
                "velocities": baseline_velocities,
            }
        support = {
            "mode_indices": {
                mode: np.arange(4, dtype=int) for mode in FINAL.MODES},
            "gt_indices": np.arange(4, dtype=int),
        }
        metrics, alignment = FINAL.trajectory_metrics(
            trajectories, {
                "positions": gt_positions,
                "rotations": gt_rotations,
                "velocities": gt_velocities,
            }, support)
        np.testing.assert_allclose(alignment, baseline_gauge, atol=1e-12)
        for mode in FINAL.MODES:
            self.assertAlmostEqual(metrics[mode]["ate_rmse_m"], 0.0,
                                   places=12)
            self.assertAlmostEqual(metrics[mode]["rotation_rmse_deg"], 0.0,
                                   places=6)
            self.assertAlmostEqual(metrics[mode]["velocity_rmse_mps"], 0.0,
                                   places=12)

    def test_markdown_contains_only_three_result_tables(self):
        sequence_results = []
        for name, group, scale in (
                ("easy", "easy_medium", 1.0),
                ("hard", "difficult", 2.0)):
            metrics = {}
            for index, mode in enumerate(FINAL.MODES):
                value = scale * (1.0 + 0.1 * index)
                metrics[mode] = {
                    "ate_rmse_m": value,
                    "rotation_rmse_deg": value,
                    "velocity_rmse_mps": value,
                }
            sequence_results.append({
                "sequence": name, "group": group, "metrics": metrics})
        markdown = FINAL.render_markdown({
            "sequence_results": sequence_results})
        self.assertEqual(markdown.count("## Table"), 3)
        self.assertEqual(markdown.count("Easy/Medium Mean"), 3)
        self.assertEqual(markdown.count("Difficult Mean"), 3)
        self.assertIn("+10.00%", markdown)


if __name__ == "__main__":
    unittest.main()
