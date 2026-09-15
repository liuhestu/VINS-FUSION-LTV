#!/usr/bin/env python3

import importlib.util
import os
import tempfile
import unittest

import numpy as np


SCRIPT_PATH = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "scripts", "analyze_euroc_asl_velocity.py")
SPEC = importlib.util.spec_from_file_location("analyze_euroc_asl_velocity", SCRIPT_PATH)
ANALYZER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(ANALYZER)


class AnalyzeEurocAslVelocityTest(unittest.TestCase):

    def _write_constant_velocity_csv(self, rotation=(1.0, 0.0, 0.0, 0.0)):
        stream = tempfile.NamedTemporaryFile(mode="w", suffix=".csv", delete=False)
        stream.write(
            "#timestamp, p_RS_R_x [m], p_RS_R_y [m], p_RS_R_z [m], "
            "q_RS_w [], q_RS_x [], q_RS_y [], q_RS_z [], "
            "v_RS_R_x [m s^-1], v_RS_R_y [m s^-1], v_RS_R_z [m s^-1]\n")
        velocity = np.array([2.0, -1.0, 0.5])
        for index in range(21):
            timestamp = index * 100000000
            position = velocity * (index * 0.1)
            stream.write(
                f"{timestamp},{position[0]},{position[1]},{position[2]},"
                f"{rotation[0]},{rotation[1]},{rotation[2]},{rotation[3]},"
                f"{velocity[0]},{velocity[1]},{velocity[2]}\n")
        stream.close()
        return stream.name

    def test_constant_velocity_statistics_and_position_check(self):
        path = self._write_constant_velocity_csv()
        try:
            summary, series = ANALYZER.analyze_ground_truth(path, 0.1)
        finally:
            os.unlink(path)

        self.assertEqual(summary["samples"], 21)
        self.assertTrue(summary["timestamp"]["strictly_increasing"])
        self.assertEqual(summary["position_difference_check"]["valid_samples"], 19)
        self.assertAlmostEqual(summary["world_velocity_mps"]["norm"]["mean"], 2.291287847, places=8)
        self.assertAlmostEqual(
            summary["position_difference_check"]
            ["official_minus_position_difference_error_mps"]["rms"], 0.0, places=10)
        np.testing.assert_allclose(series["velocity_body_mps"],
                                   np.tile([2.0, -1.0, 0.5], (21, 1)))
        np.testing.assert_allclose(series["acceleration_world_mps2"][1:], 0.0,
                                   atol=1e-12)

    def test_body_velocity_uses_transpose_of_world_body_rotation(self):
        # q_RS rotates body x onto world y. Therefore world y maps back to
        # body x under R_RS^T.
        path = self._write_constant_velocity_csv(
            rotation=(np.sqrt(0.5), 0.0, 0.0, np.sqrt(0.5)))
        try:
            _, series = ANALYZER.analyze_ground_truth(path, 0.1)
        finally:
            os.unlink(path)

        expected = np.tile([-1.0, -2.0, 0.5], (21, 1))
        np.testing.assert_allclose(series["velocity_body_mps"], expected, atol=1e-12)

    def test_time_series_csv_contains_official_and_body_velocity(self):
        path = self._write_constant_velocity_csv()
        output = tempfile.NamedTemporaryFile(suffix=".csv", delete=False).name
        try:
            _, series = ANALYZER.analyze_ground_truth(path, 0.1)
            ANALYZER.write_time_series(output, series)
            with open(output, encoding="utf-8") as stream:
                lines = stream.readlines()
        finally:
            os.unlink(path)
            os.unlink(output)

        self.assertEqual(len(lines), 22)
        self.assertIn("velocity_world_x_mps", lines[0])
        self.assertIn("velocity_body_x_mps", lines[0])


if __name__ == "__main__":
    unittest.main()
