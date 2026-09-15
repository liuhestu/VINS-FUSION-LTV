#!/usr/bin/env python3

import importlib.util
import os
import tempfile
import unittest

import numpy as np


SCRIPT_PATH = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "scripts", "evaluate_vins_euroc.py")
SPEC = importlib.util.spec_from_file_location("evaluate_vins_euroc", SCRIPT_PATH)
EVALUATOR = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(EVALUATOR)


class EvaluateVinsEurocTest(unittest.TestCase):

    def test_official_csv_reads_velocity_by_header_name(self):
        with tempfile.NamedTemporaryFile(mode="w", suffix=".csv", delete=False) as stream:
            stream.write(
                "#timestamp, p_RS_R_x [m], p_RS_R_y [m], p_RS_R_z [m], "
                "q_RS_w [], q_RS_x [], q_RS_y [], q_RS_z [], "
                "v_RS_R_x [m s^-1], v_RS_R_y [m s^-1], v_RS_R_z [m s^-1]\n")
            stream.write("1000000000,1,2,3,1,0,0,0,4,5,6\n")
            stream.write("1010000000,2,3,4,1,0,0,0,7,8,9\n")
            path = stream.name
        try:
            ground_truth = EVALUATOR.read_official_ground_truth(path)
        finally:
            os.unlink(path)

        self.assertEqual(ground_truth["source"], "official_csv")
        np.testing.assert_allclose(ground_truth["timestamps"], [1.0, 1.01])
        np.testing.assert_allclose(ground_truth["positions"], [[1, 2, 3], [2, 3, 4]])
        np.testing.assert_allclose(ground_truth["velocities"], [[4, 5, 6], [7, 8, 9]])

    def test_position_difference_is_fallback_for_constant_velocity(self):
        times = np.linspace(0.0, 2.0, 21)
        velocity = np.array([2.0, -1.0, 0.5])
        positions = np.outer(times, velocity)
        query_times = np.array([0.5, 1.0, 1.5])

        recovered, valid = EVALUATOR.position_difference_velocity(
            times, positions, query_times, 0.1)

        self.assertTrue(np.all(valid))
        np.testing.assert_allclose(recovered, np.tile(velocity, (3, 1)), atol=1e-12)

    def test_alignment_rotation_changes_velocity_but_not_translation(self):
        rotation = EVALUATOR.quaternion_to_rotation(
            np.sqrt(0.5), 0.0, 0.0, np.sqrt(0.5))
        vins_velocity = np.array([[1.0, 0.0, 0.0]])
        aligned_velocity = (rotation @ vins_velocity.T).T

        np.testing.assert_allclose(aligned_velocity, [[0.0, 1.0, 0.0]], atol=1e-12)


if __name__ == "__main__":
    unittest.main()
