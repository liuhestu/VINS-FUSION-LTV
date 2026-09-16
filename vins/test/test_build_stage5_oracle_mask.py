#!/usr/bin/env python3
import importlib.util
import os
import tempfile
import unittest
from pathlib import Path


SCRIPT = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "scripts", "build_stage5_oracle_mask.py")
SPEC = importlib.util.spec_from_file_location("build_stage5_oracle_mask", SCRIPT)
MASK = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MASK)


class BuildStage5OracleMaskTest(unittest.TestCase):

    def test_world_to_body_rotation(self):
        value = 2 ** -0.5
        velocity = MASK.rotate_world_to_body((value, 0.0, 0.0, value),
                                             (0.0, 1.0, 0.0))
        self.assertAlmostEqual(velocity[0], 1.0)
        self.assertAlmostEqual(velocity[1], 0.0)

    def test_builds_strict_zero_and_point_zero_two_masks(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            ground_truth = root / "gt.csv"
            ground_truth.write_text(
                "#timestamp [ns],q_RS_w [],q_RS_x [],q_RS_y [],q_RS_z [],"
                "v_RS_R_x [m s^-1],v_RS_R_y [m s^-1],v_RS_R_z [m s^-1]\n"
                "1000000000,1,0,0,0,0,0,0\n",
                encoding="utf-8")
            reference = root / "ltv.csv"
            reference.write_text(
                "frame_timestamp,velocity_body_x,velocity_body_y,velocity_body_z,"
                "vins_velocity_body_x,vins_velocity_body_y,vins_velocity_body_z,"
                "valid,velocity_valid,reset_reason,velocity_factor_base_eligible\n"
                "1,0.99,0,0,1,0,0,1,1,none,1\n",
                encoding="utf-8")
            rows = MASK.build_rows(reference, ground_truth)
            self.assertEqual(rows[0]["oracle_0"], 1)
            self.assertEqual(rows[0]["oracle_002"], 0)

    def test_gt_tolerance_and_ineligible_rows_fail_closed(self):
        entries = [(1_000_000_000, (0.0, 0.0, 0.0))]
        match, error = MASK.nearest_ground_truth(
            entries, [entry[0] for entry in entries], 1_005_000_000, 5_000_000)
        self.assertIsNotNone(match)
        self.assertEqual(error, 5_000_000)
        miss, _ = MASK.nearest_ground_truth(
            entries, [entry[0] for entry in entries], 1_005_000_001, 5_000_000)
        self.assertIsNone(miss)


if __name__ == "__main__":
    unittest.main()
