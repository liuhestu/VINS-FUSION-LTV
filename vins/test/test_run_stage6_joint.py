#!/usr/bin/env python3

import csv
import importlib.util
import json
import os
import sys
import tempfile
import unittest
from pathlib import Path


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))
SCRIPT = SCRIPTS / "run_stage6_joint.py"
SPEC = importlib.util.spec_from_file_location("run_stage6_joint", SCRIPT)
STAGE6 = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(STAGE6)


FIELDNAMES = [
    "gravity_factor_added", "gravity_gate_base_eligible",
    "gravity_gate_pass", "gravity_gate_reason_mask",
    "velocity_factor_added", "velocity_factor_base_eligible",
    "velocity_gate_feature_ok", "velocity_gate_innovation_ok",
    "velocity_gate_disagreement_ok", "velocity_gate_reset_ok",
    "velocity_gate_pass", "velocity_gate_reason_mask",
    "velocity_oracle_mask_loaded", "velocity_oracle_mask_hit",
    "reset_reason",
]


class Stage6JointTest(unittest.TestCase):

    def row(self, **updates):
        row = {name: 0 for name in FIELDNAMES}
        row["reset_reason"] = "none"
        row.update(updates)
        return row

    def write_diagnostics(self, root, rows):
        path = Path(root) / "ltv_debug.csv"
        with path.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=FIELDNAMES)
            writer.writeheader()
            writer.writerows(rows)
        return path

    def test_effective_config_covers_both_modes_and_frozen_parameters(self):
        keys = set(STAGE6.FROZEN_SETTINGS)
        keys.update({"output_path", "ltv_debug_csv_path"})
        for mode in STAGE6.MODES.values():
            keys.update(mode)
        base = "%YAML:1.0\n" + "".join(f"{key}: old\n" for key in sorted(keys))
        output = Path("/tmp/stage6-test")

        baseline, _ = STAGE6.effective_config(base, "baseline", output)
        joint, _ = STAGE6.effective_config(base, "joint", output)
        joint_v_gate, _ = STAGE6.effective_config(
            base, "joint_v_gate", output)

        self.assertIn("ltv_enable_gravity_factor: 0", baseline)
        self.assertIn("ltv_enable: 1", baseline)
        self.assertIn("ltv_enable_velocity_factor: 0", baseline)
        self.assertIn("ltv_enable_gravity_quality_gate: 0", baseline)
        self.assertIn("ltv_enable_gravity_factor: 1", joint)
        self.assertIn("ltv_enable: 1", joint)
        self.assertIn("ltv_enable_velocity_factor: 1", joint)
        self.assertIn("ltv_enable_gravity_quality_gate: 1", joint)
        self.assertIn("ltv_enable_velocity_quality_gate: 0", joint)
        self.assertIn("ltv_enable_velocity_oracle_gate: 0", joint)
        self.assertIn("ltv_gravity_sigma_deg: 10.0", joint)
        self.assertIn("ltv_velocity_sigma_mps: 1.0", joint)
        self.assertIn("ltv_gravity_gate_max_normalized_innovation: 0.05", joint)
        self.assertIn("ltv_enable_gravity_factor: 1", joint_v_gate)
        self.assertIn("ltv_enable_velocity_factor: 1", joint_v_gate)
        self.assertIn("ltv_enable_velocity_quality_gate: 1", joint_v_gate)
        self.assertIn("ltv_velocity_gate_min_features: 25", joint_v_gate)
        self.assertIn(
            "ltv_velocity_gate_max_normalized_innovation: 0.03", joint_v_gate)
        self.assertIn(
            "ltv_velocity_gate_max_disagreement_mps: 0.5", joint_v_gate)
        self.assertIn(
            "ltv_velocity_gate_reset_cooldown_frames: 10", joint_v_gate)

    def test_joint_diagnostics_require_gate_and_fixed_velocity(self):
        rows = [self.row(
            gravity_factor_added=1, gravity_gate_base_eligible=1,
            gravity_gate_pass=1, velocity_factor_added=1,
            velocity_factor_base_eligible=1),
                self.row(
            gravity_gate_base_eligible=1, gravity_gate_reason_mask=4,
            velocity_factor_added=1, velocity_factor_base_eligible=1)]
        with tempfile.TemporaryDirectory() as root:
            result = STAGE6.factor_diagnostics(
                self.write_diagnostics(root, rows), "joint")
        self.assertEqual(result["gravity_factor_added_count"], 1)
        self.assertEqual(result["velocity_factor_added_count"], 2)
        self.assertEqual(result["gravity_gate_coverage"], 0.5)
        self.assertEqual(result["reset_count"], 0)

    def test_joint_v_gate_diagnostics_require_all_conditions(self):
        rows = [self.row(
            gravity_factor_added=1, gravity_gate_base_eligible=1,
            gravity_gate_pass=1, velocity_factor_added=1,
            velocity_factor_base_eligible=1, velocity_gate_feature_ok=1,
            velocity_gate_innovation_ok=1, velocity_gate_disagreement_ok=1,
            velocity_gate_reset_ok=1, velocity_gate_pass=1),
                self.row(
            gravity_factor_added=1, gravity_gate_base_eligible=1,
            gravity_gate_pass=1, velocity_factor_base_eligible=1,
            velocity_gate_feature_ok=1, velocity_gate_innovation_ok=0,
            velocity_gate_disagreement_ok=1, velocity_gate_reset_ok=1,
            velocity_gate_reason_mask=4),
                self.row(
            gravity_factor_added=1, gravity_gate_base_eligible=1,
            gravity_gate_pass=1, velocity_factor_added=1,
            velocity_factor_base_eligible=1, velocity_gate_feature_ok=1,
            velocity_gate_innovation_ok=1, velocity_gate_disagreement_ok=1,
            velocity_gate_reset_ok=1, velocity_gate_pass=1)]
        with tempfile.TemporaryDirectory() as root:
            result = STAGE6.factor_diagnostics(
                self.write_diagnostics(root, rows), "joint_v_gate")
        self.assertEqual(result["velocity_factor_added_count"], 2)
        self.assertEqual(result["velocity_gate_pass_count"], 2)
        self.assertEqual(result["velocity_gate_coverage"], 2 / 3)
        self.assertEqual(result["velocity_gate_on_segment_lengths"], [1, 1])

    def test_joint_rejects_gate_bypass_and_oracle_use(self):
        bypass = [self.row(
            gravity_factor_added=1, gravity_gate_base_eligible=1,
            velocity_factor_added=1, velocity_factor_base_eligible=1)]
        oracle = [self.row(
            gravity_factor_added=1, gravity_gate_base_eligible=1,
            gravity_gate_pass=1, velocity_factor_added=1,
            velocity_factor_base_eligible=1,
            velocity_oracle_mask_loaded=1, velocity_oracle_mask_hit=1)]
        with tempfile.TemporaryDirectory() as root:
            with self.assertRaisesRegex(RuntimeError, "bypassed gate"):
                STAGE6.factor_diagnostics(
                    self.write_diagnostics(root, bypass), "joint")
            with self.assertRaisesRegex(RuntimeError, "Oracle mask"):
                STAGE6.factor_diagnostics(
                    self.write_diagnostics(root, oracle), "joint")

    def test_joint_v_gate_rejects_velocity_gate_bypass(self):
        bypass = [self.row(
            gravity_factor_added=1, gravity_gate_base_eligible=1,
            gravity_gate_pass=1, velocity_factor_added=1,
            velocity_factor_base_eligible=1, velocity_gate_feature_ok=1,
            velocity_gate_innovation_ok=0, velocity_gate_disagreement_ok=1,
            velocity_gate_reset_ok=1)]
        with tempfile.TemporaryDirectory() as root:
            with self.assertRaisesRegex(RuntimeError, "quality gate"):
                STAGE6.factor_diagnostics(
                    self.write_diagnostics(root, bypass), "joint_v_gate")

    def test_transaction_publish_and_failure_preservation(self):
        with tempfile.TemporaryDirectory() as root_string:
            root = Path(root_string)
            partial = root / "joint.partial-id"
            partial.mkdir()
            (partial / "validation.json").write_text(
                json.dumps({"ok": True}), encoding="utf-8")
            final = root / "joint"
            STAGE6.publish_validated(partial, final)
            self.assertTrue((final / "validation.json").is_file())
            self.assertFalse(partial.exists())

            failed_partial = root / "baseline.partial-id"
            failed_partial.mkdir()
            failed = STAGE6.preserve_failed(
                failed_partial, root, "baseline", "id")
            self.assertIsNotNone(failed)
            self.assertTrue(failed.is_dir())
            self.assertIn("baseline.failed-", failed.name)

    def test_baseline_sha_must_match_stage6(self):
        with tempfile.TemporaryDirectory() as root_string:
            root = Path(root_string)
            current = root / "current.csv"
            reference = root / "reference.csv"
            current.write_text("same\n", encoding="utf-8")
            reference.write_text("same\n", encoding="utf-8")
            self.assertEqual(
                STAGE6.validate_baseline_vio(current, reference),
                STAGE6.sha256(current))
            reference.write_text("different\n", encoding="utf-8")
            with self.assertRaisesRegex(RuntimeError, "differs"):
                STAGE6.validate_baseline_vio(current, reference)


if __name__ == "__main__":
    unittest.main()
