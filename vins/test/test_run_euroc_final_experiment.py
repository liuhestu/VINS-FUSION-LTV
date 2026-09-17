#!/usr/bin/env python3

import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))
SCRIPT = SCRIPTS / "run_euroc_final_experiment.py"
SPEC = importlib.util.spec_from_file_location(
    "run_euroc_final_experiment", SCRIPT)
RUN = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(RUN)


class RunEurocFinalExperimentTest(unittest.TestCase):

    def test_batch_manifest_freezes_all_modes_and_hashes(self):
        with tempfile.TemporaryDirectory() as root_string:
            root = Path(root_string)
            config = root / "config.yaml"
            replay = root / "replay"
            runner = root / "runner.py"
            for path, text in ((config, "config"), (replay, "binary"),
                               (runner, "runner")):
                path.write_text(text, encoding="utf-8")
            arguments = SimpleNamespace(
                config=config, cache_root=root / "cache", replay=replay)
            manifest = RUN.batch_manifest(arguments, runner)
            self.assertEqual(manifest["expected_run_count"], 55)
            self.assertEqual(manifest["modes"], list(RUN.MODES))
            self.assertEqual(len(manifest["config_sha256"]), 64)
            self.assertEqual(manifest["frozen_settings"][
                "ltv_enable_velocity_oracle_gate"], "0")

    def test_expected_pair_consumption_counts_five_modes(self):
        with tempfile.TemporaryDirectory() as root_string:
            root = Path(root_string)
            expected = 0
            for index, sequence in enumerate(RUN.SEQUENCES, start=1):
                directory = root / sequence
                directory.mkdir()
                (directory / "metadata.json").write_text(json.dumps({
                    "paired_count": index}), encoding="utf-8")
                expected += index
            self.assertEqual(
                RUN.expected_pair_consumption(root), expected * 5)

    def test_failed_batch_is_preserved_with_evidence(self):
        with tempfile.TemporaryDirectory() as root_string:
            root = Path(root_string)
            partial = root / "formal.partial-id"
            partial.mkdir()
            output = root / "formal"
            failed = RUN.preserve_failed(
                partial, output, {"error": "synthetic failure"})
            self.assertFalse(partial.exists())
            self.assertTrue(failed.is_dir())
            evidence = json.loads(
                (failed / "batch_failure.json").read_text(encoding="utf-8"))
            self.assertEqual(evidence["error"], "synthetic failure")


if __name__ == "__main__":
    unittest.main()
