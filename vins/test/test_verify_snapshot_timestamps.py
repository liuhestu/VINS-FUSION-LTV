#!/usr/bin/env python3
import importlib.util
import os
import tempfile
import unittest


SCRIPT_PATH = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "scripts", "verify_snapshot_timestamps.py")
SPEC = importlib.util.spec_from_file_location("verify_snapshot_timestamps", SCRIPT_PATH)
VERIFY = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(VERIFY)


class VerifySnapshotTimestampsTest(unittest.TestCase):

    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()

    def tearDown(self):
        self.directory.cleanup()

    def write_csv(self, name, timestamps):
        path = os.path.join(self.directory.name, name)
        with open(path, "w", encoding="utf-8") as stream:
            stream.write("frame_timestamp,valid\n")
            for timestamp in timestamps:
                stream.write(f"{timestamp},1\n")
        return path

    def test_reports_equal_and_divergent_grids_without_rejecting_divergence(self):
        baseline = self.write_csv("b.csv", [0.0, 1.0, 1.05])
        fixed = self.write_csv("fixed.csv", [1.0, 1.05])
        oracle = self.write_csv("oracle.csv", [1.0, 1.1])
        report = VERIFY.verify_independent([baseline, fixed, oracle])
        self.assertTrue(report[baseline]["same_grid_as_first"])
        self.assertTrue(report[fixed]["same_grid_as_first"])
        self.assertFalse(report[oracle]["same_grid_as_first"])

    def test_rejects_non_monotonic_grid(self):
        bad = self.write_csv("bad.csv", [1.0, 1.0])
        with self.assertRaisesRegex(RuntimeError, "not strictly increasing"):
            VERIFY.verify_independent([bad])


if __name__ == "__main__":
    unittest.main()
