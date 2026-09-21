#!/usr/bin/env python3
import importlib.util
import os
import sys
import tempfile
import unittest

SCRIPT = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "scripts", "stage5_cache.py")
SPEC = importlib.util.spec_from_file_location("stage5_cache", SCRIPT)
CACHE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(CACHE)

SCRIPTS = os.path.dirname(SCRIPT)
sys.path.insert(0, SCRIPTS)
PREPARE_SPEC = importlib.util.spec_from_file_location(
    "stage5_prepare_euroc", os.path.join(SCRIPTS, "stage5_prepare_euroc.py"))
PREPARE = importlib.util.module_from_spec(PREPARE_SPEC)
PREPARE_SPEC.loader.exec_module(PREPARE)


class Stage5CacheTest(unittest.TestCase):

    def test_imu_monotonicity_fails_closed(self):
        with self.assertRaisesRegex(RuntimeError, "strictly increasing"):
            CACHE.require_strictly_increasing([1, 1], "IMU")

    def test_sha256_and_csv_are_deterministic(self):
        with tempfile.TemporaryDirectory() as directory:
            path = os.path.join(directory, "manifest.csv")
            CACHE.write_csv(path, ["timestamp_ns"], [(1,), (2,)])
            self.assertEqual(
                CACHE.sha256_file(path),
                "07c787ea8877e17f9e0f1a0ac08109968fc1b36a1b96302c70f22b86e3aecb63")

    def test_imu_bracketing_drops_only_unsupported_boundaries(self):
        pairs = [
            {"pair_timestamp": 9}, {"pair_timestamp": 10},
            {"pair_timestamp": 19}, {"pair_timestamp": 20},
        ]
        filtered = PREPARE.filter_imu_bracketed_pairs(pairs, [10, 15, 20])
        self.assertEqual(
            [item["pair_timestamp"] for item in filtered], [10, 19])

    def test_imu_bracketing_margin_covers_time_shift(self):
        pairs = [{"pair_timestamp": value} for value in (10, 12, 17, 18, 19)]
        filtered = PREPARE.filter_imu_bracketed_pairs(
            pairs, [10, 15, 20], margin=2)
        self.assertEqual(
            [item["pair_timestamp"] for item in filtered], [12, 17])


if __name__ == "__main__":
    unittest.main()
