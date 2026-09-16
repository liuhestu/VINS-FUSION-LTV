#!/usr/bin/env python3
import importlib.util
import os
import tempfile
import unittest

SCRIPT = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "scripts", "stage5_cache.py")
SPEC = importlib.util.spec_from_file_location("stage5_cache", SCRIPT)
CACHE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(CACHE)


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


if __name__ == "__main__":
    unittest.main()
