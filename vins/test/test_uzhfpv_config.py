#!/usr/bin/env python3

import importlib.util
import os
import unittest

import cv2
import numpy as np
import yaml


REPOSITORY = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
CONFIG_ROOT = os.path.join(REPOSITORY, "config")
CATEGORIES = (
    "uzhfpv_indoor",
    "uzhfpv_indoor_45",
    "uzhfpv_outdoor",
    "uzhfpv_outdoor_45",
)


class UzhFpvConfigTest(unittest.TestCase):

    def test_kalibr_conversion(self):
        for category in CATEGORIES:
            directory = os.path.join(CONFIG_ROOT, category)
            with open(os.path.join(directory, "kalibr_imucam_chain.yaml"), encoding="utf-8") as stream:
                kalibr = yaml.safe_load("".join(stream.readlines()[1:]))
            config = cv2.FileStorage(
                os.path.join(directory, "uzhfpv_stereo_imu_config.yaml"),
                cv2.FILE_STORAGE_READ)
            self.assertTrue(config.isOpened(), category)
            self.assertEqual(int(config.getNode("estimate_td").real()), 1)
            self.assertAlmostEqual(
                config.getNode("td").real(), kalibr["cam0"]["timeshift_cam_imu"], places=12)
            self.assertEqual(int(config.getNode("ltv_enable").real()), 1)
            self.assertEqual(int(config.getNode("multiple_thread").real()), 0)
            self.assertEqual(int(config.getNode("freq").real()), 31)
            self.assertEqual(int(config.getNode("show_track").real()), 0)
            self.assertAlmostEqual(
                config.getNode("ltv_gravity_sigma_deg").real(), 10.0)
            self.assertAlmostEqual(
                config.getNode("ltv_velocity_sigma_mps").real(), 1.0)
            self.assertAlmostEqual(
                config.getNode("ltv_velocity_gate_max_normalized_innovation").real(),
                0.03)
            self.assertEqual(int(config.getNode("use_mask").real()),
                             int(category == "uzhfpv_outdoor"))
            for camera_index in (0, 1):
                camera_name = f"cam{camera_index}"
                expected_transform = np.linalg.inv(
                    np.asarray(kalibr[camera_name]["T_cam_imu"], dtype=float))
                np.testing.assert_allclose(
                    config.getNode(f"body_T_{camera_name}").mat(),
                    expected_transform, atol=1e-12, err_msg=category)
                camera = cv2.FileStorage(
                    os.path.join(directory, camera_name + ".yaml"),
                    cv2.FILE_STORAGE_READ)
                self.assertEqual(camera.getNode("model_type").string(), "KANNALA_BRANDT")
                projection = camera.getNode("projection_parameters")
                actual = [projection.getNode(name).real() for name in
                          ("k2", "k3", "k4", "k5", "mu", "mv", "u0", "v0")]
                expected = (kalibr[camera_name]["distortion_coeffs"] +
                            kalibr[camera_name]["intrinsics"])
                np.testing.assert_allclose(actual, expected, atol=1e-12, err_msg=category)
                camera.release()
            config.release()

    def test_outdoor_masks_are_single_channel_and_expected_size(self):
        directory = os.path.join(CONFIG_ROOT, "uzhfpv_outdoor")
        for index in (0, 1):
            mask = cv2.imread(
                os.path.join(directory, f"mask_uzhfpv_outdoor_mask{index}.png"),
                cv2.IMREAD_UNCHANGED)
            self.assertIsNotNone(mask)
            self.assertEqual(mask.shape, (480, 640))
            self.assertEqual(mask.dtype, np.uint8)

    def test_launch_sequence_classification(self):
        path = os.path.join(REPOSITORY, "vins", "launch", "uzhfpv.launch.py")
        spec = importlib.util.spec_from_file_location("uzhfpv_launch", path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        self.assertEqual(module.classify_sequence("indoor_forward_3_snapdragon_with_gt_db"),
                         "uzhfpv_indoor")
        self.assertEqual(module.classify_sequence("indoor_45_4_snapdragon_with_gt_db"),
                         "uzhfpv_indoor_45")
        self.assertEqual(module.classify_sequence("outdoor_forward_1_snapdragon_with_gt_db"),
                         "uzhfpv_outdoor")
        self.assertEqual(module.classify_sequence("outdoor_45_1_snapdragon_with_gt_db"),
                         "uzhfpv_outdoor_45")
        with self.assertRaises(RuntimeError):
            module.classify_sequence("unknown")


if __name__ == "__main__":
    unittest.main()
