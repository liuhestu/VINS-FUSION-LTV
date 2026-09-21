#!/usr/bin/env python3
"""Freeze the UZH-FPV ground-truth timestamp policy before formal ATE."""

import argparse
import json
import math
import re
from pathlib import Path

import numpy as np

from evaluate_vins_euroc import (
    audit_uzh_leica, discover_uzh_leica, match_nearest, read_bag_time_range,
    read_uzh_ground_truth, read_vins, sha256_file)
from evaluate_euroc_final import atomic_write


SEQUENCES = (
    "indoor_forward_3_snapdragon_with_gt",
    "indoor_forward_5_snapdragon_with_gt",
    "indoor_forward_6_snapdragon_with_gt",
    "indoor_forward_7_snapdragon_with_gt",
    "indoor_forward_9_snapdragon_with_gt",
    "indoor_forward_10_snapdragon_with_gt",
    "indoor_45_2_snapdragon_with_gt",
    "indoor_45_4_snapdragon_with_gt",
    "indoor_45_9_snapdragon_with_gt",
    "indoor_45_12_snapdragon_with_gt",
    "indoor_45_13_snapdragon_with_gt",
    "indoor_45_14_snapdragon_with_gt",
    "outdoor_forward_1_snapdragon_with_gt",
    "outdoor_forward_3_snapdragon_with_gt",
    "outdoor_forward_5_snapdragon_with_gt",
    "outdoor_45_1_snapdragon_with_gt",
)

CATEGORIES = {
    "indoor_forward": "uzhfpv_indoor",
    "indoor_45": "uzhfpv_indoor_45",
    "outdoor_forward": "uzhfpv_outdoor",
    "outdoor_45": "uzhfpv_outdoor_45",
}


def sequence_family(sequence):
    for family in CATEGORIES:
        if sequence.startswith(family + "_"):
            return family
    raise RuntimeError(f"unsupported UZH-FPV sequence: {sequence}")


def read_td(config_path):
    matches = re.findall(
        r"^td\s*:\s*([-+0-9.eE]+)\s*$",
        config_path.read_text(encoding="utf-8"), re.MULTILINE)
    if len(matches) != 1:
        raise RuntimeError(f"expected one td setting in {config_path}")
    value = float(matches[0])
    if not math.isfinite(value):
        raise RuntimeError(f"non-finite td in {config_path}")
    return value


def rotation_angular_velocity(timestamps, rotations):
    if len(timestamps) < 3:
        raise RuntimeError("ground truth has fewer than three rotations")
    relative = np.einsum(
        "nij,njk->nik", np.transpose(rotations[:-2], (0, 2, 1)),
        rotations[2:])
    vee = 0.5 * np.column_stack((
        relative[:, 2, 1] - relative[:, 1, 2],
        relative[:, 0, 2] - relative[:, 2, 0],
        relative[:, 1, 0] - relative[:, 0, 1]))
    delta = timestamps[2:] - timestamps[:-2]
    if np.any(delta <= 0.0):
        raise RuntimeError("ground-truth timestamps are not increasing")
    return timestamps[1:-1], vee / delta[:, None]


def normalized_correlation(left, right):
    left = left - np.mean(left)
    right = right - np.mean(right)
    denominator = math.sqrt(float(left @ left) * float(right @ right))
    return float(left @ right / denominator) if denominator > 0.0 else math.nan


def correlation_at(offset, gt_times, gt_values, imu_times, imu_values):
    valid = ((gt_times + offset >= imu_times[0]) &
             (gt_times + offset <= imu_times[-1]))
    if np.count_nonzero(valid) < 100:
        return math.nan
    interpolated = np.interp(
        gt_times[valid] + offset, imu_times, imu_values)
    return normalized_correlation(gt_values[valid], interpolated)


def estimate_offset(gt_times, gt_gyro, imu_times, imu_gyro, component):
    step = max(1, int(round(0.01 / float(np.median(np.diff(gt_times))))))
    gt_times = gt_times[::step]
    gt_gyro = gt_gyro[::step]
    motion = np.linalg.norm(gt_gyro, axis=1)
    keep = motion >= np.quantile(motion, 0.25)
    gt_times = gt_times[keep]
    if component == "norm":
        gt_values = motion[keep]
        imu_values = np.linalg.norm(imu_gyro, axis=1)
    elif component == "x":
        gt_values = gt_gyro[keep, 0]
        imu_values = imu_gyro[:, 0]
    else:
        raise ValueError(f"unsupported correlation component: {component}")

    coarse = np.arange(-0.2, 0.200001, 0.001)
    scores = np.asarray([
        correlation_at(value, gt_times, gt_values, imu_times, imu_values)
        for value in coarse])
    if not np.any(np.isfinite(scores)):
        raise RuntimeError("time-offset correlation has no valid samples")
    coarse_peak = coarse[int(np.nanargmax(scores))]
    fine = np.arange(coarse_peak - 0.002, coarse_peak + 0.002001, 0.0001)
    fine_scores = np.asarray([
        correlation_at(value, gt_times, gt_values, imu_times, imu_values)
        for value in fine])
    peak_index = int(np.nanargmax(fine_scores))
    return {
        "offset_s": float(fine[peak_index]),
        "correlation": float(fine_scores[peak_index]),
        "sample_count": int(len(gt_times)),
    }


def read_imu_csv(path):
    values = np.genfromtxt(path, delimiter=",", names=True)
    if values.size == 0:
        raise RuntimeError(f"empty IMU cache: {path}")
    times = np.asarray(values["timestamp_ns"], dtype=float) * 1e-9
    gyro = np.column_stack((values["gx"], values["gy"], values["gz"]))
    if (not np.all(np.isfinite(times)) or not np.all(np.isfinite(gyro)) or
            np.any(np.diff(times) <= 0.0)):
        raise RuntimeError(f"invalid IMU cache: {path}")
    return times, gyro


def decide_family(residuals):
    values = np.asarray(residuals, dtype=float)
    median = float(np.median(values))
    mad = float(np.median(np.abs(values - median)))
    if abs(median) <= 0.008 and mad <= 0.005:
        return "direct", 0.0, median, mad
    if (-0.100 <= median <= -0.095 and mad <= 0.005 and
            np.all(np.abs(values - median) <= 0.010)):
        return "uniform_indoor_compensation", median, median, mad
    return "rejected", math.nan, median, mad


def audit_optional_leica(path):
    if not path:
        return {"status": "not_retained"}
    audit = audit_uzh_leica(path)
    if not audit["positions_finite"]:
        raise RuntimeError(f"raw Leica positions are non-finite: {path}")
    audit["status"] = "audited"
    return audit


def audit_sequence(sequence, dataset_root, cache_root, results_root,
                   repository):
    family = sequence_family(sequence)
    category = CATEGORIES[family]
    bag = dataset_root / category / f"{sequence}_db"
    gt_path = dataset_root / "archives" / sequence / "groundtruth.txt"
    cache = cache_root / sequence
    metadata_path = cache / "metadata.json"
    baseline = results_root / sequence / "baseline" / "vio.csv"
    config = repository / "config" / category / "uzhfpv_stereo_imu_config.yaml"
    for path, label in ((bag, "bag"), (gt_path, "GT"), (metadata_path, "cache"),
                        (baseline, "baseline"), (config, "config")):
        if not path.exists():
            raise RuntimeError(f"{sequence}: missing {label}: {path}")

    metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
    if metadata.get("ground_truth_sha256") != sha256_file(str(gt_path)):
        raise RuntimeError(f"{sequence}: GT/cache hash mismatch")
    gt = read_uzh_ground_truth(str(gt_path), read_bag_time_range(str(bag)))
    imu_times, imu_gyro = read_imu_csv(cache / "imu.csv")
    gt_gyro_times, gt_gyro = rotation_angular_velocity(
        gt["timestamps"], gt["rotations"])
    primary = estimate_offset(
        gt_gyro_times, gt_gyro, imu_times, imu_gyro, "norm")
    cross_check = estimate_offset(
        gt_gyro_times, gt_gyro, imu_times, imu_gyro, "x")
    td = read_td(config)
    residual = primary["offset_s"] - td
    confidence = (
        primary["correlation"] >= 0.8 and
        cross_check["correlation"] >= 0.8 and
        abs(primary["offset_s"] - cross_check["offset_s"]) <= 0.015)
    leica_path = discover_uzh_leica(str(bag))
    leica_audit = audit_optional_leica(leica_path)
    return {
        "sequence": sequence,
        "family": family,
        "category": category,
        "bag_path": str(bag.resolve()),
        "bag_metadata_sha256": sha256_file(str(bag / "metadata.yaml")),
        "bag_time_range_s": list(read_bag_time_range(str(bag))),
        "imu_time_range_s": [float(imu_times[0]), float(imu_times[-1])],
        "gt_path": str(gt_path.resolve()),
        "gt_sha256": gt["sha256"],
        "gt_time_range_s": [float(gt["timestamps"][0]),
                            float(gt["timestamps"][-1])],
        "raw_leica_audit": leica_audit,
        "baseline_vio_path": str(baseline.resolve()),
        "baseline_vio_sha256": sha256_file(str(baseline)),
        "calibration_td_s": td,
        "raw_offset_s": primary["offset_s"],
        "raw_offset_correlation": primary["correlation"],
        "x_axis_offset_s": cross_check["offset_s"],
        "x_axis_correlation": cross_check["correlation"],
        "offset_cross_check_difference_s": abs(
            primary["offset_s"] - cross_check["offset_s"]),
        "residual_offset_s": residual,
        "offset_confident": confidence,
        "imu_sha256": metadata["imu_sha256"],
    }


def add_overlap(sequence_audit, applied_offset):
    gt = read_uzh_ground_truth(sequence_audit["gt_path"])
    gt_times = gt["timestamps"] + applied_offset
    vio_times, vio_positions, _, _ = read_vins(
        sequence_audit["baseline_vio_path"])
    finite_vio = np.all(np.isfinite(vio_positions), axis=1)
    finite_gt = np.all(np.isfinite(gt["positions"]), axis=1)
    if not np.all(finite_vio) or not np.all(finite_gt):
        raise RuntimeError(f"{sequence_audit['sequence']}: non-finite position")
    vio_indices, _, errors = match_nearest(vio_times, gt_times, 0.02)
    if len(vio_indices) < 100:
        raise RuntimeError(
            f"{sequence_audit['sequence']}: fewer than 100 GT/VIO matches")
    start = float(vio_times[vio_indices[0]])
    stop = float(vio_times[vio_indices[-1]])
    if stop - start < 5.0:
        raise RuntimeError(
            f"{sequence_audit['sequence']}: GT/VIO overlap is below 5 seconds")
    sequence_audit.update({
        "applied_gt_time_offset_s": applied_offset,
        "corrected_gt_time_range_s": [float(gt_times[0]), float(gt_times[-1])],
        "baseline_common_position_interval_s": [start, stop],
        "baseline_common_position_duration_s": stop - start,
        "baseline_common_position_samples": int(len(vio_indices)),
        "baseline_maximum_timestamp_error_s": float(np.max(errors)),
    })


def build_audit(dataset_root, cache_root, results_root, repository):
    sequences = [
        audit_sequence(sequence, dataset_root, cache_root, results_root,
                       repository)
        for sequence in SEQUENCES]
    if not all(item["offset_confident"] for item in sequences):
        failed = [item["sequence"] for item in sequences
                  if not item["offset_confident"]]
        raise RuntimeError(f"low-confidence time offsets: {failed}")

    family_decisions = {}
    for family in CATEGORIES:
        selected = [item for item in sequences if item["family"] == family]
        status, applied, median, mad = decide_family(
            [item["residual_offset_s"] for item in selected])
        if status == "uniform_indoor_compensation" and not family.startswith("indoor"):
            status, applied = "rejected", math.nan
        family_decisions[family] = {
            "status": status,
            "residual_median_s": median,
            "residual_mad_s": mad,
            "applied_gt_time_offset_s": applied,
        }
        if status == "rejected":
            raise RuntimeError(
                f"{family}: residual GT offset matches neither accepted band")
        for item in selected:
            add_overlap(item, applied)

    return {
        "schema": "uzhfpv-gt-time-offset-audit-v1",
        "offset_definition": (
            "raw maximizes corr(omega_gt(t), omega_imu(t+raw)); "
            "residual=raw-timeshift_cam_imu; t_gt_eval=t_gt+applied"),
        "thresholds": {
            "search_range_s": [-0.2, 0.2],
            "coarse_step_s": 0.001,
            "fine_step_s": 0.0001,
            "minimum_correlation": 0.8,
            "maximum_cross_check_difference_s": 0.015,
            "direct_family_median_abs_s": 0.008,
            "maximum_family_mad_s": 0.005,
            "indoor_compensation_band_s": [-0.100, -0.095],
        },
        "families": family_decisions,
        "sequences": sequences,
    }


def main():
    repository = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset-root", type=Path,
                        default=Path("/home/he/datasets/uzhfpv"))
    parser.add_argument("--cache-root", required=True, type=Path)
    parser.add_argument("--results-root", required=True, type=Path)
    parser.add_argument(
        "--output", type=Path,
        default=repository / "config/uzhfpv_gt_time_offsets.json")
    arguments = parser.parse_args()
    audit = build_audit(arguments.dataset_root, arguments.cache_root,
                        arguments.results_root, repository)
    atomic_write(arguments.output,
                 json.dumps(audit, indent=2, sort_keys=True) + "\n")
    print(json.dumps({
        "output": str(arguments.output.resolve()),
        "families": audit["families"],
    }, sort_keys=True))


if __name__ == "__main__":
    main()
