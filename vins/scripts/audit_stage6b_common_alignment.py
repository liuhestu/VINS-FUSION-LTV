#!/usr/bin/env python3
"""Re-evaluate Stage 6b orientation with each Baseline alignment held fixed."""

import argparse
import csv
import hashlib
import io
import json
import math
import os
import uuid
from pathlib import Path

import numpy as np

from evaluate_vins_euroc import (
    match_nearest, read_official_ground_truth, read_vins,
    rigid_alignment, rotation_error_rpy)


SEQUENCES = (
    "MH_01_easy", "MH_02_easy", "MH_03_medium", "MH_04_difficult",
    "MH_05_difficult", "V1_01_easy", "V1_02_medium",
    "V1_03_difficult", "V2_01_easy", "V2_02_medium",
    "V2_03_difficult")
MODES = ("baseline", "joint_v_gate")
ORIENTATION_KEYS = (
    "rotation_rmse_deg", "rotation_p95_error_deg",
    "maximum_rotation_error_deg", "roll_rmse_deg", "pitch_rmse_deg")


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def relative_change_percent(baseline, candidate):
    if not math.isfinite(baseline) or not math.isfinite(candidate):
        raise ValueError("relative-change inputs must be finite")
    if baseline == 0.0:
        raise ValueError("relative change is undefined for a zero baseline")
    return (candidate / baseline - 1.0) * 100.0


def orientation_metrics(vins_rotations, vins_indices, gt_rotations,
                        gt_indices, alignment_rotation):
    if len(vins_indices) != len(gt_indices) or len(vins_indices) < 3:
        raise ValueError("at least three paired rotations are required")
    rotation_errors = []
    rpy_errors = []
    for vins_index, gt_index in zip(vins_indices, gt_indices):
        delta = (gt_rotations[gt_index].T @ alignment_rotation @
                 vins_rotations[vins_index])
        cosine = np.clip((np.trace(delta) - 1.0) * 0.5, -1.0, 1.0)
        rotation_errors.append(math.acos(float(cosine)))
        rpy_errors.append(rotation_error_rpy(delta))
    rotation_errors = np.asarray(rotation_errors)
    rpy_errors = np.asarray(rpy_errors)
    return {
        "rotation_rmse_deg": math.degrees(
            math.sqrt(float(np.mean(rotation_errors ** 2)))),
        "rotation_p95_error_deg": math.degrees(
            float(np.percentile(rotation_errors, 95))),
        "maximum_rotation_error_deg": math.degrees(
            float(np.max(rotation_errors))),
        "roll_rmse_deg": math.degrees(
            math.sqrt(float(np.mean(rpy_errors[:, 0] ** 2)))),
        "pitch_rmse_deg": math.degrees(
            math.sqrt(float(np.mean(rpy_errors[:, 1] ** 2)))),
    }


def trajectory_data(path, ground_truth):
    timestamps, positions, rotations, _ = read_vins(path)
    vins_indices, gt_indices, _ = match_nearest(
        timestamps, ground_truth["timestamps"], 0.02)
    if len(vins_indices) < 3:
        raise RuntimeError(f"{path}: fewer than three GT-matched samples")
    alignment_rotation, _ = rigid_alignment(
        positions[vins_indices], ground_truth["positions"][gt_indices])
    return {
        "rotations": rotations,
        "vins_indices": vins_indices,
        "gt_indices": gt_indices,
        "alignment_rotation": alignment_rotation,
    }


def load_formal_metrics(path):
    metrics = {}
    with path.open(encoding="utf-8") as stream:
        for line_number, line in enumerate(stream, start=1):
            if not line.strip():
                continue
            try:
                row = json.loads(line)
                key = (row["sequence"], row["mode"])
            except (json.JSONDecodeError, KeyError) as error:
                raise RuntimeError(
                    f"{path}:{line_number}: invalid formal metric") from error
            if key in metrics:
                raise RuntimeError(f"{path}:{line_number}: duplicate {key}")
            metrics[key] = row
    expected = {(sequence, mode) for sequence in SEQUENCES for mode in MODES}
    if set(metrics) != expected:
        missing = sorted(expected - set(metrics))
        extra = sorted(set(metrics) - expected)
        raise RuntimeError(
            f"formal metrics set mismatch; missing={missing}, extra={extra}")
    return metrics


def alignment_delta(reference, candidate):
    delta = candidate @ reference.T
    cosine = np.clip((np.trace(delta) - 1.0) * 0.5, -1.0, 1.0)
    return {
        "angle_deg": math.degrees(math.acos(float(cosine))),
        "rpy_deg": [math.degrees(value) for value in rotation_error_rpy(delta)],
    }


def atomic_write(path, text):
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.partial-{uuid.uuid4().hex}")
    try:
        temporary.write_text(text, encoding="utf-8")
        os.replace(temporary, path)
    finally:
        if temporary.exists():
            temporary.unlink()


def csv_text(rows):
    fields = [
        "sequence", "group", "matched_samples",
        "baseline_rotation_rmse_deg",
        "joint_rotation_rmse_individual_deg",
        "joint_rotation_rmse_common_deg",
        "joint_rotation_change_individual_percent",
        "joint_rotation_change_common_percent",
        "baseline_rotation_p95_deg", "joint_rotation_p95_common_deg",
        "joint_rotation_p95_change_common_percent",
        "baseline_rotation_max_deg", "joint_rotation_max_common_deg",
        "joint_rotation_max_change_common_percent",
        "baseline_roll_rmse_deg", "joint_roll_rmse_common_deg",
        "joint_roll_change_common_percent",
        "baseline_pitch_rmse_deg", "joint_pitch_rmse_common_deg",
        "joint_pitch_change_common_percent",
        "joint_individual_alignment_delta_angle_deg",
        "joint_individual_alignment_delta_roll_deg",
        "joint_individual_alignment_delta_pitch_deg",
        "joint_individual_alignment_delta_yaw_deg",
    ]
    stream = io.StringIO()
    writer = csv.DictWriter(stream, fieldnames=fields, lineterminator="\n")
    writer.writeheader()
    for row in rows:
        writer.writerow({field: row[field] for field in fields})
    return stream.getvalue()


def evaluate(stage6b_root, cache_root):
    formal_path = stage6b_root / "stage6b_metrics.jsonl"
    if not formal_path.is_file():
        raise RuntimeError(f"missing formal Stage 6b metrics: {formal_path}")
    formal = load_formal_metrics(formal_path)
    results = []

    for sequence in SEQUENCES:
        metadata_path = cache_root / sequence / "metadata.json"
        if not metadata_path.is_file():
            raise RuntimeError(f"missing cache metadata: {metadata_path}")
        metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
        if metadata.get("format") != "ltv-stage5-cache-v2":
            raise RuntimeError(f"{sequence}: cache is not canonical v2")
        ground_truth_path = Path(metadata["ground_truth"])
        if (not ground_truth_path.is_file() or
                sha256(ground_truth_path) != metadata["ground_truth_sha256"]):
            raise RuntimeError(f"{sequence}: ground-truth hash mismatch")
        ground_truth = read_official_ground_truth(ground_truth_path)

        baseline = trajectory_data(
            stage6b_root / sequence / "baseline" / "vio.csv", ground_truth)
        joint = trajectory_data(
            stage6b_root / sequence / "joint_v_gate" / "vio.csv", ground_truth)
        baseline_metrics = orientation_metrics(
            baseline["rotations"], baseline["vins_indices"],
            ground_truth["rotations"], baseline["gt_indices"],
            baseline["alignment_rotation"])
        joint_individual = orientation_metrics(
            joint["rotations"], joint["vins_indices"],
            ground_truth["rotations"], joint["gt_indices"],
            joint["alignment_rotation"])
        joint_common = orientation_metrics(
            joint["rotations"], joint["vins_indices"],
            ground_truth["rotations"], joint["gt_indices"],
            baseline["alignment_rotation"])

        for mode, calculated in (
                ("baseline", baseline_metrics),
                ("joint_v_gate", joint_individual)):
            recorded = formal[(sequence, mode)]
            for key in ORIENTATION_KEYS:
                if not math.isclose(calculated[key], recorded[key],
                                    rel_tol=1e-10, abs_tol=1e-10):
                    raise RuntimeError(
                        f"{sequence}/{mode}: {key} does not reproduce "
                        "the formal Stage 6b evaluator")

        changes = {
            key: relative_change_percent(baseline_metrics[key],
                                         joint_common[key])
            for key in ORIENTATION_KEYS
        }
        original_changes = {
            key: relative_change_percent(baseline_metrics[key],
                                         joint_individual[key])
            for key in ORIENTATION_KEYS
        }
        delta = alignment_delta(
            baseline["alignment_rotation"], joint["alignment_rotation"])
        group = ("difficult" if sequence.endswith("_difficult")
                 else "easy_medium")
        result = {
            "sequence": sequence,
            "group": group,
            "matched_samples": int(len(joint["vins_indices"])),
            "baseline_common_alignment": baseline_metrics,
            "joint_individual_alignment": joint_individual,
            "joint_common_baseline_alignment": joint_common,
            "joint_change_individual_percent": original_changes,
            "joint_change_common_percent": changes,
            "joint_individual_alignment_delta_from_baseline": delta,
            "formal_ate_rmse_m": {
                mode: formal[(sequence, mode)]["ate_rmse_m"] for mode in MODES},
            "formal_velocity_rmse_mps": {
                mode: formal[(sequence, mode)]["velocity_rmse_mps"]
                for mode in MODES},
        }
        results.append(result)

    group_summary = {}
    for group in ("easy_medium", "difficult"):
        selected = [row for row in results if row["group"] == group]
        group_summary[group] = {}
        for key in ORIENTATION_KEYS:
            baseline_mean = float(np.mean([
                row["baseline_common_alignment"][key] for row in selected]))
            joint_mean = float(np.mean([
                row["joint_common_baseline_alignment"][key]
                for row in selected]))
            group_summary[group][key] = {
                "baseline_mean": baseline_mean,
                "joint_mean": joint_mean,
                "change_percent": relative_change_percent(
                    baseline_mean, joint_mean),
            }

    safety_failures = []
    for row in results:
        if row["group"] != "easy_medium":
            continue
        formal_baseline = formal[(row["sequence"], "baseline")]
        formal_joint = formal[(row["sequence"], "joint_v_gate")]
        checks = {
            "ate_rmse_m": relative_change_percent(
                formal_baseline["ate_rmse_m"], formal_joint["ate_rmse_m"]),
            "rotation_rmse_deg": row["joint_change_common_percent"][
                "rotation_rmse_deg"],
            "velocity_rmse_mps": relative_change_percent(
                formal_baseline["velocity_rmse_mps"],
                formal_joint["velocity_rmse_mps"]),
        }
        for metric, change in checks.items():
            if change > 3.0:
                safety_failures.append({
                    "sequence": row["sequence"], "metric": metric,
                    "change_percent": change})

    return {
        "schema": "ltv-stage6b-common-baseline-alignment-audit-v1",
        "alignment_policy": (
            "For each sequence, compute the rigid position-alignment rotation "
            "from Baseline and apply that same rotation to Baseline and Joint "
            "orientation. ATE and velocity remain the formal Stage 6b values."),
        "formal_metrics_sha256": sha256(formal_path),
        "sequence_results": results,
        "group_summary": group_summary,
        "safety_threshold_percent": 3.0,
        "safety_pass": not safety_failures,
        "safety_failures": safety_failures,
    }


def flatten_for_csv(audit):
    rows = []
    for result in audit["sequence_results"]:
        baseline = result["baseline_common_alignment"]
        individual = result["joint_individual_alignment"]
        common = result["joint_common_baseline_alignment"]
        individual_change = result["joint_change_individual_percent"]
        common_change = result["joint_change_common_percent"]
        delta = result["joint_individual_alignment_delta_from_baseline"]
        rows.append({
            "sequence": result["sequence"],
            "group": result["group"],
            "matched_samples": result["matched_samples"],
            "baseline_rotation_rmse_deg": baseline["rotation_rmse_deg"],
            "joint_rotation_rmse_individual_deg": individual[
                "rotation_rmse_deg"],
            "joint_rotation_rmse_common_deg": common["rotation_rmse_deg"],
            "joint_rotation_change_individual_percent": individual_change[
                "rotation_rmse_deg"],
            "joint_rotation_change_common_percent": common_change[
                "rotation_rmse_deg"],
            "baseline_rotation_p95_deg": baseline["rotation_p95_error_deg"],
            "joint_rotation_p95_common_deg": common["rotation_p95_error_deg"],
            "joint_rotation_p95_change_common_percent": common_change[
                "rotation_p95_error_deg"],
            "baseline_rotation_max_deg": baseline[
                "maximum_rotation_error_deg"],
            "joint_rotation_max_common_deg": common[
                "maximum_rotation_error_deg"],
            "joint_rotation_max_change_common_percent": common_change[
                "maximum_rotation_error_deg"],
            "baseline_roll_rmse_deg": baseline["roll_rmse_deg"],
            "joint_roll_rmse_common_deg": common["roll_rmse_deg"],
            "joint_roll_change_common_percent": common_change["roll_rmse_deg"],
            "baseline_pitch_rmse_deg": baseline["pitch_rmse_deg"],
            "joint_pitch_rmse_common_deg": common["pitch_rmse_deg"],
            "joint_pitch_change_common_percent": common_change[
                "pitch_rmse_deg"],
            "joint_individual_alignment_delta_angle_deg": delta["angle_deg"],
            "joint_individual_alignment_delta_roll_deg": delta["rpy_deg"][0],
            "joint_individual_alignment_delta_pitch_deg": delta["rpy_deg"][1],
            "joint_individual_alignment_delta_yaw_deg": delta["rpy_deg"][2],
        })
    return rows


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--stage6b-root", required=True, type=Path)
    parser.add_argument("--cache-root", required=True, type=Path)
    parser.add_argument("--output-json", type=Path)
    parser.add_argument("--output-csv", type=Path)
    arguments = parser.parse_args()
    output_json = (arguments.output_json or
                   arguments.stage6b_root / "stage6b_common_alignment_audit.json")
    output_csv = (arguments.output_csv or
                  arguments.stage6b_root / "stage6b_common_alignment_audit.csv")

    audit = evaluate(arguments.stage6b_root, arguments.cache_root)
    atomic_write(output_json, json.dumps(
        audit, indent=2, sort_keys=True, allow_nan=False) + "\n")
    atomic_write(output_csv, csv_text(flatten_for_csv(audit)))
    print(json.dumps({
        "output_json": str(output_json.resolve()),
        "output_csv": str(output_csv.resolve()),
        "sequence_count": len(audit["sequence_results"]),
        "safety_pass": audit["safety_pass"],
        "safety_failures": audit["safety_failures"],
    }, sort_keys=True))


if __name__ == "__main__":
    main()
