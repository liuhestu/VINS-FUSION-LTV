#!/usr/bin/env python3
"""Evaluate one fresh five-mode EuRoC experiment on a common time support."""

import argparse
import csv
import io
import json
import math
import os
import uuid
from pathlib import Path

import numpy as np

from evaluate_vins_euroc import (
    match_nearest, read_official_ground_truth, read_vins, rigid_alignment)
from run_stage5_velocity_oracle import sha256


SEQUENCES = (
    "MH_01_easy", "MH_02_easy", "MH_03_medium", "MH_04_difficult",
    "MH_05_difficult", "V1_01_easy", "V1_02_medium",
    "V1_03_difficult", "V2_01_easy", "V2_02_medium",
    "V2_03_difficult")
MODES = (
    "baseline", "gravity_only", "velocity_only", "joint", "joint_v_gate")
DISPLAY_NAMES = {
    "baseline": "B",
    "gravity_only": "G_gate",
    "velocity_only": "V_fixed",
    "joint": "G_gate+V_fixed",
    "joint_v_gate": "G_gate+V_gate",
}
GRAVITY_MODES = {"gravity_only", "joint", "joint_v_gate"}
VELOCITY_MODES = {"velocity_only", "joint", "joint_v_gate"}
CROSS_MODE_TOLERANCE_S = 0.001
GT_TOLERANCE_S = 0.02
MIN_COMMON_COVERAGE = 0.99
MAX_RANGE_ENDPOINT_SPREAD_S = 0.1


def atomic_write(path, text):
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.partial-{uuid.uuid4().hex}")
    try:
        temporary.write_text(text, encoding="utf-8")
        os.replace(temporary, path)
    finally:
        if temporary.exists():
            temporary.unlink()


def relative_change_percent(baseline, candidate):
    if not math.isfinite(baseline) or not math.isfinite(candidate):
        raise ValueError("relative-change inputs must be finite")
    if baseline == 0.0:
        raise ValueError("relative change is undefined for a zero baseline")
    return (candidate / baseline - 1.0) * 100.0


def match_one_to_one(anchor_times, candidate_times, tolerance_s):
    """Greedily match increasing timestamps without reusing candidates."""
    indices = np.full(len(anchor_times), -1, dtype=int)
    errors = np.full(len(anchor_times), math.nan)
    previous = -1
    for anchor_index, timestamp in enumerate(anchor_times):
        insertion = int(np.searchsorted(candidate_times, timestamp))
        candidates = [
            index for index in (insertion - 1, insertion)
            if previous < index < len(candidate_times)]
        if not candidates:
            continue
        candidate = min(
            candidates, key=lambda index: abs(candidate_times[index] - timestamp))
        error = abs(candidate_times[candidate] - timestamp)
        if error <= tolerance_s:
            indices[anchor_index] = candidate
            errors[anchor_index] = error
            previous = candidate
    return indices, errors


def common_time_support(trajectories, ground_truth):
    baseline_times = trajectories["baseline"]["timestamps"]
    mappings = {"baseline": np.arange(len(baseline_times), dtype=int)}
    errors = {"baseline": np.zeros(len(baseline_times))}
    for mode in MODES[1:]:
        mappings[mode], errors[mode] = match_one_to_one(
            baseline_times, trajectories[mode]["timestamps"],
            CROSS_MODE_TOLERANCE_S)
    gt_mapping, gt_errors = match_one_to_one(
        baseline_times, ground_truth["timestamps"], GT_TOLERANCE_S)

    valid = gt_mapping >= 0
    for mode in MODES[1:]:
        valid &= mappings[mode] >= 0
    common_count = int(np.count_nonzero(valid))
    if common_count < 3:
        raise RuntimeError("fewer than three five-mode common timestamps")

    coverages = {}
    starts = []
    stops = []
    for mode in MODES:
        timestamps = trajectories[mode]["timestamps"]
        valid_indices, _, _ = match_nearest(
            timestamps, ground_truth["timestamps"], GT_TOLERANCE_S)
        if len(valid_indices) < 3:
            raise RuntimeError(f"{mode}: fewer than three GT-valid timestamps")
        coverages[mode] = common_count / len(valid_indices)
        starts.append(float(timestamps[valid_indices[0]]))
        stops.append(float(timestamps[valid_indices[-1]]))
        if coverages[mode] + 1e-12 < MIN_COMMON_COVERAGE:
            raise RuntimeError(
                f"{mode}: common timestamp coverage {coverages[mode]:.6f} "
                f"is below {MIN_COMMON_COVERAGE:.2f}")

    start_spread = max(starts) - min(starts)
    stop_spread = max(stops) - min(stops)
    if (start_spread > MAX_RANGE_ENDPOINT_SPREAD_S + 1e-12 or
            stop_spread > MAX_RANGE_ENDPOINT_SPREAD_S + 1e-12):
        raise RuntimeError(
            "five-mode trajectory endpoint spread exceeds 0.1 seconds")

    maximum_cross_error = max(
        float(np.nanmax(errors[mode][valid])) for mode in MODES)
    return {
        "mode_indices": {
            mode: mappings[mode][valid] for mode in MODES},
        "gt_indices": gt_mapping[valid],
        "common_count": common_count,
        "coverage": coverages,
        "maximum_cross_mode_time_error_s": maximum_cross_error,
        "maximum_gt_time_error_s": float(np.nanmax(gt_errors[valid])),
        "start_spread_s": start_spread,
        "stop_spread_s": stop_spread,
    }


def rmse(values):
    values = np.asarray(values, dtype=float)
    if not len(values) or not np.all(np.isfinite(values)):
        raise RuntimeError("metric input is empty or non-finite")
    return math.sqrt(float(np.mean(values ** 2)))


def trajectory_metrics(trajectories, ground_truth, support):
    gt_indices = support["gt_indices"]
    gt_positions = ground_truth["positions"][gt_indices]
    gt_rotations = ground_truth["rotations"][gt_indices]
    gt_velocities = ground_truth["velocities"][gt_indices]
    if gt_rotations is None or gt_velocities is None:
        raise RuntimeError("official EuRoC orientation and velocity are required")

    baseline_indices = support["mode_indices"]["baseline"]
    baseline_positions = trajectories["baseline"]["positions"][baseline_indices]
    baseline_alignment, _ = rigid_alignment(baseline_positions, gt_positions)
    metrics = {}
    for mode in MODES:
        indices = support["mode_indices"][mode]
        trajectory = trajectories[mode]
        positions = trajectory["positions"][indices]
        rotations = trajectory["rotations"][indices]
        velocities = trajectory["velocities"][indices]

        position_rotation, position_translation = rigid_alignment(
            positions, gt_positions)
        aligned_positions = (
            position_rotation @ positions.T).T + position_translation
        position_errors = np.linalg.norm(aligned_positions - gt_positions, axis=1)

        rotation_errors = []
        for estimated, reference in zip(rotations, gt_rotations):
            delta = reference.T @ baseline_alignment @ estimated
            cosine = np.clip((np.trace(delta) - 1.0) * 0.5, -1.0, 1.0)
            rotation_errors.append(math.acos(float(cosine)))

        aligned_velocities = (baseline_alignment @ velocities.T).T
        velocity_errors = np.linalg.norm(
            aligned_velocities - gt_velocities, axis=1)
        metrics[mode] = {
            "ate_rmse_m": rmse(position_errors),
            "rotation_rmse_deg": math.degrees(rmse(rotation_errors)),
            "velocity_rmse_mps": rmse(velocity_errors),
        }
    return metrics, baseline_alignment


def load_run(run_path, sequence, mode):
    required = (
        "vio.csv", "ltv_debug.csv", "replay.log", "replay_summary.json",
        "validation.json", "input_manifest.json", "effective_config.yaml")
    missing = [name for name in required if not (run_path / name).is_file()]
    if missing:
        raise RuntimeError(f"{sequence}/{mode}: missing files {missing}")
    validation = json.loads(
        (run_path / "validation.json").read_text(encoding="utf-8"))
    manifest = json.loads(
        (run_path / "input_manifest.json").read_text(encoding="utf-8"))
    if manifest.get("mode") != mode:
        raise RuntimeError(f"{sequence}/{mode}: manifest mode mismatch")
    required_zero = (
        "reset_count", "solver_failure_count", "dds_error_count",
        "nan_inf_count", "velocity_oracle_mask_loaded_count",
        "velocity_oracle_mask_hit_count")
    if validation.get("replay_exit_code") != 0:
        raise RuntimeError(f"{sequence}/{mode}: replay exit code is not zero")
    for key in required_zero:
        if validation.get(key) != 0:
            raise RuntimeError(f"{sequence}/{mode}: {key} is not zero")
    if (validation.get("canonical_pair_count") !=
            validation.get("consumed_pair_count")):
        raise RuntimeError(f"{sequence}/{mode}: canonical input not fully consumed")

    gravity = validation["gravity_factor_added_count"]
    velocity = validation["velocity_factor_added_count"]
    if mode == "baseline" and (gravity or velocity):
        raise RuntimeError(f"{sequence}: Baseline added an LTV factor")
    if mode == "gravity_only" and not (gravity > 0 and velocity == 0):
        raise RuntimeError(f"{sequence}: invalid G_gate factor counts")
    if mode == "velocity_only" and not (gravity == 0 and velocity > 0):
        raise RuntimeError(f"{sequence}: invalid V_fixed factor counts")
    if mode in ("joint", "joint_v_gate") and not (gravity > 0 and velocity > 0):
        raise RuntimeError(f"{sequence}/{mode}: missing joint factors")

    timestamps, positions, rotations, velocities = read_vins(run_path / "vio.csv")
    if not all(np.all(np.isfinite(values)) for values in (
            timestamps, positions, rotations, velocities)):
        raise RuntimeError(f"{sequence}/{mode}: non-finite VIO output")
    return {
        "timestamps": timestamps,
        "positions": positions,
        "rotations": rotations,
        "velocities": velocities,
        "validation": validation,
        "manifest": manifest,
    }


def validate_manifests(sequence, runs, metadata):
    immutable_keys = (
        "pair_list_sha256", "imu_sha256", "ground_truth_sha256",
        "base_config_sha256", "replay_sha256")
    baseline = runs["baseline"]["manifest"]
    for mode in MODES:
        manifest = runs[mode]["manifest"]
        for key in immutable_keys:
            if manifest.get(key) != baseline.get(key):
                raise RuntimeError(
                    f"{sequence}/{mode}: manifest {key} differs from Baseline")
        settings = manifest.get("frozen_settings", {})
        if (settings.get("ltv_enable_velocity_oracle_gate") != "0" or
                settings.get("ltv_velocity_oracle_mask_path") != '\"\"' or
                settings.get("load_previous_pose_graph") != "0"):
            raise RuntimeError(
                f"{sequence}/{mode}: Oracle or loop closure is enabled")
    expected = {
        "pair_list_sha256": metadata["pair_list_sha256"],
        "imu_sha256": metadata["imu_sha256"],
        "ground_truth_sha256": metadata["ground_truth_sha256"],
    }
    for key, value in expected.items():
        if baseline.get(key) != value:
            raise RuntimeError(f"{sequence}: cache {key} mismatch")


def evaluate(results_root, cache_root):
    sequence_results = []
    validation_rows = []
    total_consumed = 0
    for sequence in SEQUENCES:
        metadata_path = cache_root / sequence / "metadata.json"
        if not metadata_path.is_file():
            raise RuntimeError(f"{sequence}: missing canonical metadata")
        metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
        if metadata.get("format") != "ltv-stage5-cache-v2":
            raise RuntimeError(f"{sequence}: cache is not canonical v2")
        ground_truth_path = Path(metadata["ground_truth"])
        if (not ground_truth_path.is_file() or
                sha256(ground_truth_path) != metadata["ground_truth_sha256"]):
            raise RuntimeError(f"{sequence}: official GT hash mismatch")
        ground_truth = read_official_ground_truth(ground_truth_path)

        runs = {
            mode: load_run(results_root / sequence / mode, sequence, mode)
            for mode in MODES
        }
        validate_manifests(sequence, runs, metadata)
        support = common_time_support(runs, ground_truth)
        metrics, alignment = trajectory_metrics(runs, ground_truth, support)

        sequence_results.append({
            "sequence": sequence,
            "group": ("difficult" if sequence.endswith("_difficult")
                      else "easy_medium"),
            "common_sample_count": support["common_count"],
            "common_coverage": support["coverage"],
            "maximum_cross_mode_time_error_s": support[
                "maximum_cross_mode_time_error_s"],
            "maximum_gt_time_error_s": support["maximum_gt_time_error_s"],
            "trajectory_start_spread_s": support["start_spread_s"],
            "trajectory_stop_spread_s": support["stop_spread_s"],
            "baseline_alignment_rotation": alignment.tolist(),
            "metrics": metrics,
        })

        for mode in MODES:
            validation = runs[mode]["validation"]
            consumed = validation["consumed_pair_count"]
            total_consumed += consumed
            validation_rows.append({
                "sequence": sequence,
                "mode": mode,
                "replay_exit_code": validation["replay_exit_code"],
                "canonical_pair_count": validation["canonical_pair_count"],
                "consumed_pair_count": consumed,
                "reset_count": validation["reset_count"],
                "solver_failure_count": validation["solver_failure_count"],
                "dds_error_count": validation["dds_error_count"],
                "nan_inf_count": validation["nan_inf_count"],
                "gravity_factor_count": validation[
                    "gravity_factor_added_count"],
                "velocity_factor_count": validation[
                    "velocity_factor_added_count"],
                "gravity_gate_coverage": (
                    validation["gravity_gate_coverage"]
                    if mode in GRAVITY_MODES else None),
                "velocity_gate_coverage": (
                    validation["velocity_gate_coverage"]
                    if mode == "joint_v_gate" else None),
                "common_sample_count": support["common_count"],
                "common_coverage": support["coverage"][mode],
            })

    if len(validation_rows) != len(SEQUENCES) * len(MODES):
        raise RuntimeError("final experiment does not contain 55 valid runs")
    return {
        "schema": "ltv-euroc-final-unified-v1",
        "alignment_policy": {
            "ate": "independent SE(3) position alignment on common samples",
            "rotation": "one Baseline position-alignment rotation per sequence",
            "velocity": "the same Baseline alignment rotation per sequence",
        },
        "timestamp_policy": {
            "cross_mode_tolerance_s": CROSS_MODE_TOLERANCE_S,
            "ground_truth_tolerance_s": GT_TOLERANCE_S,
            "minimum_common_coverage": MIN_COMMON_COVERAGE,
            "maximum_endpoint_spread_s": MAX_RANGE_ENDPOINT_SPREAD_S,
        },
        "sequence_results": sequence_results,
        "validation_rows": validation_rows,
        "run_count": len(validation_rows),
        "total_consumed_pair_count": total_consumed,
    }


def metric_rows(audit):
    rows = []
    for sequence in audit["sequence_results"]:
        for mode in MODES:
            row = {
                "sequence": sequence["sequence"],
                "group": sequence["group"],
                "mode": mode,
                "common_sample_count": sequence["common_sample_count"],
            }
            row.update(sequence["metrics"][mode])
            rows.append(row)
    return rows


def jsonl_text(rows):
    return "".join(
        json.dumps(row, sort_keys=True, allow_nan=False) + "\n" for row in rows)


def csv_text(rows):
    if not rows:
        raise ValueError("cannot render an empty CSV")
    stream = io.StringIO()
    writer = csv.DictWriter(
        stream, fieldnames=list(rows[0]), lineterminator="\n")
    writer.writeheader()
    writer.writerows(rows)
    return stream.getvalue()


def group_means(audit, group, metric):
    selected = [
        row for row in audit["sequence_results"] if row["group"] == group]
    return {
        mode: float(np.mean([row["metrics"][mode][metric] for row in selected]))
        for mode in MODES
    }


def table_row(name, values, precision):
    baseline = values["baseline"]
    cells = [name, f"{baseline:.{precision}f}"]
    for mode in MODES[1:]:
        cells.extend([
            f"{values[mode]:.{precision}f}",
            f"{relative_change_percent(baseline, values[mode]):+.2f}%"])
    return "| " + " | ".join(cells) + " |"


def render_markdown(audit):
    metric_sections = (
        ("Table 1 — ATE RMSE (m)", "ate_rmse_m", 6),
        ("Table 2 — Rotation RMSE (deg)", "rotation_rmse_deg", 5),
        ("Table 3 — Velocity RMSE (m/s)", "velocity_rmse_mps", 6),
    )
    lines = [
        "# EuRoC Final Unified Results", "",
        "All values come from one fresh deterministic five-mode replay. ATE uses",
        "per-mode SE(3) position alignment; Rotation and Velocity share the",
        "per-sequence Baseline alignment rotation. All metrics use the five-mode",
        "common timestamp support. `Delta = (Method - B) / B * 100%`; negative is",
        "improvement.", "",
    ]
    header = (
        "| Sequence | B | G_gate | Delta vs B | V_fixed | Delta vs B | "
        "G_gate+V_fixed | Delta vs B | G_gate+V_gate | Delta vs B |")
    separator = "|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|"
    for title, metric, precision in metric_sections:
        lines.extend([f"## {title}", "", header, separator])
        for sequence in audit["sequence_results"]:
            values = {
                mode: sequence["metrics"][mode][metric] for mode in MODES}
            lines.append(table_row(sequence["sequence"], values, precision))
        lines.append(table_row(
            "Easy/Medium Mean", group_means(audit, "easy_medium", metric),
            precision))
        lines.append(table_row(
            "Difficult Mean", group_means(audit, "difficult", metric),
            precision))
        lines.append("")
    return "\n".join(lines).rstrip() + "\n"


def write_artifacts(output_root, audit):
    atomic_write(
        output_root / "experiment_manifest.json",
        json.dumps(audit, indent=2, sort_keys=True, allow_nan=False) + "\n")
    atomic_write(
        output_root / "run_validation.jsonl",
        jsonl_text(audit["validation_rows"]))
    atomic_write(
        output_root / "run_validation.csv",
        csv_text(audit["validation_rows"]))
    atomic_write(
        output_root / "metrics.jsonl", jsonl_text(metric_rows(audit)))
    atomic_write(
        output_root / "euroc_final_results.md", render_markdown(audit))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--results-root", required=True, type=Path)
    parser.add_argument("--cache-root", required=True, type=Path)
    parser.add_argument("--markdown-output", type=Path)
    arguments = parser.parse_args()

    audit = evaluate(arguments.results_root, arguments.cache_root)
    write_artifacts(arguments.results_root, audit)
    if arguments.markdown_output is not None:
        atomic_write(arguments.markdown_output, render_markdown(audit))
    print(json.dumps({
        "run_count": audit["run_count"],
        "total_consumed_pair_count": audit["total_consumed_pair_count"],
        "results_root": str(arguments.results_root.resolve()),
    }, sort_keys=True))


if __name__ == "__main__":
    main()
