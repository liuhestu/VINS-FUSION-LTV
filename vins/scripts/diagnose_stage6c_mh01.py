#!/usr/bin/env python3
"""Diagnose the Stage 6b MH_01 pitch regression without retuning gates."""

import argparse
import csv
import json
import math
import os
import uuid
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from evaluate_vins_euroc import (
    match_nearest, read_official_ground_truth, read_vins,
    rigid_alignment, rotation_error_rpy)


MODES = ("baseline", "gravity_only", "joint_v_gate")
PITCH_WINDOW_S = 1.0
PITCH_DEGRADATION_RATIO = 1.03
PITCH_SUSTAIN_S = 1.0
LONG_GATE_TIERS_S = (1.0, 2.0, 5.0)


def to_timestamp_ns(value):
    return int(round(float(value) * 1e9))


def trailing_rmse(times, values, window_s=PITCH_WINDOW_S):
    times = np.asarray(times, dtype=float)
    values = np.asarray(values, dtype=float)
    result = np.full(len(values), np.nan)
    for index, timestamp in enumerate(times):
        if timestamp - times[0] + 1e-9 < window_s:
            continue
        first = int(np.searchsorted(times, timestamp - window_s, side="left"))
        sample = values[first:index + 1]
        if len(sample):
            result[index] = math.sqrt(float(np.mean(sample * sample)))
    return result


def find_sustained_onset(times, condition, sustain_s=PITCH_SUSTAIN_S):
    times = np.asarray(times, dtype=float)
    condition = np.asarray(condition, dtype=bool)
    for index, active in enumerate(condition):
        if not active:
            continue
        stop = int(np.searchsorted(times, times[index] + sustain_s,
                                   side="left"))
        if stop < len(times) and np.all(condition[index:stop + 1]):
            return index
    return None


def gate_segments(times, passes):
    times = np.asarray(times, dtype=float)
    passes = np.asarray(passes, dtype=bool)
    positive_steps = np.diff(times)
    positive_steps = positive_steps[positive_steps > 0.0]
    frame_dt = float(np.median(positive_steps)) if len(positive_steps) else 0.05
    segments = []
    start = None
    for index, active in enumerate(passes):
        if active and start is None:
            start = index
        if start is not None and (not active or index == len(passes) - 1):
            stop = index if active and index == len(passes) - 1 else index - 1
            frames = stop - start + 1
            segments.append({
                "start_index": start,
                "stop_index": stop,
                "frames": frames,
                "duration_s": frames * frame_dt,
            })
            start = None
    return segments


def body_velocity_residual(rotation_world_body, velocity_world,
                           velocity_ltv_body):
    return rotation_world_body.T @ velocity_world - velocity_ltv_body


def aggregate_imu(camera_timestamps_ns, imu_timestamps_ns,
                  accelerations, angular_velocities):
    camera_timestamps_ns = np.asarray(camera_timestamps_ns, dtype=np.int64)
    imu_timestamps_ns = np.asarray(imu_timestamps_ns, dtype=np.int64)
    accelerations = np.asarray(accelerations, dtype=float)
    angular_velocities = np.asarray(angular_velocities, dtype=float)
    if len(camera_timestamps_ns) < 2:
        raise ValueError("at least two camera timestamps are required")
    nominal_step = int(np.median(np.diff(camera_timestamps_ns)))
    result = []
    previous = camera_timestamps_ns[0] - nominal_step
    for timestamp in camera_timestamps_ns:
        first = int(np.searchsorted(imu_timestamps_ns, previous, side="right"))
        stop = int(np.searchsorted(imu_timestamps_ns, timestamp, side="right"))
        if stop <= first:
            raise RuntimeError(f"no IMU samples for camera timestamp {timestamp}")
        acceleration = accelerations[first:stop]
        angular_velocity = angular_velocities[first:stop]
        accel_norm = np.linalg.norm(acceleration, axis=1)
        omega_norm = np.linalg.norm(angular_velocity, axis=1)
        result.append({
            "imu_sample_count": stop - first,
            "accel_mean_x": float(np.mean(acceleration[:, 0])),
            "accel_mean_y": float(np.mean(acceleration[:, 1])),
            "accel_mean_z": float(np.mean(acceleration[:, 2])),
            "accel_norm_rms": float(math.sqrt(np.mean(accel_norm ** 2))),
            "accel_norm_max": float(np.max(accel_norm)),
            "accel_dynamic_mean": float(np.mean(np.abs(accel_norm - 9.81))),
            "accel_dynamic_max": float(np.max(np.abs(accel_norm - 9.81))),
            "omega_mean_x": float(np.mean(angular_velocity[:, 0])),
            "omega_mean_y": float(np.mean(angular_velocity[:, 1])),
            "omega_mean_z": float(np.mean(angular_velocity[:, 2])),
            "omega_norm_rms": float(math.sqrt(np.mean(omega_norm ** 2))),
            "omega_norm_max": float(np.max(omega_norm)),
        })
        previous = timestamp
    return result


def read_ltv(path):
    rows = {}
    with path.open(encoding="utf-8", newline="") as stream:
        reader = csv.DictReader(stream)
        required = {
            "frame_timestamp", "velocity_body_x", "velocity_body_y",
            "velocity_body_z", "velocity_factor_residual_norm",
            "gravity_angle_ltv_vs_vins_deg", "velocity_factor_added",
            "velocity_gate_pass"}
        if reader.fieldnames is None or not required.issubset(reader.fieldnames):
            raise RuntimeError(f"{path}: missing Stage 6c LTV fields")
        for line_number, row in enumerate(reader, start=2):
            timestamp = float(row["frame_timestamp"])
            if timestamp == 0.0:
                continue
            timestamp_ns = to_timestamp_ns(timestamp)
            if timestamp_ns in rows:
                raise RuntimeError(f"{path}:{line_number}: duplicate timestamp")
            rows[timestamp_ns] = {
                "velocity_body": np.asarray([
                    float(row["velocity_body_x"]),
                    float(row["velocity_body_y"]),
                    float(row["velocity_body_z"])]),
                "velocity_disagreement_preopt": float(
                    row["velocity_factor_residual_norm"]),
                "gravity_disagreement_deg": float(
                    row["gravity_angle_ltv_vs_vins_deg"]),
                "velocity_factor_added": int(row["velocity_factor_added"]),
                "velocity_gate_pass": int(row["velocity_gate_pass"]),
            }
    return rows


def read_imu(path):
    timestamps = []
    acceleration = []
    angular_velocity = []
    with path.open(encoding="utf-8", newline="") as stream:
        reader = csv.DictReader(stream)
        for row in reader:
            timestamps.append(int(row["timestamp_ns"]))
            acceleration.append([float(row[name]) for name in ("ax", "ay", "az")])
            angular_velocity.append(
                [float(row[name]) for name in ("gx", "gy", "gz")])
    return (np.asarray(timestamps, dtype=np.int64),
            np.asarray(acceleration), np.asarray(angular_velocity))


def trajectory_series(path, ground_truth, common_alignment=None):
    timestamps, positions, rotations, velocities = read_vins(path)
    vins_indices, gt_indices, _ = match_nearest(
        timestamps, ground_truth["timestamps"], 0.02)
    if len(vins_indices) < 3:
        raise RuntimeError(f"{path}: fewer than three GT-matched samples")
    individual_alignment, _ = rigid_alignment(
        positions[vins_indices], ground_truth["positions"][gt_indices])
    alignment = (individual_alignment if common_alignment is None
                 else common_alignment)
    pitch = np.full(len(timestamps), np.nan)
    pitch_individual = np.full(len(timestamps), np.nan)
    for vins_index, gt_index in zip(vins_indices, gt_indices):
        delta = (ground_truth["rotations"][gt_index].T @ alignment @
                 rotations[vins_index])
        pitch[vins_index] = math.degrees(rotation_error_rpy(delta)[1])
        individual_delta = (
            ground_truth["rotations"][gt_index].T @ individual_alignment @
            rotations[vins_index])
        pitch_individual[vins_index] = math.degrees(
            rotation_error_rpy(individual_delta)[1])
    return {
        "timestamps_ns": np.asarray(
            [to_timestamp_ns(value) for value in timestamps],
            dtype=np.int64),
        "pitch_error_deg": pitch,
        "pitch_error_individual_alignment_deg": pitch_individual,
        "pitch_valid_count": len(vins_indices),
        "alignment_rotation": alignment,
        "individual_alignment_rotation": individual_alignment,
        "rotations": rotations,
        "velocities_world": velocities,
    }


def finite_summary(values):
    values = np.asarray(values, dtype=float)
    values = values[np.isfinite(values)]
    if not len(values):
        return None
    return {
        "mean": float(np.mean(values)),
        "p95": float(np.percentile(values, 95)),
        "max": float(np.max(values)),
    }


def safe_number(value):
    value = float(value)
    return value if math.isfinite(value) else None


def range_summary(times, values, lower, upper):
    mask = (times >= lower) & (times <= upper)
    return finite_summary(np.asarray(values)[mask])


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--baseline", required=True, type=Path)
    parser.add_argument("--gravity-only", required=True, type=Path)
    parser.add_argument("--joint", required=True, type=Path)
    parser.add_argument("--cache", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    arguments = parser.parse_args()

    if arguments.output.exists():
        raise RuntimeError(f"refusing to overwrite Stage 6c output: {arguments.output}")
    metadata = json.loads((arguments.cache / "metadata.json").read_text(
        encoding="utf-8"))
    if metadata.get("format") != "ltv-stage5-cache-v2":
        raise RuntimeError("Stage 6c requires canonical v2 cache")
    ground_truth = read_official_ground_truth(metadata["ground_truth"])
    directories = {
        "baseline": arguments.baseline,
        "gravity_only": arguments.gravity_only,
        "joint_v_gate": arguments.joint,
    }
    trajectories = {
        "baseline": trajectory_series(
            directories["baseline"] / "vio.csv", ground_truth)}
    baseline_alignment = trajectories["baseline"]["alignment_rotation"]
    for mode in MODES[1:]:
        trajectories[mode] = trajectory_series(
            directories[mode] / "vio.csv", ground_truth, baseline_alignment)
    reference_timestamps = trajectories["baseline"]["timestamps_ns"]
    for mode in MODES[1:]:
        if not np.array_equal(
                reference_timestamps, trajectories[mode]["timestamps_ns"]):
            raise RuntimeError("B/G-only/Joint GT-matched time grids differ")

    ltv = {mode: read_ltv(directory / "ltv_debug.csv")
           for mode, directory in directories.items()}
    keep = np.asarray([
        all(timestamp in ltv[mode] for mode in MODES)
        for timestamp in reference_timestamps], dtype=bool)
    if not np.any(keep):
        raise RuntimeError("no VIO timestamp has snapshots in all three modes")
    for mode in MODES:
        for name in ("timestamps_ns", "pitch_error_deg",
                     "pitch_error_individual_alignment_deg", "rotations",
                     "velocities_world"):
            trajectories[mode][name] = trajectories[mode][name][keep]
    reference_timestamps = reference_timestamps[keep]

    times = (reference_timestamps - reference_timestamps[0]) * 1e-9
    pitch = {mode: trajectories[mode]["pitch_error_deg"] for mode in MODES}
    rolling = {mode: trailing_rmse(times, pitch[mode]) for mode in MODES}
    degradation = rolling["joint_v_gate"] > (
        PITCH_DEGRADATION_RATIO * rolling["baseline"])
    degradation &= np.isfinite(rolling["baseline"])
    onset_index = find_sustained_onset(times, degradation)
    pitch_delta_joint_baseline = (
        rolling["joint_v_gate"] - rolling["baseline"])
    if not np.any(np.isfinite(pitch_delta_joint_baseline)):
        raise RuntimeError("no finite rolling Pitch comparison")
    peak_index = int(np.nanargmax(pitch_delta_joint_baseline))

    gate_pass = np.asarray([
        ltv["joint_v_gate"][timestamp]["velocity_gate_pass"]
        for timestamp in reference_timestamps], dtype=bool)
    factor_added = np.asarray([
        ltv["joint_v_gate"][timestamp]["velocity_factor_added"]
        for timestamp in reference_timestamps], dtype=bool)
    if not np.array_equal(gate_pass, factor_added):
        raise RuntimeError("Joint V_gate pass/factor-added grids differ")
    validation = json.loads((arguments.joint / "validation.json").read_text(
        encoding="utf-8"))
    if int(np.sum(gate_pass)) != validation["velocity_gate_pass_count"]:
        raise RuntimeError("gate segment grid does not cover every V_gate pass")

    segments = gate_segments(times, gate_pass)
    segment_ids = np.full(len(times), -1, dtype=int)
    for segment_id, segment in enumerate(segments):
        segment_ids[segment["start_index"]:segment["stop_index"] + 1] = segment_id

    post_residual = {}
    for mode in MODES:
        values = []
        for index, timestamp in enumerate(reference_timestamps):
            values.append(body_velocity_residual(
                trajectories[mode]["rotations"][index],
                trajectories[mode]["velocities_world"][index],
                ltv[mode][timestamp]["velocity_body"]))
        post_residual[mode] = np.asarray(values)

    velocity_disagreement = np.asarray([
        ltv["joint_v_gate"][timestamp]["velocity_disagreement_preopt"]
        for timestamp in reference_timestamps])
    gravity_disagreement = np.asarray([
        ltv["joint_v_gate"][timestamp]["gravity_disagreement_deg"]
        for timestamp in reference_timestamps])

    imu_timestamps, accelerations, angular_velocities = read_imu(
        arguments.cache / "imu.csv")
    imu_aggregate = aggregate_imu(
        reference_timestamps, imu_timestamps, accelerations,
        angular_velocities)

    camera_rows = []
    for index, timestamp in enumerate(reference_timestamps):
        row = {
            "timestamp_ns": int(timestamp),
            "relative_time_s": float(times[index]),
            "pitch_error_common_alignment_baseline_deg": safe_number(
                pitch["baseline"][index]),
            "pitch_error_common_alignment_gravity_only_deg": safe_number(
                pitch["gravity_only"][index]),
            "pitch_error_common_alignment_joint_deg": safe_number(
                pitch["joint_v_gate"][index]),
            "pitch_error_individual_alignment_baseline_deg": safe_number(
                trajectories["baseline"][
                    "pitch_error_individual_alignment_deg"][index]),
            "pitch_error_individual_alignment_gravity_only_deg": safe_number(
                trajectories["gravity_only"][
                    "pitch_error_individual_alignment_deg"][index]),
            "pitch_error_individual_alignment_joint_deg": safe_number(
                trajectories["joint_v_gate"][
                    "pitch_error_individual_alignment_deg"][index]),
            "pitch_rolling_rmse_baseline_deg": safe_number(
                rolling["baseline"][index]),
            "pitch_rolling_rmse_gravity_only_deg": safe_number(
                rolling["gravity_only"][index]),
            "pitch_rolling_rmse_joint_deg": safe_number(
                rolling["joint_v_gate"][index]),
            "pitch_rolling_delta_joint_baseline_deg": safe_number(
                pitch_delta_joint_baseline[index]),
            "pitch_rolling_delta_gravity_baseline_deg": safe_number(
                rolling["gravity_only"][index] - rolling["baseline"][index]),
            "pitch_rolling_delta_joint_gravity_deg": safe_number(
                rolling["joint_v_gate"][index] - rolling["gravity_only"][index]),
            "velocity_gate_pass": int(gate_pass[index]),
            "velocity_factor_added": int(factor_added[index]),
            "velocity_gate_segment_id": int(segment_ids[index]),
            "velocity_disagreement_preopt_mps": float(
                velocity_disagreement[index]),
            "gravity_disagreement_deg": float(gravity_disagreement[index]),
        }
        for mode in MODES:
            residual = post_residual[mode][index]
            prefix = f"velocity_residual_postopt_{mode}"
            row.update({
                f"{prefix}_x_mps": float(residual[0]),
                f"{prefix}_y_mps": float(residual[1]),
                f"{prefix}_z_mps": float(residual[2]),
                f"{prefix}_norm_mps": float(np.linalg.norm(residual)),
            })
        row.update(imu_aggregate[index])
        camera_rows.append(row)

    segment_rows = []
    for segment_id, segment in enumerate(segments):
        start = segment["start_index"]
        stop = segment["stop_index"]
        lower = times[start]
        upper = times[stop]
        indices = slice(start, stop + 1)
        row = {
            "segment_id": segment_id,
            "start_relative_time_s": float(lower),
            "end_relative_time_s": float(upper),
            "frames": segment["frames"],
            "duration_s": segment["duration_s"],
            "at_least_1s": int(segment["duration_s"] >= 1.0),
            "at_least_2s": int(segment["duration_s"] >= 2.0),
            "at_least_5s": int(segment["duration_s"] >= 5.0),
            "pitch_delta_during_mean_deg": safe_number(
                np.nanmean(pitch_delta_joint_baseline[indices])),
            "pitch_delta_during_max_deg": safe_number(
                np.nanmax(pitch_delta_joint_baseline[indices])),
            "pitch_delta_pre2s_mean_deg": (
                range_summary(times, pitch_delta_joint_baseline,
                              lower - 2.0, lower) or {}).get("mean"),
            "pitch_delta_post2s_mean_deg": (
                range_summary(times, pitch_delta_joint_baseline,
                              upper, upper + 2.0) or {}).get("mean"),
        }
        for name, values in (
                ("omega_rms", [item["omega_norm_rms"] for item in imu_aggregate]),
                ("accel_dynamic", [item["accel_dynamic_mean"] for item in imu_aggregate]),
                ("velocity_disagreement_preopt", velocity_disagreement),
                ("gravity_disagreement", gravity_disagreement),
                ("velocity_residual_postopt_joint",
                 np.linalg.norm(post_residual["joint_v_gate"], axis=1))):
            summary = finite_summary(np.asarray(values)[indices])
            for statistic, value in summary.items():
                row[f"{name}_{statistic}"] = value
        segment_rows.append(row)

    def event_summary(index):
        if index is None:
            return None
        segment_id = int(segment_ids[index])
        result = {
            "timestamp_ns": int(reference_timestamps[index]),
            "relative_time_s": float(times[index]),
            "pitch_rolling_rmse_baseline_deg": safe_number(
                rolling["baseline"][index]),
            "pitch_rolling_rmse_gravity_only_deg": safe_number(
                rolling["gravity_only"][index]),
            "pitch_rolling_rmse_joint_deg": safe_number(
                rolling["joint_v_gate"][index]),
            "pitch_delta_joint_baseline_deg": safe_number(
                pitch_delta_joint_baseline[index]),
            "velocity_gate_pass": int(gate_pass[index]),
            "velocity_gate_segment_id": segment_id,
            "velocity_disagreement_preopt_mps": float(
                velocity_disagreement[index]),
            "gravity_disagreement_deg": float(gravity_disagreement[index]),
            "velocity_residual_postopt_joint_mps": float(
                np.linalg.norm(post_residual["joint_v_gate"][index])),
            "omega_norm_rms": imu_aggregate[index]["omega_norm_rms"],
            "accel_norm_rms": imu_aggregate[index]["accel_norm_rms"],
            "accel_dynamic_mean": imu_aggregate[index]["accel_dynamic_mean"],
        }
        for window in LONG_GATE_TIERS_S:
            first = int(np.searchsorted(times, times[index] - window,
                                       side="left"))
            result[f"velocity_gate_duty_previous_{int(window)}s"] = float(
                np.mean(gate_pass[first:index + 1]))
        if segment_id >= 0:
            segment = segments[segment_id]
            result["current_gate_segment_age_s"] = float(
                times[index] - times[segment["start_index"]])
            result["current_gate_segment_duration_s"] = float(
                segment["duration_s"])
        else:
            result["current_gate_segment_age_s"] = None
            result["current_gate_segment_duration_s"] = None
        return result

    tier_counts = {
        f"at_least_{int(tier)}s": sum(
            segment["duration_s"] >= tier for segment in segments)
        for tier in LONG_GATE_TIERS_S}
    longest_segment_id = max(
        range(len(segments)), key=lambda item: segments[item]["duration_s"])
    pitch_rmse = {
        mode: float(math.sqrt(np.nanmean(pitch[mode] ** 2))) for mode in MODES}
    pitch_rmse_individual = {
        mode: float(math.sqrt(np.nanmean(
            trajectories[mode]["pitch_error_individual_alignment_deg"] ** 2)))
        for mode in MODES}
    pitch_mean = {
        mode: float(np.nanmean(pitch[mode])) for mode in MODES}
    pitch_mean_individual = {
        mode: float(np.nanmean(
            trajectories[mode]["pitch_error_individual_alignment_deg"]))
        for mode in MODES}
    alignment_delta = {}
    for mode in MODES[1:]:
        rotation = (trajectories[mode]["individual_alignment_rotation"] @
                    baseline_alignment.T)
        cosine = float(np.clip((np.trace(rotation) - 1.0) * 0.5, -1.0, 1.0))
        alignment_delta[mode] = {
            "angle_deg": math.degrees(math.acos(cosine)),
            "roll_pitch_yaw_deg": [
                float(value) for value in np.degrees(rotation_error_rpy(rotation))],
        }
    summary = {
        "format": "ltv-stage6c-mh01-diagnosis-v1",
        "sequence": arguments.cache.name,
        "time_origin_timestamp_ns": int(reference_timestamps[0]),
        "common_camera_samples": len(reference_timestamps),
        "pitch_valid_samples": int(np.sum(np.isfinite(pitch["baseline"]))),
        "pitch_definition": {
            "rolling_window_s": PITCH_WINDOW_S,
            "degradation_ratio": PITCH_DEGRADATION_RATIO,
            "sustain_s": PITCH_SUSTAIN_S,
        },
        "pitch_rmse_common_baseline_alignment_deg": pitch_rmse,
        "pitch_rmse_individual_stage6b_alignment_deg": pitch_rmse_individual,
        "pitch_signed_mean_common_baseline_alignment_deg": pitch_mean,
        "pitch_signed_mean_individual_stage6b_alignment_deg":
            pitch_mean_individual,
        "individual_alignment_rotation_delta_from_baseline": alignment_delta,
        "pitch_relative_change_percent_individual_stage6b_alignment": {
            "gravity_only_vs_baseline":
                (pitch_rmse_individual["gravity_only"] /
                 pitch_rmse_individual["baseline"] - 1.0) * 100.0,
            "joint_vs_baseline":
                (pitch_rmse_individual["joint_v_gate"] /
                 pitch_rmse_individual["baseline"] - 1.0) * 100.0,
            "joint_vs_gravity_only":
                (pitch_rmse_individual["joint_v_gate"] /
                 pitch_rmse_individual["gravity_only"] - 1.0) * 100.0,
        },
        "pitch_degradation_onset": event_summary(onset_index),
        "pitch_degradation_peak": event_summary(peak_index),
        "velocity_gate": {
            "pass_count": int(np.sum(gate_pass)),
            "segment_count": len(segments),
            "tier_counts": tier_counts,
            "longest_segment_id": longest_segment_id,
            "longest_segment": segment_rows[longest_segment_id],
            "gate_on_pitch_delta_summary": finite_summary(
                pitch_delta_joint_baseline[gate_pass]),
            "gate_off_pitch_delta_summary": finite_summary(
                pitch_delta_joint_baseline[~gate_pass]),
        },
    }

    partial = arguments.output.parent / (
        arguments.output.name + ".partial-" + uuid.uuid4().hex)
    partial.mkdir(parents=True)
    try:
        with (partial / "mh01_camera_timeseries.csv").open(
                "w", encoding="utf-8", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(camera_rows[0]))
            writer.writeheader()
            writer.writerows(camera_rows)
        with (partial / "mh01_gate_segments.csv").open(
                "w", encoding="utf-8", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(segment_rows[0]))
            writer.writeheader()
            writer.writerows(segment_rows)
        (partial / "mh01_diagnosis.json").write_text(
            json.dumps(summary, indent=2, sort_keys=True, allow_nan=False) + "\n",
            encoding="utf-8")

        post_joint_norm = np.linalg.norm(post_residual["joint_v_gate"], axis=1)
        omega_rms = np.asarray([item["omega_norm_rms"] for item in imu_aggregate])
        accel_dynamic = np.asarray(
            [item["accel_dynamic_mean"] for item in imu_aggregate])
        fig, axes = plt.subplots(7, 1, figsize=(16, 18), sharex=True)
        for mode, label in (("baseline", "B"), ("gravity_only", "B+G_gate"),
                            ("joint_v_gate", "B+G_gate+V_gate")):
            axes[0].plot(times, rolling[mode], label=label, linewidth=1.0)
        axes[0].set_ylabel("Pitch rolling RMSE (deg)")
        axes[0].legend(loc="upper right")
        axes[1].plot(times, pitch_delta_joint_baseline, label="Joint-B")
        axes[1].plot(times, rolling["joint_v_gate"] - rolling["gravity_only"],
                     label="Joint-G-only")
        axes[1].axhline(0.0, color="black", linewidth=0.5)
        axes[1].set_ylabel("Pitch RMSE delta (deg)")
        axes[1].legend(loc="upper right")
        axes[2].step(times, gate_pass.astype(int), where="post", color="tab:green")
        axes[2].set_ylabel("V_gate")
        axes[3].plot(times, velocity_disagreement, label="pre-opt disagreement")
        axes[3].plot(times, post_joint_norm, label="post-opt residual", alpha=0.8)
        axes[3].axhline(0.5, color="red", linestyle="--", linewidth=0.8)
        axes[3].set_ylabel("Velocity (m/s)")
        axes[3].legend(loc="upper right")
        axes[4].plot(times, gravity_disagreement, color="tab:purple")
        axes[4].set_ylabel("G disagreement (deg)")
        axes[5].plot(times, omega_rms, color="tab:orange")
        axes[5].set_ylabel("omega RMS (rad/s)")
        axes[6].plot(times, accel_dynamic, color="tab:brown")
        axes[6].set_ylabel("abs(|a|-g) (m/s2)")
        axes[6].set_xlabel("Time from first common VIO output (s)")
        for axis in axes:
            if onset_index is not None:
                axis.axvline(times[onset_index], color="red", linestyle=":",
                             linewidth=1.0)
            axis.axvline(times[peak_index], color="black", linestyle=":",
                         linewidth=0.8)
            axis.grid(True, alpha=0.2)
        fig.suptitle("MH_01 Stage 6c (Pitch time series use common Baseline alignment)")
        fig.tight_layout()
        fig.savefig(partial / "mh01_overview.png", dpi=160)
        plt.close(fig)

        focus = []
        if onset_index is not None:
            focus.append(("Pitch degradation onset",
                          times[onset_index] - 5.0,
                          times[onset_index] + 10.0,
                          (times[onset_index],)))
        longest = segments[longest_segment_id]
        longest_start = times[longest["start_index"]]
        longest_stop = times[longest["stop_index"]]
        focus.append(("Longest V_gate-on segment",
                      longest_start - 2.0, longest_stop + 2.0,
                      (longest_start, longest_stop)))
        fig, axes = plt.subplots(5, len(focus), figsize=(8 * len(focus), 13),
                                 squeeze=False)
        for column, (title, lower, upper, markers) in enumerate(focus):
            mask = (times >= lower) & (times <= upper)
            axes[0, column].plot(times[mask], rolling["baseline"][mask], label="B")
            axes[0, column].plot(times[mask], rolling["gravity_only"][mask],
                                 label="G-only")
            axes[0, column].plot(times[mask], rolling["joint_v_gate"][mask],
                                 label="Joint")
            axes[0, column].legend()
            axes[0, column].set_title(title)
            axes[0, column].set_ylabel("Pitch RMSE (deg)")
            axes[1, column].step(times[mask], gate_pass[mask].astype(int), where="post")
            axes[1, column].set_ylabel("V_gate")
            axes[2, column].plot(times[mask], velocity_disagreement[mask],
                                 label="pre-opt")
            axes[2, column].plot(times[mask], post_joint_norm[mask], label="post-opt")
            axes[2, column].legend()
            axes[2, column].set_ylabel("Velocity (m/s)")
            axes[3, column].plot(times[mask], gravity_disagreement[mask],
                                 color="tab:purple")
            axes[3, column].set_ylabel("G disagreement (deg)")
            axes[4, column].plot(times[mask], omega_rms[mask], label="omega RMS")
            axes[4, column].plot(times[mask], accel_dynamic[mask],
                                 label="abs(|a|-g)")
            axes[4, column].legend()
            axes[4, column].set_ylabel("Motion")
            axes[4, column].set_xlabel("Time (s)")
            for row in range(5):
                for marker in markers:
                    axes[row, column].axvline(
                        marker, color="red", linestyle=":")
                axes[row, column].grid(True, alpha=0.2)
        fig.suptitle("Pitch time series use common Baseline alignment")
        fig.tight_layout()
        fig.savefig(partial / "mh01_focus.png", dpi=160)
        plt.close(fig)
        os.rename(partial, arguments.output)
    except Exception:
        if partial.exists():
            failed = arguments.output.parent / (
                arguments.output.name + ".failed-" + uuid.uuid4().hex)
            os.rename(partial, failed)
        raise
    print(arguments.output)


if __name__ == "__main__":
    main()
