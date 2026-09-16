#!/usr/bin/env python3
"""Build and freeze a Stage 5 Oracle mask from one passive reference run."""

import argparse
import bisect
import csv
import hashlib
import json
import math
import os
from pathlib import Path


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def rotate_world_to_body(quaternion, velocity):
    qw, qx, qy, qz = quaternion
    norm = math.sqrt(qw * qw + qx * qx + qy * qy + qz * qz)
    if not math.isfinite(norm) or norm <= 1e-12:
        raise RuntimeError("ground-truth quaternion is invalid")
    qw, qx, qy, qz = (value / norm for value in quaternion)
    vx, vy, vz = velocity
    # R(q)^T v, where q is the EuRoC body-to-world orientation.
    return (
        (1 - 2 * (qy * qy + qz * qz)) * vx
        + 2 * (qx * qy + qw * qz) * vy
        + 2 * (qx * qz - qw * qy) * vz,
        2 * (qx * qy - qw * qz) * vx
        + (1 - 2 * (qx * qx + qz * qz)) * vy
        + 2 * (qy * qz + qw * qx) * vz,
        2 * (qx * qz + qw * qy) * vx
        + 2 * (qy * qz - qw * qx) * vy
        + (1 - 2 * (qx * qx + qy * qy)) * vz,
    )


def read_ground_truth(path):
    with path.open(encoding="utf-8", newline="") as stream:
        reader = csv.DictReader(stream)
        if reader.fieldnames is None:
            raise RuntimeError("ground-truth CSV has no header")
        names = {name.strip().lstrip("#").split(" [", 1)[0]: name
                 for name in reader.fieldnames}
        required = (
            "timestamp", "q_RS_w", "q_RS_x", "q_RS_y", "q_RS_z",
            "v_RS_R_x", "v_RS_R_y", "v_RS_R_z")
        missing = [name for name in required if name not in names]
        if missing:
            raise RuntimeError(f"ground-truth CSV is missing: {', '.join(missing)}")
        entries = []
        for line_number, row in enumerate(reader, start=2):
            try:
                timestamp_ns = int(row[names["timestamp"]])
                quaternion = tuple(float(row[names[name]]) for name in required[1:5])
                velocity = tuple(float(row[names[name]]) for name in required[5:8])
                velocity_body = rotate_world_to_body(quaternion, velocity)
            except (TypeError, ValueError) as error:
                raise RuntimeError(
                    f"{path}:{line_number}: invalid ground-truth row") from error
            if (not all(math.isfinite(value) for value in velocity_body)
                    or (entries and timestamp_ns <= entries[-1][0])):
                raise RuntimeError("ground-truth timestamps/state are invalid")
            entries.append((timestamp_ns, velocity_body))
    if not entries:
        raise RuntimeError("ground-truth CSV is empty")
    return entries


def vector(row, prefix):
    values = tuple(float(row[f"{prefix}_{axis}"]) for axis in "xyz")
    if not all(math.isfinite(value) for value in values):
        raise RuntimeError(f"reference CSV contains non-finite {prefix}")
    return values


def distance(left, right):
    return math.sqrt(sum((a - b) ** 2 for a, b in zip(left, right)))


def nearest_ground_truth(entries, timestamps, timestamp_ns, tolerance_ns):
    upper = bisect.bisect_left(timestamps, timestamp_ns)
    candidates = []
    if upper < len(entries):
        candidates.append(entries[upper])
    if upper > 0:
        candidates.append(entries[upper - 1])
    nearest = min(candidates, key=lambda entry: abs(entry[0] - timestamp_ns))
    error_ns = abs(nearest[0] - timestamp_ns)
    return (nearest, error_ns) if error_ns <= tolerance_ns else (None, error_ns)


def build_rows(reference_path, ground_truth_path, tolerance_ns=5_000_000):
    ground_truth = read_ground_truth(ground_truth_path)
    gt_timestamps = [entry[0] for entry in ground_truth]
    rows = []
    previous_timestamp_ns = -1
    with reference_path.open(encoding="utf-8", newline="") as stream:
        reader = csv.DictReader(stream)
        required = {
            "frame_timestamp", "velocity_body_x", "velocity_body_y",
            "velocity_body_z", "vins_velocity_body_x", "vins_velocity_body_y",
            "vins_velocity_body_z", "valid", "velocity_valid", "reset_reason",
            "velocity_factor_base_eligible"}
        if reader.fieldnames is None or not required.issubset(reader.fieldnames):
            raise RuntimeError("reference CSV lacks Stage 5 velocity columns")
        for line_number, row in enumerate(reader, start=2):
            timestamp = float(row["frame_timestamp"])
            if timestamp == 0.0:
                continue
            if not math.isfinite(timestamp):
                raise RuntimeError(f"{reference_path}:{line_number}: invalid timestamp")
            timestamp_ns = round(timestamp * 1_000_000_000)
            if timestamp_ns <= previous_timestamp_ns:
                raise RuntimeError("reference snapshot timestamps are not strictly increasing")
            previous_timestamp_ns = timestamp_ns
            gt, error_ns = nearest_ground_truth(
                ground_truth, gt_timestamps, timestamp_ns, tolerance_ns)
            base_eligible = row["velocity_factor_base_eligible"] == "1"
            reference_valid = (
                row["valid"] == "1" and row["velocity_valid"] == "1"
                and row["reset_reason"] == "none")
            if gt is None:
                e_ltv = e_vins = advantage = ""
                oracle_0 = oracle_002 = False
            else:
                ltv_velocity = vector(row, "velocity_body")
                vins_velocity = vector(row, "vins_velocity_body")
                e_ltv = distance(ltv_velocity, gt[1])
                e_vins = distance(vins_velocity, gt[1])
                advantage = e_vins - e_ltv
                oracle_0 = reference_valid and e_ltv < e_vins
                oracle_002 = reference_valid and e_ltv + 0.02 < e_vins
            rows.append({
                "timestamp_ns": timestamp_ns,
                "frame_timestamp": format(timestamp, ".17g"),
                "gt_time_error_ns": error_ns,
                "gt_matched": int(gt is not None),
                "e_ltv": e_ltv,
                "e_vins": e_vins,
                "advantage": advantage,
                "reference_valid": int(reference_valid),
                "reference_base_eligible": int(base_eligible),
                "oracle_0": int(oracle_0),
                "oracle_002": int(oracle_002),
            })
    if not rows:
        raise RuntimeError("reference run produced no nonzero snapshots")
    return rows


def write_mask(output, rows, reference_path, ground_truth_path, tolerance_ns):
    if output.exists():
        raise RuntimeError(f"refusing to overwrite frozen Oracle mask: {output}")
    output.parent.mkdir(parents=True, exist_ok=True)
    partial = output.with_name(output.name + f".partial-{os.getpid()}")
    fieldnames = [
        "timestamp_ns", "frame_timestamp", "gt_time_error_ns", "gt_matched",
        "e_ltv", "e_vins", "advantage", "reference_valid",
        "reference_base_eligible", "oracle_0", "oracle_002"]
    try:
        with partial.open("x", encoding="utf-8", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(rows)
        os.replace(partial, output)
    finally:
        partial.unlink(missing_ok=True)

    metadata = {
        "format": "ltv-stage5-oracle-mask-v1",
        "reference_csv": str(reference_path.resolve()),
        "reference_csv_sha256": sha256(reference_path),
        "ground_truth": str(ground_truth_path.resolve()),
        "ground_truth_sha256": sha256(ground_truth_path),
        "max_gt_time_error_ns": tolerance_ns,
        "observed_max_matched_gt_time_error_ns": max(
            row["gt_time_error_ns"] for row in rows if row["gt_matched"]),
        "gt_match_count": sum(row["gt_matched"] for row in rows),
        "gt_miss_count": sum(not row["gt_matched"] for row in rows),
        "snapshot_count": len(rows),
        "reference_valid_count": sum(row["reference_valid"] for row in rows),
        "reference_base_eligible_count": sum(
            row["reference_base_eligible"] for row in rows),
        "oracle_0_count": sum(row["oracle_0"] for row in rows),
        "oracle_002_count": sum(row["oracle_002"] for row in rows),
        "mask_sha256": sha256(output),
    }
    metadata_path = output.with_suffix(".json")
    metadata_path.write_text(
        json.dumps(metadata, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return metadata


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--reference-ltv", required=True, type=Path)
    parser.add_argument("--ground-truth", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--max-time-error-ms", default=5.0, type=float)
    arguments = parser.parse_args()
    tolerance_ns = round(arguments.max_time_error_ms * 1_000_000)
    if tolerance_ns < 0:
        raise RuntimeError("max time error must be non-negative")
    rows = build_rows(
        arguments.reference_ltv, arguments.ground_truth, tolerance_ns)
    metadata = write_mask(
        arguments.output, rows, arguments.reference_ltv,
        arguments.ground_truth, tolerance_ns)
    print(json.dumps(metadata, sort_keys=True))


if __name__ == "__main__":
    main()
