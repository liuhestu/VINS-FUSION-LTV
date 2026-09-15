#!/usr/bin/env python3

"""Compare passive LTV CSV output with EuRoC body-frame ground truth."""

import argparse
import csv
import math

import numpy as np


def quaternion_to_rotation(x, y, z, w):
    quaternion = np.array([w, x, y, z], dtype=float)
    quaternion /= np.linalg.norm(quaternion)
    w, x, y, z = quaternion
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def read_groundtruth(csv_path):
    times = []
    rotations = []
    velocities = []
    with open(csv_path, newline="", encoding="utf-8") as stream:
        rows = csv.reader(line for line in stream if not line.startswith("#"))
        for row in rows:
            if len(row) < 11:
                continue
            times.append(float(row[0]) * 1e-9)
            rotations.append(quaternion_to_rotation(
                float(row[5]), float(row[6]), float(row[7]), float(row[4])))
            velocities.append([float(row[8]), float(row[9]), float(row[10])])
    times = np.asarray(times)
    rotations = np.asarray(rotations)
    velocities = np.asarray(velocities)
    return times, rotations, velocities


def read_ltv(csv_path):
    samples = []
    with open(csv_path, newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            try:
                if row["valid"] != "1":
                    continue
                samples.append((
                    float(row["frame_timestamp"]),
                    np.array([float(row["velocity_body_x"]),
                              float(row["velocity_body_y"]),
                              float(row["velocity_body_z"])]),
                    np.array([float(row["gravity_body_x"]),
                              float(row["gravity_body_y"]),
                              float(row["gravity_body_z"])]),
                ))
            except (KeyError, TypeError, ValueError):
                # A process interrupted during a write may leave one partial row.
                continue
    return samples


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("groundtruth_csv",
                        help="EuRoC state_groundtruth_estimate0/data.csv")
    parser.add_argument("ltv_csv")
    parser.add_argument("--gravity", type=float, default=9.81007)
    parser.add_argument("--max-time-error", type=float, default=0.02)
    args = parser.parse_args()

    gt_times, gt_rotations, gt_velocities = read_groundtruth(args.groundtruth_csv)
    ltv_samples = read_ltv(args.ltv_csv)
    velocity_squared_errors = []
    gravity_angle_squared_errors = []
    matched_time_errors = []
    gravity_world = np.array([0.0, 0.0, -args.gravity])

    for timestamp, estimated_velocity, estimated_gravity in ltv_samples:
        insertion = int(np.searchsorted(gt_times, timestamp))
        candidates = [index for index in (insertion - 1, insertion)
                      if 0 <= index < len(gt_times)]
        if not candidates:
            continue
        index = min(candidates, key=lambda candidate: abs(gt_times[candidate] - timestamp))
        time_error = abs(gt_times[index] - timestamp)
        if time_error > args.max_time_error:
            continue

        rotation_world_body = gt_rotations[index]
        velocity_body_gt = rotation_world_body.T @ gt_velocities[index]
        gravity_body_gt = rotation_world_body.T @ gravity_world
        velocity_squared_errors.append(np.sum((estimated_velocity - velocity_body_gt) ** 2))
        denominator = np.linalg.norm(estimated_gravity) * np.linalg.norm(gravity_body_gt)
        if denominator > 1e-12:
            cosine = np.clip(np.dot(estimated_gravity, gravity_body_gt) / denominator, -1.0, 1.0)
            gravity_angle_squared_errors.append(math.acos(cosine) ** 2)
        matched_time_errors.append(time_error)

    if not velocity_squared_errors or not gravity_angle_squared_errors:
        raise RuntimeError("no valid timestamp-matched LTV/ground-truth samples")

    velocity_rmse = math.sqrt(float(np.mean(velocity_squared_errors)))
    gravity_angle_rmse = math.sqrt(float(np.mean(gravity_angle_squared_errors)))
    print(f"matched_samples: {len(velocity_squared_errors)}")
    print(f"maximum_timestamp_error_s: {max(matched_time_errors):.9f}")
    print(f"velocity_body_rmse_mps: {velocity_rmse:.6f}")
    print(f"gravity_direction_rmse_deg: {math.degrees(gravity_angle_rmse):.6f}")


if __name__ == "__main__":
    main()
