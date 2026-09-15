#!/usr/bin/env python3

"""Summarize EuRoC ASL official velocity ground truth.

The official ``v_RS_R`` fields are the primary velocity ground truth. Position
finite differences are reported only as a CSV consistency check and never
replace the official velocity in estimator evaluation.
"""

import argparse
import csv
import json
import math
import os
import sys

import numpy as np

SCRIPT_DIRECTORY = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIRECTORY not in sys.path:
    sys.path.insert(0, SCRIPT_DIRECTORY)

from evaluate_vins_euroc import position_difference_velocity, read_official_ground_truth


def _finite_vector_rows(values):
    values = np.asarray(values, dtype=float)
    return values[np.all(np.isfinite(values), axis=1)]


def _scalar_summary(values):
    values = np.asarray(values, dtype=float)
    values = values[np.isfinite(values)]
    if not len(values):
        return None
    return {
        "count": int(len(values)),
        "min": float(np.min(values)),
        "mean": float(np.mean(values)),
        "median": float(np.median(values)),
        "p95": float(np.percentile(values, 95)),
        "max": float(np.max(values)),
        "rms": float(math.sqrt(np.mean(values ** 2))),
    }


def _vector_summary(values):
    values = _finite_vector_rows(values)
    if not len(values):
        return None
    return {
        "count": int(len(values)),
        "component_mean": [float(value) for value in np.mean(values, axis=0)],
        "component_std": [float(value) for value in np.std(values, axis=0)],
        "norm": _scalar_summary(np.linalg.norm(values, axis=1)),
    }


def _sequence_name(path):
    state_directory = os.path.dirname(path)
    mav0_directory = os.path.dirname(state_directory)
    sequence_directory = os.path.dirname(mav0_directory)
    return os.path.basename(sequence_directory)


def analyze_ground_truth(path, difference_window=0.1):
    """Analyze one official EuRoC ``state_groundtruth_estimate0/data.csv``.

    Returns a JSON-serializable summary and a time-series dictionary. The
    latter is intentionally kept separate so callers can write it to CSV
    without changing the summary schema.
    """
    if not math.isfinite(difference_window) or difference_window <= 0.0:
        raise ValueError("difference window must be positive")

    ground_truth = read_official_ground_truth(path)
    if ground_truth["velocities"] is None:
        raise RuntimeError("official ground-truth CSV has no v_RS_R velocity columns")

    times = np.asarray(ground_truth["timestamps"], dtype=float)
    positions = np.asarray(ground_truth["positions"], dtype=float)
    rotations_world_body = np.asarray(ground_truth["rotations"], dtype=float)
    velocities_world = np.asarray(ground_truth["velocities"], dtype=float)
    if len(times) < 2:
        raise RuntimeError("official ground-truth CSV needs at least two samples")

    finite_rows = (np.isfinite(times) & np.all(np.isfinite(positions), axis=1) &
                   np.all(np.isfinite(rotations_world_body), axis=(1, 2)) &
                   np.all(np.isfinite(velocities_world), axis=1))
    if not np.all(finite_rows):
        raise RuntimeError("official ground-truth CSV contains non-finite parsed values")

    time_steps = np.diff(times)
    positive_time_steps = time_steps[time_steps > 0.0]
    non_monotonic_steps = int(np.count_nonzero(time_steps <= 0.0))
    if non_monotonic_steps:
        raise RuntimeError("official ground-truth timestamps are not strictly increasing")

    velocities_body = np.einsum(
        "nji,nj->ni", rotations_world_body, velocities_world)

    accelerations_world = np.full_like(velocities_world, math.nan)
    accelerations_world[1:] = np.diff(velocities_world, axis=0) / time_steps[:, None]

    difference_velocities, difference_valid = position_difference_velocity(
        times, positions, times, difference_window)
    difference_errors = np.full(len(times), math.nan)
    difference_errors[difference_valid] = np.linalg.norm(
        velocities_world[difference_valid] - difference_velocities[difference_valid], axis=1)

    summary = {
        "sequence": _sequence_name(path),
        "ground_truth_csv": os.path.abspath(path),
        "samples": int(len(times)),
        "duration_s": float(times[-1] - times[0]),
        "timestamp": {
            "strictly_increasing": non_monotonic_steps == 0,
            "non_monotonic_steps": non_monotonic_steps,
            "dt_s": _scalar_summary(positive_time_steps),
        },
        "world_velocity_mps": _vector_summary(velocities_world),
        "body_velocity_mps": _vector_summary(velocities_body),
        "world_acceleration_mps2": _vector_summary(accelerations_world[1:]),
        "position_difference_check": {
            "difference_window_s": float(difference_window),
            "valid_samples": int(np.count_nonzero(difference_valid)),
            "official_minus_position_difference_error_mps": _scalar_summary(
                difference_errors[difference_valid]),
        },
    }
    series = {
        "timestamp_s": times,
        "velocity_world_mps": velocities_world,
        "velocity_body_mps": velocities_body,
        "speed_mps": np.linalg.norm(velocities_world, axis=1),
        "acceleration_world_mps2": accelerations_world,
        "acceleration_mps2": np.linalg.norm(accelerations_world, axis=1),
        "position_difference_velocity_world_mps": difference_velocities,
        "position_difference_error_mps": difference_errors,
    }
    return summary, series


def write_time_series(path, series):
    """Write a per-sample ASL velocity diagnostic CSV."""
    with open(path, "w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow((
            "timestamp_s", "velocity_world_x_mps", "velocity_world_y_mps",
            "velocity_world_z_mps", "velocity_body_x_mps", "velocity_body_y_mps",
            "velocity_body_z_mps", "speed_mps", "acceleration_world_x_mps2",
            "acceleration_world_y_mps2", "acceleration_world_z_mps2",
            "acceleration_mps2", "position_difference_velocity_world_x_mps",
            "position_difference_velocity_world_y_mps",
            "position_difference_velocity_world_z_mps",
            "position_difference_error_mps"))
        for index, timestamp in enumerate(series["timestamp_s"]):
            writer.writerow((
                timestamp,
                *series["velocity_world_mps"][index],
                *series["velocity_body_mps"][index],
                series["speed_mps"][index],
                *series["acceleration_world_mps2"][index],
                series["acceleration_mps2"][index],
                *series["position_difference_velocity_world_mps"][index],
                series["position_difference_error_mps"][index]))


def main():
    parser = argparse.ArgumentParser(
        description="Analyze velocity statistics in EuRoC ASL official ground truth")
    parser.add_argument("ground_truth_csv", nargs="+",
                        help="EuRoC mav0/state_groundtruth_estimate0/data.csv")
    parser.add_argument("--difference-window", type=float, default=0.1,
                        help="position-difference sanity-check window in seconds")
    parser.add_argument("--json", action="store_true", help="emit JSON instead of text")
    parser.add_argument("--time-series-csv",
                        help="write per-sample diagnostics; valid for one input CSV")
    args = parser.parse_args()

    if args.time_series_csv and len(args.ground_truth_csv) != 1:
        parser.error("--time-series-csv requires exactly one ground-truth CSV")

    summaries = []
    for path in args.ground_truth_csv:
        summary, series = analyze_ground_truth(path, args.difference_window)
        summaries.append(summary)
        if args.time_series_csv:
            write_time_series(args.time_series_csv, series)

    if args.json:
        print(json.dumps(summaries if len(summaries) > 1 else summaries[0], indent=2,
                         sort_keys=True))
        return

    for summary in summaries:
        print(f"sequence: {summary['sequence']}")
        print(f"samples: {summary['samples']}")
        print(f"duration_s: {summary['duration_s']:.6f}")
        print(f"speed_p95_mps: {summary['world_velocity_mps']['norm']['p95']:.6f}")
        print(f"speed_max_mps: {summary['world_velocity_mps']['norm']['max']:.6f}")
        print("position_difference_rmse_mps: "
              f"{summary['position_difference_check']['official_minus_position_difference_error_mps']['rms']:.6f}")


if __name__ == "__main__":
    main()
