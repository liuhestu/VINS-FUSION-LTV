#!/usr/bin/env python3

"""Evaluate a VINS trajectory against EuRoC ground truth stored in a ROS 2 bag."""

import argparse
import csv
import json
import math
import os

import numpy as np


GROUND_TRUTH_TOPICS = (
    "/vicon/firefly_sbx/firefly_sbx",
    "vicon/firefly_sbx/firefly_sbx",
    "/leica/position",
    "leica/position",
)


def quaternion_to_rotation(w, x, y, z):
    quaternion = np.asarray([w, x, y, z], dtype=float)
    quaternion /= np.linalg.norm(quaternion)
    w, x, y, z = quaternion
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def read_vins(path):
    timestamps = []
    positions = []
    rotations = []
    velocities = []
    with open(path, encoding="utf-8") as stream:
        for line in stream:
            fields = line.rstrip().split(",")
            if len(fields) < 8:
                continue
            try:
                values = [float(field) for field in fields[:8]]
            except ValueError:
                continue
            timestamps.append(values[0])
            positions.append(values[1:4])
            rotations.append(quaternion_to_rotation(*values[4:8]))
            if len(fields) >= 11:
                try:
                    velocities.append([float(field) for field in fields[8:11]])
                    continue
                except ValueError:
                    pass
            velocities.append([math.nan, math.nan, math.nan])
    if not timestamps:
        raise RuntimeError("VINS trajectory contains no valid samples")
    return (np.asarray(timestamps), np.asarray(positions), np.asarray(rotations),
            np.asarray(velocities))


def read_bag_ground_truth(bag_path, requested_topic=None):
    import rosbag2_py
    from geometry_msgs.msg import PointStamped, TransformStamped
    from rclpy.serialization import deserialize_message

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag_path, storage_id="sqlite3"),
                rosbag2_py.ConverterOptions("", ""))
    topic_types = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
    topic = requested_topic
    if topic is None:
        topic = next((candidate for candidate in GROUND_TRUTH_TOPICS
                      if candidate in topic_types), None)
    if topic not in topic_types:
        raise RuntimeError("no supported EuRoC ground-truth topic found")

    message_type = topic_types[topic]
    if message_type == "geometry_msgs/msg/TransformStamped":
        message_class = TransformStamped
        has_orientation = True
    elif message_type == "geometry_msgs/msg/PointStamped":
        message_class = PointStamped
        has_orientation = False
    else:
        raise RuntimeError(f"unsupported ground-truth message type: {message_type}")

    timestamps = []
    positions = []
    rotations = []
    while reader.has_next():
        current_topic, data, _ = reader.read_next()
        if current_topic != topic:
            continue
        message = deserialize_message(data, message_class)
        timestamps.append(message.header.stamp.sec + message.header.stamp.nanosec * 1e-9)
        if has_orientation:
            positions.append([message.transform.translation.x,
                              message.transform.translation.y,
                              message.transform.translation.z])
            rotation = message.transform.rotation
            rotations.append(quaternion_to_rotation(
                rotation.w, rotation.x, rotation.y, rotation.z))
        else:
            positions.append([message.point.x, message.point.y, message.point.z])

    return {
        "source": "rosbag_vicon" if has_orientation else "rosbag_leica",
        "topic": topic,
        "timestamps": np.asarray(timestamps),
        "positions": np.asarray(positions),
        "rotations": np.asarray(rotations) if has_orientation else None,
        "velocities": None,
    }


def _official_column_name(header):
    return header.lstrip("#").strip().split(" [", 1)[0].strip()


def read_official_ground_truth(path):
    with open(path, encoding="utf-8", newline="") as stream:
        reader = csv.reader(stream)
        header = None
        rows = []
        for row in reader:
            if not row:
                continue
            if header is None:
                header = [_official_column_name(value) for value in row]
                continue
            rows.append(row)

    if header is None or not rows:
        raise RuntimeError("official ground-truth CSV contains no samples")
    indices = {name: index for index, name in enumerate(header)}
    required = ("timestamp", "p_RS_R_x", "p_RS_R_y", "p_RS_R_z",
                "q_RS_w", "q_RS_x", "q_RS_y", "q_RS_z")
    missing = [name for name in required if name not in indices]
    if missing:
        raise RuntimeError("official ground-truth CSV missing: " + ", ".join(missing))

    timestamps = []
    positions = []
    rotations = []
    velocities = []
    velocity_names = ("v_RS_R_x", "v_RS_R_y", "v_RS_R_z")
    has_velocity = all(name in indices for name in velocity_names)
    for row in rows:
        try:
            timestamp = float(row[indices["timestamp"]]) * 1e-9
            position = [float(row[indices[name]]) for name in
                        ("p_RS_R_x", "p_RS_R_y", "p_RS_R_z")]
            rotation = quaternion_to_rotation(
                *[float(row[indices[name]]) for name in
                  ("q_RS_w", "q_RS_x", "q_RS_y", "q_RS_z")])
            if has_velocity:
                velocity = [float(row[indices[name]]) for name in velocity_names]
        except (IndexError, ValueError):
            continue
        timestamps.append(timestamp)
        positions.append(position)
        rotations.append(rotation)
        if has_velocity:
            velocities.append(velocity)
    if not timestamps:
        raise RuntimeError("official ground-truth CSV has no valid samples")
    return {
        "source": "official_csv",
        "topic": None,
        "timestamps": np.asarray(timestamps),
        "positions": np.asarray(positions),
        "rotations": np.asarray(rotations),
        "velocities": np.asarray(velocities) if has_velocity else None,
    }


def discover_official_ground_truth(bag_path):
    candidates = (
        os.path.join(bag_path, "mav0", "state_groundtruth_estimate0", "data.csv"),
        os.path.join(bag_path, "state_groundtruth_estimate0", "data.csv"),
    )
    return next((candidate for candidate in candidates if os.path.isfile(candidate)), None)


def match_nearest(query_times, reference_times, max_time_error):
    query_indices = []
    reference_indices = []
    time_errors = []
    for query_index, timestamp in enumerate(query_times):
        insertion = int(np.searchsorted(reference_times, timestamp))
        candidates = [index for index in (insertion - 1, insertion)
                      if 0 <= index < len(reference_times)]
        if not candidates:
            continue
        reference_index = min(candidates,
                              key=lambda index: abs(reference_times[index] - timestamp))
        time_error = abs(reference_times[reference_index] - timestamp)
        if time_error <= max_time_error:
            query_indices.append(query_index)
            reference_indices.append(reference_index)
            time_errors.append(time_error)
    return (np.asarray(query_indices, dtype=int),
            np.asarray(reference_indices, dtype=int), np.asarray(time_errors))


def interpolate_values(reference_times, reference_values, query_times):
    reference_times = np.asarray(reference_times)
    reference_values = np.asarray(reference_values)
    query_times = np.asarray(query_times)
    if reference_values.ndim != 2 or len(reference_times) != len(reference_values):
        raise ValueError("reference values must be an N x D array matched to timestamps")

    valid = ((query_times >= reference_times[0]) &
             (query_times <= reference_times[-1]))
    values = np.full((len(query_times), reference_values.shape[1]), math.nan)
    for axis in range(reference_values.shape[1]):
        values[valid, axis] = np.interp(query_times[valid], reference_times,
                                        reference_values[:, axis])
    return values, valid


def position_difference_velocity(reference_times, reference_positions, query_times,
                                 difference_window):
    if not math.isfinite(difference_window) or difference_window <= 0.0:
        raise ValueError("velocity difference window must be positive")
    half_window = 0.5 * difference_window
    plus_positions, plus_valid = interpolate_values(
        reference_times, reference_positions, query_times + half_window)
    minus_positions, minus_valid = interpolate_values(
        reference_times, reference_positions, query_times - half_window)
    valid = plus_valid & minus_valid
    velocities = np.full_like(plus_positions, math.nan)
    velocities[valid] = (plus_positions[valid] - minus_positions[valid]) / difference_window
    return velocities, valid


def rigid_alignment(source, target):
    source_center = np.mean(source, axis=0)
    target_center = np.mean(target, axis=0)
    covariance = (source - source_center).T @ (target - target_center)
    u, _, vt = np.linalg.svd(covariance)
    rotation = vt.T @ u.T
    if np.linalg.det(rotation) < 0.0:
        vt[-1, :] *= -1.0
        rotation = vt.T @ u.T
    translation = target_center - rotation @ source_center
    return rotation, translation


def rotation_error_rpy(rotation):
    pitch = math.asin(float(np.clip(-rotation[2, 0], -1.0, 1.0)))
    roll = math.atan2(rotation[2, 1], rotation[2, 2])
    yaw = math.atan2(rotation[1, 0], rotation[0, 0])
    return np.asarray([roll, pitch, yaw])


def mean_rotation(rotations):
    accumulator = np.sum(rotations, axis=0)
    u, _, vt = np.linalg.svd(accumulator)
    rotation = u @ vt
    if np.linalg.det(rotation) < 0.0:
        u[:, -1] *= -1.0
        rotation = u @ vt
    return rotation


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag", help="ROS 2 EuRoC bag directory")
    parser.add_argument("trajectory", help="VINS vio.csv")
    parser.add_argument("--ground-truth-topic")
    parser.add_argument("--ground-truth-csv",
                        help="EuRoC state_groundtruth_estimate0/data.csv")
    parser.add_argument("--max-time-error", type=float, default=0.02)
    parser.add_argument("--velocity-difference-window", type=float, default=0.1)
    parser.add_argument("--allow-position-velocity-fallback", action="store_true",
                        help="allow Leica position differencing for velocity metrics")
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args()

    vins_times, vins_positions, vins_rotations, vins_velocities = read_vins(args.trajectory)
    official_path = args.ground_truth_csv or discover_official_ground_truth(args.bag)
    ground_truth = (read_official_ground_truth(official_path) if official_path else
                    read_bag_ground_truth(args.bag, args.ground_truth_topic))
    gt_times = ground_truth["timestamps"]
    gt_positions = ground_truth["positions"]
    gt_rotations = ground_truth["rotations"]
    vins_indices, gt_indices, time_errors = match_nearest(
        vins_times, gt_times, args.max_time_error)
    if len(vins_indices) < 3:
        raise RuntimeError("fewer than three timestamp-matched samples")

    matched_vins_positions = vins_positions[vins_indices]
    matched_gt_positions = gt_positions[gt_indices]
    alignment_rotation, alignment_translation = rigid_alignment(
        matched_vins_positions, matched_gt_positions)
    aligned_positions = (alignment_rotation @ matched_vins_positions.T).T + alignment_translation
    position_errors = np.linalg.norm(aligned_positions - matched_gt_positions, axis=1)

    result = {
        "ground_truth_source": ground_truth["source"],
        "ground_truth_topic": ground_truth["topic"],
        "trajectory_samples": int(len(vins_times)),
        "matched_samples": int(len(vins_indices)),
        "duration_s": float(vins_times[-1] - vins_times[0]),
        "maximum_timestamp_error_s": float(np.max(time_errors)),
        "ate_rmse_m": float(math.sqrt(np.mean(position_errors ** 2))),
        "maximum_position_error_m": float(np.max(position_errors)),
    }

    if gt_rotations is not None:
        if ground_truth["source"] == "official_csv":
            body_alignment = np.eye(3)
            orientation_alignment_samples = 0
        else:
            # The EuRoC bag's Vicon pose is expressed in the vehicle body
            # frame, while VINS estimates the IMU sensor frame. Remove their
            # fixed body rotation; official CSV GT already uses IMU frame S.
            orientation_alignment_samples = min(50, len(vins_indices))
            body_alignments = []
            for vins_index, gt_index in zip(
                    vins_indices[:orientation_alignment_samples],
                    gt_indices[:orientation_alignment_samples]):
                aligned_vins_rotation = alignment_rotation @ vins_rotations[vins_index]
                body_alignments.append(aligned_vins_rotation.T @ gt_rotations[gt_index])
            body_alignment = mean_rotation(np.asarray(body_alignments))

        rotation_errors = []
        rpy_errors = []
        for vins_index, gt_index in zip(vins_indices, gt_indices):
            aligned_rotation = (alignment_rotation @ vins_rotations[vins_index] @
                                body_alignment)
            delta = gt_rotations[gt_index].T @ aligned_rotation
            cosine = np.clip((np.trace(delta) - 1.0) * 0.5, -1.0, 1.0)
            rotation_errors.append(math.acos(float(cosine)))
            rpy_errors.append(rotation_error_rpy(delta))
        rotation_errors = np.asarray(rotation_errors)
        rpy_errors = np.asarray(rpy_errors)
        result.update({
            "orientation_alignment_samples": orientation_alignment_samples,
            "rotation_rmse_deg": math.degrees(math.sqrt(np.mean(rotation_errors ** 2))),
            "maximum_rotation_error_deg": math.degrees(float(np.max(rotation_errors))),
            "roll_rmse_deg": math.degrees(math.sqrt(np.mean(rpy_errors[:, 0] ** 2))),
            "pitch_rmse_deg": math.degrees(math.sqrt(np.mean(rpy_errors[:, 1] ** 2))),
        })

    velocity_gt_source = "unavailable"
    velocity_vins_indices = np.asarray([], dtype=int)
    velocity_gt_values = np.empty((0, 3))
    if ground_truth["velocities"] is not None:
        velocity_vins_indices = vins_indices
        velocity_gt_values = ground_truth["velocities"][gt_indices]
        velocity_gt_source = "official"
    elif (ground_truth["source"] != "rosbag_leica" or
          args.allow_position_velocity_fallback):
        candidate_indices = vins_indices
        fallback_velocities, fallback_valid = position_difference_velocity(
            gt_times, gt_positions, vins_times[candidate_indices],
            args.velocity_difference_window)
        velocity_vins_indices = candidate_indices[fallback_valid]
        velocity_gt_values = fallback_velocities[fallback_valid]
        velocity_gt_source = "position_difference"

    if len(velocity_vins_indices) > 0:
        valid_velocity = (np.all(np.isfinite(vins_velocities[velocity_vins_indices]), axis=1) &
                          np.all(np.isfinite(velocity_gt_values), axis=1))
        velocity_vins_indices = velocity_vins_indices[valid_velocity]
        velocity_gt_values = velocity_gt_values[valid_velocity]
    if len(velocity_vins_indices) > 0:
        aligned_velocities = (
            alignment_rotation @ vins_velocities[velocity_vins_indices].T).T
        velocity_errors = np.linalg.norm(aligned_velocities - velocity_gt_values, axis=1)
        result.update({
            "velocity_gt_source": velocity_gt_source,
            "velocity_samples": int(len(velocity_vins_indices)),
            "velocity_rmse_mps": float(math.sqrt(np.mean(velocity_errors ** 2))),
            "velocity_p95_error_mps": float(np.percentile(velocity_errors, 95)),
            "maximum_velocity_error_mps": float(np.max(velocity_errors)),
        })
    else:
        result["velocity_gt_source"] = "unavailable"

    if args.json:
        print(json.dumps(result, sort_keys=True))
    else:
        for key, value in result.items():
            print(f"{key}: {value}")


if __name__ == "__main__":
    main()
