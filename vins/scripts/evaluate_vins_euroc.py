#!/usr/bin/env python3

"""Evaluate a VINS trajectory against EuRoC ground truth stored in a ROS 2 bag."""

import argparse
import json
import math

import numpy as np
import rosbag2_py
from geometry_msgs.msg import PointStamped, TransformStamped
from rclpy.serialization import deserialize_message


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
    if not timestamps:
        raise RuntimeError("VINS trajectory contains no valid samples")
    return np.asarray(timestamps), np.asarray(positions), np.asarray(rotations)


def read_ground_truth(bag_path, requested_topic=None):
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

    return (topic, np.asarray(timestamps), np.asarray(positions),
            np.asarray(rotations) if has_orientation else None)


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
    parser.add_argument("--max-time-error", type=float, default=0.02)
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args()

    vins_times, vins_positions, vins_rotations = read_vins(args.trajectory)
    topic, gt_times, gt_positions, gt_rotations = read_ground_truth(
        args.bag, args.ground_truth_topic)
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
        "ground_truth_topic": topic,
        "trajectory_samples": int(len(vins_times)),
        "matched_samples": int(len(vins_indices)),
        "duration_s": float(vins_times[-1] - vins_times[0]),
        "maximum_timestamp_error_s": float(np.max(time_errors)),
        "ate_rmse_m": float(math.sqrt(np.mean(position_errors ** 2))),
        "maximum_position_error_m": float(np.max(position_errors)),
    }

    if gt_rotations is not None:
        # The EuRoC bag's Vicon pose is expressed in the vehicle body frame,
        # while VINS estimates the IMU sensor frame. Remove their fixed body
        # rotation using the first matched samples; subsequent errors still
        # measure time-varying attitude drift.
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

    if args.json:
        print(json.dumps(result, sort_keys=True))
    else:
        for key, value in result.items():
            print(f"{key}: {value}")


if __name__ == "__main__":
    main()
