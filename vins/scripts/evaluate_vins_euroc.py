#!/usr/bin/env python3

"""Evaluate a VINS trajectory against EuRoC ground truth stored in a ROS 2 bag."""

import argparse
import csv
import datetime
import hashlib
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

UZH_GROUND_TRUTH_TOPICS = (
    "/groundtruth/pose",
    "groundtruth/pose",
    "/groundtruth/odometry",
    "groundtruth/odometry",
)


def sha256_file(path):
    digest = hashlib.sha256()
    with open(path, "rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def read_bag_time_range(bag_path):
    metadata_path = os.path.join(bag_path, "metadata.yaml")
    try:
        import yaml
        with open(metadata_path, encoding="utf-8") as stream:
            metadata = yaml.safe_load(stream)["rosbag2_bagfile_information"]
        start_ns = metadata["starting_time"]["nanoseconds_since_epoch"]
        duration_ns = metadata["duration"]["nanoseconds"]
    except (ImportError, KeyError, OSError, TypeError, ValueError) as error:
        raise RuntimeError(f"cannot read ROS 2 bag time range: {metadata_path}") from error
    return start_ns * 1e-9, (start_ns + duration_ns) * 1e-9


def discover_uzh_ground_truth(bag_path):
    sequence = os.path.basename(os.path.normpath(bag_path))
    if sequence.endswith("_db"):
        sequence = sequence[:-3]
    prefixes = ("indoor_forward_", "indoor_45_", "outdoor_forward_", "outdoor_45_")
    if not sequence.startswith(prefixes):
        return None
    dataset_root = os.path.dirname(os.path.dirname(os.path.abspath(bag_path)))
    candidate = os.path.join(dataset_root, "archives", sequence, "groundtruth.txt")
    return candidate if os.path.isfile(candidate) else None


def read_uzh_ground_truth(path, bag_time_range=None):
    timestamps = []
    positions = []
    rotations = []
    with open(path, encoding="utf-8") as stream:
        for line_number, line in enumerate(stream, 1):
            stripped = line.strip()
            if not stripped or stripped.startswith("#"):
                continue
            fields = stripped.split()
            if len(fields) != 8:
                raise RuntimeError(
                    f"UZH ground truth line {line_number} has {len(fields)} fields, expected 8")
            try:
                values = np.asarray([float(field) for field in fields], dtype=float)
            except ValueError as error:
                raise RuntimeError(
                    f"UZH ground truth line {line_number} contains a non-numeric value") from error
            if not np.all(np.isfinite(values)):
                raise RuntimeError(
                    f"UZH ground truth line {line_number} contains a non-finite value")
            if timestamps and values[0] <= timestamps[-1]:
                raise RuntimeError(
                    f"UZH ground-truth timestamps are not strictly increasing at line {line_number}")
            quaternion = values[4:8]
            quaternion_norm = float(np.linalg.norm(quaternion))
            if quaternion_norm < 1e-12 or abs(quaternion_norm - 1.0) > 1e-3:
                raise RuntimeError(
                    f"UZH ground truth line {line_number} has an invalid quaternion")
            timestamps.append(values[0])
            positions.append(values[1:4])
            # File order is qx qy qz qw; retain orientation for traceability.
            rotations.append(quaternion_to_rotation(
                quaternion[3], quaternion[0], quaternion[1], quaternion[2]))
    if not timestamps:
        raise RuntimeError("UZH ground-truth file contains no samples")
    timestamps = np.asarray(timestamps)
    if bag_time_range is not None:
        bag_start, bag_end = bag_time_range
        if timestamps[-1] < bag_start or timestamps[0] > bag_end:
            raise RuntimeError(
                "UZH ground truth does not overlap the ROS 2 bag time range")
    return {
        "source": "uzh_archives",
        "source_type": "uzh_eight_column",
        "source_path": os.path.abspath(path),
        "sha256": sha256_file(path),
        "topic": None,
        "timestamps": timestamps,
        "positions": np.asarray(positions),
        "rotations": np.asarray(rotations),
        "velocities": None,
    }


def discover_uzh_leica(bag_path):
    sequence = os.path.basename(os.path.normpath(bag_path))
    if sequence.endswith("_db"):
        sequence = sequence[:-3]
    if sequence.endswith("_snapdragon_with_gt"):
        sequence = sequence[:-len("_snapdragon_with_gt")]
    dataset_root = os.path.dirname(os.path.dirname(os.path.abspath(bag_path)))
    candidate = os.path.join(dataset_root, "groundtruth_official", sequence, "leica.txt")
    return candidate if os.path.isfile(candidate) else None


def _uzh_sequence_name(bag_path):
    sequence = os.path.basename(os.path.normpath(bag_path))
    if sequence.endswith("_db"):
        sequence = sequence[:-3]
    return sequence


def load_uzh_time_offset_manifest(path, bag_path, ground_truth):
    if not os.path.isfile(path):
        raise RuntimeError(
            f"UZH time-offset audit manifest is missing: {path}; "
            "run audit_uzhfpv_time_offsets.py before formal ATE")
    try:
        with open(path, encoding="utf-8") as stream:
            manifest = json.load(stream)
    except (OSError, ValueError) as error:
        raise RuntimeError(f"cannot read UZH time-offset manifest: {path}") from error
    if manifest.get("schema") != "uzhfpv-gt-time-offset-audit-v1":
        raise RuntimeError("unsupported UZH time-offset manifest schema")
    sequence_name = _uzh_sequence_name(bag_path)
    entry = next((item for item in manifest.get("sequences", [])
                  if item.get("sequence") == sequence_name), None)
    if entry is None:
        raise RuntimeError(
            f"UZH time-offset manifest has no entry for {sequence_name}")
    if entry.get("gt_sha256") != ground_truth.get("sha256"):
        raise RuntimeError("UZH time-offset manifest GT hash mismatch")
    metadata_path = os.path.join(bag_path, "metadata.yaml")
    if (not os.path.isfile(metadata_path) or
            entry.get("bag_metadata_sha256") != sha256_file(metadata_path)):
        raise RuntimeError("UZH time-offset manifest bag metadata hash mismatch")
    offset = entry.get("applied_gt_time_offset_s")
    if not isinstance(offset, (int, float)) or not math.isfinite(offset):
        raise RuntimeError("UZH time-offset manifest has an invalid applied offset")
    return manifest, entry, float(offset)


def audit_uzh_leica(path):
    timestamps = []
    finite_positions = True
    sample_count = 0
    with open(path, encoding="utf-8", errors="replace", newline="") as stream:
        for row in csv.reader(stream):
            if len(row) < 13 or row[0] != "3" or row[6] == "":
                continue
            try:
                timestamp = datetime.datetime.strptime(
                    row[6], "%Y-%m-%d %H:%M:%S.%f").replace(
                        tzinfo=datetime.timezone.utc).timestamp()
                position = [float(value) for value in row[10:13]]
            except (ValueError, IndexError):
                continue
            sample_count += 1
            timestamps.append(timestamp)
            finite_positions = finite_positions and all(math.isfinite(value) for value in position)
    if not sample_count:
        raise RuntimeError(f"raw Leica file contains no parseable samples: {path}")
    return {
        "path": os.path.abspath(path),
        "sha256": sha256_file(path),
        "samples": sample_count,
        "start_time_utc_s": min(timestamps),
        "end_time_utc_s": max(timestamps),
        "positions_finite": finite_positions,
    }


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
    from geometry_msgs.msg import PointStamped, PoseStamped, TransformStamped
    from nav_msgs.msg import Odometry
    from rclpy.serialization import deserialize_message

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag_path, storage_id="sqlite3"),
                rosbag2_py.ConverterOptions("", ""))
    topic_types = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
    topic = requested_topic
    if topic is None:
        topic = next((candidate for candidate in GROUND_TRUTH_TOPICS + UZH_GROUND_TRUTH_TOPICS
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
    elif message_type == "geometry_msgs/msg/PoseStamped":
        message_class = PoseStamped
        has_orientation = True
    elif message_type == "nav_msgs/msg/Odometry":
        message_class = Odometry
        has_orientation = True
    else:
        raise RuntimeError(f"unsupported ground-truth message type: {message_type}")

    timestamps = []
    positions = []
    rotations = []
    source_digest = hashlib.sha256()
    while reader.has_next():
        current_topic, data, _ = reader.read_next()
        if current_topic != topic:
            continue
        source_digest.update(data)
        message = deserialize_message(data, message_class)
        timestamps.append(message.header.stamp.sec + message.header.stamp.nanosec * 1e-9)
        if message_type == "geometry_msgs/msg/TransformStamped":
            positions.append([message.transform.translation.x,
                              message.transform.translation.y,
                              message.transform.translation.z])
            rotation = message.transform.rotation
            rotations.append(quaternion_to_rotation(
                rotation.w, rotation.x, rotation.y, rotation.z))
        elif message_type == "geometry_msgs/msg/PointStamped":
            positions.append([message.point.x, message.point.y, message.point.z])
        else:
            pose = message.pose if message_type == "geometry_msgs/msg/PoseStamped" else message.pose.pose
            positions.append([pose.position.x, pose.position.y, pose.position.z])
            rotations.append(quaternion_to_rotation(
                pose.orientation.w, pose.orientation.x,
                pose.orientation.y, pose.orientation.z))

    is_uzh = topic in UZH_GROUND_TRUTH_TOPICS
    if not timestamps:
        raise RuntimeError(f"ground-truth topic contains no messages: {topic}")
    if is_uzh:
        timestamp_values = np.asarray(timestamps)
        position_values = np.asarray(positions)
        if not np.all(np.isfinite(timestamp_values)) or not np.all(np.isfinite(position_values)):
            raise RuntimeError("UZH bag ground truth contains non-finite values")
        if np.any(np.diff(timestamp_values) <= 0.0):
            raise RuntimeError("UZH bag ground-truth timestamps are not strictly increasing")

    return {
        "source": ("uzh_rosbag_groundtruth" if is_uzh else
                   ("rosbag_vicon" if has_orientation else "rosbag_leica")),
        "source_type": message_type,
        "source_path": os.path.abspath(bag_path),
        "sha256": source_digest.hexdigest(),
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
    parser.add_argument("--ground-truth-uzh",
                        help="UZH-FPV eight-column groundtruth.txt")
    parser.add_argument(
        "--uzh-time-offset-manifest",
        help="frozen UZH timestamp audit; defaults to config/uzhfpv_gt_time_offsets.json")
    parser.add_argument("--max-time-error", type=float, default=0.02)
    parser.add_argument("--velocity-difference-window", type=float, default=0.1)
    parser.add_argument("--allow-position-velocity-fallback", action="store_true",
                        help="allow Leica position differencing for velocity metrics")
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args()

    vins_times, vins_positions, vins_rotations, vins_velocities = read_vins(args.trajectory)
    uzh_path = args.ground_truth_uzh or discover_uzh_ground_truth(args.bag)
    official_path = args.ground_truth_csv or discover_official_ground_truth(args.bag)
    if args.ground_truth_csv:
        ground_truth = read_official_ground_truth(args.ground_truth_csv)
    elif uzh_path:
        ground_truth = read_uzh_ground_truth(uzh_path, read_bag_time_range(args.bag))
    elif official_path:
        ground_truth = read_official_ground_truth(official_path)
    else:
        ground_truth = read_bag_ground_truth(args.bag, args.ground_truth_topic)
    is_uzh = ground_truth["source"].startswith("uzh_")
    time_audit_manifest = None
    time_audit_entry = None
    applied_gt_time_offset = 0.0
    if is_uzh:
        default_manifest = os.path.join(
            os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
            "config", "uzhfpv_gt_time_offsets.json")
        manifest_path = args.uzh_time_offset_manifest or default_manifest
        time_audit_manifest, time_audit_entry, applied_gt_time_offset = (
            load_uzh_time_offset_manifest(
                manifest_path, args.bag, ground_truth))
    gt_times = ground_truth["timestamps"] + applied_gt_time_offset
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
        "position_p95_error_m": float(np.percentile(position_errors, 95)),
        "maximum_position_error_m": float(np.max(position_errors)),
    }

    if is_uzh:
        result.update({
            "ground_truth_path": ground_truth["source_path"],
            "ground_truth_type": ground_truth["source_type"],
            "ground_truth_sha256": ground_truth["sha256"],
            "ground_truth_original_time_range_s": [
                float(ground_truth["timestamps"][0]),
                float(ground_truth["timestamps"][-1])],
            "ground_truth_evaluation_time_range_s": [
                float(gt_times[0]), float(gt_times[-1])],
            "estimated_raw_time_offset_s": time_audit_entry["raw_offset_s"],
            "calibration_td_s": time_audit_entry["calibration_td_s"],
            "estimated_residual_gt_offset_s": time_audit_entry[
                "residual_offset_s"],
            "applied_gt_time_offset_s": applied_gt_time_offset,
            "time_offset_manifest_sha256": sha256_file(
                args.uzh_time_offset_manifest or default_manifest),
            "time_offset_decision": time_audit_manifest["families"][
                time_audit_entry["family"]]["status"],
            "orientation_metrics_status": "excluded_known_ground_truth_issue",
            "velocity_metrics_status": "unavailable",
        })
        leica_path = discover_uzh_leica(args.bag)
        if leica_path:
            result["raw_leica_audit"] = audit_uzh_leica(leica_path)

    if gt_rotations is not None and not is_uzh:
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
            "rotation_p95_error_deg": math.degrees(
                float(np.percentile(rotation_errors, 95))),
            "maximum_rotation_error_deg": math.degrees(float(np.max(rotation_errors))),
            "roll_rmse_deg": math.degrees(math.sqrt(np.mean(rpy_errors[:, 0] ** 2))),
            "pitch_rmse_deg": math.degrees(math.sqrt(np.mean(rpy_errors[:, 1] ** 2))),
        })

    velocity_gt_source = "unavailable"
    velocity_vins_indices = np.asarray([], dtype=int)
    velocity_gt_values = np.empty((0, 3))
    if is_uzh:
        pass
    elif ground_truth["velocities"] is not None:
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
    elif not is_uzh:
        result["velocity_gt_source"] = "unavailable"

    if args.json:
        print(json.dumps(result, sort_keys=True))
    else:
        for key, value in result.items():
            print(f"{key}: {value}")


if __name__ == "__main__":
    main()
