#!/usr/bin/env python3
"""Create immutable canonical Stage 5 input from a ROS 2 EuRoC bag."""
import argparse
import csv
import json
import shutil
import subprocess
from pathlib import Path

import cv2
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from stage5_cache import require_strictly_increasing, sha256_file, write_csv


def timestamp(header):
    value = int(header.stamp.sec) * 1_000_000_000 + int(header.stamp.nanosec)
    if value < 0:
        raise RuntimeError("negative header timestamp")
    return value


def image(message):
    if message.encoding not in ("mono8", "8UC1") or message.step != message.width:
        raise RuntimeError("expected tightly packed mono8 image")
    raw = np.frombuffer(message.data, dtype=np.uint8)
    if raw.size < message.height * message.width:
        raise RuntimeError("truncated image")
    return raw.reshape(message.height, message.step)[:, :message.width].copy()


def read_bag(args):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(args.bag), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""))
    types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    wanted = (args.imu_topic, args.left_topic, args.right_topic)
    if any(topic not in types for topic in wanted):
        raise RuntimeError("bag does not contain all requested IMU/stereo topics")
    classes = {topic: get_message(types[topic]) for topic in wanted}
    imu, left, right = [], [], []
    topic_indexes = {args.left_topic: 0, args.right_topic: 0}
    while reader.has_next():
        topic, serialized, _ = reader.read_next()
        if topic not in classes:
            continue
        message = deserialize_message(serialized, classes[topic])
        stamp = timestamp(message.header)
        if topic == args.imu_topic:
            values = (
                message.linear_acceleration.x, message.linear_acceleration.y,
                message.linear_acceleration.z, message.angular_velocity.x,
                message.angular_velocity.y, message.angular_velocity.z)
            if not all(np.isfinite(values)):
                raise RuntimeError("IMU contains NaN/Inf")
            imu.append((stamp, values))
        else:
            source_index = topic_indexes[topic]
            topic_indexes[topic] += 1
            target = left if topic == args.left_topic else right
            target.append((stamp, source_index, image(message)))
    imu.sort(key=lambda item: item[0])
    left.sort(key=lambda item: item[0])
    right.sort(key=lambda item: item[0])
    require_strictly_increasing([item[0] for item in imu], "IMU")
    require_strictly_increasing([item[0] for item in left], "left image")
    require_strictly_increasing([item[0] for item in right], "right image")
    return imu, left, right


def read_pair_indexes(path):
    with path.open(encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))
    required = {
        "pair_index", "pair_timestamp", "left_timestamp", "right_timestamp",
        "left_index", "right_index"}
    if not rows or not required.issubset(rows[0]):
        raise RuntimeError("shared stereo pairer produced an invalid manifest")
    parsed = []
    for expected, row in enumerate(rows):
        values = {name: int(row[name]) for name in required}
        if values["pair_index"] != expected:
            raise RuntimeError("shared stereo pairer produced non-contiguous indexes")
        parsed.append(values)
    return parsed


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag", required=True, type=Path)
    parser.add_argument("--ground-truth", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--stereo-pairer", default="stage5_pair_stereo")
    parser.add_argument("--imu-topic", default="/imu0")
    parser.add_argument("--left-topic", default="/cam0/image_raw")
    parser.add_argument("--right-topic", default="/cam1/image_raw")
    args = parser.parse_args()
    if not args.bag.exists() or not args.ground_truth.is_file():
        raise RuntimeError("source bag or official ASL ground truth is missing")
    if args.output.exists():
        raise RuntimeError(f"cache destination already exists: {args.output}")
    temporary = args.output.with_name(args.output.name + ".partial")
    if temporary.exists():
        raise RuntimeError(f"stale partial cache exists: {temporary}")
    try:
        imu, left, right = read_bag(args)
        if not imu or not left or not right:
            raise RuntimeError("bag has no IMU/stereo data")
        temporary.mkdir(parents=True)
        (temporary / "images").mkdir()
        left_csv = temporary / "left_timestamps.csv"
        right_csv = temporary / "right_timestamps.csv"
        raw_pairs = temporary / "pair_indexes.csv"
        write_csv(left_csv, ["timestamp_ns", "source_index"],
                  [(stamp, source_index) for stamp, source_index, _ in left])
        write_csv(right_csv, ["timestamp_ns", "source_index"],
                  [(stamp, source_index) for stamp, source_index, _ in right])
        subprocess.run(
            [args.stereo_pairer, str(left_csv), str(right_csv), str(raw_pairs)],
            check=True)
        pairs = read_pair_indexes(raw_pairs)
        if not pairs:
            raise RuntimeError("shared stereo synchronizer produced no pairs")

        left_by_index = {source_index: (stamp, pixels)
                         for stamp, source_index, pixels in left}
        right_by_index = {source_index: (stamp, pixels)
                          for stamp, source_index, pixels in right}
        canonical_rows = []
        for pair in pairs:
            left_stamp, left_image = left_by_index[pair["left_index"]]
            right_stamp, right_image = right_by_index[pair["right_index"]]
            if (left_stamp != pair["left_timestamp"] or
                    right_stamp != pair["right_timestamp"]):
                raise RuntimeError("pairer/source timestamp mismatch")
            left_png = f"images/{pair['pair_index']:06d}_left.png"
            right_png = f"images/{pair['pair_index']:06d}_right.png"
            if (not cv2.imwrite(str(temporary / left_png), left_image) or
                    not cv2.imwrite(str(temporary / right_png), right_image)):
                raise RuntimeError("failed to write lossless PNG")
            canonical_rows.append((
                pair["pair_index"], pair["pair_timestamp"], left_stamp,
                right_stamp, pair["left_index"], pair["right_index"],
                left_png, right_png))

        canonical = temporary / "canonical_stereo_pairs.csv"
        write_csv(canonical, [
            "pair_index", "pair_timestamp", "left_timestamp",
            "right_timestamp", "left_index", "right_index", "left_png",
            "right_png"], canonical_rows)
        write_csv(temporary / "imu.csv", [
            "timestamp_ns", "ax", "ay", "az", "gx", "gy", "gz"],
            [(stamp, *values) for stamp, values in imu])
        left_count, right_count, paired_count = len(left), len(right), len(pairs)
        metadata = {
            "format": "ltv-stage5-cache-v2",
            "bag": str(args.bag.resolve()),
            "ground_truth": str(args.ground_truth.resolve()),
            "ground_truth_sha256": sha256_file(args.ground_truth),
            "left_input_count": left_count,
            "right_input_count": right_count,
            "paired_count": paired_count,
            "dropped_left_count": left_count - paired_count,
            "dropped_right_count": right_count - paired_count,
            "left_coverage": paired_count / left_count,
            "right_coverage": paired_count / right_count,
            "frame_time_range_ns": [
                canonical_rows[0][1], canonical_rows[-1][1]],
            "pair_list_sha256": sha256_file(canonical),
            "imu_count": len(imu),
            "imu_time_range_ns": [imu[0][0], imu[-1][0]],
            "imu_sha256": sha256_file(temporary / "imu.csv"),
            "stereo_pairer": str(Path(args.stereo_pairer).resolve()),
        }
        (temporary / "metadata.json").write_text(
            json.dumps(metadata, indent=2, sort_keys=True) + "\n",
            encoding="utf-8")
        left_csv.unlink()
        right_csv.unlink()
        raw_pairs.unlink()
        temporary.rename(args.output)
    except Exception:
        shutil.rmtree(temporary, ignore_errors=True)
        raise


if __name__ == "__main__":
    main()
