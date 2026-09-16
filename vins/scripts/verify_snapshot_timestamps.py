#!/usr/bin/env python3
"""Validate each Stage 5 snapshot stream without requiring equal grids."""

import argparse
import csv
import json
from pathlib import Path


def read_timestamps(path):
    with open(path, encoding="utf-8", newline="") as stream:
        reader = csv.DictReader(stream)
        if reader.fieldnames is None or "frame_timestamp" not in reader.fieldnames:
            raise RuntimeError(f"{path}: missing frame_timestamp column")
        timestamps = []
        for line_number, row in enumerate(reader, start=2):
            try:
                timestamp = float(row["frame_timestamp"])
            except (TypeError, ValueError) as error:
                raise RuntimeError(
                    f"{path}:{line_number}: invalid frame_timestamp") from error
            if timestamp != 0.0:
                timestamps.append(timestamp)
    if not timestamps:
        raise RuntimeError(f"{path}: no nonzero snapshot timestamps")
    if any(current <= previous for previous, current in
           zip(timestamps, timestamps[1:])):
        raise RuntimeError(f"{path}: timestamps are not strictly increasing")
    return timestamps


def verify_independent(paths):
    streams = {str(path): read_timestamps(path) for path in paths}
    reference = next(iter(streams.values()))
    report = {}
    for path, timestamps in streams.items():
        report[path] = {
            "count": len(timestamps),
            "first": timestamps[0],
            "last": timestamps[-1],
            "same_grid_as_first": timestamps == reference,
        }
    return report


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("csv", nargs="+", help="Stage 5 LTV CSV files")
    parser.add_argument("--output", type=Path)
    arguments = parser.parse_args()
    report = verify_independent(arguments.csv)
    rendered = json.dumps(report, indent=2, sort_keys=True) + "\n"
    if arguments.output:
        arguments.output.write_text(rendered, encoding="utf-8")
    print(rendered, end="")


if __name__ == "__main__":
    main()
