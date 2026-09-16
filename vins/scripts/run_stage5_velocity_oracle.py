#!/usr/bin/env python3
"""Run one Stage 5 mode transactionally against an immutable cache."""

import argparse
import csv
import hashlib
import json
import math
import os
import re
import subprocess
import tempfile
import uuid
from datetime import datetime, timezone
from pathlib import Path


MODES = {
    "reference": (0, 0, "oracle_0"),
    "v_fixed": (1, 0, "oracle_0"),
    "v_oracle_0": (1, 1, "oracle_0"),
    "v_oracle_002": (1, 1, "oracle_002"),
}


def replace_setting(text, key, value):
    text, count = re.subn(
        rf"^{re.escape(key)}\s*:.*$", f"{key}: {value}", text,
        flags=re.MULTILINE)
    if count != 1:
        raise RuntimeError(f"expected exactly one {key} setting, found {count}")
    return text


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def validate_csv(path, timestamp_column, time_range=None, nonnumeric=()):
    with path.open(encoding="utf-8", newline="") as stream:
        reader = csv.DictReader(stream)
        if reader.fieldnames is None or timestamp_column not in reader.fieldnames:
            raise RuntimeError(f"{path}: missing {timestamp_column}")
        count = 0
        timestamps = []
        for line_number, row in enumerate(reader, start=2):
            count += 1
            for name, value in row.items():
                if name in nonnumeric:
                    continue
                if name is None or value in (None, ""):
                    raise RuntimeError(
                        f"{path}:{line_number}: missing numeric CSV field")
                try:
                    number = float(value)
                except ValueError as error:
                    raise RuntimeError(
                        f"{path}:{line_number}: non-numeric {name}") from error
                if not math.isfinite(number):
                    raise RuntimeError(
                        f"{path}:{line_number}: NaN/Inf in {name}")
            timestamp = float(row[timestamp_column])
            if timestamp != 0.0:
                timestamps.append(timestamp)
        if count == 0:
            raise RuntimeError(f"{path}: no data rows")
    if any(current <= previous for previous, current in zip(timestamps, timestamps[1:])):
        raise RuntimeError(f"{path}: nonzero timestamps are not strictly increasing")
    if timestamps and time_range is not None:
        lower, upper = (value * 1e-9 for value in time_range)
        tolerance = 1e-6
        if timestamps[0] < lower - tolerance or timestamps[-1] > upper + tolerance:
            raise RuntimeError(f"{path}: output timestamp lies outside canonical input")
    return count, timestamps


def validate_vio(path, time_range):
    count = 0
    timestamps = []
    with path.open(encoding="utf-8") as stream:
        for line_number, line in enumerate(stream, start=1):
            fields = line.rstrip().split(",")
            if len(fields) < 11:
                raise RuntimeError(f"{path}:{line_number}: incomplete trajectory row")
            try:
                values = [float(value) for value in fields[:11]]
            except ValueError as error:
                raise RuntimeError(
                    f"{path}:{line_number}: non-numeric trajectory row") from error
            if not all(math.isfinite(value) for value in values):
                raise RuntimeError(f"{path}:{line_number}: NaN/Inf trajectory row")
            timestamps.append(values[0])
            count += 1
    if count == 0:
        raise RuntimeError(f"{path}: no trajectory rows")
    if any(current <= previous for previous, current in zip(timestamps, timestamps[1:])):
        raise RuntimeError(f"{path}: timestamps are not strictly increasing")
    lower, upper = (value * 1e-9 for value in time_range)
    tolerance = 1e-6
    if timestamps[0] < lower - tolerance or timestamps[-1] > upper + tolerance:
        raise RuntimeError(f"{path}: timestamp lies outside canonical input")
    return count, timestamps


def oracle_diagnostics(path, mode):
    counts = {
        "snapshot_count": 0,
        "base_eligible_count": 0,
        "oracle_mask_hit_count": 0,
        "oracle_pass_count": 0,
        "factor_added_count": 0,
    }
    with path.open(encoding="utf-8", newline="") as stream:
        reader = csv.DictReader(stream)
        required = {
            "velocity_factor_base_eligible", "velocity_oracle_mask_loaded",
            "velocity_oracle_mask_hit", "velocity_oracle_pass",
            "velocity_factor_added"}
        if reader.fieldnames is None or not required.issubset(reader.fieldnames):
            raise RuntimeError("LTV CSV lacks frozen Oracle diagnostics")
        for line_number, row in enumerate(reader, start=2):
            values = {name: row[name] == "1" for name in required}
            counts["snapshot_count"] += 1
            counts["base_eligible_count"] += values["velocity_factor_base_eligible"]
            oracle_mode = mode.startswith("v_oracle")
            if oracle_mode:
                counts["oracle_mask_hit_count"] += values["velocity_oracle_mask_hit"]
                counts["oracle_pass_count"] += values["velocity_oracle_pass"]
            counts["factor_added_count"] += values["velocity_factor_added"]
            if oracle_mode and values["velocity_factor_added"]:
                if not all((
                        values["velocity_factor_base_eligible"],
                        values["velocity_oracle_mask_loaded"],
                        values["velocity_oracle_mask_hit"],
                        values["velocity_oracle_pass"])):
                    raise RuntimeError(
                        f"{path}:{line_number}: factor bypassed frozen mask")
            if mode == "reference" and values["velocity_factor_added"]:
                raise RuntimeError(f"{path}:{line_number}: baseline added velocity factor")
    counts["oracle_mask_miss_count"] = (
        counts["base_eligible_count"] - counts["oracle_mask_hit_count"]
        if mode.startswith("v_oracle") else 0)
    return counts


def validate_output(output, metadata, mode):
    summary_path = output / "replay_summary.json"
    if not summary_path.is_file():
        raise RuntimeError("offline replay did not write replay_summary.json")
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    expected = metadata["paired_count"]
    if (summary.get("canonical_pair_count") != expected or
            summary.get("consumed_pair_count") != expected):
        raise RuntimeError("replay did not consume every canonical stereo pair")

    vio_count, _ = validate_vio(
        output / "vio.csv", metadata["frame_time_range_ns"])
    ltv_count, snapshots = validate_csv(
        output / "ltv_debug.csv", "frame_timestamp",
        metadata["frame_time_range_ns"], nonnumeric={"reset_reason"})
    diagnostics = oracle_diagnostics(output / "ltv_debug.csv", mode)
    diagnostics.update({
        "vio_row_count": vio_count,
        "ltv_row_count": ltv_count,
        "nonzero_snapshot_count": len(snapshots),
    })

    log = (output / "replay.log").read_text(encoding="utf-8", errors="replace")
    forbidden = (
        "Solver is unusable", "Failed to delete datawriter", "SIGSEGV",
        "Error in destruction of rcl publisher handle")
    found = [marker for marker in forbidden if marker in log]
    if found:
        raise RuntimeError(f"replay log contains failure markers: {found}")
    return diagnostics


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", required=True, choices=sorted(MODES))
    parser.add_argument("--config", required=True, type=Path)
    parser.add_argument("--cache", required=True, type=Path)
    parser.add_argument("--oracle-mask", type=Path)
    parser.add_argument("--stage5-replay", default="stage5_replay")
    parser.add_argument(
        "--output-root", default="/home/he/output/ltv_stage5_minimal_fix",
        type=Path)
    arguments = parser.parse_args()

    metadata_path = arguments.cache / "metadata.json"
    canonical = arguments.cache / "canonical_stereo_pairs.csv"
    imu = arguments.cache / "imu.csv"
    if not metadata_path.is_file() or not canonical.is_file() or not imu.is_file():
        raise RuntimeError("cache is incomplete; run stage5_prepare_euroc first")
    metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
    if metadata.get("format") != "ltv-stage5-cache-v2":
        raise RuntimeError("Stage 5 cache format is not canonical v2")
    if (metadata.get("pair_list_sha256") != sha256(canonical) or
            metadata.get("imu_sha256") != sha256(imu)):
        raise RuntimeError("cache manifest hash mismatch; refusing modified cache")
    ground_truth = Path(metadata["ground_truth"])
    if (not ground_truth.is_file() or
            metadata.get("ground_truth_sha256") != sha256(ground_truth)):
        raise RuntimeError("official GT changed or is unavailable")

    velocity, oracle, mask_column = MODES[arguments.mode]
    if oracle and (arguments.oracle_mask is None or not arguments.oracle_mask.is_file()):
        raise RuntimeError("Oracle modes require an existing frozen --oracle-mask")
    mask_sha = sha256(arguments.oracle_mask) if oracle else None

    sequence_root = arguments.output_root / arguments.cache.name
    sequence_root.mkdir(parents=True, exist_ok=True)
    final = sequence_root / arguments.mode
    if final.exists():
        raise RuntimeError(f"refusing to overwrite formal result: {final}")
    run_id = uuid.uuid4().hex
    partial = sequence_root / f"{arguments.mode}.partial-{run_id}"
    partial.mkdir()

    base_config_sha = sha256(arguments.config)
    config_text = arguments.config.read_text(encoding="utf-8")
    settings = {
        "output_path": f'"{partial}"',
        "freq": "20",
        "show_track": "0",
        "save_image": "0",
        "multiple_thread": "0",
        "ltv_log_debug": "1",
        "ltv_enable_gravity_factor": "0",
        "ltv_enable_velocity_factor": str(velocity),
        "ltv_enable_velocity_oracle_gate": str(oracle),
        "ltv_debug_csv_path": f'"{partial / "ltv_debug.csv"}"',
        "ltv_velocity_oracle_mask_path": (
            f'"{arguments.oracle_mask.resolve()}"' if oracle else '""'),
        "ltv_velocity_oracle_mask_column": f'"{mask_column}"',
    }
    for key, value in settings.items():
        config_text = replace_setting(config_text, key, value)
    effective_config_sha = hashlib.sha256(config_text.encode()).hexdigest()
    (partial / "effective_config.yaml").write_text(config_text, encoding="utf-8")
    input_manifest = {
        "cache": str(arguments.cache.resolve()),
        "pair_list_sha256": metadata["pair_list_sha256"],
        "imu_sha256": metadata["imu_sha256"],
        "ground_truth_sha256": metadata["ground_truth_sha256"],
        "base_config": str(arguments.config.resolve()),
        "base_config_sha256": base_config_sha,
        "effective_config_sha256": effective_config_sha,
        "mode": arguments.mode,
        "oracle_mask": str(arguments.oracle_mask.resolve()) if oracle else None,
        "oracle_mask_sha256": mask_sha,
        "oracle_mask_column": mask_column if oracle else None,
    }
    (partial / "input_manifest.json").write_text(
        json.dumps(input_manifest, indent=2, sort_keys=True) + "\n",
        encoding="utf-8")

    temporary_config = tempfile.NamedTemporaryFile(
        mode="w", suffix=".yaml", prefix="stage5_",
        dir=str(arguments.config.resolve().parent), delete=False)
    temporary_path = Path(temporary_config.name)
    try:
        temporary_config.write(config_text)
        temporary_config.close()
        with (partial / "replay.log").open("w", encoding="utf-8") as log:
            subprocess.run(
                [arguments.stage5_replay, str(temporary_path),
                 str(arguments.cache.resolve())],
                check=True, stdout=log, stderr=subprocess.STDOUT)
        if sha256(arguments.config) != base_config_sha:
            raise RuntimeError("base config changed during replay")
        if (sha256(canonical) != metadata["pair_list_sha256"] or
                sha256(imu) != metadata["imu_sha256"]):
            raise RuntimeError("canonical input changed during replay")
        if oracle and sha256(arguments.oracle_mask) != mask_sha:
            raise RuntimeError("frozen Oracle mask changed during replay")
        diagnostics = validate_output(partial, metadata, arguments.mode)
        (partial / "validation.json").write_text(
            json.dumps(diagnostics, indent=2, sort_keys=True) + "\n",
            encoding="utf-8")
        os.rename(partial, final)
        print(final)
    except Exception:
        if partial.exists():
            timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
            failed = sequence_root / f"{arguments.mode}.failed-{timestamp}-{run_id}"
            os.rename(partial, failed)
            print(f"preserved failed run at {failed}")
        raise
    finally:
        temporary_path.unlink(missing_ok=True)


if __name__ == "__main__":
    main()
