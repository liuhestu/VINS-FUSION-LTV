#!/usr/bin/env python3
"""Run one frozen LTV factor mode as an atomic deterministic replay."""

import argparse
import csv
import hashlib
import json
import os
import subprocess
import tempfile
import uuid
from collections import Counter
from datetime import datetime, timezone
from pathlib import Path

from run_stage5_velocity_oracle import (
    replace_setting, sha256, validate_csv, validate_vio)


MODES = {
    "baseline": {
        "ltv_enable_gravity_factor": "0",
        "ltv_enable_velocity_factor": "0",
        "ltv_enable_gravity_quality_gate": "0",
        "ltv_enable_velocity_quality_gate": "0",
    },
    "joint": {
        "ltv_enable_gravity_factor": "1",
        "ltv_enable_velocity_factor": "1",
        "ltv_enable_gravity_quality_gate": "1",
        "ltv_enable_velocity_quality_gate": "0",
    },
    "joint_v_gate": {
        "ltv_enable_gravity_factor": "1",
        "ltv_enable_velocity_factor": "1",
        "ltv_enable_gravity_quality_gate": "1",
        "ltv_enable_velocity_quality_gate": "1",
    },
    "gravity_only": {
        "ltv_enable_gravity_factor": "1",
        "ltv_enable_velocity_factor": "0",
        "ltv_enable_gravity_quality_gate": "1",
        "ltv_enable_velocity_quality_gate": "0",
    },
    "velocity_only": {
        "ltv_enable_gravity_factor": "0",
        "ltv_enable_velocity_factor": "1",
        "ltv_enable_gravity_quality_gate": "0",
        "ltv_enable_velocity_quality_gate": "0",
    },
}

FROZEN_SETTINGS = {
    "ltv_enable": "1",
    "freq": "20",
    "show_track": "0",
    "save_image": "0",
    "multiple_thread": "0",
    "load_previous_pose_graph": "0",
    "ltv_log_debug": "1",
    "ltv_enable_velocity_oracle_gate": "0",
    "ltv_velocity_oracle_mask_path": '\"\"',
    "ltv_gravity_sigma_deg": "10.0",
    "ltv_velocity_sigma_mps": "1.0",
    "ltv_gravity_huber_delta": "2.0",
    "ltv_velocity_huber_delta": "2.0",
    "ltv_gravity_gate_min_features": "15",
    "ltv_gravity_gate_max_eta_norm_error": "0.2",
    "ltv_gravity_gate_max_normalized_innovation": "0.05",
    "ltv_gravity_gate_reset_cooldown_frames": "0",
    "ltv_velocity_gate_min_features": "25",
    "ltv_velocity_gate_max_normalized_innovation": "0.03",
    "ltv_velocity_gate_max_disagreement_mps": "0.5",
    "ltv_velocity_gate_reset_cooldown_frames": "10",
}

FAILURE_MARKERS = (
    "Solver is unusable",
    "Failed to delete datawriter",
    "SIGSEGV",
    "Error in destruction of rcl publisher handle",
    "NaN detected",
    "nan detected",
)


def effective_config(base_text, mode, output):
    if mode not in MODES:
        raise ValueError(f"unsupported Stage 6/6b mode: {mode}")
    settings = dict(FROZEN_SETTINGS)
    settings.update(MODES[mode])
    settings.update({
        "output_path": f'\"{output}\"',
        "ltv_debug_csv_path": f'\"{output / "ltv_debug.csv"}\"',
    })
    result = base_text
    for key, value in settings.items():
        result = replace_setting(result, key, value)
    return result, settings


def factor_diagnostics(path, mode):
    required = {
        "gravity_factor_added", "gravity_gate_base_eligible",
        "gravity_gate_pass", "gravity_gate_reason_mask",
        "velocity_factor_added", "velocity_factor_base_eligible",
        "velocity_gate_feature_ok", "velocity_gate_innovation_ok",
        "velocity_gate_disagreement_ok", "velocity_gate_reset_ok",
        "velocity_gate_pass", "velocity_gate_reason_mask",
        "velocity_oracle_mask_loaded", "velocity_oracle_mask_hit",
        "reset_reason",
    }
    counts = Counter({
        "snapshot_count": 0,
        "gravity_base_eligible_count": 0,
        "gravity_gate_pass_count": 0,
        "gravity_factor_added_count": 0,
        "velocity_base_eligible_count": 0,
        "velocity_gate_pass_count": 0,
        "velocity_factor_added_count": 0,
        "velocity_oracle_mask_loaded_count": 0,
        "velocity_oracle_mask_hit_count": 0,
        "reset_count": 0,
    })
    gravity_reasons = Counter()
    velocity_reasons = Counter()
    velocity_gate_on_segments = []
    velocity_gate_on_streak = 0
    with path.open(encoding="utf-8", newline="") as stream:
        reader = csv.DictReader(stream)
        if reader.fieldnames is None or not required.issubset(reader.fieldnames):
            raise RuntimeError("LTV CSV lacks Stage 6 diagnostics")
        for line_number, row in enumerate(reader, start=2):
            boolean_names = required - {
                "gravity_gate_reason_mask", "velocity_gate_reason_mask",
                "reset_reason"}
            try:
                values = {name: int(row[name]) for name in boolean_names}
                gravity_reason_mask = int(row["gravity_gate_reason_mask"])
                velocity_reason_mask = int(row["velocity_gate_reason_mask"])
            except (TypeError, ValueError) as error:
                raise RuntimeError(
                    f"{path}:{line_number}: invalid Stage 6 diagnostics") from error
            if any(value not in (0, 1) for value in values.values()):
                raise RuntimeError(
                    f"{path}:{line_number}: non-boolean Stage 6 diagnostic")

            counts["snapshot_count"] += 1
            counts["gravity_base_eligible_count"] += values[
                "gravity_gate_base_eligible"]
            counts["gravity_gate_pass_count"] += values["gravity_gate_pass"]
            counts["gravity_factor_added_count"] += values["gravity_factor_added"]
            counts["velocity_base_eligible_count"] += values[
                "velocity_factor_base_eligible"]
            counts["velocity_gate_pass_count"] += values["velocity_gate_pass"]
            counts["velocity_factor_added_count"] += values[
                "velocity_factor_added"]
            counts["velocity_oracle_mask_loaded_count"] += values[
                "velocity_oracle_mask_loaded"]
            counts["velocity_oracle_mask_hit_count"] += values[
                "velocity_oracle_mask_hit"]
            if row["reset_reason"] != "none":
                counts["reset_count"] += 1
            gravity_reasons[str(gravity_reason_mask)] += 1
            velocity_reasons[str(velocity_reason_mask)] += 1
            if values["velocity_gate_pass"]:
                velocity_gate_on_streak += 1
            elif velocity_gate_on_streak:
                velocity_gate_on_segments.append(velocity_gate_on_streak)
                velocity_gate_on_streak = 0

            if values["velocity_oracle_mask_loaded"] or values[
                    "velocity_oracle_mask_hit"]:
                raise RuntimeError(
                    f"{path}:{line_number}: Stage 6 used an Oracle mask")
            if values["gravity_factor_added"] and not (
                    values["gravity_gate_base_eligible"] and
                    values["gravity_gate_pass"]):
                raise RuntimeError(
                    f"{path}:{line_number}: gravity factor bypassed gate")
            if (values["velocity_factor_added"] and
                    not values["velocity_factor_base_eligible"]):
                raise RuntimeError(
                    f"{path}:{line_number}: velocity factor bypassed eligibility")
            if mode == "joint_v_gate" and values["velocity_factor_added"] and not (
                    values["velocity_factor_base_eligible"] and
                    values["velocity_gate_feature_ok"] and
                    values["velocity_gate_innovation_ok"] and
                    values["velocity_gate_disagreement_ok"] and
                    values["velocity_gate_reset_ok"] and
                    values["velocity_gate_pass"]):
                raise RuntimeError(
                    f"{path}:{line_number}: velocity factor bypassed quality gate")

    if velocity_gate_on_streak:
        velocity_gate_on_segments.append(velocity_gate_on_streak)

    if counts["snapshot_count"] == 0:
        raise RuntimeError("LTV CSV contains no snapshots")
    if mode == "baseline" and (
            counts["gravity_factor_added_count"] != 0 or
            counts["velocity_factor_added_count"] != 0):
        raise RuntimeError("baseline added an LTV factor")
    if mode == "gravity_only":
        if counts["gravity_factor_added_count"] == 0:
            raise RuntimeError("gravity_only mode added no gravity factors")
        if (counts["gravity_factor_added_count"] !=
                counts["gravity_gate_pass_count"]):
            raise RuntimeError(
                "gravity_only gravity factor/gate pass counts differ")
        if counts["velocity_factor_added_count"] != 0:
            raise RuntimeError("gravity_only mode added a velocity factor")
    if mode == "velocity_only":
        if counts["gravity_factor_added_count"] != 0:
            raise RuntimeError("velocity_only mode added a gravity factor")
        if counts["velocity_factor_added_count"] == 0:
            raise RuntimeError("velocity_only mode added no velocity factors")
        if (counts["velocity_factor_added_count"] !=
                counts["velocity_base_eligible_count"]):
            raise RuntimeError(
                "velocity_only fixed velocity factor count is inconsistent")
    if mode in ("joint", "joint_v_gate"):
        if counts["gravity_factor_added_count"] == 0:
            raise RuntimeError("joint mode added no gravity factors")
        if counts["velocity_factor_added_count"] == 0:
            raise RuntimeError("joint mode added no velocity factors")
        if (counts["gravity_factor_added_count"] !=
                counts["gravity_gate_pass_count"]):
            raise RuntimeError("joint gravity factor/gate pass counts differ")
        if mode == "joint" and (counts["velocity_factor_added_count"] !=
                                counts["velocity_base_eligible_count"]):
            raise RuntimeError("joint fixed velocity factor count is inconsistent")
        if mode == "joint_v_gate" and (
                counts["velocity_factor_added_count"] !=
                counts["velocity_gate_pass_count"]):
            raise RuntimeError("joint_v_gate velocity factor/gate pass counts differ")
        if mode == "joint_v_gate" and not (
                0 < counts["velocity_gate_pass_count"] <
                counts["velocity_base_eligible_count"]):
            raise RuntimeError(
                "joint_v_gate velocity gate coverage must be strictly between 0 and 1")

    result = dict(counts)
    result["gravity_gate_reason_mask_counts"] = dict(
        sorted(gravity_reasons.items()))
    result["velocity_gate_reason_mask_counts"] = dict(
        sorted(velocity_reasons.items()))
    result["velocity_gate_on_segment_lengths"] = velocity_gate_on_segments
    result["velocity_gate_on_segment_count"] = len(velocity_gate_on_segments)
    result["velocity_gate_max_on_segment"] = max(
        velocity_gate_on_segments, default=0)
    denominator = counts["gravity_base_eligible_count"]
    result["gravity_gate_coverage"] = (
        counts["gravity_gate_pass_count"] / denominator if denominator else 0.0)
    velocity_denominator = counts["velocity_base_eligible_count"]
    result["velocity_gate_coverage"] = (
        counts["velocity_gate_pass_count"] / velocity_denominator
        if velocity_denominator else 0.0)
    return result


def validate_baseline_vio(current, reference):
    if not reference.is_file():
        raise RuntimeError(f"Stage 6 baseline is unavailable: {reference}")
    current_sha = sha256(current)
    reference_sha = sha256(reference)
    if current_sha != reference_sha:
        raise RuntimeError("Stage 6b baseline vio.csv differs from Stage 6 baseline")
    return current_sha


def validate_log(path):
    log = path.read_text(encoding="utf-8", errors="replace")
    found = [marker for marker in FAILURE_MARKERS if marker in log]
    if found:
        raise RuntimeError(f"replay log contains failure markers: {found}")
    return {
        "solver_failure_count": log.count("Solver is unusable"),
        "dds_error_count": (
            log.count("Failed to delete datawriter") +
            log.count("Error in destruction of rcl publisher handle")),
        "nan_inf_count": 0,
    }


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
    diagnostics = factor_diagnostics(output / "ltv_debug.csv", mode)
    diagnostics.update(validate_log(output / "replay.log"))
    diagnostics.update({
        "canonical_pair_count": expected,
        "consumed_pair_count": summary["consumed_pair_count"],
        "vio_row_count": vio_count,
        "ltv_row_count": ltv_count,
        "nonzero_snapshot_count": len(snapshots),
    })
    return diagnostics


def publish_validated(partial, final):
    if final.exists():
        raise RuntimeError(f"refusing to overwrite formal result: {final}")
    os.rename(partial, final)


def preserve_failed(partial, sequence_root, mode, run_id):
    if not partial.exists():
        return None
    timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    failed = sequence_root / f"{mode}.failed-{timestamp}-{run_id}"
    os.rename(partial, failed)
    return failed


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", required=True, choices=sorted(MODES))
    parser.add_argument("--config", required=True, type=Path)
    parser.add_argument("--cache", required=True, type=Path)
    parser.add_argument("--replay", required=True)
    parser.add_argument("--output-root", required=True, type=Path)
    parser.add_argument(
        "--stage6-baseline-root", type=Path,
        help="optional historical baseline root for an explicit SHA check")
    arguments = parser.parse_args()

    metadata_path = arguments.cache / "metadata.json"
    canonical = arguments.cache / "canonical_stereo_pairs.csv"
    imu = arguments.cache / "imu.csv"
    if not metadata_path.is_file() or not canonical.is_file() or not imu.is_file():
        raise RuntimeError("cache is incomplete; run stage5_prepare_euroc first")
    metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
    if metadata.get("format") != "ltv-stage5-cache-v2":
        raise RuntimeError("Stage 6 requires canonical v2 cache")
    if (metadata.get("pair_list_sha256") != sha256(canonical) or
            metadata.get("imu_sha256") != sha256(imu)):
        raise RuntimeError("cache manifest hash mismatch; refusing modified cache")
    ground_truth = Path(metadata["ground_truth"])
    if (not ground_truth.is_file() or
            metadata.get("ground_truth_sha256") != sha256(ground_truth)):
        raise RuntimeError("official GT changed or is unavailable")

    sequence_root = arguments.output_root / arguments.cache.name
    sequence_root.mkdir(parents=True, exist_ok=True)
    final = sequence_root / arguments.mode
    if final.exists():
        raise RuntimeError(f"refusing to overwrite formal result: {final}")
    run_id = uuid.uuid4().hex
    partial = sequence_root / f"{arguments.mode}.partial-{run_id}"
    partial.mkdir()

    base_config_sha = sha256(arguments.config)
    config_text, settings = effective_config(
        arguments.config.read_text(encoding="utf-8"), arguments.mode, partial)
    effective_config_sha = hashlib.sha256(config_text.encode()).hexdigest()
    (partial / "effective_config.yaml").write_text(config_text, encoding="utf-8")
    input_manifest = {
        "cache": str(arguments.cache.resolve()),
        "pair_list_sha256": metadata["pair_list_sha256"],
        "imu_sha256": metadata["imu_sha256"],
        "ground_truth_sha256": metadata["ground_truth_sha256"],
        "base_config": str(arguments.config.resolve()),
        "base_config_sha256": base_config_sha,
        "replay": str(Path(arguments.replay).resolve()),
        "replay_sha256": sha256(Path(arguments.replay)),
        "effective_config_sha256": effective_config_sha,
        "mode": arguments.mode,
        "frozen_settings": settings,
    }
    (partial / "input_manifest.json").write_text(
        json.dumps(input_manifest, indent=2, sort_keys=True) + "\n",
        encoding="utf-8")

    temporary_config = tempfile.NamedTemporaryFile(
        mode="w", suffix=".yaml", prefix="stage6_",
        dir=str(arguments.config.resolve().parent), delete=False)
    temporary_path = Path(temporary_config.name)
    try:
        temporary_config.write(config_text)
        temporary_config.close()
        with (partial / "replay.log").open("w", encoding="utf-8") as log:
            completed = subprocess.run(
                [arguments.replay, str(temporary_path),
                 str(arguments.cache.resolve())],
                check=True, stdout=log, stderr=subprocess.STDOUT)
        if sha256(arguments.config) != base_config_sha:
            raise RuntimeError("base config changed during replay")
        if (sha256(canonical) != metadata["pair_list_sha256"] or
                sha256(imu) != metadata["imu_sha256"]):
            raise RuntimeError("canonical input changed during replay")
        if sha256(ground_truth) != metadata["ground_truth_sha256"]:
            raise RuntimeError("official GT changed during replay")
        diagnostics = validate_output(partial, metadata, arguments.mode)
        diagnostics["replay_exit_code"] = completed.returncode
        if (arguments.mode == "baseline" and
                arguments.stage6_baseline_root is not None):
            diagnostics["stage6_baseline_vio_sha256"] = validate_baseline_vio(
                partial / "vio.csv",
                arguments.stage6_baseline_root / arguments.cache.name /
                "baseline" / "vio.csv")
        (partial / "validation.json").write_text(
            json.dumps(diagnostics, indent=2, sort_keys=True) + "\n",
            encoding="utf-8")
        publish_validated(partial, final)
        print(final)
    except Exception:
        failed = preserve_failed(partial, sequence_root, arguments.mode, run_id)
        if failed is not None:
            print(f"preserved failed run at {failed}")
        raise
    finally:
        temporary_path.unlink(missing_ok=True)


if __name__ == "__main__":
    main()
