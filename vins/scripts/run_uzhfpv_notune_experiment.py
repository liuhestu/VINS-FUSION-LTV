#!/usr/bin/env python3
"""Prepare, run, audit, and publish the frozen UZH-FPV no-tune study."""

import argparse
import concurrent.futures
import json
import os
import subprocess
import sys
import uuid
from datetime import datetime, timezone
from pathlib import Path

from audit_uzhfpv_time_offsets import (
    CATEGORIES, SEQUENCES, build_audit, sequence_family)
from evaluate_euroc_final import MODES, atomic_write
from evaluate_uzhfpv_notune import evaluate, write_artifacts
from run_stage5_velocity_oracle import sha256
from run_stage6_joint import FROZEN_SETTINGS


def run(command):
    return subprocess.run(command, check=False).returncode


def prepare_caches(arguments, scripts, repository):
    if arguments.cache_root.exists():
        for sequence in SEQUENCES:
            if not (arguments.cache_root / sequence / "metadata.json").is_file():
                raise RuntimeError(f"incomplete existing cache: {sequence}")
        return
    partial = arguments.cache_root.with_name(
        f"{arguments.cache_root.name}.partial-{uuid.uuid4().hex}")
    partial.mkdir(parents=True)
    try:
        for index, sequence in enumerate(SEQUENCES, 1):
            family = sequence_family(sequence)
            category = CATEGORIES[family]
            bag = arguments.dataset_root / category / f"{sequence}_db"
            ground_truth = (
                arguments.dataset_root / "archives" / sequence / "groundtruth.txt")
            print(f"[cache {index:02d}/{len(SEQUENCES)}] {sequence}", flush=True)
            command = [
                sys.executable, str(scripts / "stage5_prepare_euroc.py"),
                "--bag", str(bag), "--ground-truth", str(ground_truth),
                "--output", str(partial / sequence),
                "--stereo-pairer", str(arguments.stereo_pairer),
                "--imu-topic", "/snappy_imu",
                "--left-topic", "/snappy_cam/stereo_l",
                "--right-topic", "/snappy_cam/stereo_r",
                "--require-imu-bracketing",
                "--imu-bracketing-margin-s", "0.05",
            ]
            if run(command) != 0:
                raise RuntimeError(f"cache preparation failed: {sequence}")
        os.rename(partial, arguments.cache_root)
    except Exception:
        failed = arguments.cache_root.with_name(
            f"{arguments.cache_root.name}.failed-"
            f"{datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')}")
        if partial.exists():
            os.rename(partial, failed)
        raise


def execution_manifest(arguments, repository, scripts):
    settings = dict(FROZEN_SETTINGS)
    del settings["freq"]
    return {
        "schema": "ltv-uzhfpv-notune-run-v1",
        "dataset_root": str(arguments.dataset_root.resolve()),
        "cache_root": str(arguments.cache_root.resolve()),
        "sequences": list(SEQUENCES),
        "modes": list(MODES),
        "expected_run_count": len(SEQUENCES) * len(MODES),
        "frozen_settings": settings,
        "preserved_front_end_frequency": 31,
        "parameter_source": str(
            (repository / "docs/euroc_notune_results.md").resolve()),
        "parameter_source_sha256": sha256(
            repository / "docs/euroc_notune_results.md"),
        "runner_sha256": sha256(scripts / "run_stage6_joint.py"),
        "replay_sha256": sha256(arguments.replay),
        "evaluator_sha256": sha256(scripts / "evaluate_uzhfpv_notune.py"),
        "time_auditor_sha256": sha256(
            scripts / "audit_uzhfpv_time_offsets.py"),
        "configs": {
            category: {
                "path": str((repository / "config" / category /
                             "uzhfpv_stereo_imu_config.yaml").resolve()),
                "sha256": sha256(repository / "config" / category /
                                 "uzhfpv_stereo_imu_config.yaml"),
            } for category in CATEGORIES.values()},
    }


def run_mode(arguments, scripts, repository, partial, sequence, mode):
    family = sequence_family(sequence)
    config = (repository / "config" / CATEGORIES[family] /
              "uzhfpv_stereo_imu_config.yaml")
    command = [
        sys.executable, str(scripts / "run_stage6_joint.py"),
        "--mode", mode, "--config", str(config),
        "--cache", str(arguments.cache_root / sequence),
        "--replay", str(arguments.replay),
        "--output-root", str(partial), "--preserve-config-frequency",
    ]
    return run(command)


def run_baselines(arguments, scripts, repository, partial):
    failures = []
    with concurrent.futures.ThreadPoolExecutor(
            max_workers=arguments.workers) as executor:
        futures = {
            executor.submit(run_mode, arguments, scripts, repository, partial,
                            sequence, "baseline"): sequence
            for sequence in SEQUENCES}
        for completed, future in enumerate(
                concurrent.futures.as_completed(futures), 1):
            sequence = futures[future]
            return_code = future.result()
            print(f"[baseline {completed:02d}/{len(SEQUENCES)}] {sequence} "
                  f"exit={return_code}", flush=True)
            if return_code != 0:
                failures.append({"sequence": sequence, "mode": "baseline"})
    return failures


def run_sequence_modes(arguments, scripts, repository, partial, sequence):
    failures = []
    for mode in MODES[1:]:
        return_code = run_mode(
            arguments, scripts, repository, partial, sequence, mode)
        if return_code != 0:
            failures.append({"sequence": sequence, "mode": mode})
    return failures


def run_remaining_modes(arguments, scripts, repository, partial):
    failures = []
    with concurrent.futures.ThreadPoolExecutor(
            max_workers=arguments.workers) as executor:
        futures = {
            executor.submit(run_sequence_modes, arguments, scripts, repository,
                            partial, sequence): sequence
            for sequence in SEQUENCES}
        for completed, future in enumerate(
                concurrent.futures.as_completed(futures), 1):
            sequence = futures[future]
            sequence_failures = future.result()
            failures.extend(sequence_failures)
            print(f"[sequence {completed:02d}/{len(SEQUENCES)}] {sequence} "
                  f"remaining modes complete; failures={len(sequence_failures)}",
                  flush=True)
    return failures


def main():
    scripts = Path(__file__).resolve().parent
    repository = scripts.parents[1]
    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset-root", type=Path,
                        default=Path("/home/he/datasets/uzhfpv"))
    parser.add_argument("--cache-root", type=Path,
                        default=Path("/home/he/output/ltv_uzhfpv_notune_cache"))
    parser.add_argument("--output-root", type=Path,
                        default=Path("/home/he/output/ltv_uzhfpv_notune"))
    parser.add_argument("--replay", type=Path,
                        default=Path("/home/he/vins_fusion_ltv_ws/build/vins/stage5_replay"))
    parser.add_argument("--stereo-pairer", type=Path,
                        default=Path("/home/he/vins_fusion_ltv_ws/build/vins/stage5_pair_stereo"))
    parser.add_argument("--markdown-output", type=Path,
                        default=repository / "docs/uzhfpv_notune_results.md")
    parser.add_argument("--time-audit-output", type=Path,
                        default=repository / "config/uzhfpv_gt_time_offsets.json")
    parser.add_argument("--workers", type=int, default=8)
    arguments = parser.parse_args()
    if arguments.workers < 1:
        raise RuntimeError("workers must be positive")

    for path, label in ((arguments.replay, "replay"),
                        (arguments.stereo_pairer, "stereo pairer")):
        if not path.is_file():
            raise RuntimeError(f"missing {label}: {path}")
    if arguments.output_root.exists():
        raise RuntimeError(f"refusing to overwrite {arguments.output_root}")
    if arguments.markdown_output.exists():
        raise RuntimeError(f"refusing to overwrite {arguments.markdown_output}")

    prepare_caches(arguments, scripts, repository)
    partial = arguments.output_root.with_name(
        f"{arguments.output_root.name}.partial-{uuid.uuid4().hex}")
    partial.mkdir(parents=True)
    execution = execution_manifest(arguments, repository, scripts)
    failures = []
    try:
        print(f"running baselines with {arguments.workers} sequence workers",
              flush=True)
        failures.extend(run_baselines(
            arguments, scripts, repository, partial))
        if failures:
            raise RuntimeError(f"baseline phase failed: {failures}")

        audit = build_audit(arguments.dataset_root, arguments.cache_root,
                            partial, repository)
        atomic_write(arguments.time_audit_output,
                     json.dumps(audit, indent=2, sort_keys=True) + "\n")
        execution["time_audit"] = str(arguments.time_audit_output.resolve())
        execution["time_audit_sha256"] = sha256(arguments.time_audit_output)
        atomic_write(partial / "batch_manifest.json",
                     json.dumps(execution, indent=2, sort_keys=True) + "\n")

        failures.extend(run_remaining_modes(
            arguments, scripts, repository, partial))
        execution["failed_runs"] = failures
        atomic_write(partial / "batch_manifest.json",
                     json.dumps(execution, indent=2, sort_keys=True) + "\n")
        audit_result = evaluate(
            partial, arguments.cache_root, arguments.time_audit_output)
        temporary_markdown = partial / "uzhfpv_notune_results.md"
        write_artifacts(partial, audit_result, execution, temporary_markdown)
        os.rename(partial, arguments.output_root)
        atomic_write(arguments.markdown_output,
                     (arguments.output_root / "uzhfpv_notune_results.md").read_text(
                         encoding="utf-8"))
        print(json.dumps({
            "output_root": str(arguments.output_root.resolve()),
            "markdown": str(arguments.markdown_output.resolve()),
            "eligible_sequences": audit_result["eligible_sequence_count"],
            "failed_runs": failures,
        }, sort_keys=True), flush=True)
    except Exception as error:
        if partial.exists():
            atomic_write(partial / "batch_failure.json", json.dumps({
                "error": str(error), "failed_runs": failures,
            }, indent=2, sort_keys=True) + "\n")
            failed = arguments.output_root.with_name(
                f"{arguments.output_root.name}.failed-"
                f"{datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')}-"
                f"{uuid.uuid4().hex[:8]}")
            os.rename(partial, failed)
            print(f"preserved failed experiment at {failed}", flush=True)
        raise


if __name__ == "__main__":
    main()
