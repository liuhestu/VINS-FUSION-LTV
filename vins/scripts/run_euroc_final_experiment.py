#!/usr/bin/env python3
"""Run and publish the fresh 55-run EuRoC final experiment transactionally."""

import argparse
import json
import os
import subprocess
import sys
import uuid
from datetime import datetime, timezone
from pathlib import Path

from evaluate_euroc_final import (
    MODES, SEQUENCES, atomic_write, evaluate, render_markdown, write_artifacts)
from run_stage5_velocity_oracle import sha256
from run_stage6_joint import FROZEN_SETTINGS


def batch_manifest(arguments, runner):
    return {
        "schema": "ltv-euroc-final-unified-run-v1",
        "config": str(arguments.config.resolve()),
        "config_sha256": sha256(arguments.config),
        "cache_root": str(arguments.cache_root.resolve()),
        "replay": str(arguments.replay.resolve()),
        "replay_sha256": sha256(arguments.replay),
        "runner": str(runner.resolve()),
        "runner_sha256": sha256(runner),
        "evaluator_sha256": sha256(
            Path(__file__).resolve().parent / "evaluate_euroc_final.py"),
        "sequences": list(SEQUENCES),
        "modes": list(MODES),
        "frozen_settings": FROZEN_SETTINGS,
        "expected_run_count": len(SEQUENCES) * len(MODES),
    }


def preserve_failed(partial, output_root, failure):
    if not partial.exists():
        return None
    atomic_write(
        partial / "batch_failure.json",
        json.dumps(failure, indent=2, sort_keys=True) + "\n")
    timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    failed = output_root.with_name(
        f"{output_root.name}.failed-{timestamp}-{uuid.uuid4().hex[:8]}")
    os.rename(partial, failed)
    return failed


def expected_pair_consumption(cache_root):
    one_pass = 0
    for sequence in SEQUENCES:
        metadata = json.loads(
            (cache_root / sequence / "metadata.json").read_text(
                encoding="utf-8"))
        one_pass += int(metadata["paired_count"])
    return one_pass * len(MODES)


def main():
    scripts = Path(__file__).resolve().parent
    repository = scripts.parents[1]
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--config", type=Path,
        default=repository / "config/euroc/euroc_stereo_imu_ltv_config.yaml")
    parser.add_argument(
        "--cache-root", type=Path,
        default=Path("/home/he/output/ltv_stage5_minimal_fix/cache"))
    parser.add_argument(
        "--replay", type=Path,
        default=Path("/home/he/vins_fusion_ltv_ws/build/vins/stage5_replay"))
    parser.add_argument(
        "--runner", type=Path, default=scripts / "run_stage6_joint.py")
    parser.add_argument(
        "--output-root", type=Path,
        default=Path("/home/he/output/ltv_euroc_final_unified"))
    parser.add_argument(
        "--markdown-output", type=Path,
        default=repository / "docs/euroc_final_results.md")
    arguments = parser.parse_args()

    for path, label in (
            (arguments.config, "config"), (arguments.replay, "replay"),
            (arguments.runner, "runner")):
        if not path.is_file():
            raise RuntimeError(f"missing {label}: {path}")
    if arguments.output_root.exists():
        raise RuntimeError(
            f"refusing to overwrite formal result: {arguments.output_root}")
    if arguments.markdown_output.exists():
        raise RuntimeError(
            f"refusing to overwrite final table: {arguments.markdown_output}")

    arguments.output_root.parent.mkdir(parents=True, exist_ok=True)
    partial = arguments.output_root.with_name(
        f"{arguments.output_root.name}.partial-{uuid.uuid4().hex}")
    partial.mkdir()
    manifest = batch_manifest(arguments, arguments.runner)
    atomic_write(
        partial / "batch_manifest.json",
        json.dumps(manifest, indent=2, sort_keys=True) + "\n")

    current = None
    try:
        total = len(SEQUENCES) * len(MODES)
        completed = 0
        for sequence in SEQUENCES:
            cache = arguments.cache_root / sequence
            for mode in MODES:
                completed += 1
                current = {"sequence": sequence, "mode": mode}
                print(
                    f"[{completed:02d}/{total}] {sequence} {mode}",
                    flush=True)
                command = [
                    sys.executable, str(arguments.runner.resolve()),
                    "--mode", mode,
                    "--config", str(arguments.config.resolve()),
                    "--cache", str(cache.resolve()),
                    "--replay", str(arguments.replay.resolve()),
                    "--output-root", str(partial.resolve()),
                ]
                result = subprocess.run(command, check=False)
                if result.returncode != 0:
                    raise RuntimeError(
                        f"{sequence}/{mode}: runner exited {result.returncode}")

        audit = evaluate(partial, arguments.cache_root)
        expected_consumption = expected_pair_consumption(arguments.cache_root)
        if audit["total_consumed_pair_count"] != expected_consumption:
            raise RuntimeError(
                "total canonical pair consumption differs from experiment plan")
        audit["execution"] = manifest
        write_artifacts(partial, audit)
        os.rename(partial, arguments.output_root)
        atomic_write(
            arguments.markdown_output, render_markdown(audit))
        print(json.dumps({
            "markdown": str(arguments.markdown_output.resolve()),
            "output_root": str(arguments.output_root.resolve()),
            "run_count": audit["run_count"],
            "total_consumed_pair_count": audit[
                "total_consumed_pair_count"],
        }, sort_keys=True), flush=True)
    except Exception as error:
        failure = {
            "error": str(error),
            "current_run": current,
        }
        failed = preserve_failed(partial, arguments.output_root, failure)
        if failed is not None:
            print(f"preserved failed experiment at {failed}", flush=True)
        raise


if __name__ == "__main__":
    main()
