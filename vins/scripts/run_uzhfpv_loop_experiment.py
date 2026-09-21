#!/usr/bin/env python3
"""Run the frozen UZH-FPV indoor-forward Loop-on study at 1x bag rate."""

import argparse
import hashlib
import json
import os
import shutil
import signal
import subprocess
import sys
import time
import uuid
from pathlib import Path

from run_stage5_velocity_oracle import replace_setting, sha256
from run_stage6_joint import FROZEN_SETTINGS, MODES


SEQUENCES = (
    "indoor_forward_3_snapdragon_with_gt",
    "indoor_forward_5_snapdragon_with_gt",
    "indoor_forward_6_snapdragon_with_gt",
    "indoor_forward_7_snapdragon_with_gt",
    "indoor_forward_9_snapdragon_with_gt",
    "indoor_forward_10_snapdragon_with_gt",
)
LOOP_MODES = ("baseline", "gravity_only", "velocity_only", "joint", "joint_v_gate")
DISPLAY_NAMES = {
    "baseline": "B_loop",
    "gravity_only": "G_gate_loop",
    "velocity_only": "V_fixed_loop",
    "joint": "G_gate+V_fixed_loop",
    "joint_v_gate": "G_gate+V_gate_loop",
}


def effective_config(base_text, mode, output, graph_path, repository):
    settings = dict(FROZEN_SETTINGS)
    # UZH's calibrated 31 Hz front-end is part of the frozen dataset adapter;
    # only LTV and Loop settings are changed for this study.
    settings.pop("freq", None)
    settings.update(MODES[mode])
    settings.update({
        "output_path": f'"{output}"',
        "ltv_debug_csv_path": f'"{output / "ltv_debug.csv"}"',
        "load_previous_pose_graph": "0",
        "pose_graph_save_path": f'"{graph_path}{os.sep}"',
    })
    result = base_text
    for key, value in settings.items():
        result = replace_setting(result, key, value)
    return result, settings


def stop_process(process, timeout=10):
    if process is None or process.poll() is not None:
        return
    process.send_signal(signal.SIGINT)
    try:
        process.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        process.terminate()
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()


def run_one(arguments, repository, sequence, mode):
    category = "uzhfpv_indoor"
    bag = arguments.dataset_root / category / f"{sequence}_db"
    base_config = repository / "config" / category / "uzhfpv_stereo_imu_config.yaml"
    sequence_root = arguments.output_root / sequence
    final = sequence_root / mode
    if final.exists():
        raise RuntimeError(f"refusing to overwrite {final}")
    run_id = uuid.uuid4().hex
    partial = sequence_root / f"{mode}.partial-{run_id}"
    partial.mkdir(parents=True)
    graph_path = partial / "pose_graph"
    graph_path.mkdir()
    config_text, settings = effective_config(
        base_config.read_text(encoding="utf-8"), mode, partial, graph_path,
        repository)
    effective = partial / "effective_config.yaml"
    effective.write_text(config_text, encoding="utf-8")
    shutil.copy2(repository / "config" / "uzhfpv_indoor" / "cam0.yaml", partial / "cam0.yaml")
    shutil.copy2(repository / "config" / "uzhfpv_indoor" / "cam1.yaml", partial / "cam1.yaml")
    (partial / "input_manifest.json").write_text(json.dumps({
        "schema": "uzhfpv-loop-run-v1",
        "sequence": sequence,
        "mode": mode,
        "display_name": DISPLAY_NAMES[mode],
        "bag": str(bag.resolve()),
        "bag_metadata_sha256": sha256(bag / "metadata.yaml"),
        "base_config": str(base_config.resolve()),
        "base_config_sha256": sha256(base_config),
        "effective_config_sha256": hashlib.sha256(config_text.encode()).hexdigest(),
        "frozen_settings": settings,
        "loop_enabled": True,
        "bag_rate": arguments.rate,
    }, indent=2, sort_keys=True) + "\n", encoding="utf-8")

    logs = {name: (partial / f"{name}.log").open("w", encoding="utf-8")
            for name in ("vins", "loop", "bag")}
    vins = loop = bag_process = None
    started = time.time()
    try:
        env = os.environ.copy()
        env["ROS_LOG_DIR"] = str(partial / "ros_log")
        env["VINS_FUSION_SUPPORT_FILES"] = str(repository / "support_files")
        (partial / "ros_log").mkdir()
        vins = subprocess.Popen(
            [str(arguments.vins), str(effective), "--ros-args",
             "-r", "odometry:=/vins_estimator/odometry",
             "-r", "keyframe_pose:=/vins_estimator/keyframe_pose",
             "-r", "keyframe_point:=/vins_estimator/keyframe_point",
             "-r", "extrinsic:=/vins_estimator/extrinsic",
             "-r", "imu_propagate:=/vins_estimator/imu_propagate",
             "-r", "path:=/vins_estimator/path",
             "-r", "point_cloud:=/vins_estimator/point_cloud",
             "-r", "margin_cloud:=/vins_estimator/margin_cloud",
             "-r", "image_track:=/vins_estimator/image_track"], stdout=logs["vins"],
            stderr=subprocess.STDOUT, env=env)
        time.sleep(arguments.node_start_delay)
        loop = subprocess.Popen(
            [str(arguments.loop), str(effective)], stdin=subprocess.PIPE,
            stdout=logs["loop"], stderr=subprocess.STDOUT, env=env)
        time.sleep(arguments.node_start_delay)
        bag_process = subprocess.Popen(
            ["ros2", "bag", "play", str(bag), "--rate", str(arguments.rate)],
            stdout=logs["bag"], stderr=subprocess.STDOUT, env=env)
        bag_return = bag_process.wait()
        # Loop Fusion has two asynchronous queues and a 2 s update period.
        # Leave a deterministic drain interval before asking it to save.
        time.sleep(arguments.drain_delay)
        if loop.poll() is None and loop.stdin is not None:
            loop.stdin.write(b"s\n")
            loop.stdin.flush()
        try:
            loop_return = loop.wait(timeout=arguments.loop_timeout)
        except subprocess.TimeoutExpired:
            # Saving a dense pose graph can outlive the nominal drain window.
            # Interrupt once more, then classify based on the artifacts below.
            stop_process(loop, timeout=arguments.loop_shutdown_timeout)
            loop_return = loop.returncode
        stop_process(vins, timeout=arguments.vins_timeout)
        vins_return = vins.returncode
        csv_path = partial / "vio_loop.csv"
        graph_file = graph_path / "pose_graph.txt"
        # The upstream node throws during destruction of its detached optimizer
        # thread after it has already saved the complete trajectory.  Accept
        # that known SIGABRT only when both trajectory and pose-graph artifacts
        # are present; any earlier crash remains a failed run.
        loop_shutdown_ok = loop_return == 0 or (
            loop_return in (-2, -6, -9, -11, -15) and csv_path.is_file() and graph_file.is_file())
        complete = bag_return == 0 and loop_shutdown_ok and csv_path.is_file()
        result = {
            "sequence": sequence, "mode": mode, "status": "complete" if complete else "failed",
            "bag_exit_code": bag_return, "loop_exit_code": loop_return,
            "loop_shutdown_status": "clean" if loop_return == 0 else "known_post_save_crash",
            "vins_exit_code": vins_return, "vio_loop_exists": csv_path.is_file(),
            "pose_graph_exists": graph_file.is_file(),
            "wall_duration_s": time.time() - started,
        }
        (partial / "run_summary.json").write_text(
            json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        if complete:
            os.rename(partial, final)
        else:
            failed = sequence_root / f"{mode}.failed-{run_id}"
            os.rename(partial, failed)
            raise RuntimeError(f"{sequence}/{mode} failed: {result}")
        return result
    except Exception:
        stop_process(bag_process)
        stop_process(loop)
        stop_process(vins)
        raise
    finally:
        for stream in logs.values():
            stream.close()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset-root", type=Path, default=Path("/home/he/datasets/uzhfpv"))
    parser.add_argument("--output-root", type=Path, default=Path("/home/he/output/uzhfpv_loop"))
    parser.add_argument("--rate", type=float, default=1.0)
    parser.add_argument("--node-start-delay", type=float, default=3.0)
    parser.add_argument("--drain-delay", type=float, default=8.0)
    parser.add_argument("--loop-timeout", type=float, default=60.0)
    parser.add_argument("--loop-shutdown-timeout", type=float, default=30.0)
    parser.add_argument("--vins-timeout", type=float, default=20.0)
    parser.add_argument("--sequence", action="append", choices=SEQUENCES)
    parser.add_argument("--mode", action="append", choices=LOOP_MODES)
    parser.add_argument("--resume", action="store_true")
    arguments = parser.parse_args()
    if arguments.rate <= 0 or arguments.node_start_delay < 0 or arguments.drain_delay < 0:
        raise RuntimeError("rate must be positive and delays must be non-negative")
    repository = Path(__file__).resolve().parents[2]
    arguments.vins = Path("/home/he/vins_fusion_ltv_ws/build/vins/vins_node")
    arguments.loop = Path("/home/he/vins_fusion_ltv_ws/build/loop_fusion/loop_fusion_node")
    for path in (arguments.vins, arguments.loop):
        if not path.is_file():
            raise RuntimeError(f"missing executable: {path}")
    selected_sequences = arguments.sequence or list(SEQUENCES)
    selected_modes = arguments.mode or list(LOOP_MODES)
    if arguments.output_root.exists() and not arguments.resume:
        raise RuntimeError(f"refusing to overwrite {arguments.output_root}")
    arguments.output_root.mkdir(parents=True, exist_ok=True)
    rows = []
    for sequence in selected_sequences:
        for mode in selected_modes:
            existing = arguments.output_root / sequence / mode / "run_summary.json"
            if arguments.resume and existing.is_file():
                summary = json.loads(existing.read_text(encoding="utf-8"))
                if summary.get("status") == "complete":
                    print(f"[loop] {sequence} {DISPLAY_NAMES[mode]} (resume: skip)", flush=True)
                    rows.append(summary)
                    continue
            print(f"[loop] {sequence} {DISPLAY_NAMES[mode]}", flush=True)
            rows.append(run_one(arguments, repository, sequence, mode))
    (arguments.output_root / "batch_manifest.json").write_text(json.dumps({
        "schema": "uzhfpv-loop-study-v1", "rate": arguments.rate,
        "sequences": selected_sequences, "modes": selected_modes,
        "runs": rows, "parameter_source": str((repository / "docs/euroc_notune_results.md").resolve()),
        "parameter_source_sha256": sha256(repository / "docs/euroc_notune_results.md"),
    }, indent=2, sort_keys=True) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
