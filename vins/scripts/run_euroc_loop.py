#!/usr/bin/env python3
"""Run the ROS 2 VINS + Loop Fusion EuRoC baseline.

Each sequence is isolated in an absolute output directory.  The runner keeps
the raw node logs, effective configuration, pose graph, and a JSON manifest so
that a failed or degraded sequence can be audited without rerunning it.
"""

import argparse
import hashlib
import json
import os
import signal
import shutil
import subprocess
import time
from pathlib import Path


SEQUENCES = ("MH_01_easy", "MH_02_easy", "MH_03_medium", "MH_04_difficult",
             "MH_05_difficult", "V1_01_easy", "V1_02_medium", "V1_03_difficult",
             "V2_01_easy", "V2_02_medium", "V2_03_difficult")


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def stop(process, timeout):
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


def bag_path(root, sequence):
    for candidate in (root / sequence, root / (sequence + "_db"),
                      root / (sequence + ".db3"), root / (sequence + ".bag")):
        if candidate.exists():
            return candidate
    raise FileNotFoundError(f"no ROS 2 bag found for {sequence} below {root}")


def run_one(args, sequence):
    bag = bag_path(args.dataset_root, sequence)
    output = args.output_root.resolve() / sequence
    if output.exists() and not args.resume:
        raise RuntimeError(f"refusing to overwrite {output}; use --resume")
    output.mkdir(parents=True, exist_ok=True)
    config = args.config.resolve()
    effective = output / "euroc_stereo_imu_config.yaml"
    text = config.read_text(encoding="utf-8")
    text = text.replace('output_path: "~/output/"', f'output_path: "{output}/"')
    text = text.replace('load_previous_pose_graph: 0', 'load_previous_pose_graph: 0')
    text = text.replace('pose_graph_save_path: "~/output/pose_graph/"',
                        f'pose_graph_save_path: "{output / "pose_graph"}/"')
    effective.write_text(text, encoding="utf-8")
    (output / "pose_graph").mkdir(exist_ok=True)
    for calibration in ("cam0_mei.yaml", "cam1_mei.yaml"):
        source = config.parent / calibration
        if not source.is_file():
            raise FileNotFoundError(source)
        shutil.copy2(source, output / calibration)
    manifest = {
        "schema": "euroc-loop-run-v1", "sequence": sequence,
        "bag": str(bag.resolve()), "bag_sha256": sha256(bag / "metadata.yaml")
        if (bag / "metadata.yaml").is_file() else None,
        "config": str(config), "config_sha256": sha256(config),
        "effective_config_sha256": hashlib.sha256(text.encode()).hexdigest(),
        "ltv_enabled": False, "loop_enabled": True, "rate": args.rate,
    }
    (output / "run_manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    env = os.environ.copy()
    env["ROS_LOG_DIR"] = str(output / "ros_log")
    env.setdefault("VINS_FUSION_SUPPORT_FILES", str(Path(__file__).resolve().parents[2] / "support_files"))
    (output / "ros_log").mkdir(exist_ok=True)
    vins_log = (output / "vins.log").open("w", encoding="utf-8")
    loop_log = (output / "loop.log").open("w", encoding="utf-8")
    bag_log = (output / "bag.log").open("w", encoding="utf-8")
    vins = loop = bag_process = None
    try:
        vins = subprocess.Popen([str(args.vins), str(effective), "--ros-args",
            "-r", "odometry:=/vins_estimator/odometry", "-r", "keyframe_pose:=/vins_estimator/keyframe_pose",
            "-r", "keyframe_point:=/vins_estimator/keyframe_point", "-r", "extrinsic:=/vins_estimator/extrinsic"],
            stdout=vins_log, stderr=subprocess.STDOUT, env=env)
        time.sleep(args.start_delay)
        loop = subprocess.Popen([str(args.loop), str(effective)], stdin=subprocess.PIPE,
                                stdout=loop_log, stderr=subprocess.STDOUT, env=env)
        time.sleep(args.start_delay)
        bag_process = subprocess.Popen(["ros2", "bag", "play", str(bag), "--rate", str(args.rate)],
                                       stdout=bag_log, stderr=subprocess.STDOUT, env=env)
        bag_code = bag_process.wait()
        time.sleep(args.drain_delay)
        if loop.poll() is None and loop.stdin:
            loop.stdin.write(b"s\n")
            loop.stdin.flush()
        try:
            loop_code = loop.wait(timeout=args.loop_timeout)
        except subprocess.TimeoutExpired:
            stop(loop, args.shutdown_timeout)
            loop_code = loop.returncode
        stop(vins, args.shutdown_timeout)
        manifest.update({"bag_exit_code": bag_code, "loop_exit_code": loop_code,
                         "vins_exit_code": vins.returncode,
                         "pose_graph": str(output / "pose_graph" / "pose_graph.txt"),
                         "loop_diagnostics": str(output / "loop.log")})
        (output / "run_manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
        if bag_code != 0 or not (output / "pose_graph" / "pose_graph.txt").is_file():
            raise RuntimeError(f"{sequence} failed; inspect {output}")
    finally:
        stop(bag_process, 5); stop(loop, args.shutdown_timeout); stop(vins, args.shutdown_timeout)
        vins_log.close(); loop_log.close(); bag_log.close()


def main():
    parser = argparse.ArgumentParser()
    repository = Path(__file__).resolve().parents[2]
    workspace = repository.parents[1]
    parser.add_argument("--dataset-root", type=Path, required=True)
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--config", type=Path, default=repository / "config/euroc/euroc_stereo_imu_config.yaml")
    parser.add_argument("--vins", type=Path, default=workspace / "install/vins/lib/vins/vins_node")
    parser.add_argument("--loop", type=Path, default=workspace / "install/loop_fusion/lib/loop_fusion/loop_fusion_node")
    parser.add_argument("--sequence", action="append", choices=SEQUENCES)
    parser.add_argument("--rate", type=float, default=1.0)
    parser.add_argument("--start-delay", type=float, default=3.0)
    parser.add_argument("--drain-delay", type=float, default=8.0)
    parser.add_argument("--loop-timeout", type=float, default=60.0)
    parser.add_argument("--shutdown-timeout", type=float, default=20.0)
    parser.add_argument("--resume", action="store_true")
    args = parser.parse_args()
    args.output_root.mkdir(parents=True, exist_ok=True)
    for sequence in args.sequence or SEQUENCES:
        run_one(args, sequence)


if __name__ == "__main__":
    main()
