#!/usr/bin/env python3
"""Evaluate final Loop Fusion trajectories for the UZH indoor-forward study."""

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path

import numpy as np

from evaluate_euroc_final import match_one_to_one, relative_change_percent
from evaluate_vins_euroc import read_uzh_ground_truth, rigid_alignment
from run_uzhfpv_loop_experiment import DISPLAY_NAMES, LOOP_MODES, SEQUENCES


def read_loop_csv(path):
    rows = []
    with path.open(encoding="utf-8") as stream:
        for line_number, line in enumerate(stream, 1):
            if not line.strip():
                continue
            fields = [value.strip() for value in line.split(",")]
            if fields and fields[-1] == "":
                fields.pop()
            if len(fields) != 8:
                raise RuntimeError(f"{path}:{line_number}: expected 8 columns")
            values = np.asarray([float(value) for value in fields], dtype=float)
            if not np.all(np.isfinite(values)):
                raise RuntimeError(f"{path}:{line_number}: non-finite value")
            rows.append(values)
    if len(rows) < 3:
        raise RuntimeError(f"{path}: fewer than three Loop trajectory samples")
    values = np.asarray(rows)
    timestamps = values[:, 0] * 1e-9
    if np.any(np.diff(timestamps) <= 0):
        raise RuntimeError(f"{path}: timestamps are not strictly increasing")
    quaternions = values[:, 4:8]
    norms = np.linalg.norm(quaternions, axis=1)
    if np.any(norms < 1e-12) or np.any(np.abs(norms - 1.0) > 1e-3):
        raise RuntimeError(f"{path}: invalid quaternion")
    return {"timestamps": timestamps, "positions": values[:, 1:4]}


def evaluate_one(sequence_root, sequence, audit_entry, modes, no_loop_root,
                 tolerance=0.02):
    gt_path = Path(audit_entry["gt_path"])
    gt = read_uzh_ground_truth(str(gt_path))
    gt["timestamps"] += audit_entry.get("applied_gt_time_offset_s", 0.0)
    loaded = {}
    run_summaries = {}
    for mode in modes:
        run_root = sequence_root / mode
        summary_path = run_root / "run_summary.json"
        if not summary_path.is_file():
            raise RuntimeError(f"{sequence}/{mode}: missing run summary")
        summary = json.loads(summary_path.read_text(encoding="utf-8"))
        if summary.get("status") != "complete":
            raise RuntimeError(f"{sequence}/{mode}: run status is not complete")
        loaded[mode] = read_loop_csv(run_root / "vio_loop.csv")
        run_summaries[mode] = summary
    # Loop Fusion emits sparse keyframes and the exact keyframe decision can
    # vary slightly between factor modes.  Match each mode independently to
    # GT, then compare the intersection of GT sample indices; requiring equal
    # estimator timestamps would incorrectly discard valid Loop trajectories.
    gt_maps = {}
    gt_errors = {}
    valid_gt_times = {}
    for mode in modes:
        gt_maps[mode], gt_errors[mode] = match_one_to_one(
            loaded[mode]["timestamps"], gt["timestamps"], tolerance)
        valid = gt_maps[mode] >= 0
        valid_gt_times[mode] = gt["timestamps"][gt_maps[mode][valid]]
    common_start = max(float(times[0]) for times in valid_gt_times.values())
    common_end = min(float(times[-1]) for times in valid_gt_times.values())
    if common_end <= common_start:
        raise RuntimeError(f"{sequence}: fewer than three common timestamps")
    metrics = {}
    timestamp_errors = []
    sample_counts = []
    for mode in modes:
        selected = [i for i, gt_index in enumerate(gt_maps[mode])
                    if gt_index >= 0 and common_start <= gt["timestamps"][gt_index] <= common_end]
        if len(selected) < 3:
            raise RuntimeError(f"{sequence}/{mode}: fewer than three common timestamps")
        sample_counts.append(len(selected))
        estimate = loaded[mode]["positions"][selected]
        reference = gt["positions"][gt_maps[mode][selected]]
        timestamp_errors.extend(np.abs(
            loaded[mode]["timestamps"][selected] -
            gt["timestamps"][gt_maps[mode][selected]]))
        rotation, translation = rigid_alignment(estimate, reference)
        aligned = (rotation @ estimate.T).T + translation
        distances = np.linalg.norm(aligned - reference, axis=1)
        metrics[mode] = {
            "ate_rmse_m": math.sqrt(float(np.mean(distances ** 2))),
            "position_p95_error_m": float(np.percentile(distances, 95)),
            "maximum_position_error_m": float(np.max(distances)),
            "position_p95_error_m": float(np.percentile(distances, 95)),
            "maximum_position_error_m": float(np.max(distances)),
        }
    diagnostics = {}
    for mode in modes:
        summary = run_summaries[mode]
        diagnostics[mode] = summary.get("diagnostics", {})
    reference_counts = [diagnostics[mode].get("no_loop_reference_vio_samples", 0)
                        for mode in modes]
    reference_count = max(reference_counts) if reference_counts else 0
    sample_ratios = {
        mode: (diagnostics[mode].get("vio_samples", 0) / reference_count
               if reference_count else 0.0)
        for mode in modes}
    keyframes = [diagnostics[mode].get("keyframe_count", 0) for mode in modes]
    keyframe_ratio = (min(keyframes) / max(keyframes)
                      if keyframes and max(keyframes) else 0.0)
    integrity = {
        "no_obvious_image_drops": all(
            diagnostics[mode].get("throw_img0_count", 0) == 0 and
            diagnostics[mode].get("throw_img1_count", 0) == 0
            for mode in modes),
        "vio_samples_close_to_no_loop": all(
            sample_ratios[mode] >= 0.95 for mode in modes),
        "keyframe_counts_consistent": keyframe_ratio >= 0.90,
    }
    integrity["eligible_for_loop_judgment"] = all(integrity.values())
    return {
        "sequence": sequence,
        "common_sample_count": min(sample_counts),
        "maximum_gt_time_error_s": float(np.max(timestamp_errors)),
        "maximum_cross_mode_time_error_s": float(np.max(timestamp_errors)),
        "trajectory_samples": {mode: int(len(loaded[mode]["timestamps"])) for mode in modes},
        "diagnostics": diagnostics,
        "no_loop_reference_vio_samples": reference_count,
        "vio_sample_ratios": sample_ratios,
        "keyframe_ratio_min_over_max": keyframe_ratio,
        "integrity": integrity,
        "metrics": metrics,
        "gt_source_path": str(gt_path),
        "gt_source_sha256": hashlib.sha256(gt_path.read_bytes()).hexdigest(),
        "loop_shutdown_status": {mode: run_summaries[mode].get("loop_shutdown_status") for mode in modes},
        "wall_duration_s": {mode: run_summaries[mode].get("wall_duration_s") for mode in modes},
    }


def render(result, manifest):
    rows = result["sequence_results"]
    modes = manifest["modes"]
    eligible = [row for row in rows if row.get("eligible") and
                row.get("integrity", {}).get("eligible_for_loop_judgment")]
    names = [DISPLAY_NAMES[mode] for mode in modes]
    lines = [
        "# UZH-FPV Indoor-Forward Loop-on Results", "",
        f"本报告运行完整 VINS-Fusion + Loop Fusion，bag 以 {manifest['rate']}× 回放；",
        "本轮仅比较 B_loop 和 G_gate+V_gate_loop，Loop 参数保持一致，",
        "LTV 参数冻结自 `docs/euroc_notune_results.md`。正式指标仅为位置 ATE。", "",
        "`Δ = (Method - B_loop) / B_loop × 100%`；负值表示改善。", "",
        "## 结论", "",
    ]
    if eligible:
        macro = {mode: float(np.mean([row["metrics"][mode]["ate_rmse_m"] for row in eligible])) for mode in modes}
        lines.append("Indoor-forward Global Macro ATE RMSE：" + ", ".join(
            f"{DISPLAY_NAMES[mode]} `{macro[mode]:.6f} m` ({relative_change_percent(macro['baseline'], macro[mode]):+.2f}%)"
            for mode in modes))
        lines.append("")
    lines += [
        f"完整性通过序列：{len(eligible)}/{len(rows)}。", "",
        "## ATE RMSE (m)", "",
        "| Sequence | " + " | ".join(names) + " |",
        "|---|" + "---:|" * len(modes),
    ]
    for row in rows:
        if row.get("eligible"):
            lines.append("| " + row["sequence"] + " | " + " | ".join(
                f"{row['metrics'][mode]['ate_rmse_m']:.6f}" for mode in modes) + " |")
        else:
            lines.append(f"| {row['sequence']} | N/A |" + " N/A |" * (len(modes) - 1))
    lines += ["", "## Position P95 Error (m)", "", "| Sequence | " + " | ".join(names) + " |", "|---|" + "---:|" * len(modes)]
    for row in rows:
        if row.get("eligible"):
            lines.append("| " + row["sequence"] + " | " + " | ".join(f"{row['metrics'][mode]['position_p95_error_m']:.6f}" for mode in modes) + " |")
        else:
            lines.append(f"| {row['sequence']} | N/A |" + " N/A |" * (len(modes) - 1))
    lines += ["", "## Maximum Position Error (m)", "", "| Sequence | " + " | ".join(names) + " |", "|---|" + "---:|" * len(modes)]
    for row in rows:
        if row.get("eligible"):
            lines.append("| " + row["sequence"] + " | " + " | ".join(f"{row['metrics'][mode]['maximum_position_error_m']:.6f}" for mode in modes) + " |")
        else:
            lines.append(f"| {row['sequence']} | N/A |" + " N/A |" * (len(modes) - 1))
    lines += ["", "## 前端与回环完整性", "", "| Sequence | throw img0/img1 | VIO samples / no-loop | Keyframes | Loop detections | Eligible |", "|---|---:|---:|---:|---:|---|"]
    for row in rows:
        if row.get("eligible"):
            d = row["diagnostics"]
            drops = ", ".join(f"{DISPLAY_NAMES[m]}:{d[m].get('throw_img0_count', 0)}/{d[m].get('throw_img1_count', 0)}" for m in modes)
            samples = ", ".join(f"{DISPLAY_NAMES[m]}:{d[m].get('vio_samples', 0)}/{row['no_loop_reference_vio_samples']}" for m in modes)
            kfs = ", ".join(str(d[m].get("keyframe_count", 0)) for m in modes)
            loops = ", ".join(str(d[m].get("loop_detection_count", 0)) for m in modes)
            lines.append(f"| {row['sequence']} | {drops} | {samples} | {kfs} | {loops} | {'yes' if row['integrity']['eligible_for_loop_judgment'] else 'no'} |")
        else:
            lines.append(f"| {row['sequence']} | N/A | — | — | — | no |")
    lines += ["", "## 运行与时间完整性", "", "| Sequence | Common samples | Max estimator–GT error (ms) | Shutdown artifacts |", "|---|---:|---:|---|"]
    for row in rows:
        if row.get("eligible"):
            shutdown = ", ".join(sorted(set(row.get("loop_shutdown_status", {}).values())))
            lines.append(f"| {row['sequence']} | {row['common_sample_count']} | {row['maximum_gt_time_error_s']*1000:.3f} | {shutdown} |")
        else:
            lines.append(f"| {row['sequence']} | — | — | — |")
    lines += ["", "## 复现信息", "", f"- 参数源 SHA-256：`{manifest['parameter_source_sha256']}`", f"- rosbag rate：`{manifest['rate']}×`", "- Loop 输出：每次运行的 `vio_loop.csv`（关键帧最终优化轨迹）", "- 姿态指标：`excluded_known_ground_truth_issue`", "- 速度指标：`unavailable`", ""]
    return "\n".join(lines)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--results-root", type=Path, default=Path("/home/he/output/uzhfpv_loop"))
    parser.add_argument("--time-audit", type=Path, default=Path("config/uzhfpv_gt_time_offsets.json"))
    parser.add_argument("--output", type=Path, default=Path("docs/uzhfpv_loop_results.md"))
    args = parser.parse_args()
    manifest = json.loads((args.results_root / "batch_manifest.json").read_text(encoding="utf-8"))
    audit = json.loads(args.time_audit.read_text(encoding="utf-8"))
    entries = {item["sequence"]: item for item in audit["sequences"]}
    results = []
    modes = manifest["modes"]
    if modes != ["baseline", "joint_v_gate"]:
        raise RuntimeError(f"this rerun requires baseline and joint_v_gate, got {modes}")
    no_loop_root = Path(manifest.get("no_loop_reference_root", "/home/he/output/ltv_uzhfpv_notune"))
    for sequence in manifest["sequences"]:
        try:
            results.append({"eligible": True, **evaluate_one(args.results_root / sequence, sequence, entries[sequence], modes, no_loop_root)})
        except Exception as error:
            results.append({"eligible": False, "sequence": sequence, "failure": str(error)})
    payload = {"schema": "uzhfpv-loop-results-v1", "sequence_results": results, "manifest": manifest}
    args.output.write_text(render(payload, manifest), encoding="utf-8")
    (args.results_root / "evaluation.json").write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
