#!/usr/bin/env python3
"""Evaluate the frozen five-mode UZH-FPV no-tune experiment."""

import argparse
import csv
import io
import json
import math
from pathlib import Path

import numpy as np

from audit_uzhfpv_time_offsets import CATEGORIES, SEQUENCES, sequence_family
from evaluate_euroc_final import (
    MODES, DISPLAY_NAMES, atomic_write, common_time_support, jsonl_text,
    load_run, relative_change_percent, validate_manifests)
from evaluate_vins_euroc import read_uzh_ground_truth, rigid_alignment


METRICS = (
    ("ATE RMSE (m)", "ate_rmse_m"),
    ("Position P95 Error (m)", "position_p95_error_m"),
    ("Maximum Position Error (m)", "maximum_position_error_m"),
)


def load_time_audit(path):
    audit = json.loads(path.read_text(encoding="utf-8"))
    if audit.get("schema") != "uzhfpv-gt-time-offset-audit-v1":
        raise RuntimeError("unsupported UZH-FPV time-audit schema")
    entries = {item["sequence"]: item for item in audit["sequences"]}
    if set(entries) != set(SEQUENCES):
        raise RuntimeError("time audit does not cover the fixed 16 sequences")
    return audit, entries


def position_metrics(runs, ground_truth, support):
    gt_positions = ground_truth["positions"][support["gt_indices"]]
    metrics = {}
    errors_by_mode = {}
    for mode in MODES:
        positions = runs[mode]["positions"][support["mode_indices"][mode]]
        rotation, translation = rigid_alignment(positions, gt_positions)
        aligned = (rotation @ positions.T).T + translation
        errors = np.linalg.norm(aligned - gt_positions, axis=1)
        if not np.all(np.isfinite(errors)):
            raise RuntimeError(f"{mode}: non-finite position errors")
        errors_by_mode[mode] = errors
        metrics[mode] = {
            "ate_rmse_m": math.sqrt(float(np.mean(errors ** 2))),
            "position_p95_error_m": float(np.percentile(errors, 95)),
            "maximum_position_error_m": float(np.max(errors)),
        }
    return metrics, errors_by_mode


def run_wall_duration(run_path):
    """Return replay wall time from the transaction's first/last artifacts."""
    started = run_path / "input_manifest.json"
    finished = run_path / "validation.json"
    if not started.is_file() or not finished.is_file():
        return None
    duration = finished.stat().st_mtime - started.stat().st_mtime
    return float(duration) if duration >= 0.0 else None


def preserved_failure_detail(sequence_root):
    details = []
    for failed in sorted(sequence_root.glob("*.failed-*")):
        replay_log = failed / "replay.log"
        last_line = None
        if replay_log.is_file():
            lines = [line.strip() for line in replay_log.read_text(
                encoding="utf-8", errors="replace").splitlines()
                     if line.strip()]
            if lines:
                last_line = lines[-1]
        detail = failed.name
        if last_line:
            detail += f" ({last_line})"
        details.append(detail)
    return "; ".join(details)


def evaluate(results_root, cache_root, time_audit_path):
    time_audit, time_entries = load_time_audit(time_audit_path)
    sequence_results = []
    pooled = {mode: [] for mode in MODES}
    validations = []
    for sequence in SEQUENCES:
        family = sequence_family(sequence)
        metadata_path = cache_root / sequence / "metadata.json"
        if not metadata_path.is_file():
            sequence_results.append({
                "sequence": sequence, "family": family, "eligible": False,
                "failure": "missing canonical cache"})
            continue
        metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
        try:
            runs = {}
            wall_durations = {}
            for mode in MODES:
                run_path = results_root / sequence / mode
                runs[mode] = load_run(run_path, sequence, mode)
                wall_durations[mode] = run_wall_duration(run_path)
            validate_manifests(sequence, runs, metadata)
            gt = read_uzh_ground_truth(str(Path(metadata["ground_truth"])))
            entry = time_entries[sequence]
            if gt["sha256"] != entry["gt_sha256"]:
                raise RuntimeError("GT hash differs from time audit")
            gt["timestamps"] = (
                gt["timestamps"] + entry["applied_gt_time_offset_s"])
            support = common_time_support(runs, gt)
            metrics, errors = position_metrics(runs, gt, support)
            for mode in MODES:
                pooled[mode].append(errors[mode])
                validation = dict(runs[mode]["validation"])
                validation.update({
                    "sequence": sequence,
                    "mode": mode,
                    "common_sample_count": support["common_count"],
                    "common_coverage": support["coverage"][mode],
                    "wall_duration_s": wall_durations[mode],
                })
                validations.append(validation)
            sequence_results.append({
                "sequence": sequence,
                "family": family,
                "eligible": True,
                "failure": None,
                "common_sample_count": support["common_count"],
                "maximum_gt_time_error_s": support["maximum_gt_time_error_s"],
                "maximum_cross_mode_time_error_s": support[
                    "maximum_cross_mode_time_error_s"],
                "metrics": metrics,
                "wall_duration_s": wall_durations,
                "dataset_duration_s": (
                    metadata["frame_time_range_ns"][1]
                    - metadata["frame_time_range_ns"][0]) / 1e9,
            })
        except Exception as error:
            failure = str(error)
            preserved = preserved_failure_detail(results_root / sequence)
            if preserved:
                failure += f"; preserved failure: {preserved}"
            sequence_results.append({
                "sequence": sequence, "family": family, "eligible": False,
                "failure": failure})

    eligible = [item for item in sequence_results if item["eligible"]]
    if not eligible:
        raise RuntimeError("no sequence has five valid no-tune modes")

    def aggregate(selected, metric):
        return {mode: float(np.mean([
            row["metrics"][mode][metric] for row in selected]))
                for mode in MODES}

    aggregates = {family: {
        "eligible": sum(item["eligible"] and item["family"] == family
                        for item in sequence_results),
        "total": sum(item["family"] == family for item in sequence_results),
        "metrics": {key: aggregate(
            [item for item in eligible if item["family"] == family], key)
            for _, key in METRICS}
        if any(item["family"] == family for item in eligible) else None,
    } for family in CATEGORIES}
    aggregates["global_macro"] = {
        "eligible": len(eligible), "total": len(SEQUENCES),
        "metrics": {key: aggregate(eligible, key) for _, key in METRICS},
    }
    micro = {}
    for mode in MODES:
        values = np.concatenate(pooled[mode])
        micro[mode] = {
            "ate_rmse_m": math.sqrt(float(np.mean(values ** 2))),
            "position_p95_error_m": float(np.percentile(values, 95)),
            "maximum_position_error_m": float(np.max(values)),
        }
    aggregates["global_micro"] = {
        "eligible": len(eligible), "total": len(SEQUENCES), "metrics": {
            key: {mode: micro[mode][key] for mode in MODES}
            for _, key in METRICS}}
    return {
        "schema": "ltv-uzhfpv-notune-v1",
        "sequence_results": sequence_results,
        "aggregates": aggregates,
        "validations": validations,
        "eligible_sequence_count": len(eligible),
        "total_sequence_count": len(SEQUENCES),
        "orientation_metrics_status": "excluded_known_ground_truth_issue",
        "velocity_metrics_status": "unavailable",
        "time_audit": time_audit,
    }


def table_row(label, values):
    baseline = values["baseline"]
    cells = [label, f"{baseline:.6f}"]
    for mode in MODES[1:]:
        cells.extend((f"{values[mode]:.6f}",
                      f"{relative_change_percent(baseline, values[mode]):+.2f}%"))
    return "| " + " | ".join(cells) + " |"


def render_markdown(audit, execution):
    macro = audit["aggregates"]["global_macro"]["metrics"]
    ate = macro["ate_rmse_m"]
    p95 = macro["position_p95_error_m"]
    maximum = macro["maximum_position_error_m"]
    ate_changes = {mode: relative_change_percent(ate["baseline"], ate[mode])
                   for mode in MODES[1:]}
    p95_joint_gate_change = relative_change_percent(
        p95["baseline"], p95["joint_v_gate"])
    maximum_joint_gate_change = relative_change_percent(
        maximum["baseline"], maximum["joint_v_gate"])
    lines = [
        "# UZH-FPV No-tune Results", "",
        "本报告使用 EuRoC No-tune 的冻结 LTV 参数；UZH-FPV 标定、IMU 噪声、",
        "相机–IMU `td`、mask 与前端频率保持数据集配置。所有正式指标均来自",
        "五模式共同时间支持上的无尺度 SE(3) 位置对齐。", "",
        "`Δ = (Method - B) / B × 100%`；负值表示改善。姿态指标因已知 GT 问题",
        "排除，速度指标不可用。", "",
        "## 冻结与复现信息", "",
        f"- 有效序列：{audit['eligible_sequence_count']}/{audit['total_sequence_count']}",
        f"- 参数源 SHA-256：`{execution['parameter_source_sha256']}`",
        f"- 时间审计 SHA-256：`{execution['time_audit_sha256']}`",
        f"- Replay SHA-256：`{execution['replay_sha256']}`", "",
        "## 结论", "",
        f"在 {audit['eligible_sequence_count']}/{audit['total_sequence_count']} 条严格有效序列的 Global Macro 上，所有冻结 LTV "
        "模式的 ATE RMSE 都差于 Baseline："
        f"G_gate `{ate_changes['gravity_only']:+.2f}%`、"
        f"V_fixed `{ate_changes['velocity_only']:+.2f}%`、"
        f"G_gate+V_fixed `{ate_changes['joint']:+.2f}%`、"
        f"G_gate+V_gate `{ate_changes['joint_v_gate']:+.2f}%`。因此本次 "
        "No-tune 测试不支持 LTV 改善 UZH-FPV 总体位置 ATE 的结论。", "",
        "G_gate+V_gate 的 P95 与最大位置误差 Global Macro 分别为 "
        f"`{p95_joint_gate_change:+.2f}%` 和 "
        f"`{maximum_joint_gate_change:+.2f}%`，但其 ATE 仍退化，不能据此判为总体改善。", "",
        "`outdoor_forward_1` 因 gravity-only 可重复的末帧 drain 失败而整条 "
        "N/A，未混入聚合结果。", "",
        "## GT 时间戳审计", "",
        "偏移定义：`raw` 最大化 `ω_GT(t)` 与 `ω_IMU(t+raw)` 的相关性；",
        "`residual = raw - td`，正式评估使用 `t_GT + applied`。", "",
        "| Sequence | GT range (s) | Bag range (s) | Raw (ms) | td (ms) | Residual (ms) | Applied (ms) | Corr | GT/VIO overlap (s) |",
        "|---|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for item in audit["time_audit"]["sequences"]:
        gt_range = item["gt_time_range_s"]
        bag_range = item["bag_time_range_s"]
        lines.append(
            f"| {item['sequence']} | {gt_range[0]:.3f}–{gt_range[1]:.3f} | "
            f"{bag_range[0]:.3f}–{bag_range[1]:.3f} | "
            f"{item['raw_offset_s'] * 1000:+.1f} | "
            f"{item['calibration_td_s'] * 1000:+.1f} | "
            f"{item['residual_offset_s'] * 1000:+.1f} | "
            f"{item['applied_gt_time_offset_s'] * 1000:+.1f} | "
            f"{item['raw_offset_correlation']:.4f} | "
            f"{item['baseline_common_position_duration_s']:.3f} |")
    lines.extend(["", "### 家族决策", ""])
    for family, decision in audit["time_audit"]["families"].items():
        lines.append(
            f"- `{family}`: `{decision['status']}`, residual median "
            f"{decision['residual_median_s'] * 1000:+.2f} ms, "
            f"MAD {decision['residual_mad_s'] * 1000:.2f} ms, applied "
            f"{decision['applied_gt_time_offset_s'] * 1000:+.2f} ms.")
    lines.append("")

    lines.extend((
        "### 原始 Leica 审计", "",
        "Leica 仅作为来源审计证据，不直接转换为 IMU 位姿，也不用于 ATE。", "",
        "| Sequence | Samples | UTC range (s) | Finite position | SHA-256 |",
        "|---|---:|---:|---|---|"))
    for item in audit["time_audit"]["sequences"]:
        leica = item.get("raw_leica_audit", {"status": "not_retained"})
        if leica.get("status") == "not_retained":
            lines.append(
                f"| {item['sequence']} | N/A | N/A | not retained | N/A |")
        else:
            lines.append(
                f"| {item['sequence']} | {leica['samples']} | "
                f"{leica['start_time_utc_s']:.3f}–{leica['end_time_utc_s']:.3f} | "
                f"{'yes' if leica['positions_finite'] else 'no'} | "
                f"`{leica['sha256']}` |")
    lines.append("")

    lines.extend((
        "## 离线运行耗时", "",
        "`wall / data` 由每次运行的 `input_manifest.json` 与 "
        "`validation.json` 文件时间计算；它衡量本次离线 replay 的吞吐，"
        "不参与 ATE。", "",
        "| Sequence | Data (s) | B wall (s) | B wall/data | "
        "G_gate wall (s) | V_fixed wall (s) | Joint wall (s) | "
        "Joint V-gate wall (s) |",
        "|---|---:|---:|---:|---:|---:|---:|---:|"))
    for item in audit["sequence_results"]:
        if not item["eligible"]:
            continue
        wall = item["wall_duration_s"]
        data_duration = item["dataset_duration_s"]
        ratio = wall["baseline"] / data_duration
        lines.append(
            f"| {item['sequence']} | {data_duration:.3f} | "
            f"{wall['baseline']:.1f} | {ratio:.2f}× | "
            f"{wall['gravity_only']:.1f} | {wall['velocity_only']:.1f} | "
            f"{wall['joint']:.1f} | {wall['joint_v_gate']:.1f} |")
    lines.extend((
        "", "特别地，`indoor_forward_5` 的数据有效时长约 150 秒（原始 bag "
        "约 156 秒），但单次运行约需 40–50 分钟。这是已确认的 CPU 满载"
        "离线性能异常，而不是 bag 时长或进程挂起；本报告保留冻结参数，"
        "未用降采样或调参掩盖该现象。", ""))

    header = ("| Sequence | B | G_gate | Δ vs B | V_fixed | Δ vs B | "
              "G_gate+V_fixed | Δ vs B | G_gate+V_gate | Δ vs B |")
    separator = "|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|"
    for title, key in METRICS:
        lines.extend((f"## {title}", "", header, separator))
        for item in audit["sequence_results"]:
            if item["eligible"]:
                lines.append(table_row(item["sequence"], {
                    mode: item["metrics"][mode][key] for mode in MODES}))
            else:
                lines.append("| " + item["sequence"] + " | " +
                             " | ".join(["N/A"] * 9) + " |")
        for family in CATEGORIES:
            group = audit["aggregates"][family]
            if group["metrics"] is not None:
                lines.append(table_row(
                    f"{family} Macro ({group['eligible']}/{group['total']})",
                    group["metrics"][key]))
        macro = audit["aggregates"]["global_macro"]
        lines.append(table_row(
            f"Global Macro ({macro['eligible']}/{macro['total']})",
            macro["metrics"][key]))
        micro = audit["aggregates"]["global_micro"]
        lines.append(table_row(
            f"Global Micro ({micro['eligible']}/{micro['total']})",
            micro["metrics"][key]))
        lines.append("")

    lines.extend(("## 运行状态", "",
                  "| Sequence | Status | Detail |", "|---|---|---|"))
    for item in audit["sequence_results"]:
        detail = (f"{item['common_sample_count']} common samples"
                  if item["eligible"] else item["failure"].replace("|", "\\|"))
        lines.append(
            f"| {item['sequence']} | "
            f"{'complete' if item['eligible'] else 'N/A'} | {detail} |")
    lines.extend(("", "## 指标边界", "",
                  "- `orientation_metrics_status: excluded_known_ground_truth_issue`",
                  "- `velocity_metrics_status: unavailable`", ""))
    return "\n".join(lines)


def write_artifacts(output_root, audit, execution, markdown_output):
    audit["execution"] = execution
    atomic_write(output_root / "experiment_manifest.json",
                 json.dumps(audit, indent=2, sort_keys=True) + "\n")
    metric_rows = []
    for item in audit["sequence_results"]:
        if item["eligible"]:
            for mode in MODES:
                row = {"sequence": item["sequence"], "family": item["family"],
                       "mode": mode}
                row.update(item["metrics"][mode])
                metric_rows.append(row)
    atomic_write(output_root / "metrics.jsonl", jsonl_text(metric_rows))
    if audit["validations"]:
        stream = io.StringIO()
        fields = sorted(set().union(*(row.keys() for row in audit["validations"])))
        writer = csv.DictWriter(stream, fields, extrasaction="ignore", lineterminator="\n")
        writer.writeheader()
        writer.writerows(audit["validations"])
        atomic_write(output_root / "run_validation.csv", stream.getvalue())
    markdown = render_markdown(audit, execution)
    atomic_write(output_root / "uzhfpv_notune_results.md", markdown)
    atomic_write(markdown_output, markdown)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--results-root", required=True, type=Path)
    parser.add_argument("--cache-root", required=True, type=Path)
    parser.add_argument("--time-audit", required=True, type=Path)
    parser.add_argument("--execution-manifest", required=True, type=Path)
    parser.add_argument("--markdown-output", required=True, type=Path)
    args = parser.parse_args()
    execution = json.loads(args.execution_manifest.read_text(encoding="utf-8"))
    audit = evaluate(args.results_root, args.cache_root, args.time_audit)
    write_artifacts(args.results_root, audit, execution, args.markdown_output)
    print(json.dumps({"eligible": audit["eligible_sequence_count"],
                      "total": audit["total_sequence_count"],
                      "markdown": str(args.markdown_output.resolve())}, sort_keys=True))


if __name__ == "__main__":
    main()
