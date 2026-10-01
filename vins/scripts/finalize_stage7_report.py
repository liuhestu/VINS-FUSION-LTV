#!/usr/bin/env python3
"""Refresh presentation and error series from verified Stage7 artifacts (no replay)."""
import argparse
import csv
import json
from pathlib import Path
import shutil
import numpy as np
from run_stage7_tuning import Runner, write_json, REPO
from evaluate_stage7_tuning import evaluate
from evaluate_vins_euroc import read_vins, read_official_ground_truth, rigid_alignment
from evaluate_euroc_final import SEQUENCES, atomic_write


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--batch',type=Path,required=True)
    args = parser.parse_args()
    root = args.batch
    plan = json.loads((root/'stage7_plan.json').read_text())
    selection = json.loads((root/'selection.json').read_text())
    final = selection['final']
    records = [json.loads(l) for l in (root/'trial_results.jsonl').read_text().splitlines()]
    paths = {}
    for r in records:
        if r['status']!='success' or r['repeat'] is not None:
            continue
        name = 'B' if r['mode']=='baseline' else 'G' if r['mode']=='gravity_only' else r['candidate']
        paths[name,r['sequence']] = Path(r['path'])
    names = list(dict.fromkeys(['B','G','W0',final]))
    finalist_names = json.loads((root/'finalists.json').read_text())
    finalist_rows = []
    for sequence in plan['D0']+plan['D1']:
        selected = {n:paths[n,sequence] for n in ['B','W0']+finalist_names}
        metadata = json.loads((paths['B',sequence]/'input_manifest.json').read_text())['metadata']
        row = evaluate(selected,Path(metadata['ground_truth']))
        row['sequence'] = sequence
        finalist_rows.append(row)
    from evaluate_stage7_tuning import score
    write_json(root/'D1_common_evaluation.json',finalist_rows)
    write_json(root/'D1_ranking.json',[dict(candidate=n,**score(finalist_rows,n)) for n in ['W0']+finalist_names])
    rows = []
    for sequence in SEQUENCES:
        selected = {n:paths[n,sequence] for n in names}
        metadata = json.loads((paths['B',sequence]/'input_manifest.json').read_text())['metadata']
        row = evaluate(selected,Path(metadata['ground_truth']))
        row['sequence'] = sequence
        rows.append(row)
        gt = read_official_ground_truth(Path(metadata['ground_truth']))
        gi = np.array(row['support']['gt_indices'])
        series = root/'error_series'/f'{sequence}.csv'
        series.parent.mkdir(exist_ok=True)
        with series.open('w') as stream:
            writer = csv.writer(stream)
            writer.writerow(['mode','timestamp_s','ate_error_m'])
            for n in names:
                t,p,_,_ = read_vins(selected[n]/'vio.csv')
                indices = np.array(row['support']['mode_indices'][n])
                rotation,translation = rigid_alignment(p[indices],gt['positions'][gi])
                errors = np.linalg.norm((rotation@p[indices].T).T+translation-gt['positions'][gi],axis=1)
                writer.writerows(zip([n]*len(indices),t[indices],errors))
    write_json(root/'final_evaluation.json',rows)
    repeats = json.loads((root/'repeat_results.json').read_text())
    acceptance = json.loads((root/'release/acceptance.json').read_text())
    runner = Runner(argparse.Namespace(output_root=root,config=REPO/'config/euroc/euroc_stereo_imu_ltv_config.yaml',
        replay=Path('/home/he/vins_fusion_ltv_ws/build/vins/stage5_replay'),cache_root=Path('/home/he/output/ltv_stage5_minimal_fix/cache')))
    runner.report(final,rows,repeats,Path(acceptance['path']) if acceptance['path'] else None)
    text = (root/'stage7_report.md').read_text()
    table = ['## 逐序列 ATE 与相对变化','', '| Sequence | B | G-only | Δ vs B | W0 | Δ vs B | Final Joint | Δ vs B | Δ vs W0 | Δ vs G-only |',
        '|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|']
    for r in rows:
        m = r['metrics']
        b,g,w,f = [m[n]['ate_rmse_m'] for n in ['B','G','W0',final]]
        table.append(f"| {r['sequence']} | {b:.6f} | {g:.6f} | {100*(g/b-1):+.2f}% | {w:.6f} | {100*(w/b-1):+.2f}% | {f:.6f} | {100*(f/b-1):+.2f}% | {100*(f/w-1):+.2f}% | {100*(f/g-1):+.2f}% |")
    for label,subset in [('Easy/Medium Mean',[r for r in rows if not r['sequence'].endswith('difficult')]), ('Difficult Mean',[r for r in rows if r['sequence'].endswith('difficult')]),('All Mean',rows)]:
        b,g,w,f = [np.mean([r['metrics'][n]['ate_rmse_m'] for r in subset]) for n in ['B','G','W0',final]]
        table.append(f'| {label} | {b:.6f} | {g:.6f} | {100*(g/b-1):+.2f}% | {w:.6f} | {100*(w/b-1):+.2f}% | {f:.6f} | {100*(f/b-1):+.2f}% | {100*(f/w-1):+.2f}% | {100*(f/g-1):+.2f}% |')
    text += '\n'+'\n'.join(table)+'\n\n逐时刻 ATE 误差已导出 `error_series/*.csv`，用于检查 >10% 尾部警告。\n'
    aliases = root/'candidate_aliases.json'
    if aliases.exists():
        text += '\nW5 去重记录：`'+json.dumps(json.loads(aliases.read_text()),ensure_ascii=False)+'`。\n'
    text += '\n## D0 全部候选\n\n| Candidate | J | Worst Δ vs B | Engineering | Screening |\n|---|---:|---:|---|---|\n'
    for result in json.loads((root/'D0_ranking.json').read_text()):
        text += f"| {result['candidate']} | {result.get('J', '—')} | {result.get('worst', '—')} | {result['engineering_pass']} | {result['screening_pass']} |\n"
        if 'error' in result:
            text += '\n淘汰原因：'+result['error']+'\n\n'
    text += '\n## 最终各模式运行诊断\n\n| Sequence | Mode | G factors | V factors | G coverage | V coverage | Wall time (s) |\n|---|---|---:|---:|---:|---:|---:|\n'
    for sequence in SEQUENCES:
        for name in names:
            validation = json.loads((paths[name,sequence]/'validation.json').read_text())
            text += f"| {sequence} | {name} | {validation['gravity_factor_added_count']} | {validation['velocity_factor_added_count']} | {validation['gravity_gate_coverage']:.4f} | {validation['velocity_gate_coverage']:.4f} | {validation['wall_time_s']:.2f} |\n"
    text += '\nGate coverage 为日志中的判定通过率；禁用分支时不解释为已加入因素比例。Huber 比例仅表示加入因素 snapshot 的 logged weighted norm > delta，不表示所有滑窗 residual 的求解统计。\n'
    evidence_path = root/'release/ros_visualization_evidence.json'
    if evidence_path.exists():
        evidence = json.loads(evidence_path.read_text())
        text += '\n## ROS 与 RViz 验收证据\n\n```json\n'+json.dumps(evidence,indent=2)+'\n```\n'
    failures = root/'release/visualization_failures.json'
    if failures.exists():
        text += '\n## 可视化验收失败与恢复\n\n```json\n'+json.dumps(json.loads(failures.read_text()),indent=2)+'\n```\n一次补验收回放使用 W5 去重节省的预算，累计 78/80。参数和 estimator 二进制未改变。\n'
    text += '\n完整配置：`'+str(root/'release/euroc_stage7.yaml')+'`。参数 SHA：`'+(root/'stage7_final_parameters.sha256').read_text().strip()+'`。\n'
    atomic_write(root/'stage7_report.md',text)
    atomic_write(REPO/'docs/euroc_tuned_results.md',text)
    for filename in ['euroc_stage7.yaml','cam0_mei.yaml','cam1_mei.yaml']:
        shutil.copy2(root/'release'/filename,REPO/'config/tuning'/filename)
    for filename in ['stage7_final_parameters.yaml','stage7_final_parameters.sha256']:
        shutil.copy2(root/filename,REPO/'config/tuning'/filename)
    write_json(root/'report_refresh.json',dict(original_evaluator_in_selection=selection['common_evidence'][0].get('evaluator_sha256'),
        refreshed_evaluator_dependencies=rows[0]['evaluator_dependencies'], note='identical metric definitions; refreshed report and per-frame errors only'))


if __name__=='__main__':
    main()
