#!/usr/bin/env python3
"""Variable-set EuRoC evaluation; never accumulate intersections across trials."""
import json
import math
from pathlib import Path
import numpy as np
from evaluate_euroc_final import (match_one_to_one, match_nearest, rigid_alignment,
    read_official_ground_truth, read_vins, rmse, CROSS_MODE_TOLERANCE_S,
    GT_TOLERANCE_S, MIN_COMMON_COVERAGE, MAX_RANGE_ENDPOINT_SPREAD_S)


def common_support(runs, gt):
    anchors = runs['B']['timestamps']
    mappings = {}
    errors = {}
    for name, run in runs.items():
        mappings[name], errors[name] = match_one_to_one(
            anchors, run['timestamps'], CROSS_MODE_TOLERANCE_S)
    gi, ge = match_one_to_one(anchors, gt['timestamps'], GT_TOLERANCE_S)
    valid = gi >= 0
    for mapping in mappings.values():
        valid &= mapping >= 0
    count = int(valid.sum())
    if count < 3:
        raise RuntimeError('fewer than three common samples')
    coverage, starts, stops = {}, [], []
    for name, run in runs.items():
        indices, _, _ = match_nearest(run['timestamps'], gt['timestamps'], GT_TOLERANCE_S)
        if len(indices) < 3:
            raise RuntimeError(f'{name}: insufficient GT support')
        coverage[name] = count / len(indices)
        if coverage[name] + 1e-12 < MIN_COMMON_COVERAGE:
            raise RuntimeError(f'{name}: coverage {coverage[name]} below 99%')
        starts.append(float(run['timestamps'][indices[0]]))
        stops.append(float(run['timestamps'][indices[-1]]))
    if max(starts)-min(starts) > MAX_RANGE_ENDPOINT_SPREAD_S + 1e-12 or max(stops)-min(stops) > MAX_RANGE_ENDPOINT_SPREAD_S + 1e-12:
        raise RuntimeError('endpoint spread exceeds 0.1s')
    return dict(mode_indices={n: m[valid] for n, m in mappings.items()},
                gt_indices=gi[valid], anchor_indices=np.flatnonzero(valid),
                missing_anchor_indices=np.flatnonzero((gi >= 0) & ~valid),
                coverage=coverage, common_count=count,
                maximum_cross_mode_time_error_s=max(float(errors[n][valid].max()) for n in runs),
                maximum_gt_time_error_s=float(ge[valid].max()),
                start_spread_s=max(starts)-min(starts), stop_spread_s=max(stops)-min(stops))


def metrics(runs, gt, support):
    gi = support['gt_indices']
    gp, gr, gv = (gt[k][gi] for k in ('positions', 'rotations', 'velocities'))
    bi = support['mode_indices']['B']
    alignment, _ = rigid_alignment(runs['B']['positions'][bi], gp)
    result = {}
    for name, run in runs.items():
        indices = support['mode_indices'][name]
        p, r, v = (run[k][indices] for k in ('positions', 'rotations', 'velocities'))
        rotation, translation = rigid_alignment(p, gp)
        pe = np.linalg.norm((rotation @ p.T).T + translation - gp, axis=1)
        re = [math.acos(float(np.clip((np.trace(ref.T @ alignment @ est)-1)/2, -1, 1))) for est, ref in zip(r, gr)]
        ve = np.linalg.norm((alignment @ v.T).T - gv, axis=1)
        result[name] = dict(ate_rmse_m=rmse(pe), ate_p95_m=float(np.percentile(pe, 95)),
            ate_max_m=float(pe.max()), rotation_rmse_deg=math.degrees(rmse(re)), velocity_rmse_mps=rmse(ve))
    return result


def evaluate(paths, gt_path):
    runs = {}
    identities = []
    for name, path in paths.items():
        t, p, r, v = read_vins(Path(path) / 'vio.csv')
        if not all(np.all(np.isfinite(x)) for x in (t, p, r, v)) or np.any(np.diff(t) <= 0):
            raise RuntimeError(f'{name}: invalid trajectory')
        runs[name] = dict(timestamps=t, positions=p, rotations=r, velocities=v)
        manifest = json.loads((Path(path)/'input_manifest.json').read_text())
        identities.append({k: manifest[k] for k in ('pair_list_sha256', 'imu_sha256', 'ground_truth_sha256', 'replay_sha256', 'calibration_sha256', 'base_config_sha256', 'dependencies')})
    if any(i != identities[0] for i in identities[1:]):
        raise RuntimeError('mixed replay/input/calibration identities')
    gt = read_official_ground_truth(Path(gt_path))
    support = common_support(runs, gt)
    return dict(metrics=metrics(runs, gt, support), support={k: ({n: v.tolist() for n,v in x.items()} if k == 'mode_indices' else x.tolist() if isinstance(x, np.ndarray) else x) for k,x in support.items()},
                velocity_gt_source='official CSV velocity columns',
                evaluator_dependencies={name: __import__('run_stage5_velocity_oracle').sha256(Path(__file__).parent/name)
                    for name in ('evaluate_stage7_tuning.py','evaluate_euroc_final.py','evaluate_vins_euroc.py')})


def score(rows, name):
    relative = [r['metrics'][name]['ate_rmse_m']/r['metrics']['B']['ate_rmse_m']-1 for r in rows]
    tails = [r['metrics'][name]['ate_p95_m']/r['metrics']['B']['ate_p95_m']-1 for r in rows]
    return dict(J=float(np.mean(relative)), worst=max(relative), p95_worst=max(tails),
                screening_pass=max(relative) <= 0.03 + 1e-12)


def ranking_key(score_value, parameters, original):
    distance = sum(abs(parameters[k]/original[k]-1) for k in original)
    return (score_value['J'], score_value['worst'], score_value['p95_worst'], distance, json.dumps(parameters, sort_keys=True))


def group_summary(rows, names, metric):
    means = {n: float(np.mean([r['metrics'][n][metric] for r in rows])) for n in names}
    return dict(means=means, delta_vs_B_percent={n:100*(v/means['B']-1) for n,v in means.items()})


def main():
    import argparse
    from evaluate_euroc_final import atomic_write
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', action='append', required=True, metavar='NAME=DIR')
    parser.add_argument('--ground-truth', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    paths = {}
    for run in args.run:
        name, path = run.split('=',1)
        if name in paths:
            raise ValueError('duplicate mode name')
        paths[name] = Path(path)
    if 'B' not in paths:
        raise ValueError('a B anchor run is required')
    atomic_write(args.output,json.dumps(evaluate(paths,args.ground_truth),indent=2,allow_nan=False)+'\n')


if __name__ == '__main__':
    main()
