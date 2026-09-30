#!/usr/bin/env python3
"""Evaluate saved EuRoC pose graphs on identical keyframes, with SE(3) alignment."""
import argparse
import json
import re
from pathlib import Path
import numpy as np
from evaluate_vins_euroc import (read_official_ground_truth, match_nearest,
                                 rigid_alignment, quaternion_to_rotation, sha256_file)


def statistics(errors):
    a = np.asarray(errors)
    return dict(rmse=float(np.sqrt(np.mean(a*a))), p95=float(np.percentile(a,95)), maximum=float(a.max()))


def evaluate(root, dataset, sequence):
    run = root / sequence
    graph = run / 'pose_graph/pose_graph.txt'
    g = np.loadtxt(graph, ndmin=2)
    gt_path = dataset / 'ASL' / sequence / 'mav0/state_groundtruth_estimate0/data.csv'
    gt = read_official_ground_truth(str(gt_path))
    idx, ref, dt = match_nearest(g[:,1], gt['timestamps'], .02)
    if len(idx) < 3 or not np.isfinite(g).all():
        raise ValueError('invalid graph or insufficient GT matches')
    result = dict(sequence=sequence, keyframes=len(g), matched=len(idx),
                  time_range_s=[float(g[idx[0],1]),float(g[idx[-1],1])],
                  max_time_error_s=float(max(dt)), gt_sha256=sha256_file(gt_path),
                  graph_sha256=sha256_file(graph))
    for name, cols in [('vio',slice(2,5)), ('loop',slice(5,8))]:
        p=g[idx,cols]; target=gt['positions'][ref]
        r,t=rigid_alignment(p,target)
        result[name]=statistics(np.linalg.norm(p@r.T+t-target,axis=1))
    log=(run/'loop.log').read_text(errors='replace')
    # Intermediate log lines currently carry rejection labels even on accepted
    # candidates. Deduplicate by current/candidate; the last record is final.
    records={}
    for line in log.splitlines():
        if not line.startswith('LOOP_DIAGNOSTIC current='): continue
        fields=dict(re.findall(r'(\w+)=([^\s]+)',line))
        key=(fields['current'],fields['candidate'])
        records.setdefault(key,{}).update(fields)
    accepted=[v for v in records.values() if v.get('status')=='accepted_loop']
    edges=g[g[:,16]>=0]
    result['loops']=len(edges)
    result['accepted_log_count']=len(accepted)
    if len(edges)!=len(accepted): raise ValueError('accepted count differs from graph')
    norms=np.linalg.norm(edges[:,20:24],axis=1)
    if len(norms) and np.max(abs(norms-1))>1e-3: raise ValueError('invalid loop quaternion')
    result['max_loop_quaternion_norm_error']=float(max(abs(norms-1))) if len(norms) else None
    result['zero_loop_translations']=int(np.sum(np.linalg.norm(edges[:,17:20],axis=1)==0))
    for label,rows in [('all_pnp',list(records.values())),('accepted_pnp',accepted)]:
        inputs=sum(int(v.get('pnp_input',0)) for v in rows)
        inliers=sum(int(v.get('pnp_inliers',0)) for v in rows)
        result[label]=dict(inputs=inputs,inliers=inliers,rate=inliers/inputs if inputs else None)
    by_id={int(row[0]):row for row in g}
    for mode,pc,qc in [('vio',2,8),('loop',5,12)]:
        translations=[]; yaws=[]
        for row in edges:
            old=by_id[int(row[16])]
            ri=quaternion_to_rotation(*old[qc:qc+4]); rj=quaternion_to_rotation(*row[qc:qc+4])
            translations.append(np.linalg.norm(ri.T@(row[pc:pc+3]-old[pc:pc+3])-row[17:20]))
            yaw=np.degrees(np.arctan2(rj[1,0],rj[0,0])-np.arctan2(ri[1,0],ri[0,0]))-row[24]
            yaws.append(abs((yaw+180)%360-180))
        result[mode+'_edge_residual']=dict(translation_m=statistics(translations),yaw_deg=statistics(yaws)) if len(edges) else None
    for name in ['vio','vio_loop']:
        result[name+'_rows']=len((run/(name+'.csv')).read_text().splitlines())
    vins=(run/'vins.log').read_text(errors='replace')
    result['reported_stereo_drops']={side:vins.count('throw '+side) for side in ['img0','img1']}
    result['manifest']=json.loads((run/'run_manifest.json').read_text())
    if any(result['manifest'].get(k)!=0 for k in ['bag_exit_code','loop_exit_code','vins_exit_code']):
        raise ValueError('unclean run')
    result['ate_improved']=result['loop']['rmse']<result['vio']['rmse']
    return result


def main():
    p=argparse.ArgumentParser()
    p.add_argument('root',type=Path); p.add_argument('--dataset',type=Path,default=Path('/home/he/datasets/euroc'))
    p.add_argument('--sequence',action='append',required=True)
    args=p.parse_args()
    results=[evaluate(args.root,args.dataset,s) for s in args.sequence]
    (args.root/'evaluation.json').write_text(json.dumps(results,indent=2)+'\n')
    for r in results:
        print(r['sequence'],r['vio'],r['loop'],'loops',r['loops'])

if __name__=='__main__': main()
