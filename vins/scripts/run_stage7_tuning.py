#!/usr/bin/env python3
"""Budgeted, serial, resumable Stage7 search and full-system publication."""
import argparse
import csv
import fcntl
import hashlib
import json
import os
import re
import shutil
import subprocess
import tempfile
import time
import uuid
from pathlib import Path

from run_stage6_joint import FROZEN_SETTINGS, effective_config, validate_output
from run_stage5_velocity_oracle import sha256, replace_setting
from evaluate_euroc_final import SEQUENCES, atomic_write, csv_text
from evaluate_stage7_tuning import evaluate, score, ranking_key

REPO = Path(__file__).resolve().parents[2]
PROTOCOL = REPO / 'config/tuning/stage7_protocol.json'


def write_json(path, value):
    atomic_write(path, json.dumps(value, indent=2, sort_keys=True, allow_nan=False)+'\n')


def append(path, value):
    with path.open('a') as stream:
        stream.write(json.dumps(value, sort_keys=True, allow_nan=False)+'\n')
        stream.flush()
        os.fsync(stream.fileno())


def parameters(request, protocol):
    allowed = protocol['allowed']
    if set(request) - set(allowed):
        raise ValueError(f'unknown/disallowed parameters: {set(request)-set(allowed)}')
    for key, value in request.items():
        if isinstance(value, bool) or value not in allowed[key]:
            raise ValueError(f'disallowed value: {key}={value}')
    result = {k: float(FROZEN_SETTINGS[k]) for k in allowed}
    result.update(request)
    return result


def config(base, mode, output, values):
    settings = dict(FROZEN_SETTINGS)
    settings.update({k: str(v) for k,v in values.items()})
    return effective_config(base, mode, output, settings)


def normalized(text):
    for key in ('output_path', 'ltv_debug_csv_path'):
        text = replace_setting(text, key, '"<OUTPUT>"')
    return text


def cache_key(identity, text):
    return hashlib.sha256(json.dumps(dict(identity=identity, config=normalized(text)), sort_keys=True).encode()).hexdigest()


def dependencies(binary):
    output = subprocess.check_output(['ldd', str(binary)], text=True)
    paths = re.findall(r'(?:=>\s+)?(/\S+)\s+\(', output)
    return {str(Path(p).resolve()): sha256(Path(p)) for p in paths}


def budget_start(root, limit, reserve, record):
    ledger = root/'budget.jsonl'
    used = len(ledger.read_text().splitlines()) if ledger.exists() else 0
    if used + 1 + reserve > limit:
        raise RuntimeError(f'budget exhausted: used={used}, reserve={reserve}, limit={limit}')
    append(ledger, dict(record, ordinal=used+1, started_utc=time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime())))
    return used+1


def verify_cache(cache):
    metadata = json.loads((cache/'metadata.json').read_text())
    if metadata.get('format') != 'ltv-stage5-cache-v2':
        raise RuntimeError('noncanonical cache')
    for key, path in [('pair_list_sha256',cache/'canonical_stereo_pairs.csv'), ('imu_sha256',cache/'imu.csv'), ('ground_truth_sha256',Path(metadata['ground_truth']))]:
        if sha256(path) != metadata[key]:
            raise RuntimeError(f'cache hash mismatch: {path}')
    return metadata


def successful(path, key):
    try:
        manifest = json.loads((path/'input_manifest.json').read_text())
        validation = json.loads((path/'validation.json').read_text())
        if manifest['cache_key'] != key or validation['replay_exit_code'] != 0:
            return False
        for name, digest in manifest['artifacts'].items():
            if sha256(path/name) != digest:
                return False
        check = validate_output(path, manifest['metadata'], manifest['mode'], strict_stage6=False)
        return all(check[k] == 0 for k in ('reset_count', 'solver_failure_count','dds_error_count','nan_inf_count'))
    except (OSError, ValueError, KeyError, RuntimeError):
        return False


class Runner:
    def __init__(self, args):
        self.args = args
        self.root = args.output_root.resolve()
        self.protocol = json.loads(PROTOCOL.read_text())
        self.base = args.config.read_text()
        self.original = parameters({}, self.protocol)
        self.binary_sha = sha256(args.replay)
        self.deps = dependencies(args.replay)
        self.calib = {}
        for key in ('cam0_calib','cam1_calib'):
            value = re.search(r'^'+key+r':\s*"([^"]+)"', self.base, re.M).group(1)
            path = args.config.parent/value
            self.calib[str(path.resolve())] = sha256(path)
        self.source = dict(commit=subprocess.check_output(['git','rev-parse','HEAD'],cwd=REPO,text=True).strip(),
            diff_sha256=hashlib.sha256(subprocess.check_output(['git','diff','HEAD'],cwd=REPO)).hexdigest(),
            status=subprocess.check_output(['git','status','--porcelain'],cwd=REPO,text=True))
        self.metadata = {s: verify_cache(args.cache_root/s) for s in SEQUENCES}
        self.registry = {}
        if (self.root/'candidate_registry.jsonl').exists():
            for line in (self.root/'candidate_registry.jsonl').read_text().splitlines():
                r = json.loads(line)
                self.registry[r['id']] = r['parameters']

    def register(self, name, request, reason):
        values = parameters(request, self.protocol)
        if name in self.registry:
            if self.registry[name] != values:
                raise RuntimeError('candidate changed on resume')
            return name
        for previous, existing in self.registry.items():
            if existing == values:
                return previous
        self.registry[name] = values
        append(self.root/'candidate_registry.jsonl',dict(id=name,parameters=values,reason=reason))
        return name

    def trial(self, name, sequence, mode='joint_v_gate', reserve=0, repeat=None, base=None):
        values = self.registry[name]
        metadata = self.metadata[sequence]
        identity = dict(sequence=sequence, replay_sha256=self.binary_sha, dependencies=self.deps,
            calibration_sha256=self.calib, base_config_sha256=sha256(self.args.config),
            **{k: metadata[k] for k in ('pair_list_sha256','imu_sha256','ground_truth_sha256')})
        base_text = self.base if base is None else base
        text, settings = config(base_text,mode,Path('/unused'),values)
        key = cache_key(identity,text)
        final = self.root/'trials'/sequence/(key if repeat is None else key+'-'+repeat)
        if final.exists():
            if not successful(final,key):
                raise RuntimeError(f'published cache failed verification: {final}')
            return final
        # Interrupted starts consume budget and remain auditable; do not auto-retry failures.
        ledger = self.root/'budget.jsonl'
        attempt = dict(candidate=name,sequence=sequence,mode=mode,cache_key=key,repeat=repeat)
        if ledger.exists() and any(all(json.loads(line).get(k)==v for k,v in attempt.items()) for line in ledger.read_text().splitlines()):
            return None
        if sha256(self.args.replay) != self.binary_sha or dependencies(self.args.replay) != self.deps:
            raise RuntimeError('frozen replay/dependencies changed')
        partial = final.with_name(final.name+'.partial-'+uuid.uuid4().hex)
        partial.mkdir(parents=True)
        text, settings = config(base_text,mode,partial,values)
        atomic_write(partial/'runtime_config.yaml',text)
        write_json(partial/'requested_parameters.json',values)
        write_json(partial/'resolved_parameters.json',{k:settings[k] for k in values})
        manifest = dict(identity,cache_key=key,mode=mode,metadata=metadata,source=self.source,
            frozen_settings=settings, runtime_config_sha256=sha256(partial/'runtime_config.yaml'))
        started = time.monotonic()
        temporary_path = None
        try:
            with tempfile.NamedTemporaryFile(mode='w',suffix='.yaml',prefix='stage7_',dir=self.args.config.parent,delete=False) as temp:
                temp.write(text)
                temporary_path = Path(temp.name)
            manifest['actual_config_path'] = str(temporary_path)
            manifest['actual_config_sha256'] = sha256(temporary_path)
            write_json(partial/'input_manifest.json',manifest)
            ordinal = budget_start(self.root,self.protocol['max_new_replays'],reserve,attempt)
            print(f'[{ordinal}/80] {name} {mode} {sequence}',flush=True)
            with (partial/'replay.log').open('w') as log:
                subprocess.run([str(self.args.replay),str(temporary_path),str(self.args.cache_root/sequence)],stdout=log,stderr=subprocess.STDOUT,check=True)
            if verify_cache(self.args.cache_root/sequence) != metadata or sha256(self.args.replay) != self.binary_sha or sha256(self.args.config) != identity['base_config_sha256']:
                raise RuntimeError('input or binary changed during replay')
            validation = validate_output(partial,metadata,mode,strict_stage6=False)
            for k in ('reset_count','solver_failure_count','dds_error_count','nan_inf_count'):
                if validation[k] != 0:
                    raise RuntimeError(f'engineering failure: {k}')
            validation.update(replay_exit_code=0,wall_time_s=time.monotonic()-started,
                degenerate_gate=[branch for branch in ('gravity','velocity') if validation[branch+'_gate_coverage'] in (0,1)])
            # Fraction of added-factor snapshots whose logged weighted norm exceeds Huber delta.
            rows = list(csv.DictReader((partial/'ltv_debug.csv').open()))
            validation['robust_region_fraction'] = {}
            for branch in ('gravity','velocity'):
                active = [r for r in rows if int(r[branch+'_factor_added'])]
                validation['robust_region_fraction'][branch] = sum(float(r[branch+'_factor_weighted_residual_norm']) > 2 for r in active)/len(active) if active else None
            write_json(partial/'validation.json',validation)
            published, _ = config(base_text,mode,final,values)
            # Copy calibrations beside reusable YAML; parser resolves relative paths.
            for path in self.calib:
                shutil.copy2(path,partial/Path(path).name)
            atomic_write(partial/'effective_config.yaml',published)
            manifest['published_config_sha256'] = sha256(partial/'effective_config.yaml')
            manifest['artifacts'] = {p.name:sha256(p) for p in partial.iterdir() if p.is_file() and p.name != 'input_manifest.json'}
            write_json(partial/'input_manifest.json',manifest)
            os.rename(partial,final)
            append(self.root/'trial_results.jsonl',dict(attempt,status='success',path=str(final),wall_time_s=validation['wall_time_s']))
            return final
        except (subprocess.CalledProcessError, RuntimeError, OSError) as error:
            failure = dict(attempt,status='failed',error=str(error),wall_time_s=time.monotonic()-started)
            write_json(partial/'failure.json',failure)
            failed = partial.with_name(partial.name.replace('.partial-','.failed-'))
            os.rename(partial,failed)
            append(self.root/'trial_results.jsonl',dict(failure,path=str(failed)))
            print('FAILED '+str(error),flush=True)
            return None
        finally:
            if temporary_path is not None:
                temporary_path.unlink(missing_ok=True)

    def comparisons(self, names, sequences, paths):
        result = []
        for s in sequences:
            selected = {n:paths.get((n,s)) for n in names}
            if any(p is None for p in selected.values()):
                raise RuntimeError(f'incomplete comparison: {names} {s}')
            row = evaluate(selected,Path(self.metadata[s]['ground_truth']))
            row['sequence'] = s
            result.append(row)
        return result

    def rank(self, names, sequences, paths, label):
        rows = []
        for name in names:
            try:
                evidence = self.comparisons(['B','W0'] + ([] if name=='W0' else [name]),sequences,paths)
                sc = score(evidence,name)
                rows.append(dict(candidate=name,engineering_pass=True,**sc))
                write_json(self.root/'evaluations'/f'{label}-{name}.json',evidence)
            except RuntimeError as error:
                rows.append(dict(candidate=name,engineering_pass=False,screening_pass=False,error=str(error)))
        atomic_write(self.root/f'{label}_ranking.json',json.dumps(rows,indent=2)+'\n')
        good = [r for r in rows if r['engineering_pass'] and r['screening_pass']]
        good.sort(key=lambda r:ranking_key(r,self.registry[r['candidate']],self.original))
        return good

    def gate_candidates(self, weight, paths):
        choices = []
        for dimension, key in enumerate(list(self.protocol['allowed'])[2:]):
            alternatives = [v for v in self.protocol['allowed'][key] if v != self.original[key]]
            estimates = []
            for value in alternatives:
                changed = 0
                for s in self.protocol['D0']:
                    with (paths['W0',s]/'ltv_debug.csv').open() as stream:
                        for r in csv.DictReader(stream):
                            branch = 'gravity' if 'gravity' in key else 'velocity'
                            if 'eta_norm' in key:
                                passed = abs(float(r['gravity_norm'])-9.81007) <= value
                                fields = ['feature','innovation','reset']
                            elif 'disagreement' in key:
                                passed = float(r['velocity_factor_residual_norm']) <= value
                                fields = ['feature','innovation','reset']
                            else:
                                passed = float(r['innovation_norm'])/max(1,float(r['observed_features']))**0.5 <= value
                                fields = ['feature','eta_norm','reset'] if branch=='gravity' else ['feature','disagreement','reset']
                            eligible = int(r['gravity_gate_base_eligible' if branch=='gravity' else 'velocity_factor_base_eligible']) and all(int(r[branch+'_gate_'+f+'_ok']) for f in fields)
                            new_pass = bool(eligible and passed)
                            changed += new_pass != bool(int(r[branch+'_gate_pass']))
                estimates.append(dict(key=key,value=value,estimated_changed_frames=changed))
            best = max(estimates,key=lambda r:r['estimated_changed_frames'])
            request = dict(self.registry[weight], **{key:best['value']})
            name = self.register(f'T{dimension+1}',request,'largest W0 pass-count change; document-order tie')
            choices.append(dict(best,candidate=name,alternatives=estimates))
        write_json(self.root/'gate_selection.json',choices)
        return [r['candidate'] for r in choices]

    def run(self):
        p = self.protocol
        paths = {}
        self.register('W0',{},'frozen no-tune Joint')
        # B shares the six default numeric values; its mode disables both factors.
        controls = [(n,s) for s in list(p['D0'])+[s for s in SEQUENCES if s not in p['D0']] for n in ('B','W0')]
        for index,(name,s) in enumerate(controls):
            result = self.trial('W0',s,'baseline' if name=='B' else 'joint_v_gate',reserve=len(controls)-index-1+31)
            if result is None:
                raise RuntimeError('required B/W0 failed; batch incomplete')
            paths[name,s] = result
            if s==p['D0'][0] and name=='W0':
                audit = {}
                for n, mode in [('B','baseline'),('W0','joint_v_gate')]:
                    old = Path('/home/he/output/ltv_euroc_final_unified')/s/mode
                    audit[n] = dict(old_binary=json.loads((old/'input_manifest.json').read_text())['replay_sha256'],
                        new_binary=self.binary_sha,old_vio_sha=sha256(old/'vio.csv'),new_vio_sha=sha256(paths[n,s]/'vio.csv'),
                        old_summary=json.loads((old/'replay_summary.json').read_text()),new_summary=json.loads((paths[n,s]/'replay_summary.json').read_text()))
                write_json(self.root/'representative_binary_audit.json',audit)
        # Persist fixed B/GT anchors independently of candidate support.
        for s in SEQUENCES:
            write_json(self.root/'anchors'/f'{s}.json',self.comparisons(['B'],[s],paths)[0])
        weights = ['W0']
        for name,key,value in [('W1','ltv_gravity_sigma_deg',5.),('W2','ltv_gravity_sigma_deg',20.),('W3','ltv_velocity_sigma_mps',.5),('W4','ltv_velocity_sigma_mps',2.)]:
            weights.append(self.register(name,{key:value},'single-axis weight search'))
            for s in p['D0']:
                paths[name,s] = self.trial(name,s,reserve=31)
        rank = self.rank(weights,p['D0'],paths,'weights')
        def axis_best(names):
            options = [r for r in rank if r['candidate'] in names]
            return options[0]['candidate'] if options else 'W0'
        g,v = axis_best(['W0','W1','W2']),axis_best(['W0','W3','W4'])
        w5 = self.register('W5',{'ltv_gravity_sigma_deg':self.registry[g]['ltv_gravity_sigma_deg'],
            'ltv_velocity_sigma_mps':self.registry[v]['ltv_velocity_sigma_mps']},f'combine {g} and {v}')
        if w5 not in weights:
            weights.append(w5)
            for s in p['D0']:
                paths[w5,s] = self.trial(w5,s,reserve=31)
        rank = self.rank(weights,p['D0'],paths,'weights_final')
        w0_score = score(self.comparisons(['B','W0'],p['D0'],paths),'W0')
        weight = rank[0]['candidate'] if rank and rank[0]['J'] < w0_score['J'] else 'W0'
        gates = self.gate_candidates(weight,paths)
        for name in gates:
            for s in p['D0']:
                paths[name,s] = self.trial(name,s,reserve=31)
        candidates = list(dict.fromkeys(weights+gates))
        rank = self.rank(candidates,p['D0'],paths,'D0')
        finalists = [r['candidate'] for r in rank if r['candidate'] != 'W0'][:2]
        write_json(self.root/'finalists.json',finalists)
        for name in finalists:
            for s in p['D1']:
                paths[name,s] = self.trial(name,s,reserve=15+len(p['remaining'])*len(finalists))
        self.rank(['W0']+finalists,p['D0']+p['D1'],paths,'D1')
        for name in finalists:
            for s in p['remaining']:
                paths[name,s] = self.trial(name,s,reserve=15)
        self.rank(['W0']+finalists,SEQUENCES,paths,'all11_pairwise')
        # Selection uses one shared support for both finalists, B and W0.
        complete = [n for n in finalists if all(paths.get((n,s)) is not None for s in SEQUENCES)]
        selection = self.comparisons(['B','W0']+complete,SEQUENCES,paths)
        scored = [dict(candidate=n,**score(selection,n)) for n in ['W0']+complete]
        scored.sort(key=lambda r:ranking_key(r,self.registry[r['candidate']],self.original))
        w0_j = score(selection,'W0')['J']
        winners = [r for r in scored if r['candidate']!='W0' and r['screening_pass'] and r['J'] < w0_j]
        final = winners[0]['candidate'] if winners else 'W0'
        write_json(self.root/'selection.json',dict(final=final,ranking=scored,common_evidence=selection,
            conclusion='improved qualified all11 candidate' if winners else '预算内未找到更优参数'))
        atomic_write(self.root/'candidate_ranking.csv',csv_text(scored))
        self.freeze_parameters(final)
        for s in SEQUENCES:
            paths['G',s] = self.trial(final,s,'gravity_only',reserve=4)
            if paths['G',s] is None:
                raise RuntimeError('required G-only control failed')
        final_rows = self.comparisons(['B','W0',final,'G'] if final!='W0' else ['B','W0','G'],SEQUENCES,paths)
        write_json(self.root/'final_evaluation.json',final_rows)
        repeats = []
        for i in range(3):
            result = self.trial(final,p['repeat_sequence'],reserve=1,repeat=f'repeat-{i+1}')
            repeats.append(dict(index=i+1,path=str(result) if result else None,
                evidence=evaluate({'B':paths['B',p['repeat_sequence']], 'repeat':result},Path(self.metadata[p['repeat_sequence']]['ground_truth'])) if result else None,
                vio_identical=sha256(result/'vio.csv')==sha256(paths[final,p['repeat_sequence']]/'vio.csv') if result else False))
        write_json(self.root/'repeat_results.json',repeats)
        release = self.root/'release'
        release.mkdir(exist_ok=True)
        output = release/'output'
        output.mkdir(exist_ok=True)
        text,_ = config(self.base,'joint_v_gate',output,self.registry[final])
        atomic_write(release/'euroc_stage7.yaml',text)
        for path in self.calib:
            shutil.copy2(path,release/Path(path).name)
        self.freeze_parameters(final)
        write_json(release/'release_manifest.json',dict(candidate=final,parameters=self.registry[final],
            config_sha256=sha256(release/'euroc_stage7.yaml'),replay_sha256=self.binary_sha,dependencies=self.deps,
            calibration_sha256=self.calib))
        acceptance = self.trial(final,p['delivery_sequence'],repeat='delivery',base=(release/'euroc_stage7.yaml').read_text())
        write_json(release/'acceptance.json',dict(offline_replay_pass=acceptance is not None,
            path=str(acceptance) if acceptance else None, ros_topics_verified=False,rviz_verified=False,
            note='offline feeder registers publishers; live ROS/RViz evidence is recorded separately'))
        self.report(final,final_rows,repeats,acceptance)

    def freeze_parameters(self, final):
        text = '%YAML:1.0\n'+''.join(f'{k}: {v}\n' for k,v in sorted(self.registry[final].items()))
        path = self.root/'stage7_final_parameters.yaml'
        atomic_write(path,text)
        atomic_write(self.root/'stage7_final_parameters.sha256',sha256(path)+'  stage7_final_parameters.yaml\n')
        write_json(self.root/'parameter_freeze.json',dict(candidate=final,parameters=self.registry[final],
            sha256=sha256(path),selection_sha256=sha256(self.root/'selection.json'),
            stage='all11 selection, before same-parameter G-only control'))

    def report(self, final, rows, repeats, acceptance):
        names = ['B','G','W0']+([final] if final!='W0' else [])
        lines = ['# Stage7 EuRoC 调参结果','',
            'EuRoC 全 11 序列是调参集。ATE 使用各模式独立无 scale 的 SE(3) 对齐；Rotation/Velocity 使用共同 B 对齐旋转。速度 GT 来自官方 CSV velocity 列。', '',
            f'唯一发布候选：`{final}`。Replay SHA：`{self.binary_sha}`。',
            '预算内未找到更优参数。' if final=='W0' else '全 11 序列共同支持集上，候选合格且 J 严格低于 W0。',
            f"新回放启动数（含失败）：{len((self.root/'budget.jsonl').read_text().splitlines())}/80。",'']
        for metric in ('ate_rmse_m','ate_p95_m','ate_max_m','rotation_rmse_deg','velocity_rmse_mps'):
            lines += [f'## {metric}', '', '| Sequence | '+' | '.join(names)+' |','|---|'+'---:|'*len(names)]
            for r in rows:
                lines.append('| '+r['sequence']+' | '+' | '.join(f"{r['metrics'][n][metric]:.6f}" for n in names)+' |')
            group_deltas = []
            for label, subset in [('Easy/Medium Mean',[r for r in rows if not r['sequence'].endswith('difficult')]),('Difficult Mean',[r for r in rows if r['sequence'].endswith('difficult')]),('All Mean',rows)]:
                means = {n:sum(r['metrics'][n][metric] for r in subset)/len(subset) for n in names}
                lines.append('| '+label+' | '+' | '.join(f'{means[n]:.6f}' for n in names)+' |')
                group_deltas.append(label+' Δ vs B (ratio of group means): '+', '.join(f'{n} {(means[n]/means["B"]-1)*100:+.2f}%' for n in names if n!='B'))
            lines += ['']+group_deltas+['']
        final_name = final
        j = score(rows,final_name)['J']
        w0 = score(rows,'W0')['J']
        gj = score(rows,'G')['J']
        visualization_path = self.root/'release/ros_visualization_evidence.json'
        visualization = json.loads(visualization_path.read_text()) if visualization_path.exists() else {}
        topics_verified = bool(visualization.get('topic_counts')) and all(v>0 for v in visualization['topic_counts'].values())
        rviz_verified = bool(visualization.get('screenshot_verified'))
        live_path = self.root/'release/live_startup.json'
        live_verified = live_path.exists() and json.loads(live_path.read_text()).get('startup_pass',False)
        records = [json.loads(l) for l in (self.root/'trial_results.jsonl').read_text().splitlines()]
        velocity_factor_total = sum(json.loads((Path(r['path'])/'validation.json').read_text())['velocity_factor_added_count']
            for r in records if r['status']=='success' and r['mode']=='joint_v_gate' and r['candidate']==final and r['repeat'] is None)
        conclusions = dict(system_offline_runnable=acceptance is not None,system_live_startup=live_verified,improved_vs_W0=j<w0,
            improved_vs_B=j<0,velocity_incremental_value=j<gj and velocity_factor_total>0,
            velocity_factor_total=velocity_factor_total,final_J=j,W0_J=w0,G_only_J=gj,
            ros_topics_verified=topics_verified,rviz_verified=rviz_verified)
        macro_final = sum(r['metrics'][final]['ate_rmse_m'] for r in rows)/len(rows)
        macro_w0 = sum(r['metrics']['W0']['ate_rmse_m'] for r in rows)/len(rows)
        worst_w0 = max(r['metrics'][final]['ate_rmse_m']/r['metrics']['W0']['ate_rmse_m']-1 for r in rows)
        conclusions.update(J_change_vs_W0_percentage_points=100*(j-w0),
            macro_ate_change_vs_W0_percent=100*(macro_final/macro_w0-1),
            macro_ate_improved_vs_W0=macro_final<macro_w0,worst_sequence_change_vs_W0_percent=100*worst_w0)
        write_json(self.root/'conclusions.json',conclusions)
        lines += ['## 结论', '', '```json',json.dumps(conclusions,indent=2),'```','',
            '离线 ATE 不代表实时链路性能。ROS topic 和 RViz 验收单独记录，不由离线结果推定。',
            f'选参目标是逐序列相对 B 的等权 J；绝对 ATE 算术均值相对 W0 为 {conclusions["macro_ate_change_vs_W0_percent"]:+.3f}%，最坏单序列相对 W0 为 {100*worst_w0:+.3f}%。筛选 +3% 门槛针对 B。',
            '这不构成统计显著性或未见数据泛化的结论。EuRoC 是调参集，三次复测只覆盖固定 V1_01 序列。', '',
            '## 覆盖率、尾部与耗时', '']
        for r in rows:
            lines.append(f"- {r['sequence']}: samples={r['support']['common_count']}, coverage={r['support']['coverage']}")
            for n in names:
                m,b = r['metrics'][n],r['metrics']['B']
                if m['ate_p95_m']/b['ate_p95_m']>1.1 or m['ate_max_m']/b['ate_max_m']>1.1:
                    lines.append(f'  Tail warning: {n} P95/max exceeds B by >10%; inspect saved common-support trajectories.')
        records = [json.loads(l) for l in (self.root/'trial_results.jsonl').read_text().splitlines()]
        lines += ['',f"总 replay wall time: {sum(r['wall_time_s'] for r in records):.1f} s。逐次耗时、Gate reason-mask/factor counts/robust-region incidence 见 trial_results.jsonl 与各 validation.json。",'',
            '## 全部复测', '', '```json',json.dumps([dict(index=r['index'],path=r['path'],vio_identical=r['vio_identical'],metrics=r['evidence']['metrics'] if r['evidence'] else None) for r in repeats],indent=2),'```','',
            '## Estimator 回放失败记录','', '```json',json.dumps([r for r in records if r['status']=='failed'],indent=2),'```','',
            '排名、所有候选参数与淘汰原因见 *_ranking.json、candidate_registry.jsonl。历史 Stage6/notune 未修改。']
        atomic_write(self.root/'stage7_report.md','\n'.join(lines)+'\n')
        atomic_write(REPO/'docs/euroc_tuned_results.md','\n'.join(lines)+'\n')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config',type=Path,default=REPO/'config/euroc/euroc_stereo_imu_ltv_config.yaml')
    parser.add_argument('--cache-root',type=Path,default=Path('/home/he/output/ltv_stage5_minimal_fix/cache'))
    parser.add_argument('--replay',type=Path,default=Path('/home/he/vins_fusion_ltv_ws/build/vins/stage5_replay'))
    parser.add_argument('--output-root',type=Path,required=True)
    parser.add_argument('--dry-run',action='store_true')
    args = parser.parse_args()
    for key in ('config','cache_root','replay','output_root'):
        setattr(args,key,getattr(args,key).resolve())
    args.output_root.mkdir(parents=True,exist_ok=True)
    with (args.output_root/'.lock').open('w') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        runner = Runner(args)
        plan = dict(runner.protocol,binary_sha256=runner.binary_sha,dependencies=runner.deps,
            config_sha256=sha256(args.config),calibration_sha256=runner.calib,
            branch='rebuild B/W0 all11',max_unique_candidate_configs=10,
            source=runner.source,protocol_sha256=sha256(PROTOCOL))
        plan_path = args.output_root/'stage7_plan.json'
        if plan_path.exists():
            old = json.loads(plan_path.read_text())
            for k in ('binary_sha256','dependencies','config_sha256','protocol_sha256','calibration_sha256'):
                if old[k] != plan[k]:
                    raise RuntimeError(f'frozen plan changed: {k}')
        else:
            write_json(plan_path,plan)
        if args.dry_run:
            schedule = dict(plan,paths={key:str(getattr(args,key)) for key in ('config','cache_root','replay','output_root')},
                weight_candidates=[dict(id=name,parameters=parameters(request,runner.protocol),sequences=runner.protocol['D0'])
                    for name,request in [('W0',{}),('W1',{'ltv_gravity_sigma_deg':5.}),('W2',{'ltv_gravity_sigma_deg':20.}),('W3',{'ltv_velocity_sigma_mps':.5}),('W4',{'ltv_velocity_sigma_mps':2.})]],
                W5='combine independently best qualified axes; skip duplicate',
                gate_selection='one largest estimated W0 pass-count change per dimension; tie by protocol order',
                control_trials=[dict(candidate=n,sequences=list(SEQUENCES)) for n in ('B','W0')],
                time_budget='measure first sequence; do not promise fixed wall time',
                protected_future_replays=31)
            write_json(args.output_root/'dry_run_schedule.json',schedule)
            print(json.dumps(schedule,indent=2))
        else:
            runner.run()


if __name__ == '__main__':
    main()
