#!/usr/bin/env python3
import csv
import json
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import run_stage7_tuning as run
import evaluate_stage7_tuning as ev
import evaluate_euroc_final as old
import test_run_stage6_joint as stage6_tests


class Stage7Test(unittest.TestCase):
    def setUp(self):
        self.protocol = json.loads(run.PROTOCOL.read_text())
        self.base = (run.REPO/'config/euroc/euroc_stereo_imu_ltv_config.yaml').read_text()

    def test_override_and_old_default(self):
        default = run.parameters({},self.protocol)
        text,settings = run.config(self.base,'joint_v_gate',Path('/tmp/out'),default)
        prior,_ = run.effective_config(self.base,'joint_v_gate',Path('/tmp/out'))
        self.assertEqual(run.normalized(text),run.normalized(prior))
        values = run.parameters({'ltv_gravity_sigma_deg':5.},self.protocol)
        text,settings = run.config(self.base,'joint_v_gate',Path('/tmp/out'),values)
        self.assertIn('ltv_gravity_sigma_deg: 5.0',text)
        self.assertEqual(settings['freq'],'20')
        for request in [{'acc_n':.2},{'ltv_enable_velocity_oracle_gate':1},{'typo':1},{'ltv_velocity_sigma_mps':4},{'ltv_velocity_sigma_mps':True}]:
            with self.assertRaises(ValueError):
                run.parameters(request,self.protocol)

    def test_cache_identity_paths_and_binary(self):
        values = run.parameters({},self.protocol)
        a,_ = run.config(self.base,'joint_v_gate',Path('/tmp/a'),values)
        b,_ = run.config(self.base,'joint_v_gate',Path('/tmp/b'),values)
        self.assertEqual(run.cache_key({'binary':'a'},a),run.cache_key({'binary':'a'},b))
        self.assertNotEqual(run.cache_key({'binary':'a'},a),run.cache_key({'binary':'b'},b))
        with tempfile.TemporaryDirectory() as root:
            self.assertFalse(run.successful(Path(root),'key'))
            (Path(root)/'failure.json').write_text('{}')
            self.assertFalse(run.successful(Path(root),'key'))

    def test_budget_counts_failures_reserves_and_hard_limit(self):
        self.assertEqual(sum(self.protocol['budget_allocation'].values()),80)
        for controls, search in ((22,27),(0,45)):
            limit = 80
            self.assertLessEqual(controls+search+31,limit)
            with tempfile.TemporaryDirectory() as root:
                root = Path(root)
                for i in range(controls+search):
                    run.budget_start(root,limit,31,{'purpose':'control_or_search'})
                for i in range(limit-31-controls-search):
                    run.budget_start(root,limit,31,{'failed':True})
                with self.assertRaisesRegex(RuntimeError,'budget exhausted'):
                    run.budget_start(root,limit,31,{})
                for i in range(31):
                    run.budget_start(root,limit,0,{})
                with self.assertRaises(RuntimeError):
                    run.budget_start(root,limit,0,{})

    def test_degenerate_gates_keep_bypass_checks(self):
        helper = stage6_tests.Stage6JointTest()
        for coverage in (0,1):
            row = helper.row(gravity_gate_base_eligible=1,gravity_factor_added=coverage,
                gravity_gate_pass=coverage,velocity_factor_base_eligible=1,
                velocity_factor_added=coverage,velocity_gate_pass=coverage,
                velocity_gate_feature_ok=1,velocity_gate_innovation_ok=1,
                velocity_gate_disagreement_ok=1,velocity_gate_reset_ok=1)
            with tempfile.TemporaryDirectory() as root:
                path = helper.write_diagnostics(root,[row])
                result = __import__('run_stage6_joint').factor_diagnostics(path,'joint_v_gate',False)
                self.assertEqual(result['velocity_gate_coverage'],coverage)
                row['velocity_gate_pass']=0
                row['velocity_factor_added']=1
                path = helper.write_diagnostics(root,[row])
                with self.assertRaisesRegex(RuntimeError,'quality gate'):
                    __import__('run_stage6_joint').factor_diagnostics(path,'joint_v_gate',False)

    def test_missing_frames_rejected_and_support_exact(self):
        t = np.arange(100)*.05
        gt = {'timestamps':t}
        support = ev.common_support({'B':{'timestamps':t},'C':{'timestamps':np.delete(t,50)}},gt)
        self.assertEqual(support['common_count'],99)
        self.assertEqual(support['missing_anchor_indices'].tolist(),[50])
        with self.assertRaisesRegex(RuntimeError,'below 99%'):
            ev.common_support({'B':{'timestamps':t},'C':{'timestamps':np.delete(t,[40,50])}},gt)
        long = np.arange(1000)*.05
        with self.assertRaisesRegex(RuntimeError,'endpoint'):
            ev.common_support({'B':{'timestamps':long},'C':{'timestamps':long[3:]}},{'timestamps':long})

    def test_missing_hard_frame_recomputes_baseline_too(self):
        t = np.arange(100)*.05
        p = np.column_stack([np.sin(t),np.cos(t),t])
        rotation = np.tile(np.eye(3),(100,1,1))
        velocity = np.ones((100,3))
        estimated = p + np.column_stack([.01*np.sin(2*t),np.zeros(100),np.zeros(100)])
        estimated[50,0] += 10
        gt = dict(timestamps=t,positions=p,rotations=rotation,velocities=velocity)
        baseline = dict(gt,positions=estimated)
        candidate = {k:np.delete(v,50,axis=0) for k,v in baseline.items()}
        support = ev.common_support({'B':baseline,'C':candidate},gt)
        common_metrics = ev.metrics({'B':baseline,'C':candidate},gt,support)
        full_metrics = ev.metrics({'B':baseline},gt,ev.common_support({'B':baseline},gt))
        self.assertGreater(full_metrics['B']['ate_rmse_m'],common_metrics['B']['ate_rmse_m'])
        self.assertAlmostEqual(common_metrics['B']['ate_rmse_m'],common_metrics['C']['ate_rmse_m'])
        self.assertAlmostEqual(ev.score([{'metrics':common_metrics}],'C')['J'],0)

    def test_successful_skip_and_interrupted_attempt(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            config_file = root/'base.yaml'
            config_file.write_text(self.base)
            runner = run.Runner.__new__(run.Runner)
            runner.root = root
            runner.args = type('Args',(),dict(config=config_file,cache_root=root,replay=root/'binary'))()
            runner.base = self.base
            runner.registry = {'W0':run.parameters({},self.protocol)}
            runner.metadata = {'S':{k:'digest' for k in ('pair_list_sha256','imu_sha256','ground_truth_sha256')}}
            runner.binary_sha = 'binary'
            runner.deps = {}
            runner.calib = {}
            identity = dict(sequence='S',replay_sha256='binary',dependencies={},calibration_sha256={},base_config_sha256=run.sha256(config_file),**runner.metadata['S'])
            text,_ = run.config(self.base,'joint_v_gate',Path('/unused'),runner.registry['W0'])
            key = run.cache_key(identity,text)
            final = root/'trials/S'/key
            final.mkdir(parents=True)
            with patch.object(run,'successful',return_value=True), patch.object(run.subprocess,'run') as replay:
                self.assertEqual(runner.trial('W0','S'),final)
                replay.assert_not_called()
            with patch.object(run,'successful',return_value=False):
                with self.assertRaisesRegex(RuntimeError,'cache failed'):
                    runner.trial('W0','S')
            final.rmdir()
            run.append(root/'budget.jsonl',dict(candidate='W0',sequence='S',mode='joint_v_gate',cache_key=key,repeat=None))
            self.assertIsNone(runner.trial('W0','S'))

    def test_group_delta_is_ratio_of_means(self):
        rows = [{'metrics':{'B':{'ate':1},'C':{'ate':.5}}}, {'metrics':{'B':{'ate':10},'C':{'ate':11}}}]
        result = ev.group_summary(rows,['B','C'],'ate')
        self.assertAlmostEqual(result['means']['C'],5.75)
        self.assertAlmostEqual(result['delta_vs_B_percent']['C'],100*(11.5/11-1))

    def test_metrics_match_old_evaluator_and_tails(self):
        t = np.arange(100)*.05
        points = np.column_stack([np.sin(t),np.cos(t),t])
        gt = dict(timestamps=t,positions=points,rotations=np.tile(np.eye(3),(100,1,1)),velocities=np.ones((100,3)))
        runs = {n:dict(gt,positions=points + np.column_stack([.01*np.sin(2*t),np.zeros(100),np.zeros(100)])) for n in ['B','C']}
        support = ev.common_support(runs,gt)
        m = ev.metrics(runs,gt,support)
        old_runs = {mode:runs['B'] for mode in old.MODES}
        expected,_ = old.trajectory_metrics(old_runs,gt,old.common_time_support(old_runs,gt))
        for key in expected['baseline']:
            self.assertAlmostEqual(m['B'][key],expected['baseline'][key])
        self.assertGreaterEqual(m['B']['ate_max_m'],m['B']['ate_p95_m'])
        score = ev.score([{'metrics':{'B':{'ate_rmse_m':1.,'ate_p95_m':2.},'C':{'ate_rmse_m':.8,'ate_p95_m':1.8}}},
                          {'metrics':{'B':{'ate_rmse_m':10.,'ate_p95_m':20.},'C':{'ate_rmse_m':11.,'ate_p95_m':22.}}}],'C')
        self.assertAlmostEqual(score['J'],-.05)
        self.assertFalse(score['screening_pass'])


if __name__=='__main__':
    unittest.main()
