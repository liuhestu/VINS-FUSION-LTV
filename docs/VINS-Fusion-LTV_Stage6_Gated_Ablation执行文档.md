# VINS-Fusion + LTV Stage 6 执行文档：Joint Structure Validation

## 目标与结论边界

Stage 6 只验证固定联合结构在同一 deterministic replay 下的整体兼容性：

\[
B \quad vs \quad B+G_{\text{gate}}+V_{\text{fixed}}
\]

其中 `B` 是开启 passive LTV observer、但不向优化器加入 LTV factor 的等价
Baseline；Joint 同时开启现有 Gravity Quality Gate 和固定 Velocity factor。Stage 5 的
measurement-accuracy Oracle 已验证失败，因此本阶段删除 Oracle、2×2 ablation、learned
gate 和 RL 分支。只比较 Joint 对 B 能判断最终结构是否稳定兼容，不能提供严格的单因子
交互因果消融。

## 单 factor 背景证据（不作 Stage 6 正式判定）

Stage 4 的 `G_gate` 和补齐后的 Stage 5 `V_fixed` 表只用于解释 Joint 结果。每条序列记录
它们相对各自 Passive reference 的 ATE、Rotation 与 Velocity 变化，但禁止把不同阶段的
运行拼成 Joint 结果。Stage 6 的正式判定只来自本阶段新跑的 B 与 Joint。

已知背景如下：Stage 4 `G_gate` 在 MH_04 的 ATE 改善 6.13%，但 V2_03 未保留
`G_fixed` 的主要收益；Stage 5 证明瞬时 velocity accuracy 不适合作为非学习式 gate
判据，故只保留 `V_fixed`。完整逐序列参考表见 `stage_test_result.md`。

## 数据、模式与冻结参数

完整运行 EuRoC 11 序列：

```text
MH_01_easy      MH_02_easy      MH_03_medium
MH_04_difficult MH_05_difficult
V1_01_easy      V1_02_medium    V1_03_difficult
V2_01_easy      V2_02_medium    V2_03_difficult
```

每个模式读取同一份 Stage 5 canonical v2 cache，单线程、20 Hz、离线确定性消费；官方
GT 为 `/home/he/datasets/euroc/ASL/<sequence>/mav0/state_groundtruth_estimate0/data.csv`。

| 模式 | Gravity factor | Gravity Gate | Velocity factor | Velocity Oracle |
|---|---:|---:|---:|---:|
| `baseline` | OFF | OFF | OFF | OFF |
| `joint` | ON | ON | fixed | OFF |

冻结参数：

```text
sigma_g = 10 deg                 sigma_v = 1.0 m/s
gravity_huber = 2.0              velocity_huber = 2.0
gate_min_features = 15           gate_max_eta_norm_error = 0.2
gate_max_normalized_innovation = 0.05
gate_reset_cooldown_frames = 0
```

## 事务 runner 与运行命令

`vins/scripts/run_stage6_joint.py` 的固定接口为：

```bash
python3 vins/scripts/run_stage6_joint.py \
  --mode baseline \
  --config config/euroc/euroc_stereo_imu_ltv_config.yaml \
  --cache /home/he/output/ltv_stage5_minimal_fix/cache/MH_01_easy \
  --replay /home/he/vins_fusion_ltv_ws/install/vins/lib/vins/stage5_replay \
  --output-root /home/he/output/ltv_stage6
```

将 `--mode` 改为 `joint` 运行联合结构。runner 先写
`<mode>.partial-<uuid>`；全部校验通过后原子改名为 `<mode>`，异常则保留为
`<mode>.failed-<UTC>-<uuid>`。正式目录拒绝覆盖。

发布前必须满足：

- canonical pair 全部消费，cache、GT 和 base config 的 SHA-256 在运行前后不变；
- `vio.csv`、`ltv_debug.csv` 全部数值有限，非零时间戳严格递增；
- Baseline 两类 factor 加入量均为 0；Joint 两类 factor 加入量均非 0；
- Joint 的 Gravity factor 仅在 `base_eligible && gate_pass` 时加入，且加入数等于
  gate pass 数；fixed Velocity 加入数等于 velocity base eligible 数；
- Oracle mask 未加载、未命中，日志无 solver、DDS、SIGSEGV 或 NaN 标志。

## 统一评价与报告

统一 evaluator 对同一官方 GT、时间匹配和 SE(3) alignment 输出：

```text
ATE RMSE / P95 / max
Rotation RMSE / P95 / max
roll / pitch RMSE
Velocity RMSE / P95 / max
```

逐序列同时报告 Gravity Gate coverage、Gravity/Velocity factor 数、reset、solver
failure、DDS error 和 NaN/Inf。分组汇总使用：

- easy/medium：MH_01、MH_02、MH_03、V1_01、V1_02、V2_01、V2_02；
- difficult：MH_04、MH_05、V1_03、V2_03。

相对变化统一定义为 `(Joint - B) / B * 100%`，负值表示误差改善。

## 验收规则

- 22 次 replay 正常退出且输入消费完整；Joint 不新增 reset、solver failure、DDS 错误
  或 NaN/Inf。
- easy/medium 每条序列的 ATE、Rotation、Velocity RMSE 相对 B 均不得退化超过 3%。
- difficult 单独判断：至少一个主指标改善，且其他主指标均无超过 3% 的退化，才计为
  正向局部证据。
- 任一结构或稳定性条件失败，Stage 6 定性失败并停止，不执行 Stage 7。

输出固定在 `/home/he/output/ltv_stage6/`，不提交运行产物。Stage 6 完成后只更新
`stage_test_result.md`，不自动启动 Stage 7。
