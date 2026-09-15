# VINS-Fusion + LTV Stage 4 执行文档：Gravity Quality Gate

## 目标

Stage 2 表明固定 `sigma_g = 10 deg` 的 Gravity factor 工程稳定，但收益依赖场景。Stage 4 只验证一个可解释的 binary Quality Gate：

```text
GOOD -> 保持原 Gravity factor 和 sigma_g = 10 deg
BAD  -> 当前 snapshot 不加入 Gravity factor
```

不修改 LTV observer 方程、IMU/视觉 factor、VINS 状态布局或 marginalization。不实现连续权重、learned gate、RL、Velocity factor 或同帧双向反馈。

## 实际数据与统一运行规则

完整 EuRoC 验证集固定为：

```text
MH_01_easy      MH_02_easy      MH_03_medium
MH_04_difficult MH_05_difficult
V1_01_easy      V1_02_medium    V1_03_difficult
V2_01_easy      V2_02_medium    V2_03_difficult
```

每个序列的输入固定对应：

```text
ROS 2 bag: /home/he/datasets/euroc/<sequence>_db
官方 GT:   /home/he/datasets/euroc/ASL/<sequence>/mav0/state_groundtruth_estimate0/data.csv
```

所有模式均关闭 loop closure、以 `1.0×` 回放，并用 `--ground-truth-csv` 显式传入同名官方 GT。每个模式、每个序列运行一次；不得用旧 Stage 2 evaluator 数字代替本阶段结果。`V1_03_difficult`、`V2_03_difficult` 及全部 MH 序列是必跑重点，不能只运行 V1_01/V2_02/V2_03。

输出放在 `/home/he/output/ltv_stage4/<sequence>/<mode>/`，不提交运行产物。

## Gate 与日志

现有 `ltvGravityFactorEligible()` 保留为基础 eligibility。新增 Quality Gate 只能在其结果为真时放行，并且默认关闭：

```yaml
ltv_enable_gravity_quality_gate: 0
ltv_gravity_gate_min_features: <显式值>
ltv_gravity_gate_max_eta_norm_error: <显式值>
ltv_gravity_gate_max_normalized_innovation: <显式值或禁用>
ltv_gravity_gate_reset_cooldown_frames: <显式值>
```

第一版固定为：

```text
base eligible
AND observed_features >= gate_min_features
AND abs(||eta_hat|| - ||g||) <= gate_max_eta_norm_error
AND normalized innovation <= gate_max_normalized_innovation（若启用）
AND no reset in cooldown window
```

`innovation_norm` 不得直接跨序列使用原始绝对值；若启用，使用 `innovation_norm / sqrt(max(1, observed_features))`。协方差只记录和离线分析，未完成尺度标定前不作第一版 hard gate。

Gate 决策必须在 snapshot 产生时冻结，不能在每次 sliding-window optimization 中重新依赖可变全局状态。追加以下 CSV 字段，并保证 `reason_mask` 能解释每个拒绝项：

```text
gravity_gate_base_eligible
gravity_gate_feature_ok
gravity_gate_eta_norm_ok
gravity_gate_innovation_ok
gravity_gate_reset_ok
gravity_gate_pass
gravity_gate_reason_mask
```

先完成日志-only run：gate 开关关闭但仍计算并写出决策字段。根据全量 Passive 日志选择保守初值后冻结配置；后续 B/G_fixed/G_gate 实验中不得再按单个序列调阈值。

## 实验矩阵与验收

每个 11 序列均运行：

| 模式 | LTV | Gravity factor | Quality Gate |
|---|---:|---:|---:|
| B | 0 | 0 | 0 |
| G_fixed | 1 | 1 | 0 |
| G_gate | 1 | 1 | 1 |

统一记录 ATE RMSE/P95/max、Rotation RMSE/P95/max、roll/pitch RMSE、Gravity factor coverage、gate pass coverage、连续 gate-on 段、reset、NaN/Inf 和 solver failure。所有结果由同一官方 GT、时间匹配和 alignment 规则产生。

验收规则：

- G_gate 在正常序列相对 B 的 ATE、Rotation RMSE、roll/pitch RMSE 退化不超过 3%。
- 对 Vicon difficult 与 MH 困难序列，G_gate 必须保留 G_fixed 的正向收益，或将 G_fixed 相对 B 的负向主指标改善至少 1%。
- Gate coverage 不得接近全开或全关；分别报告 11 个序列的 pass 比例与 reason 分布。
- 默认关闭时，G_gate 路径必须与 G_fixed trajectory 字节级一致。

完成 Stage 4 后提交代码、测试与全量报告并停止。只有 Gate 具有选择性且满足上述稳定性条件，才允许进入 Stage 6 的 Gravity 分支。
