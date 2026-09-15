# VINS-Fusion + LTV Stage 6 执行文档：Gated Ablation

## 前置条件与目标

Stage 6 只在 Stage 4 的 Gravity Quality Gate 通过稳定性验收后执行。Velocity Oracle 只有在 Stage 5 显示相对 fixed Velocity 的一致正向上限收益时才进入本阶段。

目标是使用同一 evaluator、同一官方 GT 和完整 EuRoC 11 序列，区分收益来自 factor 本身还是来自选择性使用 factor。Stage 6 不引入 marginalization、连续自适应权重、learned gate、RL、新状态或 LTV landmark factor。

## 数据与统一评价

每个模式、每个序列都使用：

```text
ROS 2 bag: /home/he/datasets/euroc/<sequence>_db
官方 GT:   /home/he/datasets/euroc/ASL/<sequence>/mav0/state_groundtruth_estimate0/data.csv
```

运行完整 11 序列：MH_01、MH_02、MH_03、MH_04、MH_05、V1_01、V1_02、V1_03、V2_01、V2_02、V2_03。loop closure OFF，回放 `1.0×`。`V1_03_difficult`、`V2_03_difficult` 以及 MH_04/MH_05 必须单独分析，不得被全局平均值掩盖。

统一 evaluator 必须从同一官方 CSV、同一最近邻时间匹配和同一 alignment 规则输出：

```text
ATE RMSE / P95 / max
Rotation RMSE / P95 / max
roll / pitch RMSE
Velocity RMSE / P95 / max
```

如果当前 evaluator 缺少 position 或 rotation P95，先补齐 evaluator 和其自动化测试，再开始 Stage 6 实验。禁止拼接旧 Stage 2 与新 Stage 3 的不同 evaluator 数字。

## 消融矩阵

固定 reference controls：

| 模式 | Gravity | Velocity |
|---|---:|---:|
| B | OFF | OFF |
| G_fixed | fixed `10 deg` | OFF |
| G_gate | Quality Gate | OFF |
| V_fixed | OFF | fixed `1.0 m/s` |

只有 Stage 5 通过时，增加 Oracle upper-bound 分支：

| 模式 | Gravity | Velocity |
|---|---:|---:|
| V_oracle | OFF | Oracle Gate |
| G_gate + V_oracle | Quality Gate | Oracle Gate |

`V_oracle` 和 `G_gate + V_oracle` 必须明确标记为非部署 upper bound。若 Stage 5 未通过，Stage 6 仅报告 B/G_fixed/G_gate，并将 Velocity 标注为 neutral/negative result；不为了凑齐 2×2 而运行 Oracle。

## 报告与判定

每个模式、每个序列同时报告：

```text
Gravity factor coverage
Velocity factor coverage
gate coverage / reason distribution
mask hit / miss（如适用）
reset count、NaN / Inf、solver failure
```

对 Vicon difficult 与 MH 困难段输出位置、姿态、速度误差时间序列，并标记 gravity gate、oracle gate、feature count、normalized innovation、motion intensity 和 reset。除了整体 RMSE，也分析剧烈运动前、中、后的连续片段。

进入后续 marginalization、continuous gate、learned gate 或 RL 的最低条件为：

- 正常序列相对 Baseline 的主指标退化不超过 3%；
- 至少一个可部署的 gated factor 在多个困难序列上呈现可重复正收益；
- gate 覆盖率、拒绝原因和误差变化一致且可解释；
- factor 不会频繁把错误信息加入窗口。

完整运行输出放入 `/home/he/output/ltv_stage6/`，不提交运行产物。只提交统一 evaluator、必要脚本、汇总报告和独立测试；Stage 6 完成后停止，不自动进入 Stage 7。
