# VINS-Fusion + LTV Stage 5 执行文档：Velocity Oracle Gate

## 目标

Stage 3 的固定 Velocity factor 工程稳定但未达到收益门槛。Stage 5 先用官方 GT 构造离线 Oracle，验证“仅在 LTV velocity 优于 VINS velocity 时加入 factor”是否存在值得继续研究的上限。

Oracle 仅是离线反事实实验：不得被描述为在线或可部署算法，不得把 GT 读入 estimator，不得进入默认配置、marginalization、learned gate 或 RL。

## 实际数据与实验集

全量 11 序列、bag、官方 GT、loop closure OFF、`1.0×` 回放规则与 Stage 4 完全相同：

```text
bag: /home/he/datasets/euroc/<sequence>_db
GT:  /home/he/datasets/euroc/ASL/<sequence>/mav0/state_groundtruth_estimate0/data.csv
```

所有 velocity 指标直接使用官方 CSV 的 `v_RS_R_x/y/z`，不使用位置差分 fallback。`V1_03_difficult`、`V2_03_difficult` 和 MH_01–MH_05 必须全部纳入分析与回放。

输出分别放入 `/home/he/output/ltv_stage5/<sequence>/`；oracle mask、分析 JSON/CSV 和运行轨迹均不提交仓库。

## Oracle 定义与 mask

从同一时间戳的 Baseline + Passive LTV 输出计算：

\[
v_{B,GT}=R_{WB,GT}^{T}V_{W,GT}
\]

\[
e_{LTV}=||\hat v_{B,LTV}-v_{B,GT}||,
\qquad
e_{VINS}=||R_{WB,VINS}^{T}V_{W,VINS}-v_{B,GT}||
\]

\[
A_v=e_{VINS}-e_{LTV}
\]

第一版 mask 条件固定为 `A_v > 0`。`A_v > 0.02 m/s` 只作离线敏感性统计，第一轮不做 margin sweep。

`build_velocity_oracle_mask.py` 必须：

- 校验 GT、VINS 和 LTV snapshot 均来自同一 sequence；
- 用最近邻时间匹配，误差必须不大于 `ltv_snapshot_max_time_error`；
- 只为 valid、velocity-valid 且无 reset 的 snapshot 生成候选行；
- 输出 `timestamp, oracle_velocity_on, advantage_mps, ltv_error_mps, vins_error_mps`；
- 对缺失、重复、时间不匹配行输出 off；不得默认为 fixed Velocity factor。

Run A 生成 mask，Run B 按冻结 mask 回放。Run B 中如 snapshot 与 mask 未在容差内匹配，必须 fail-closed，不加 Velocity factor，并记录 mask-miss。Gravity factor 始终关闭：

```yaml
ltv_enable_gravity_factor: 0
ltv_enable_velocity_factor: 1
ltv_enable_velocity_oracle_gate: 1
ltv_velocity_oracle_mask_path: <sequence mask path>
```

## 分析、实验与判定

每个序列先离线报告：

```text
P(A_v > 0)
P(A_v > 0.02 m/s)
positive advantage median / P90
oracle-positive 连续段长度
与 feature count、normalized innovation、motion intensity、gravity disagreement 的关系
```

再对完整 11 序列运行：

| 模式 | Gravity | Velocity |
|---|---:|---:|
| B | OFF | OFF |
| V_fixed | OFF | 固定 `sigma_v = 1.0 m/s` |
| V_oracle | OFF | Oracle mask |

统一使用官方 GT 输出 position、orientation 和 velocity 的 RMSE/P95/max，以及 oracle coverage、mask hit/miss、factor coverage、reset、NaN/Inf、solver failure。完整报告必须单列 Vicon difficult 与每个 MH 序列，不能只汇总平均值。

若 V_oracle 相比 V_fixed 没有跨多个困难序列的一致改善，或仍整体变差，则停止 Velocity Quality Gate、learned gate 与 RL 路线；Stage 6 不包含 Velocity 分支。若 V_oracle 在困难序列显示清晰且可解释的改善，才允许作为 Stage 6 的 upper-bound control。

## 工程验收

- mask parser、时间容差、缺失 mask fail-closed、GT 不进入 estimator 必须有自动化测试。
- Oracle gate 默认关闭时，trajectory 必须与 V_fixed 字节级一致。
- 只增加独立 Oracle mask 模块，不修改 LtvObserver 方程、Velocity residual、VINS state layout 或 marginalization。
- 完成后提交代码、测试和全量 11 序列报告并停止，不自动进入 Stage 6。
