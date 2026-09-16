# VINS-Fusion + LTV 阶段验证指标

本文记录各阶段验证指标。EuRoC 正式实验均关闭 loop closure，并使用 `1×` 回放速度。

## 模式开关

| 模式 | `ltv_enable` | `ltv_enable_gravity_factor` | `ltv_enable_velocity_factor` | 含义 |
|---|---:|---:|---:|---|
| Baseline | 0 | 0 | 0 | 原始 VINS-Fusion，LTV 完全关闭 |
| Passive LTV | 1 | 0 | 0 | LTV Observer 只运行和记录，不向优化器加入 factor |
| LTV Gravity | 1 | 1 | 0 | 仅加入 LTV Gravity factor |
| LTV Velocity | 1 | 0 | 1 | 仅加入 LTV Velocity factor |

## 阶段状态

| 阶段 | 状态 | 指标结论 |
|---|---|---|
| Stage 1 — Passive LTV | 通过 | Observer 输出有限，且关闭 factor 时 VINS 轨迹不变 |
| Stage 2 — Gravity Factor | 初步通过 | 部分困难序列姿态改善，但收益不一致，适合作为条件性弱约束 |
| Stage 3 — Velocity Factor | 工程通过，收益未通过 | 三个序列稳定运行，但困难序列未达到约 3% 的改善门槛 |
| Stage 4 — Gravity Quality Gate | 工程通过，效果未通过 | Gate 选择性工作且稳定，但未保留 V2_03 固定 Gravity 的收益 |

## Stage 1 — Passive LTV

### V1_01_easy Observer 指标

| 指标 | 结果 |
|---|---:|
| CSV frames | 1445 |
| valid frames | 1232 |
| gravity-valid frames | 1229 |
| reset | 0 |
| NaN / Inf | 0 |
| max `||v_hat_B||` | 2.603 m/s |
| max `||eta_hat||` | 10.085 m/s² |
| camera update mean | 5.52 ms |
| camera update max | 13.09 ms |
| max adaptive Euler substeps | 20 |

### V1_01_easy Ground Truth 指标

| 指标 | 结果 |
|---|---:|
| matched frames | 1222 |
| body velocity RMSE | 0.500 m/s |
| gravity direction RMSE | 2.69° |
| maximum timestamp error | 0.238 μs |

这里使用正确的 EuRoC body-frame GT。早期约 `110.9°` 的 gravity error 来自 Vicon sensor/body frame 使用错误，不纳入结论。

### Baseline 与 Passive LTV 轨迹一致性

| 序列 | Baseline 与 Passive `vio.csv` | 结论 |
|---|---|---|
| V1_01_easy | 字节级一致 | Passive LTV 未改变 VINS 轨迹 |
| V2_02_medium | 字节级一致 | Passive LTV 未改变 VINS 轨迹 |
| V2_03_difficult | 字节级一致 | Passive LTV 未改变 VINS 轨迹 |

## Stage 2 — Gravity Factor

### V1_01_easy

| 模式 | ATE RMSE | Max position | Rotation RMSE | Max rotation | Roll RMSE | Pitch RMSE |
|---|---:|---:|---:|---:|---:|---:|
| Baseline | 0.133820 m | 0.256842 m | 3.2868° | 8.1414° | 2.5474° | 1.5707° |
| LTV Gravity, `sigma=10°` | 0.133904 m | 0.257003 m | 3.3044° | 7.9697° | 2.5542° | 1.5820° |

LTV Gravity 相对 Baseline：

| 指标 | 变化 |
|---|---:|
| ATE RMSE | +0.063% |
| Max position | +0.063% |
| Rotation RMSE | +0.536% |
| Max rotation | **-2.11%** |
| Roll RMSE | +0.268% |
| Pitch RMSE | +0.724% |

V1_01 的 ATE 和 rotation regression 均小于约 3% 的开发阶段上限。

### V2_02_medium

| 模式 | ATE RMSE | Max position | Rotation RMSE | Max rotation | Roll RMSE | Pitch RMSE |
|---|---:|---:|---:|---:|---:|---:|
| Baseline | 0.205770 m | 0.437832 m | 9.1862° | 25.3411° | 4.2516° | 5.0604° |
| LTV Gravity, `sigma=10°` | 0.207000 m | 0.434972 m | 8.9376° | 25.2797° | 4.2026° | 5.0658° |

LTV Gravity 相对 Baseline：

| 指标 | 变化 |
|---|---:|
| ATE RMSE | +0.60% |
| Max position | **-0.65%** |
| Rotation RMSE | **-2.71%** |
| Max rotation | **-0.24%** |
| Roll RMSE | **-1.15%** |
| Pitch RMSE | +0.11% |

### V2_03_difficult

| 模式 | ATE RMSE | Max position | Rotation RMSE | Max rotation | Roll RMSE | Pitch RMSE |
|---|---:|---:|---:|---:|---:|---:|
| Baseline | 0.377647 m | 0.736419 m | 9.8043° | 26.2327° | 3.0754° | 5.8105° |
| LTV Gravity, `sigma=10°` | 0.380349 m | 0.737272 m | 9.6950° | 26.1731° | 3.0787° | 5.8118° |

LTV Gravity 相对 Baseline：

| 指标 | 变化 |
|---|---:|
| ATE RMSE | +0.72% |
| Max position | +0.12% |
| Rotation RMSE | **-1.11%** |
| Max rotation | **-0.23%** |
| Roll RMSE | +0.11% |
| Pitch RMSE | +0.02% |

### 其他困难序列

MH_04 和 MH_05 的 Baseline/LTV Gravity 输出时间网格相差约 50 ms。下表使用共同 GT 时间插值后分别对齐，避免把输出相位差误判为算法效果。

| 序列 | Baseline ATE | LTV Gravity ATE | ATE 变化 | 备注 |
|---|---:|---:|---:|---|
| MH_04_difficult | 0.474221 m | 0.448445 m | **-5.44%** | Leica，仅位置指标 |
| MH_05_difficult | 0.372959 m | 0.381473 m | +2.28% | Leica，仅位置指标 |
| V1_03_difficult | 0.124146 m | 0.124438 m | +0.24% | Rotation RMSE +0.49% |

### V1_01 Gravity Sigma Sweep

| Gravity sigma | ATE RMSE | Max position | Rotation RMSE | Max rotation | Roll RMSE | Pitch RMSE |
|---:|---:|---:|---:|---:|---:|---:|
| 20° | 0.133797 m | 0.256797 m | 3.2903° | 8.0547° | 2.5476° | 1.5729° |
| 10° | 0.133904 m | 0.257003 m | 3.3044° | 7.9697° | 2.5542° | 1.5820° |
| 5° | 0.133697 m | 0.256559 m | 3.3227° | 7.0169° | 2.5587° | 1.6075° |
| 3° | 0.133507 m | 0.256310 m | 3.3625° | 6.7505° | 2.5623° | 1.6468° |

权重增强时，ATE 与最大姿态误差略有改善，但 rotation、roll 和 pitch RMSE 整体趋于恶化。

### Gravity Factor 数据覆盖

| 序列 | LTV rows | Gravity-valid | Factors added | Mean angle | Max angle | Max weighted residual |
|---|---:|---:|---:|---:|---:|---:|
| V1_01 | 1456 | 1229 | 1229 | 1.039° | 3.338° | 0.334 |
| MH_04 | 1015 | 900 | 900 | 1.724° | 6.256° | 0.625 |
| MH_05 | 1135 | 1069 | 1069 | 1.651° | 6.471° | 0.647 |
| V1_03 | 1073 | 222 | 222 | 2.196° | 7.023° | 0.702 |
| V2_02 | 1173 | 775 | 775 | 1.979° | 6.750° | 0.675 |
| V2_03 | 959 | 221 | 221 | 2.677° | 9.330° | 0.932 |

所有已加入 factor 的 snapshot timestamp error 均为 `0`，未发现 NaN/Inf。

V2_03 的 Baseline 和 LTV Gravity 均在相同位置记录了 83 个 `timestamp_gap`，所以不计为 Gravity factor 引入的额外 reset。早期 `2×` 回放数据未纳入正式统计。

## 指标结论

- Stage 1 Passive LTV：通过。
- Stage 2 Gravity Factor：初步通过。
- Gravity 对 V1_01 的退化远低于 3%，并在 V2_02、V2_03 上降低 rotation RMSE。
- MH_04 改善而 MH_05 恶化，说明 Gravity factor 的收益具有明显场景依赖性。
- Gravity 更适合作为条件性弱约束和 Quality Gate / RL 输入，不应假定固定开启即可稳定提升 VINS。


## Stage 3 — Velocity Factor

三个序列均使用对应的 EuRoC 官方 `state_groundtruth_estimate0/data.csv`，速度真值取 `v_RS_R_x/y/z`。LTV Velocity 运行中 Gravity factor 加入数均为 `0`。

### V1_01_easy

| 模式 | ATE RMSE | Max position | Rotation RMSE | Max rotation | Roll RMSE | Pitch RMSE |
|---|---:|---:|---:|---:|---:|---:|
| Baseline | 0.113173 m | 0.195780 m | 6.39810° | 11.77909° | 4.21403° | 1.87284° |
| LTV Velocity | 0.113174 m | 0.195790 m | 6.39808° | 11.77787° | 4.21404° | 1.87279° |

| 模式 | Velocity RMSE | Velocity P95 | Max velocity error |
|---|---:|---:|---:|
| Baseline | 0.032028 m/s | 0.055169 m/s | 0.088381 m/s |
| LTV Velocity | 0.032036 m/s | 0.055188 m/s | 0.088432 m/s |

LTV Velocity 相对 Baseline：

| ATE RMSE | Rotation RMSE | Velocity RMSE | Velocity P95 |
|---:|---:|---:|---:|
| +0.0010% | -0.0004% | +0.0255% | +0.0349% |

### V2_02_medium

| 模式 | ATE RMSE | Max position | Rotation RMSE | Max rotation | Roll RMSE | Pitch RMSE |
|---|---:|---:|---:|---:|---:|---:|
| Baseline | 0.120188 m | 0.327147 m | 2.69707° | 11.25885° | 2.37596° | 0.73283° |
| LTV Velocity | 0.120226 m | 0.327207 m | 2.69941° | 11.26612° | 2.37846° | 0.73310° |

| 模式 | Velocity RMSE | Velocity P95 | Max velocity error |
|---|---:|---:|---:|
| Baseline | 0.052676 m/s | 0.093611 m/s | 0.259546 m/s |
| LTV Velocity | 0.052714 m/s | 0.093681 m/s | 0.259710 m/s |

LTV Velocity 相对 Baseline：

| ATE RMSE | Rotation RMSE | Velocity RMSE | Velocity P95 |
|---:|---:|---:|---:|
| +0.0314% | +0.0866% | +0.0723% | +0.0744% |

### V2_03_difficult

| 模式 | ATE RMSE | Max position | Rotation RMSE | Max rotation | Roll RMSE | Pitch RMSE |
|---|---:|---:|---:|---:|---:|---:|
| Baseline | 0.330690 m | 0.586325 m | 3.09572° | 16.29764° | 2.96489° | 0.50827° |
| LTV Velocity | 0.331875 m | 0.586151 m | 3.09674° | 16.27119° | 2.96495° | 0.51343° |

| 模式 | Velocity RMSE | Velocity P95 | Max velocity error |
|---|---:|---:|---:|
| Baseline | 0.060931 m/s | 0.121667 m/s | 0.270255 m/s |
| LTV Velocity | 0.060645 m/s | 0.122099 m/s | 0.269952 m/s |

LTV Velocity 相对 Baseline：

| ATE RMSE | Rotation RMSE | Velocity RMSE | Velocity P95 |
|---:|---:|---:|---:|
| +0.3584% | +0.0330% | **-0.4703%** | +0.3552% |

### Velocity Factor 数据覆盖

| 序列 | Trajectory samples | Matched GT | LTV rows | Factors added | Coverage | Gravity factors | Reset | NaN / Inf |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| V1_01 | 1446 | 1436 | 1456 | 1232 | 84.62% | 0 | 0 | 0 |
| V2_02 | 1163 | 1155 | 1173 | 778 | 66.33% | 0 | 0 | 0 |
| V2_03 | 949 | 945 | 959 | 229 | 23.88% | 0 | 83 | 0 |

三序列 GT 最大时间戳误差均为 `0.238 μs`。V2_03 的 83 个 `timestamp_gap` 与 Baseline 相同，不计为 Velocity factor 引入的新增 reset。

### Stage 3 指标结论

- 工程稳定性通过：三个序列均完整生成轨迹，无 NaN/Inf、无新增异常 reset，Gravity factor 全程关闭。
- V1_01 不退化条件通过：ATE、Rotation RMSE 和 Velocity RMSE 变化均远小于 3%。
- 困难序列收益条件未通过：V2_03 Velocity RMSE 改善 0.4703%，V2_02 未改善，均未达到约 3% 的明确改善门槛。
- Velocity factor 保持默认关闭，不进入 Gravity+Velocity 联合验证。

## Stage 4 — Gravity Quality Gate

全量 EuRoC 11 个序列均关闭 loop closure、以 `1×` 回放，并用各序列官方
`mav0/state_groundtruth_estimate0/data.csv` 评估。此处的参照组 `Passive LTV`
为 Stage 1 已验证不向优化器加入 factor 的模式；不重复执行全量
`ltv_enable=0`。三组均关闭 Velocity factor：Passive LTV、固定 Gravity
（`sigma=10°`）和 Gate Gravity。

Gate 使用冻结到 snapshot 的判定：基础 Gravity eligibility、观测特征数、
`abs(||eta_hat|| - ||g||) <= 0.20 m/s²`、归一化 innovation `<= 0.05`，以及
reset cooldown。默认配置仍保持 Gate 关闭。

### 全量轨迹指标

表中变化为相对 Passive LTV；负值表示改善。`G_fixed` 是固定开启 Gravity，
`G_gate` 是启用 Quality Gate 后的 Gravity。

| 序列 | Passive ATE m | G_fixed ATE m | G_gate ATE m | G_fixed ATE 变化 | G_gate ATE 变化 | Passive Rotation RMSE (°) | G_fixed Rotation RMSE (°) | G_gate Rotation RMSE (°) | G_fixed Rotation 变化 | G_gate Rotation 变化 |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| MH_01_easy | 0.252745 | 0.253115 | 0.253083 | +0.15% | +0.13% | 2.10166 | 2.10310 | 2.10342 | +0.07% | +0.08% |
| MH_02_easy | 0.216940 | 0.216886 | 0.216951 | -0.03% | +0.01% | 1.89353 | 1.89319 | 1.89298 | -0.02% | -0.03% |
| MH_03_medium | 0.292403 | 0.292617 | 0.292482 | +0.07% | +0.03% | 1.43859 | 1.43733 | 1.43917 | -0.09% | +0.04% |
| MH_04_difficult | 0.445944 | 0.445953 | 0.418617 | +0.00% | **-6.13%** | 2.35336 | 2.35296 | 2.34451 | -0.02% | -0.38% |
| MH_05_difficult | 0.307750 | 0.307421 | 0.307741 | -0.11% | -0.00% | 1.88946 | 1.88494 | 1.88904 | -0.24% | -0.02% |
| V1_01_easy | 0.113173 | 0.113272 | 0.113198 | +0.09% | +0.02% | 6.39810 | 6.39984 | 6.39888 | +0.03% | +0.01% |
| V1_02_medium | 0.107725 | 0.107492 | 0.107740 | -0.22% | +0.01% | 2.99126 | 2.98330 | 2.99136 | -0.27% | +0.00% |
| V1_03_difficult | 0.114160 | 0.114562 | 0.114498 | +0.35% | +0.30% | 7.04597 | 7.07343 | 7.06741 | +0.39% | +0.30% |
| V2_01_easy | 0.100983 | 0.102650 | 0.101133 | +1.65% | +0.15% | 3.11298 | 3.24192 | 3.11713 | +4.14% | +0.13% |
| V2_02_medium | 0.120188 | 0.122054 | 0.122757 | +1.55% | +2.14% | 2.69707 | 2.60967 | 2.64686 | **-3.24%** | -1.86% |
| V2_03_difficult | 0.330690 | 0.320866 | 0.331615 | **-2.97%** | +0.28% | 3.09572 | 2.79854 | 3.07761 | **-9.60%** | -0.58% |

`MH_04_difficult` 的 Gate ATE 明显改善，但 pitch RMSE 从 `0.74122°` 到
`0.76793°`（`+3.60%`）。Gate 还抑制了 V2_01 的固定 Gravity 回归：Rotation
RMSE 从 `+4.14%` 降至 `+0.13%`，Roll RMSE 从 `+5.20%` 降至 `+0.26%`。

### Roll / Pitch RMSE 汇总

以下为相对姿态误差分解后的绝对 RMSE，单位为度（°）。

| 序列 | Passive Roll RMSE | G_fixed Roll RMSE | G_gate Roll RMSE | Passive Pitch RMSE | G_fixed Pitch RMSE | G_gate Pitch RMSE |
|---|---:|---:|---:|---:|---:|---:|
| MH_01_easy | 1.56361 | 1.56463 | 1.56532 | 0.70857 | 0.70811 | 0.70813 |
| MH_02_easy | 1.51760 | 1.51735 | 1.51698 | 0.86500 | 0.86494 | 0.86497 |
| MH_03_medium | 1.24709 | 1.24543 | 1.24768 | 0.50493 | 0.50464 | 0.50492 |
| MH_04_difficult | 2.16840 | 2.16793 | 2.15850 | 0.74122 | 0.74130 | 0.76793 |
| MH_05_difficult | 1.61343 | 1.61348 | 1.61287 | 0.86746 | 0.85906 | 0.86753 |
| V1_01_easy | 4.21403 | 4.21234 | 4.21130 | 1.87284 | 1.87791 | 1.87737 |
| V1_02_medium | 2.74433 | 2.73888 | 2.74447 | 0.63901 | 0.62893 | 0.63884 |
| V1_03_difficult | 6.31657 | 6.34182 | 6.33612 | 1.73399 | 1.74260 | 1.74124 |
| V2_01_easy | 2.90946 | 3.06065 | 2.91717 | 1.02055 | 0.97586 | 1.01094 |
| V2_02_medium | 2.37596 | 2.27581 | 2.31982 | 0.73283 | 0.73214 | 0.73165 |
| V2_03_difficult | 2.96489 | 2.65195 | 2.94477 | 0.50827 | 0.48483 | 0.51146 |

### Gate 覆盖与稳定性

| 序列 | LTV rows | Base eligible | Gate factors | Gate / base |
|---|---:|---:|---:|---:|
| MH_01_easy | 1841 | 1810 | 1626 | 89.83% |
| MH_02_easy | 1520 | 1435 | 1306 | 91.01% |
| MH_03_medium | 1349 | 1277 | 898 | 70.32% |
| MH_04_difficult | 1015 | 920 | 754 | 81.96% |
| MH_05_difficult | 1135 | 1082 | 887 | 81.98% |
| V1_01_easy | 1456 | 1229 | 1148 | 93.41% |
| V1_02_medium | 855 | 448 | 290 | 64.73% |
| V1_03_difficult | 1073 | 222 | 156 | 70.27% |
| V2_01_easy | 1139 | 1008 | 939 | 93.15% |
| V2_02_medium | 1173 | 775 | 501 | 64.65% |
| V2_03_difficult | 959 | 221 | 88 | 39.82% |

所有运行均完整生成轨迹和 LTV CSV，未发现 NaN/Inf 或 Gate 引入的异常 reset。
Gate 既没有在全部 frame 开启，也没有完全关闭；V2_03 的低覆盖主要来自其本身
基础 eligibility 稀疏及已有 timestamp gap。

### Stage 4 指标结论

- 工程与可观测性通过：Gate 判定在 snapshot 生成时冻结，日志含各条件和原因位，
  11 个序列稳定完成。
- 选择性通过：相对基础 eligible 的实际加入比例为 39.82%–93.41%，并可抑制
  V2_01 的固定 Gravity 姿态回归。
- 效果验收未通过：V2_03 中固定 Gravity 的 ATE、Rotation、Roll 与 Pitch 改善
  未被 Gate 保留；Gate 结果分别变为 `+0.28%`、`-0.58%`、`-0.68%`、`+0.63%`。
- 因此 Gate 目前仅作为默认关闭的诊断性条件弱约束；不据此进入 Stage 6 的
  Gravity 分支联合验证，需先针对困难序列重新校准或扩展 gate 特征。
- Gravity factor 本身有信息增益，Gate 也确实能做选择性抑制，但是阈值没有选好。



## Stage 5 — Velocity Oracle Gate

本次严格按 `VINS-Fusion-LTV_Stage5_Velocity_Oracle_Gate执行文档.md`
收紧范围，只比较 Passive reference、V_fixed、V_oracle_any_advantage 和
V_oracle_margin_002；未开启 Gravity，未修改 Velocity residual/Jacobian、LTV 方程或
marginalization，也未执行联合 factor、2×2 ablation、RL 或联合调参。

模式名称说明：`V_oracle_any_advantage` 表示 `A=e_VINS-e_LTV>0`（runner 内部 ID：
`v_oracle_0`）；`V_oracle_margin_002` 表示 `A>0.02 m/s`（内部 ID：
`v_oracle_002`）。下文表格使用语义化名称；mask CSV 的列名仍保持 `oracle_0`、
`oracle_002` 以兼容运行时配置。


### Frozen Oracle mask

mask 仅由 Passive reference + official GT 生成。Oracle replay 仅按 snapshot
timestamp 精确查表，不读 GT、不做 nearest/interpolation，也不用 Oracle run
自身状态重算 decision。

| 序列 | valid reference snapshot | GT match / miss | max matched GT error | any advantage / margin 0.02 | positive advantage median / P90 |
|---|---:|---:|---:|---:|---:|
| V1_01_easy | 2,735 | 2,872 / 29 | 4.999936 ms | 22 / 8 | 0.01693 / 0.02283 m/s |
| V1_03_difficult | 846 | 2,094 / 44 | 0.000256 ms | 18 / 14 | 0.03166 / 0.05346 m/s |
| V2_03_difficult | 909 | 1,890 / 20 | 0.000256 ms | 0 / 0 | N/A |
| MH_01_easy | 3,635 | 3,638 / 33 | 0.000256 ms | 465 / 0 | 0.01016 / 0.01519 m/s |
| MH_02_easy | 2,990 | 2,999 / 30 | 0.000256 ms | 221 / 0 | 0.00544 / 0.00837 m/s |
| MH_03_medium | 2,670 | 2,631 / 58 | 0.000256 ms | 187 / 26 | 0.00897 / 0.10651 m/s |
| MH_04_difficult | 1,960 | 1,976 / 45 | 0.000256 ms | 36 / 16 | 0.02274 / 0.06971 m/s |
| MH_05_difficult | 2,196 | 2,221 / 41 | 0.000256 ms | 39 / 1 | 0.00162 / 0.00380 m/s |

Oracle 运行的 mask miss 全部为 0；实际加入 factor 数与当前运行的
`base_eligible && oracle_pass` 数一致。MH 的 any-advantage / margin-0.02 factor 数分别为
MH_01 `465/0`、MH_02 `221/0`、MH_03 `187/26`、MH_04 `36/16`、
MH_05 `39/1`。MH_01/MH_02 的 margin-0.02 因无 pass 点而与 Passive 完全一致。

### Official GT 指标

| 序列 | 模式 | Velocity RMSE (m/s) | P95 (m/s) | max (m/s) | ATE RMSE (m) | Rotation RMSE (deg) |
|---|---|---:|---:|---:|---:|---:|
| V1_01 | Passive | 0.033174 | 0.058023 | 0.096377 | 0.126207 | 6.70357 |
| V1_01 | V_fixed | 0.033259 | 0.058769 | 0.088814 | 0.124125 | 6.77790 |
| V1_01 | V_oracle_any_advantage | 0.033174 | 0.058023 | 0.096377 | 0.126206 | 6.70356 |
| V1_01 | V_oracle_margin_002 | 0.033174 | 0.058023 | 0.096377 | 0.126207 | 6.70357 |
| V1_03 | Passive | 0.087199 | 0.174761 | 0.367287 | 0.160582 | 6.89629 |
| V1_03 | V_fixed | 0.087147 | 0.168852 | 0.371378 | 0.171710 | 7.16734 |
| V1_03 | V_oracle_any_advantage | 0.089223 | 0.176065 | 0.369695 | 0.166623 | 7.06044 |
| V1_03 | V_oracle_margin_002 | 0.089359 | 0.176880 | 0.373356 | 0.164695 | 7.06247 |
| V2_03 | Passive | 0.087891 | 0.180064 | 0.339329 | 0.334255 | 6.44686 |
| V2_03 | V_fixed | 0.085872 | 0.170275 | 0.335001 | 0.317372 | 6.18295 |
| V2_03 | V_oracle_any_advantage | 0.087891 | 0.180064 | 0.339329 | 0.334255 | 6.44686 |
| V2_03 | V_oracle_margin_002 | 0.087891 | 0.180064 | 0.339329 | 0.334255 | 6.44686 |
| MH_01 | Passive | 0.036554 | 0.067823 | 0.123197 | 0.269967 | 2.35540 |
| MH_01 | V_fixed | 0.037645 | 0.069453 | 0.115387 | 0.279788 | 2.87614 |
| MH_01 | V_oracle_any_advantage | 0.036545 | 0.067914 | 0.119021 | 0.253809 | 2.47427 |
| MH_01 | V_oracle_margin_002 | 0.036554 | 0.067823 | 0.123197 | 0.269967 | 2.35540 |
| MH_02 | Passive | 0.034854 | 0.065625 | 0.128128 | 0.194630 | 1.89043 |
| MH_02 | V_fixed | 0.035197 | 0.065550 | 0.124771 | 0.196772 | 1.88168 |
| MH_02 | V_oracle_any_advantage | 0.035263 | 0.066199 | 0.129753 | 0.192189 | 1.93602 |
| MH_02 | V_oracle_margin_002 | 0.034854 | 0.065625 | 0.128128 | 0.194630 | 1.89043 |
| MH_03 | Passive | 0.069108 | 0.138145 | 0.186780 | 0.385766 | 1.57583 |
| MH_03 | V_fixed | 0.067395 | 0.132809 | 0.190434 | 0.382094 | 1.58009 |
| MH_03 | V_oracle_any_advantage | 0.067516 | 0.135460 | 0.188733 | 0.396151 | 1.62483 |
| MH_03 | V_oracle_margin_002 | 0.069108 | 0.138145 | 0.186780 | 0.385766 | 1.57583 |
| MH_04 | Passive | 0.080783 | 0.149541 | 0.239676 | 0.545678 | 3.25887 |
| MH_04 | V_fixed | 0.081708 | 0.152660 | 0.237325 | 0.535529 | 3.08404 |
| MH_04 | V_oracle_any_advantage | 0.080324 | 0.149492 | 0.243298 | 0.532055 | 3.18920 |
| MH_04 | V_oracle_margin_002 | 0.080783 | 0.149541 | 0.239676 | 0.545678 | 3.25887 |
| MH_05 | Passive | 0.070919 | 0.128590 | 0.157676 | 0.377180 | 2.57725 |
| MH_05 | V_fixed | 0.071876 | 0.128838 | 0.167051 | 0.377217 | 2.64106 |
| MH_05 | V_oracle_any_advantage | 0.071125 | 0.127018 | 0.158209 | 0.375436 | 2.56368 |
| MH_05 | V_oracle_margin_002 | 0.071108 | 0.128614 | 0.158144 | 0.392357 | 2.55044 |

相对 Passive 的 Velocity RMSE 改善率（正数为改善）：

| 序列 | V_fixed | V_oracle_any_advantage | V_oracle_margin_002 |
|---|---:|---:|---:|
| V1_01 | -0.256% | +0.0008% | 0.000% |
| V1_03 | +0.060% | **-2.321%** | **-2.477%** |
| V2_03 | +2.297% | 0.000% | 0.000% |
| MH_01 | -2.985% | +0.022% | 0.000% |
| MH_02 | -0.986% | -1.175% | 0.000% |
| MH_03 | +2.479% | +2.303% | +0.0004% |
| MH_04 | -1.144% | +0.568% | +0.0001% |
| MH_05 | -1.350% | -0.292% | -0.267% |

### Stage 5 结论

八条已覆盖序列上的 frozen measurement-accuracy Oracle 没有显示一致、明显的
Velocity RMSE 增益：

- V1_03 的 any-advantage / margin-0.02 反而退化 2.321%/2.477%；V2_03 没有正 advantage 点；
- MH_03 的 any-advantage 改善 2.303%，但略低于 V_fixed 的 2.479%，同时 ATE 和
  Rotation 分别退化 2.692% 和 3.110%；
- MH_04 的 any-advantage 仅改善 0.568%；MH_01 几乎不变，MH_02/MH_05 退化；
- margin-0.02 在 MH_01/MH_02 无开启点，在 MH_03/MH_04/MH_05 仅加入
  26/16/1 个 factor，整体没有可重复的 Velocity 收益；
- 所有 Oracle 的 mask miss 为 0，32 次正式回放均完整消费 canonical input，因而该
  结果不能归因于输入缺失或 mask lookup 失败。

因此，对 Stage 5 当前**具体问题**的答案是否定的：在当前 LTV velocity、Velocity
factor、`sigma_v=1.0 m/s` 和“Passive 时刻瞬时 body-velocity 更接近 GT”的 frozen
gate 定义下，没有观察到约 3% 以上、跨困难序列一致且不损害其他主要指标的明显增益。

但这个结论有严格责任边界。当前 gate 是 measurement-accuracy hindsight gate，不是
最终优化收益的理论 upper bound；factor 对滑窗状态和后续线性化的耦合解释了为何
“局部 velocity 更准”仍可能使 ATE/Rotation 变差。因此本结果不支持沿**当前判据**继续
设计在线 Velocity Quality Gate，也不能据此否定所有 optimization-benefit 或逐 factor
反事实 Oracle。后者需要不同实验设计，且不属于本 Stage 5 的最小范围。
