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
