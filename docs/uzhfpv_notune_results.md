# UZH-FPV No-tune Results

本报告使用 EuRoC No-tune 的冻结 LTV 参数；UZH-FPV 标定、IMU 噪声、
相机–IMU `td`、mask 与前端频率保持数据集配置。所有正式指标均来自
五模式共同时间支持上的无尺度 SE(3) 位置对齐。

`Δ = (Method - B) / B × 100%`；负值表示改善。姿态指标因已知 GT 问题
排除，速度指标不可用。

## 冻结与复现信息

- 有效序列：15/16
- 参数源 SHA-256：`feb616db61d3529f4fa4cceca40c08c9bf12fdb114272e3f6a6ce467c0ae8e80`
- 时间审计 SHA-256：`b25bdad67cd04a038992a23336bd4298a3a6c346f736f3ab93ac1bcac8aeb4a4`
- Replay SHA-256：`2c27d667b7384d742d6aaafec1e658a353004483ed89bb703c588ed90d8c7f1b`

## 结论

在 15/16 条严格有效序列的 Global Macro 上，所有冻结 LTV 模式的 ATE RMSE 都差于 Baseline：G_gate `+19.81%`、V_fixed `+40.72%`、G_gate+V_fixed `+79.52%`、G_gate+V_gate `+3.10%`。因此本次 No-tune 测试不支持 LTV 改善 UZH-FPV 总体位置 ATE 的结论。

G_gate+V_gate 的 P95 与最大位置误差 Global Macro 分别为 `-3.39%` 和 `-1.49%`，但其 ATE 仍退化，不能据此判为总体改善。

`outdoor_forward_1` 因 gravity-only 可重复的末帧 drain 失败而整条 N/A，未混入聚合结果。

## GT 时间戳审计

偏移定义：`raw` 最大化 `ω_GT(t)` 与 `ω_IMU(t+raw)` 的相关性；
`residual = raw - td`，正式评估使用 `t_GT + applied`。

| Sequence | GT range (s) | Bag range (s) | Raw (ms) | td (ms) | Residual (ms) | Applied (ms) | Corr | GT/VIO overlap (s) |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| indoor_forward_3_snapdragon_with_gt | 4908.792–4958.290 | 4878.790–4970.894 | -14.4 | -16.7 | +2.3 | +0.0 | 0.9899 | 49.462 |
| indoor_forward_5_snapdragon_with_gt | 1540821125.816–1540821145.114 | 1540821095.598–1540821245.680 | -11.1 | -16.7 | +5.6 | +0.0 | 0.9834 | 19.307 |
| indoor_forward_6_snapdragon_with_gt | 1540821387.483–1540821417.681 | 1540821358.196–1540821427.824 | -16.2 | -16.7 | +0.5 | +0.0 | 0.9856 | 30.188 |
| indoor_forward_7_snapdragon_with_gt | 1540821844.731–1540821911.429 | 1540821814.386–1540821932.470 | -15.9 | -16.7 | +0.8 | +0.0 | 0.9946 | 66.713 |
| indoor_forward_9_snapdragon_with_gt | 1540822844.483–1540822873.281 | 1540822816.292–1540822893.126 | -11.4 | -16.7 | +5.3 | +0.0 | 0.9848 | 28.795 |
| indoor_forward_10_snapdragon_with_gt | 1540823082.195–1540823112.093 | 1540823051.509–1540823127.794 | -7.9 | -16.7 | +8.8 | +0.0 | 0.9826 | 29.856 |
| indoor_45_2_snapdragon_with_gt | 1416.606–1466.004 | 1398.502–1473.178 | -19.6 | -14.8 | -4.8 | +0.0 | 0.9851 | 49.396 |
| indoor_45_4_snapdragon_with_gt | 1808.489–1851.687 | 1790.156–1856.382 | -7.9 | -14.8 | +6.9 | +0.0 | 0.9815 | 43.192 |
| indoor_45_9_snapdragon_with_gt | 2023.467–2044.865 | 2005.394–2077.176 | -12.4 | -14.8 | +2.4 | +0.0 | 0.9876 | 21.398 |
| indoor_45_12_snapdragon_with_gt | 622.687–662.985 | 601.476–674.006 | -10.0 | -14.8 | +4.8 | +0.0 | 0.9698 | 40.305 |
| indoor_45_13_snapdragon_with_gt | 819.407–856.805 | 800.324–862.260 | -18.1 | -14.8 | -3.3 | +0.0 | 0.9815 | 37.387 |
| indoor_45_14_snapdragon_with_gt | 948.563–985.561 | 929.546–994.082 | -15.4 | -14.8 | -0.6 | +0.0 | 0.9877 | 36.989 |
| outdoor_forward_1_snapdragon_with_gt | 552.460–595.058 | 523.077–608.854 | +12.3 | -8.0 | +20.3 | +0.0 | 0.9643 | 42.594 |
| outdoor_forward_3_snapdragon_with_gt | 1061.095–1146.793 | 1028.813–1160.081 | -3.2 | -8.0 | +4.8 | +0.0 | 0.9724 | 85.686 |
| outdoor_forward_5_snapdragon_with_gt | 573.454–591.052 | 526.006–617.176 | -1.3 | -8.0 | +6.7 | +0.0 | 0.9826 | 17.615 |
| outdoor_45_1_snapdragon_with_gt | 310.644–330.642 | 287.124–369.786 | -4.6 | -8.6 | +4.0 | +0.0 | 0.9096 | 20.003 |

### 家族决策

- `indoor_45`: `direct`, residual median +0.95 ms, MAD 4.05 ms, applied +0.00 ms.
- `indoor_forward`: `direct`, residual median +3.78 ms, MAD 2.40 ms, applied +0.00 ms.
- `outdoor_45`: `direct`, residual median +4.04 ms, MAD 0.00 ms, applied +0.00 ms.
- `outdoor_forward`: `direct`, residual median +6.70 ms, MAD 1.90 ms, applied +0.00 ms.

### 原始 Leica 审计

Leica 仅作为来源审计证据，不直接转换为 IMU 位姿，也不用于 ATE。

| Sequence | Samples | UTC range (s) | Finite position | SHA-256 |
|---|---:|---:|---|---|
| indoor_forward_3_snapdragon_with_gt | 1662 | 1540823831.711–1540823924.488 | yes | `e754a41ca0f89736d0e8c06bf9110c6360363f3293f8c1bb9b1677964051b5e3` |
| indoor_forward_5_snapdragon_with_gt | 3124 | 1540824687.402–1540824852.769 | yes | `1ba3080c475073cc9a1e498a48d9a553d7c084dc9b44b4f659db3867d1cb1d16` |
| indoor_forward_6_snapdragon_with_gt | 1645 | 1540824948.552–1540825035.125 | yes | `73b002ee28b99349d2ea35b7fa806e79c63387a1c10e272706159db379d7c139` |
| indoor_forward_7_snapdragon_with_gt | 2319 | 1540825405.546–1540825528.934 | yes | `5de7d7ee4316f9bbf920d360dc2f949bb3cb180349582825ad5050cb7a98d71d` |
| indoor_forward_9_snapdragon_with_gt | 1672 | 1540826400.938–1540826488.741 | yes | `0c7147eccdd3ad48383290a5593708732afc270136c2a5cb60d2692d6fa1b3ed` |
| indoor_forward_10_snapdragon_with_gt | 2167 | 1540826646.375–1540826735.791 | yes | `26f480fd337eb6982826cae8bdc95d20df8d8e8b99d611df27bddc4cbdfbd7e3` |
| indoor_45_2_snapdragon_with_gt | 1862 | 1545309288.999–1545309388.168 | yes | `8b432bfceb00816b37032ee6b39e43e6f9d8f2e1b743e3746689db189aa345c6` |
| indoor_45_4_snapdragon_with_gt | 1591 | 1545309686.160–1545309772.382 | yes | `043e6615307b3f3eb83035894193f6dc837f611114957a6fb1c3cbf3fcd5cf34` |
| indoor_45_9_snapdragon_with_gt | 1368 | 1545311841.193–1545311918.491 | yes | `3971c026ae5e348241c7584d8ab578d49a1f9e395b48c26c44743ffda7acc970` |
| indoor_45_12_snapdragon_with_gt | 1591 | 1545317073.809–1545317158.361 | yes | `7bea4fbdd4f68845f1c085cffa8dcb97678d7d2e66766f101a1c4e122601e87b` |
| indoor_45_13_snapdragon_with_gt | 1368 | 1545317274.019–1545317345.873 | yes | `c5b7ad2fc1db6cd83753e5ecf9e80b9a729aa3c60cd5a54dd3c5a8ce62e5dc81` |
| indoor_45_14_snapdragon_with_gt | 1331 | 1545317399.225–1545317475.572 | yes | `e519e7b1475e205e8ce13271e9a10402effe7b25065afb0a853ab454e1fb6228` |
| outdoor_forward_1_snapdragon_with_gt | 6654 | 1540120344.972–1540120540.877 | yes | `a285f2a066a7608603e4ac15f950d9040b9b1cc88982bf55f9dd427e57c24a2c` |
| outdoor_forward_3_snapdragon_with_gt | 3062 | 1540120868.806–1540121014.525 | yes | `4db3fe6ecd8ced8d13e67290ebda7cf43c7d5682d6d6fa8e56c7e0b866a213df` |
| outdoor_forward_5_snapdragon_with_gt | 2513 | 1540122671.669–1540122772.555 | yes | `467b81a28231de2f720ead03d67a2ababaf49103cdc5e0640e2e3abcf9f18f8d` |
| outdoor_45_1_snapdragon_with_gt | 1217 | 1553767979.080–1553768038.539 | yes | `4eba733b88a786d6313dae4ca6d77e2baab9255d31a8db80b0c2e383aad4e373` |

## 离线运行耗时

`wall / data` 由每次运行的 `input_manifest.json` 与 `validation.json` 文件时间计算；它衡量本次离线 replay 的吞吐，不参与 ATE。

| Sequence | Data (s) | B wall (s) | B wall/data | G_gate wall (s) | V_fixed wall (s) | Joint wall (s) | Joint V-gate wall (s) |
|---|---:|---:|---:|---:|---:|---:|---:|
| indoor_forward_3_snapdragon_with_gt | 91.956 | 1676.0 | 18.23× | 1698.7 | 1772.3 | 1636.1 | 1752.3 |
| indoor_forward_5_snapdragon_with_gt | 149.943 | 2734.2 | 18.23× | 2912.4 | 2926.1 | 2679.4 | 2387.6 |
| indoor_forward_6_snapdragon_with_gt | 69.498 | 1257.8 | 18.10× | 1251.2 | 1270.5 | 1296.4 | 1242.5 |
| indoor_forward_7_snapdragon_with_gt | 117.965 | 1992.4 | 16.89× | 2028.9 | 1957.5 | 1991.7 | 1925.9 |
| indoor_forward_9_snapdragon_with_gt | 76.697 | 1362.8 | 17.77× | 1297.4 | 1320.4 | 1319.0 | 1299.2 |
| indoor_forward_10_snapdragon_with_gt | 76.166 | 1488.3 | 19.54× | 1435.6 | 1455.0 | 1405.7 | 1481.6 |
| indoor_45_2_snapdragon_with_gt | 74.542 | 906.2 | 12.16× | 942.0 | 934.1 | 924.1 | 963.0 |
| indoor_45_4_snapdragon_with_gt | 66.115 | 793.7 | 12.00× | 839.1 | 850.5 | 865.9 | 927.7 |
| indoor_45_9_snapdragon_with_gt | 71.622 | 868.5 | 12.13× | 872.4 | 1073.8 | 902.6 | 956.9 |
| indoor_45_12_snapdragon_with_gt | 72.386 | 878.2 | 12.13× | 855.9 | 851.3 | 869.3 | 892.5 |
| indoor_45_13_snapdragon_with_gt | 61.803 | 744.2 | 12.04× | 712.8 | 785.0 | 808.8 | 716.0 |
| indoor_45_14_snapdragon_with_gt | 64.390 | 741.8 | 11.52× | 735.5 | 805.1 | 724.9 | 708.5 |
| outdoor_forward_3_snapdragon_with_gt | 131.133 | 372.8 | 2.84× | 368.3 | 374.1 | 365.8 | 330.8 |
| outdoor_forward_5_snapdragon_with_gt | 91.027 | 290.5 | 3.19× | 280.1 | 283.7 | 280.2 | 264.7 |
| outdoor_45_1_snapdragon_with_gt | 82.534 | 957.9 | 11.61× | 1031.6 | 938.1 | 934.5 | 934.0 |

特别地，`indoor_forward_5` 的数据有效时长约 150 秒（原始 bag 约 156 秒），但单次运行约需 40–50 分钟。这是已确认的 CPU 满载离线性能异常，而不是 bag 时长或进程挂起；本报告保留冻结参数，未用降采样或调参掩盖该现象。

## ATE RMSE (m)

| Sequence | B | G_gate | Δ vs B | V_fixed | Δ vs B | G_gate+V_fixed | Δ vs B | G_gate+V_gate | Δ vs B |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| indoor_forward_3_snapdragon_with_gt | 5.835645 | 51.717279 | +786.23% | 5.864911 | +0.50% | 99.942727 | +1612.63% | 5.648894 | -3.20% |
| indoor_forward_5_snapdragon_with_gt | 1.438678 | 1.407330 | -2.18% | 1.446672 | +0.56% | 1.367377 | -4.96% | 1.337710 | -7.02% |
| indoor_forward_6_snapdragon_with_gt | 2.366664 | 2.250376 | -4.91% | 2.318432 | -2.04% | 2.114151 | -10.67% | 2.218529 | -6.26% |
| indoor_forward_7_snapdragon_with_gt | 18.931642 | 16.654432 | -12.03% | 15.782119 | -16.64% | 16.678813 | -11.90% | 19.309730 | +2.00% |
| indoor_forward_9_snapdragon_with_gt | 10.430962 | 10.800769 | +3.55% | 12.111354 | +16.11% | 12.607693 | +20.87% | 12.660124 | +21.37% |
| indoor_forward_10_snapdragon_with_gt | 1.544480 | 3.170895 | +105.31% | 1.250708 | -19.02% | 1.860741 | +20.48% | 1.231914 | -20.24% |
| indoor_45_2_snapdragon_with_gt | 35.062731 | 35.210496 | +0.42% | 36.144079 | +3.08% | 34.831175 | -0.66% | 35.506340 | +1.27% |
| indoor_45_4_snapdragon_with_gt | 17.396681 | 15.518385 | -10.80% | 17.917375 | +2.99% | 16.812857 | -3.36% | 17.459819 | +0.36% |
| indoor_45_9_snapdragon_with_gt | 9.812277 | 9.476097 | -3.43% | 8.028281 | -18.18% | 8.313110 | -15.28% | 8.920630 | -9.09% |
| indoor_45_12_snapdragon_with_gt | 55.785932 | 55.016626 | -1.38% | 60.110885 | +7.75% | 60.677417 | +8.77% | 54.046242 | -3.12% |
| indoor_45_13_snapdragon_with_gt | 2.625759 | 2.822365 | +7.49% | 2.666128 | +1.54% | 3.013471 | +14.77% | 2.721358 | +3.64% |
| indoor_45_14_snapdragon_with_gt | 8.985223 | 8.821584 | -1.82% | 8.827341 | -1.76% | 7.682079 | -14.50% | 9.207256 | +2.47% |
| outdoor_forward_1_snapdragon_with_gt | N/A | N/A | N/A | N/A | N/A | N/A | N/A | N/A | N/A |
| outdoor_forward_3_snapdragon_with_gt | 288.303680 | 305.906797 | +6.11% | 468.275377 | +62.42% | 538.569719 | +86.81% | 299.173126 | +3.77% |
| outdoor_forward_5_snapdragon_with_gt | 14.912510 | 51.110849 | +242.74% | 31.300451 | +109.89% | 56.259267 | +277.26% | 18.457475 | +23.77% |
| outdoor_45_1_snapdragon_with_gt | 14.336036 | 14.530906 | +1.36% | 14.337865 | +0.01% | 14.923391 | +4.10% | 14.966319 | +4.40% |
| indoor_forward Macro (6/6) | 6.758012 | 14.333514 | +112.10% | 6.462366 | -4.37% | 22.428584 | +231.88% | 7.067817 | +4.58% |
| indoor_45 Macro (6/6) | 21.611434 | 21.144259 | -2.16% | 22.282348 | +3.10% | 21.888352 | +1.28% | 21.310274 | -1.39% |
| outdoor_forward Macro (2/3) | 151.608095 | 178.508823 | +17.74% | 249.787914 | +64.76% | 297.414493 | +96.17% | 158.815301 | +4.75% |
| outdoor_45 Macro (1/1) | 14.336036 | 14.530906 | +1.36% | 14.337865 | +0.01% | 14.923391 | +4.10% | 14.966319 | +4.40% |
| Global Macro (15/16) | 32.517927 | 38.961012 | +19.81% | 45.758798 | +40.72% | 58.376933 | +79.52% | 33.524364 | +3.10% |
| Global Micro (15/16) | 111.729102 | 119.511714 | +6.97% | 179.738234 | +60.87% | 208.588802 | +86.69% | 115.790693 | +3.64% |

## Position P95 Error (m)

| Sequence | B | G_gate | Δ vs B | V_fixed | Δ vs B | G_gate+V_fixed | Δ vs B | G_gate+V_gate | Δ vs B |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| indoor_forward_3_snapdragon_with_gt | 10.002322 | 103.678595 | +936.55% | 9.981602 | -0.21% | 195.827924 | +1857.82% | 9.565319 | -4.37% |
| indoor_forward_5_snapdragon_with_gt | 2.184873 | 2.053622 | -6.01% | 2.125829 | -2.70% | 2.023576 | -7.38% | 2.000899 | -8.42% |
| indoor_forward_6_snapdragon_with_gt | 4.481351 | 4.142034 | -7.57% | 4.046495 | -9.70% | 3.745245 | -16.43% | 4.123854 | -7.98% |
| indoor_forward_7_snapdragon_with_gt | 27.247334 | 22.530018 | -17.31% | 21.762971 | -20.13% | 22.336717 | -18.02% | 27.341024 | +0.34% |
| indoor_forward_9_snapdragon_with_gt | 21.295018 | 22.018910 | +3.40% | 24.760063 | +16.27% | 25.787195 | +21.09% | 25.901778 | +21.63% |
| indoor_forward_10_snapdragon_with_gt | 2.240236 | 4.280749 | +91.08% | 1.772703 | -20.87% | 2.631563 | +17.47% | 1.966328 | -12.23% |
| indoor_45_2_snapdragon_with_gt | 71.314758 | 71.923303 | +0.85% | 73.485624 | +3.04% | 70.901973 | -0.58% | 72.413109 | +1.54% |
| indoor_45_4_snapdragon_with_gt | 34.403878 | 31.025924 | -9.82% | 35.311407 | +2.64% | 33.169523 | -3.59% | 34.635053 | +0.67% |
| indoor_45_9_snapdragon_with_gt | 15.018868 | 14.839659 | -1.19% | 11.954426 | -20.40% | 12.284078 | -18.21% | 14.069860 | -6.32% |
| indoor_45_12_snapdragon_with_gt | 99.975163 | 98.761952 | -1.21% | 108.679137 | +8.71% | 108.576591 | +8.60% | 96.970149 | -3.01% |
| indoor_45_13_snapdragon_with_gt | 4.460961 | 5.458896 | +22.37% | 5.255857 | +17.82% | 6.166461 | +38.23% | 4.627902 | +3.74% |
| indoor_45_14_snapdragon_with_gt | 15.728646 | 15.441328 | -1.83% | 15.136203 | -3.77% | 13.246704 | -15.78% | 15.841351 | +0.72% |
| outdoor_forward_1_snapdragon_with_gt | N/A | N/A | N/A | N/A | N/A | N/A | N/A | N/A | N/A |
| outdoor_forward_3_snapdragon_with_gt | 528.252805 | 549.688784 | +4.06% | 850.585968 | +61.02% | 983.455722 | +86.17% | 476.152276 | -9.86% |
| outdoor_forward_5_snapdragon_with_gt | 22.299208 | 85.483177 | +283.35% | 74.018931 | +231.94% | 90.387734 | +305.34% | 41.738186 | +87.17% |
| outdoor_45_1_snapdragon_with_gt | 28.901852 | 29.330270 | +1.48% | 28.894912 | -0.02% | 29.799409 | +3.11% | 30.350306 | +5.01% |
| indoor_forward Macro (6/6) | 11.241856 | 26.450655 | +135.29% | 10.741611 | -4.45% | 42.058704 | +274.13% | 11.816534 | +5.11% |
| indoor_45 Macro (6/6) | 40.150379 | 39.575177 | -1.43% | 41.637109 | +3.70% | 40.724222 | +1.43% | 39.759571 | -0.97% |
| outdoor_forward Macro (2/3) | 275.276006 | 317.585981 | +15.37% | 462.302449 | +67.94% | 536.921728 | +95.05% | 258.945231 | -5.93% |
| outdoor_45 Macro (1/1) | 28.901852 | 29.330270 | +1.48% | 28.894912 | -0.02% | 29.799409 | +3.11% | 30.350306 | +5.01% |
| Global Macro (15/16) | 59.187152 | 70.710481 | +19.47% | 84.518142 | +42.80% | 106.689361 | +80.26% | 57.179826 | -3.39% |
| Global Micro (15/16) | 310.332957 | 333.765958 | +7.55% | 538.763360 | +73.61% | 622.010939 | +100.43% | 352.223713 | +13.50% |

## Maximum Position Error (m)

| Sequence | B | G_gate | Δ vs B | V_fixed | Δ vs B | G_gate+V_fixed | Δ vs B | G_gate+V_gate | Δ vs B |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| indoor_forward_3_snapdragon_with_gt | 10.104149 | 122.039945 | +1107.82% | 10.089313 | -0.15% | 229.512445 | +2171.47% | 9.692269 | -4.08% |
| indoor_forward_5_snapdragon_with_gt | 2.279532 | 2.123095 | -6.86% | 2.206806 | -3.19% | 2.094778 | -8.10% | 2.078713 | -8.81% |
| indoor_forward_6_snapdragon_with_gt | 5.376515 | 4.922288 | -8.45% | 4.766850 | -11.34% | 4.407815 | -18.02% | 4.963567 | -7.68% |
| indoor_forward_7_snapdragon_with_gt | 28.851452 | 25.752737 | -10.74% | 25.142813 | -12.85% | 24.724353 | -14.30% | 28.729359 | -0.42% |
| indoor_forward_9_snapdragon_with_gt | 24.979933 | 25.813990 | +3.34% | 28.954869 | +15.91% | 30.173729 | +20.79% | 30.320868 | +21.38% |
| indoor_forward_10_snapdragon_with_gt | 2.310833 | 4.339564 | +87.79% | 1.846340 | -20.10% | 2.734006 | +18.31% | 2.108205 | -8.77% |
| indoor_45_2_snapdragon_with_gt | 89.331197 | 90.297350 | +1.08% | 92.029881 | +3.02% | 88.918401 | -0.46% | 90.824281 | +1.67% |
| indoor_45_4_snapdragon_with_gt | 40.342791 | 36.144307 | -10.41% | 41.563420 | +3.03% | 39.033987 | -3.24% | 40.577974 | +0.58% |
| indoor_45_9_snapdragon_with_gt | 16.343151 | 16.453632 | +0.68% | 12.132667 | -25.76% | 12.413483 | -24.04% | 15.578711 | -4.68% |
| indoor_45_12_snapdragon_with_gt | 120.361215 | 118.820385 | -1.28% | 130.945365 | +8.79% | 130.291934 | +8.25% | 116.733150 | -3.01% |
| indoor_45_13_snapdragon_with_gt | 6.202501 | 7.580546 | +22.22% | 7.205441 | +16.17% | 8.245343 | +32.94% | 6.293859 | +1.47% |
| indoor_45_14_snapdragon_with_gt | 18.040770 | 17.782995 | -1.43% | 17.110733 | -5.16% | 15.261780 | -15.40% | 17.961754 | -0.44% |
| outdoor_forward_1_snapdragon_with_gt | N/A | N/A | N/A | N/A | N/A | N/A | N/A | N/A | N/A |
| outdoor_forward_3_snapdragon_with_gt | 585.737586 | 628.234798 | +7.26% | 946.181226 | +61.54% | 1084.129700 | +85.09% | 540.570272 | -7.71% |
| outdoor_forward_5_snapdragon_with_gt | 30.583100 | 91.651379 | +199.68% | 109.014148 | +256.45% | 105.498340 | +244.96% | 57.313705 | +87.40% |
| outdoor_45_1_snapdragon_with_gt | 34.860476 | 35.434760 | +1.65% | 34.887977 | +0.08% | 35.738359 | +2.52% | 36.873096 | +5.77% |
| indoor_forward Macro (6/6) | 12.317069 | 30.831936 | +150.32% | 12.167832 | -1.21% | 48.941188 | +297.34% | 12.982163 | +5.40% |
| indoor_45 Macro (6/6) | 48.436938 | 47.846536 | -1.22% | 50.164584 | +3.57% | 49.027488 | +1.22% | 47.994955 | -0.91% |
| outdoor_forward Macro (2/3) | 308.160343 | 359.943088 | +16.80% | 527.597687 | +71.21% | 594.814020 | +93.02% | 298.941989 | -2.99% |
| outdoor_45 Macro (1/1) | 34.860476 | 35.434760 | +1.65% | 34.887977 | +0.08% | 35.738359 | +2.52% | 36.873096 | +5.77% |
| Global Macro (15/16) | 67.713680 | 81.826118 | +20.84% | 97.605190 | +44.14% | 120.878564 | +78.51% | 66.707985 | -1.49% |
| Global Micro (15/16) | 585.737586 | 628.234798 | +7.26% | 946.181226 | +61.54% | 1084.129700 | +85.09% | 540.570272 | -7.71% |

## 运行状态

| Sequence | Status | Detail |
|---|---|---|
| indoor_forward_3_snapdragon_with_gt | complete | 1333 common samples |
| indoor_forward_5_snapdragon_with_gt | complete | 530 common samples |
| indoor_forward_6_snapdragon_with_gt | complete | 817 common samples |
| indoor_forward_7_snapdragon_with_gt | complete | 1736 common samples |
| indoor_forward_9_snapdragon_with_gt | complete | 767 common samples |
| indoor_forward_10_snapdragon_with_gt | complete | 820 common samples |
| indoor_45_2_snapdragon_with_gt | complete | 1265 common samples |
| indoor_45_4_snapdragon_with_gt | complete | 1154 common samples |
| indoor_45_9_snapdragon_with_gt | complete | 579 common samples |
| indoor_45_12_snapdragon_with_gt | complete | 1077 common samples |
| indoor_45_13_snapdragon_with_gt | complete | 1037 common samples |
| indoor_45_14_snapdragon_with_gt | complete | 1026 common samples |
| outdoor_forward_1_snapdragon_with_gt | N/A | outdoor_forward_1_snapdragon_with_gt/gravity_only: missing files ['vio.csv', 'ltv_debug.csv', 'replay.log', 'replay_summary.json', 'validation.json', 'input_manifest.json', 'effective_config.yaml']; preserved failure: gravity_only.failed-20260921T100936Z-7bd409c2301049498dcd29eaebabfe0f (stage5_replay: estimator did not drain all queued features); gravity_only.failed-20260921T113830Z-9b06ccd5240146c6a3f8fd95d7182110 (stage5_replay: estimator did not drain all queued features) |
| outdoor_forward_3_snapdragon_with_gt | complete | 2224 common samples |
| outdoor_forward_5_snapdragon_with_gt | complete | 446 common samples |
| outdoor_45_1_snapdragon_with_gt | complete | 507 common samples |

## 指标边界

- `orientation_metrics_status: excluded_known_ground_truth_issue`
- `velocity_metrics_status: unavailable`
