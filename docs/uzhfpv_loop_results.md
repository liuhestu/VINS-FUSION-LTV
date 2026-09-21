# UZH-FPV Indoor-Forward Loop-on Results

本报告运行完整 VINS-Fusion + Loop Fusion，bag 以 1× 回放；五组使用完全相同的 Loop 参数，
LTV 参数冻结自 `docs/euroc_notune_results.md`。正式指标仅为位置 ATE。

`Δ = (Method - B_loop) / B_loop × 100%`；负值表示改善。

## 结论

Indoor-forward Global Macro ATE RMSE：B_loop `296.206211 m` (+0.00%), G_gate_loop `234.101768 m` (-20.97%), V_fixed_loop `249.068019 m` (-15.91%), G_gate+V_fixed_loop `238.669557 m` (-19.42%), G_gate+V_gate_loop `229.228594 m` (-22.61%)

有效序列：6/6。

## ATE RMSE (m)

| Sequence | B_loop | G_gate_loop | V_fixed_loop | G_gate+V_fixed_loop | G_gate+V_gate_loop |
|---|---:|---:|---:|---:|---:|
| indoor_forward_3_snapdragon_with_gt | 97.187972 | 34.895271 | 110.397982 | 175.004059 | 26.769961 |
| indoor_forward_5_snapdragon_with_gt | 19.926286 | 22.933546 | 35.221350 | 28.686463 | 4.608879 |
| indoor_forward_6_snapdragon_with_gt | 66.149749 | 145.526485 | 70.080400 | 151.580338 | 137.076285 |
| indoor_forward_7_snapdragon_with_gt | 1317.550415 | 863.560444 | 957.624987 | 693.255634 | 817.256591 |
| indoor_forward_9_snapdragon_with_gt | 5.133451 | 51.982325 | 8.977697 | 6.143109 | 50.359821 |
| indoor_forward_10_snapdragon_with_gt | 271.289392 | 285.712537 | 312.105698 | 377.347741 | 339.300028 |

## Position P95 Error (m)

| Sequence | B_loop | G_gate_loop | V_fixed_loop | G_gate+V_fixed_loop | G_gate+V_gate_loop |
|---|---:|---:|---:|---:|---:|
| indoor_forward_3_snapdragon_with_gt | 166.950761 | 60.151211 | 189.865133 | 339.104828 | 44.456393 |
| indoor_forward_5_snapdragon_with_gt | 35.935852 | 41.762107 | 59.513510 | 52.513288 | 8.370991 |
| indoor_forward_6_snapdragon_with_gt | 125.524915 | 214.935041 | 121.873352 | 294.939207 | 192.152492 |
| indoor_forward_7_snapdragon_with_gt | 2311.495370 | 1528.191951 | 1723.466063 | 1222.584462 | 1468.568267 |
| indoor_forward_9_snapdragon_with_gt | 9.685063 | 93.921855 | 16.412588 | 10.707834 | 86.521579 |
| indoor_forward_10_snapdragon_with_gt | 432.101259 | 412.607241 | 517.290226 | 634.356334 | 548.723233 |

## Maximum Position Error (m)

| Sequence | B_loop | G_gate_loop | V_fixed_loop | G_gate+V_fixed_loop | G_gate+V_gate_loop |
|---|---:|---:|---:|---:|---:|
| indoor_forward_3_snapdragon_with_gt | 201.293806 | 77.848974 | 235.010043 | 387.581389 | 51.664260 |
| indoor_forward_5_snapdragon_with_gt | 41.282963 | 51.026546 | 72.423430 | 65.942424 | 10.664606 |
| indoor_forward_6_snapdragon_with_gt | 138.851547 | 225.798538 | 131.235382 | 314.309229 | 244.762914 |
| indoor_forward_7_snapdragon_with_gt | 2829.925783 | 1896.098112 | 2058.541695 | 1505.932766 | 1766.422902 |
| indoor_forward_9_snapdragon_with_gt | 10.508786 | 106.601132 | 19.326342 | 11.498711 | 91.175005 |
| indoor_forward_10_snapdragon_with_gt | 528.495523 | 422.508852 | 646.913368 | 672.209222 | 693.182332 |

## 运行与时间完整性

| Sequence | Status | Common samples | Max estimator–GT error (ms) | Shutdown artifacts |
|---|---|---:|---:|---|
| indoor_forward_3_snapdragon_with_gt | complete | 32 | 0.999 | known_post_save_crash |
| indoor_forward_5_snapdragon_with_gt | complete | 7 | 0.910 | known_post_save_crash |
| indoor_forward_6_snapdragon_with_gt | complete | 16 | 0.992 | known_post_save_crash |
| indoor_forward_7_snapdragon_with_gt | complete | 76 | 0.996 | known_post_save_crash |
| indoor_forward_9_snapdragon_with_gt | complete | 18 | 0.993 | known_post_save_crash |
| indoor_forward_10_snapdragon_with_gt | complete | 15 | 0.997 | known_post_save_crash |

## 复现信息

- 参数源 SHA-256：`feb616db61d3529f4fa4cceca40c08c9bf12fdb114272e3f6a6ce467c0ae8e80`
- Loop 输出：每次运行的 `vio_loop.csv`（关键帧最终优化轨迹）
- 姿态指标：`excluded_known_ground_truth_issue`
- 速度指标：`unavailable`
