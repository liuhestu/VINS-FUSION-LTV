# Stage7 EuRoC 调参结果

EuRoC 全 11 序列是调参集。ATE 使用各模式独立无 scale 的 SE(3) 对齐；Rotation/Velocity 使用共同 B 对齐旋转。速度 GT 来自官方 CSV velocity 列。

唯一发布候选：`T2`。Replay SHA：`779da8043e36a5bed374fe28985158bd8827e29180abde78886d2f9fc54a3464`。
全 11 序列共同支持集上，候选合格且 J 严格低于 W0。
新回放启动数（含失败）：78/80。

## ate_rmse_m

| Sequence | B | G | W0 | T2 |
|---|---:|---:|---:|---:|
| MH_01_easy | 0.269967 | 0.270341 | 0.273753 | 0.268823 |
| MH_02_easy | 0.194630 | 0.182957 | 0.186180 | 0.186622 |
| MH_03_medium | 0.385766 | 0.375820 | 0.385435 | 0.379822 |
| MH_04_difficult | 0.545678 | 0.564262 | 0.552884 | 0.544925 |
| MH_05_difficult | 0.377180 | 0.370307 | 0.363146 | 0.385862 |
| V1_01_easy | 0.126207 | 0.127814 | 0.126092 | 0.125202 |
| V1_02_medium | 0.127580 | 0.127351 | 0.128745 | 0.129772 |
| V1_03_difficult | 0.160582 | 0.160496 | 0.163859 | 0.159529 |
| V2_01_easy | 0.127949 | 0.116949 | 0.115424 | 0.110998 |
| V2_02_medium | 0.183381 | 0.174155 | 0.161052 | 0.158212 |
| V2_03_difficult | 0.334255 | 0.329113 | 0.310905 | 0.322434 |
| Easy/Medium Mean | 0.202211 | 0.196484 | 0.196669 | 0.194207 |
| Difficult Mean | 0.354424 | 0.356045 | 0.347699 | 0.353188 |
| All Mean | 0.257561 | 0.254506 | 0.251589 | 0.252018 |

Easy/Medium Mean Δ vs B (ratio of group means): G -2.83%, W0 -2.74%, T2 -3.96%
Difficult Mean Δ vs B (ratio of group means): G +0.46%, W0 -1.90%, T2 -0.35%
All Mean Δ vs B (ratio of group means): G -1.19%, W0 -2.32%, T2 -2.15%

## ate_p95_m

| Sequence | B | G | W0 | T2 |
|---|---:|---:|---:|---:|
| MH_01_easy | 0.379357 | 0.391190 | 0.382840 | 0.388139 |
| MH_02_easy | 0.284324 | 0.274797 | 0.274277 | 0.270846 |
| MH_03_medium | 0.579299 | 0.564685 | 0.577295 | 0.571914 |
| MH_04_difficult | 0.818931 | 0.868272 | 0.863848 | 0.826398 |
| MH_05_difficult | 0.541890 | 0.561577 | 0.534446 | 0.575901 |
| V1_01_easy | 0.188991 | 0.189416 | 0.185769 | 0.187782 |
| V1_02_medium | 0.221447 | 0.222051 | 0.226223 | 0.224892 |
| V1_03_difficult | 0.258128 | 0.261702 | 0.265229 | 0.259060 |
| V2_01_easy | 0.288032 | 0.241071 | 0.246870 | 0.231534 |
| V2_02_medium | 0.305161 | 0.292376 | 0.276630 | 0.267835 |
| V2_03_difficult | 0.639988 | 0.624202 | 0.595356 | 0.608239 |
| Easy/Medium Mean | 0.320944 | 0.310798 | 0.309986 | 0.306134 |
| Difficult Mean | 0.564734 | 0.578938 | 0.564720 | 0.567400 |
| All Mean | 0.409595 | 0.408303 | 0.402617 | 0.401140 |

Easy/Medium Mean Δ vs B (ratio of group means): G -3.16%, W0 -3.41%, T2 -4.61%
Difficult Mean Δ vs B (ratio of group means): G +2.52%, W0 -0.00%, T2 +0.47%
All Mean Δ vs B (ratio of group means): G -0.32%, W0 -1.70%, T2 -2.06%

## ate_max_m

| Sequence | B | G | W0 | T2 |
|---|---:|---:|---:|---:|
| MH_01_easy | 0.399528 | 0.410337 | 0.402748 | 0.408584 |
| MH_02_easy | 0.302397 | 0.289979 | 0.301645 | 0.297192 |
| MH_03_medium | 0.613438 | 0.593216 | 0.619373 | 0.611727 |
| MH_04_difficult | 0.904799 | 0.940969 | 0.943660 | 0.899666 |
| MH_05_difficult | 0.632912 | 0.616039 | 0.621154 | 0.631738 |
| V1_01_easy | 0.220468 | 0.227382 | 0.219035 | 0.222304 |
| V1_02_medium | 0.232063 | 0.233986 | 0.239806 | 0.233937 |
| V1_03_difficult | 0.410752 | 0.424341 | 0.425629 | 0.423011 |
| V2_01_easy | 0.294032 | 0.246839 | 0.252726 | 0.235936 |
| V2_02_medium | 0.573218 | 0.572447 | 0.506513 | 0.518181 |
| V2_03_difficult | 0.646516 | 0.630599 | 0.601896 | 0.627847 |
| Easy/Medium Mean | 0.376449 | 0.367741 | 0.363121 | 0.361123 |
| Difficult Mean | 0.648745 | 0.652987 | 0.648085 | 0.645566 |
| All Mean | 0.475466 | 0.471467 | 0.466744 | 0.464557 |

Easy/Medium Mean Δ vs B (ratio of group means): G -2.31%, W0 -3.54%, T2 -4.07%
Difficult Mean Δ vs B (ratio of group means): G +0.65%, W0 -0.10%, T2 -0.49%
All Mean Δ vs B (ratio of group means): G -0.84%, W0 -1.83%, T2 -2.29%

## rotation_rmse_deg

| Sequence | B | G | W0 | T2 |
|---|---:|---:|---:|---:|
| MH_01_easy | 2.355399 | 2.296233 | 2.295003 | 2.292381 |
| MH_02_easy | 1.890432 | 1.952197 | 1.944132 | 1.953814 |
| MH_03_medium | 1.575831 | 1.398875 | 1.433035 | 1.730284 |
| MH_04_difficult | 3.258865 | 3.273228 | 3.290114 | 3.270098 |
| MH_05_difficult | 2.577248 | 2.584541 | 2.585059 | 2.583109 |
| V1_01_easy | 6.703572 | 6.493299 | 6.629495 | 6.482942 |
| V1_02_medium | 2.826267 | 2.976923 | 3.186908 | 2.777642 |
| V1_03_difficult | 6.896294 | 6.871036 | 6.954588 | 7.070687 |
| V2_01_easy | 3.742888 | 3.548983 | 3.608175 | 3.580777 |
| V2_02_medium | 5.697935 | 5.659198 | 5.796085 | 5.757308 |
| V2_03_difficult | 6.446860 | 6.382207 | 6.466139 | 6.603417 |
| Easy/Medium Mean | 3.541761 | 3.475101 | 3.556119 | 3.510735 |
| Difficult Mean | 4.794817 | 4.777753 | 4.823975 | 4.881828 |
| All Mean | 3.997417 | 3.948793 | 4.017158 | 4.009314 |

Easy/Medium Mean Δ vs B (ratio of group means): G -1.88%, W0 +0.41%, T2 -0.88%
Difficult Mean Δ vs B (ratio of group means): G -0.36%, W0 +0.61%, T2 +1.81%
All Mean Δ vs B (ratio of group means): G -1.22%, W0 +0.49%, T2 +0.30%

## velocity_rmse_mps

| Sequence | B | G | W0 | T2 |
|---|---:|---:|---:|---:|
| MH_01_easy | 0.036554 | 0.036891 | 0.036400 | 0.036215 |
| MH_02_easy | 0.034854 | 0.034475 | 0.034579 | 0.034550 |
| MH_03_medium | 0.069108 | 0.068070 | 0.067452 | 0.069985 |
| MH_04_difficult | 0.080783 | 0.079985 | 0.081644 | 0.080258 |
| MH_05_difficult | 0.070919 | 0.072138 | 0.070287 | 0.071760 |
| V1_01_easy | 0.033174 | 0.032883 | 0.032948 | 0.033002 |
| V1_02_medium | 0.046105 | 0.046843 | 0.047326 | 0.046532 |
| V1_03_difficult | 0.087199 | 0.086509 | 0.087916 | 0.088925 |
| V2_01_easy | 0.026954 | 0.027168 | 0.028382 | 0.028442 |
| V2_02_medium | 0.087306 | 0.088901 | 0.091881 | 0.091606 |
| V2_03_difficult | 0.087891 | 0.087972 | 0.091893 | 0.093884 |
| Easy/Medium Mean | 0.047722 | 0.047890 | 0.048424 | 0.048619 |
| Difficult Mean | 0.081698 | 0.081651 | 0.082935 | 0.083707 |
| All Mean | 0.060077 | 0.060167 | 0.060973 | 0.061378 |

Easy/Medium Mean Δ vs B (ratio of group means): G +0.35%, W0 +1.47%, T2 +1.88%
Difficult Mean Δ vs B (ratio of group means): G -0.06%, W0 +1.51%, T2 +2.46%
All Mean Δ vs B (ratio of group means): G +0.15%, W0 +1.49%, T2 +2.17%

## 结论

```json
{
  "system_offline_runnable": true,
  "system_live_startup": true,
  "improved_vs_W0": true,
  "improved_vs_B": true,
  "velocity_incremental_value": true,
  "velocity_factor_total": 9748,
  "final_J": -0.031054288072460367,
  "W0_J": -0.02864909535291354,
  "G_only_J": -0.01907342529044299,
  "ros_topics_verified": true,
  "rviz_verified": true,
  "J_change_vs_W0_percentage_points": -0.24051927195468253,
  "macro_ate_change_vs_W0_percent": 0.17073395017952464,
  "macro_ate_improved_vs_W0": false,
  "worst_sequence_change_vs_W0_percent": 6.255392884494926
}
```

离线 ATE 不代表实时链路性能。ROS topic 和 RViz 验收单独记录，不由离线结果推定。
选参目标是逐序列相对 B 的等权 J；绝对 ATE 算术均值相对 W0 为 +0.171%，最坏单序列相对 W0 为 +6.255%。筛选 +3% 门槛针对 B。
这不构成统计显著性或未见数据泛化的结论。EuRoC 是调参集，三次复测只覆盖固定 V1_01 序列。

## 覆盖率、尾部与耗时

- MH_01_easy: samples=3639, coverage={'B': 1.0, 'G': 1.0, 'W0': 1.0, 'T2': 1.0}
- MH_02_easy: samples=3000, coverage={'B': 1.0, 'G': 1.0, 'W0': 1.0, 'T2': 1.0}
- MH_03_medium: samples=2631, coverage={'B': 1.0, 'G': 1.0, 'W0': 1.0, 'T2': 1.0}
- MH_04_difficult: samples=1976, coverage={'B': 1.0, 'G': 1.0, 'W0': 1.0, 'T2': 1.0}
- MH_05_difficult: samples=2222, coverage={'B': 1.0, 'G': 1.0, 'W0': 1.0, 'T2': 1.0}
- V1_01_easy: samples=2872, coverage={'B': 1.0, 'G': 1.0, 'W0': 1.0, 'T2': 1.0}
- V1_02_medium: samples=1671, coverage={'B': 1.0, 'G': 1.0, 'W0': 1.0, 'T2': 1.0}
- V1_03_difficult: samples=2094, coverage={'B': 1.0, 'G': 1.0, 'W0': 1.0, 'T2': 1.0}
- V2_01_easy: samples=2240, coverage={'B': 1.0, 'G': 1.0, 'W0': 1.0, 'T2': 1.0}
- V2_02_medium: samples=2310, coverage={'B': 1.0, 'G': 1.0, 'W0': 1.0, 'T2': 1.0}
- V2_03_difficult: samples=1890, coverage={'B': 1.0, 'G': 1.0, 'W0': 1.0, 'T2': 1.0}

总 replay wall time: 5964.3 s。逐次耗时、Gate reason-mask/factor counts/robust-region incidence 见 trial_results.jsonl 与各 validation.json。

## 全部复测

```json
[
  {
    "index": 1,
    "path": "/home/he/output/ltv_stage7_tuning/all11_20261001/trials/V1_01_easy/5cf0514d06eb3235b0590f723d4ef8fa8d665c0c43c2ea2cc689126b767d6cc0-repeat-1",
    "vio_identical": true,
    "metrics": {
      "B": {
        "ate_max_m": 0.2204678608747015,
        "ate_p95_m": 0.1889905864342497,
        "ate_rmse_m": 0.12620672500581537,
        "rotation_rmse_deg": 6.703571871958577,
        "velocity_rmse_mps": 0.03317433413999172
      },
      "repeat": {
        "ate_max_m": 0.22230419203795307,
        "ate_p95_m": 0.18778195377732854,
        "ate_rmse_m": 0.12520154515436424,
        "rotation_rmse_deg": 6.4829423777380235,
        "velocity_rmse_mps": 0.03300232264548017
      }
    }
  },
  {
    "index": 2,
    "path": "/home/he/output/ltv_stage7_tuning/all11_20261001/trials/V1_01_easy/5cf0514d06eb3235b0590f723d4ef8fa8d665c0c43c2ea2cc689126b767d6cc0-repeat-2",
    "vio_identical": true,
    "metrics": {
      "B": {
        "ate_max_m": 0.2204678608747015,
        "ate_p95_m": 0.1889905864342497,
        "ate_rmse_m": 0.12620672500581537,
        "rotation_rmse_deg": 6.703571871958577,
        "velocity_rmse_mps": 0.03317433413999172
      },
      "repeat": {
        "ate_max_m": 0.22230419203795307,
        "ate_p95_m": 0.18778195377732854,
        "ate_rmse_m": 0.12520154515436424,
        "rotation_rmse_deg": 6.4829423777380235,
        "velocity_rmse_mps": 0.03300232264548017
      }
    }
  },
  {
    "index": 3,
    "path": "/home/he/output/ltv_stage7_tuning/all11_20261001/trials/V1_01_easy/5cf0514d06eb3235b0590f723d4ef8fa8d665c0c43c2ea2cc689126b767d6cc0-repeat-3",
    "vio_identical": true,
    "metrics": {
      "B": {
        "ate_max_m": 0.2204678608747015,
        "ate_p95_m": 0.1889905864342497,
        "ate_rmse_m": 0.12620672500581537,
        "rotation_rmse_deg": 6.703571871958577,
        "velocity_rmse_mps": 0.03317433413999172
      },
      "repeat": {
        "ate_max_m": 0.22230419203795307,
        "ate_p95_m": 0.18778195377732854,
        "ate_rmse_m": 0.12520154515436424,
        "rotation_rmse_deg": 6.4829423777380235,
        "velocity_rmse_mps": 0.03300232264548017
      }
    }
  }
]
```

## Estimator 回放失败记录

```json
[]
```

排名、所有候选参数与淘汰原因见 *_ranking.json、candidate_registry.jsonl。历史 Stage6/notune 未修改。

## 逐序列 ATE 与相对变化

| Sequence | B | G-only | Δ vs B | W0 | Δ vs B | Final Joint | Δ vs B | Δ vs W0 | Δ vs G-only |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| MH_01_easy | 0.269967 | 0.270341 | +0.14% | 0.273753 | +1.40% | 0.268823 | -0.42% | -1.80% | -0.56% |
| MH_02_easy | 0.194630 | 0.182957 | -6.00% | 0.186180 | -4.34% | 0.186622 | -4.11% | +0.24% | +2.00% |
| MH_03_medium | 0.385766 | 0.375820 | -2.58% | 0.385435 | -0.09% | 0.379822 | -1.54% | -1.46% | +1.06% |
| MH_04_difficult | 0.545678 | 0.564262 | +3.41% | 0.552884 | +1.32% | 0.544925 | -0.14% | -1.44% | -3.43% |
| MH_05_difficult | 0.377180 | 0.370307 | -1.82% | 0.363146 | -3.72% | 0.385862 | +2.30% | +6.26% | +4.20% |
| V1_01_easy | 0.126207 | 0.127814 | +1.27% | 0.126092 | -0.09% | 0.125202 | -0.80% | -0.71% | -2.04% |
| V1_02_medium | 0.127580 | 0.127351 | -0.18% | 0.128745 | +0.91% | 0.129772 | +1.72% | +0.80% | +1.90% |
| V1_03_difficult | 0.160582 | 0.160496 | -0.05% | 0.163859 | +2.04% | 0.159529 | -0.66% | -2.64% | -0.60% |
| V2_01_easy | 0.127949 | 0.116949 | -8.60% | 0.115424 | -9.79% | 0.110998 | -13.25% | -3.83% | -5.09% |
| V2_02_medium | 0.183381 | 0.174155 | -5.03% | 0.161052 | -12.18% | 0.158212 | -13.72% | -1.76% | -9.15% |
| V2_03_difficult | 0.334255 | 0.329113 | -1.54% | 0.310905 | -6.99% | 0.322434 | -3.54% | +3.71% | -2.03% |
| Easy/Medium Mean | 0.202211 | 0.196484 | -2.83% | 0.196669 | -2.74% | 0.194207 | -3.96% | -1.25% | -1.16% |
| Difficult Mean | 0.354424 | 0.356045 | +0.46% | 0.347699 | -1.90% | 0.353188 | -0.35% | +1.58% | -0.80% |
| All Mean | 0.257561 | 0.254506 | -1.19% | 0.251589 | -2.32% | 0.252018 | -2.15% | +0.17% | -0.98% |

逐时刻 ATE 误差已导出 `error_series/*.csv`，用于检查 >10% 尾部警告。

W5 去重记录：`{"W5": {"alias": "W0", "gravity_axis_best": "W0", "velocity_axis_best": "W0", "reason": "combined best qualified axes duplicate W0; no replacement candidate", "replays_saved": 3}}`。

## D0 全部候选

| Candidate | J | Worst Δ vs B | Engineering | Screening |
|---|---:|---:|---|---|
| W0 | -0.019185618649510554 | 0.013206686846070737 | True | True |
| W1 | -0.013437557261630317 | 0.010713027504859252 | True | True |
| W2 | -0.0022167154271749054 | 0.07360200308280884 | True | False |
| W3 | -0.002122898719589409 | 0.035390750927761117 | True | False |
| W4 | 0.006897374402700292 | 0.024397513237549795 | True | True |
| T1 | -0.005163534496630429 | -7.072513991823737e-05 | True | True |
| T2 | -0.014903226670637112 | -0.0013799771252667181 | True | True |
| T3 | 0.0018511596792936125 | 0.02033956761617639 | True | True |
| T4 | -0.005531070161013211 | 0.03275527182801019 | True | False |

## 最终各模式运行诊断

| Sequence | Mode | G factors | V factors | G coverage | V coverage | Wall time (s) |
|---|---|---:|---:|---:|---:|---:|
| MH_01_easy | B | 0 | 0 | 0.9031 | 0.5609 | 116.32 |
| MH_01_easy | G | 3135 | 0 | 0.8679 | 0.5645 | 115.24 |
| MH_01_easy | W0 | 3263 | 2033 | 0.9034 | 0.5593 | 114.91 |
| MH_01_easy | T2 | 3127 | 2036 | 0.8657 | 0.5601 | 116.26 |
| MH_02_easy | B | 0 | 0 | 0.9215 | 0.4615 | 99.70 |
| MH_02_easy | G | 2567 | 0 | 0.8649 | 0.4632 | 99.01 |
| MH_02_easy | W0 | 2735 | 1387 | 0.9215 | 0.4639 | 99.27 |
| MH_02_easy | T2 | 2560 | 1375 | 0.8625 | 0.4599 | 99.65 |
| MH_03_medium | B | 0 | 0 | 0.7271 | 0.3037 | 89.16 |
| MH_03_medium | G | 1599 | 0 | 0.6043 | 0.3034 | 88.31 |
| MH_03_medium | W0 | 1911 | 799 | 0.7277 | 0.3015 | 88.40 |
| MH_03_medium | T2 | 1580 | 802 | 0.6017 | 0.3026 | 89.51 |
| MH_04_difficult | B | 0 | 0 | 0.8146 | 0.2526 | 64.33 |
| MH_04_difficult | G | 1434 | 0 | 0.7488 | 0.2465 | 63.67 |
| MH_04_difficult | W0 | 1550 | 483 | 0.8094 | 0.2491 | 63.75 |
| MH_04_difficult | T2 | 1463 | 498 | 0.7553 | 0.2540 | 63.68 |
| MH_05_difficult | B | 0 | 0 | 0.8407 | 0.2231 | 71.69 |
| MH_05_difficult | G | 1692 | 0 | 0.7719 | 0.2202 | 69.54 |
| MH_05_difficult | W0 | 1829 | 490 | 0.8344 | 0.2211 | 70.76 |
| MH_05_difficult | T2 | 1692 | 491 | 0.7797 | 0.2238 | 70.22 |
| V1_01_easy | B | 0 | 0 | 0.9432 | 0.6508 | 92.42 |
| V1_01_easy | G | 2486 | 0 | 0.9238 | 0.6549 | 91.60 |
| V1_01_easy | W0 | 2558 | 1784 | 0.9436 | 0.6523 | 92.03 |
| V1_01_easy | T2 | 2495 | 1804 | 0.9203 | 0.6596 | 91.76 |
| V1_02_medium | B | 0 | 0 | 0.7368 | 0.1904 | 52.90 |
| V1_02_medium | G | 592 | 0 | 0.4650 | 0.1897 | 51.54 |
| V1_02_medium | W0 | 939 | 248 | 0.7376 | 0.1912 | 56.84 |
| V1_02_medium | T2 | 593 | 244 | 0.4655 | 0.1880 | 51.91 |
| V1_03_difficult | B | 0 | 0 | 0.7640 | 0.2683 | 66.36 |
| V1_03_difficult | G | 414 | 0 | 0.4911 | 0.2641 | 58.42 |
| V1_03_difficult | W0 | 625 | 229 | 0.7603 | 0.2707 | 64.78 |
| V1_03_difficult | T2 | 412 | 231 | 0.4922 | 0.2683 | 58.47 |
| V2_01_easy | B | 0 | 0 | 0.9375 | 0.7678 | 78.09 |
| V2_01_easy | G | 1969 | 0 | 0.9235 | 0.7662 | 75.39 |
| V2_01_easy | W0 | 1994 | 1655 | 0.9370 | 0.7691 | 76.58 |
| V2_01_easy | T2 | 1971 | 1668 | 0.9254 | 0.7744 | 75.51 |
| V2_02_medium | B | 0 | 0 | 0.7255 | 0.2449 | 74.87 |
| V2_02_medium | G | 866 | 0 | 0.4653 | 0.2297 | 73.87 |
| V2_02_medium | W0 | 1338 | 450 | 0.7252 | 0.2408 | 74.41 |
| V2_02_medium | T2 | 868 | 419 | 0.4717 | 0.2248 | 73.58 |
| V2_03_difficult | B | 0 | 0 | 0.7593 | 0.2013 | 53.94 |
| V2_03_difficult | G | 388 | 0 | 0.4455 | 0.2056 | 53.58 |
| V2_03_difficult | W0 | 643 | 179 | 0.7600 | 0.2057 | 53.57 |
| V2_03_difficult | T2 | 390 | 180 | 0.4498 | 0.2020 | 53.61 |

Gate coverage 为日志中的判定通过率；禁用分支时不解释为已加入因素比例。Huber 比例仅表示加入因素 snapshot 的 logged weighted norm > delta，不表示所有滑窗 residual 的求解统计。

## ROS 与 RViz 验收证据

```json
{
  "last_messages": {
    "odometry": {
      "frame_id": "world",
      "timestamp_s": 1403715418.362143
    },
    "path": {
      "frame_id": "world",
      "pose_count": 2894,
      "timestamp_s": 1403715418.4121432
    },
    "point_cloud": {
      "frame_id": "world",
      "timestamp_s": 1403715418.4121432
    }
  },
  "observer_elapsed_s": 111.95405371300876,
  "rviz_command": [
    "ros2",
    "run",
    "rviz2",
    "rviz2",
    "-d",
    "/home/he/vins_fusion_ltv_ws/src/vins_fusion_ltv/config/tuning/stage7.rviz",
    "--ros-args",
    "-r",
    "/vins_estimator/path:=/path",
    "-r",
    "/vins_estimator/odometry:=/odometry",
    "-r",
    "/vins_estimator/point_cloud:=/point_cloud",
    "-r",
    "/vins_estimator/margin_cloud:=/margin_cloud",
    "-r",
    "/vins_estimator/key_poses:=/key_poses",
    "-r",
    "/vins_estimator/camera_pose:=/camera_pose",
    "-r",
    "/vins_estimator/camera_pose_visual:=/camera_pose_visual",
    "-r",
    "/vins_estimator/keyframe_pose:=/keyframe_pose",
    "-r",
    "/vins_estimator/keyframe_point:=/keyframe_point",
    "-r",
    "/vins_estimator/extrinsic:=/extrinsic",
    "-r",
    "/vins_estimator/image_track:=/image_track"
  ],
  "rviz_running": true,
  "screenshot": "/home/he/output/ltv_stage7_tuning/all11_20261001/release/rviz_delivery.png",
  "screenshot_note": "manually inspected: world frame OK, visible green VIO path and yellow points; recovered from direct RViz-window XWD, BGR24 with row padding",
  "screenshot_verified": true,
  "topic_counts": {
    "odometry": 977,
    "path": 978,
    "point_cloud": 987
  }
}
```

## 可视化验收失败与恢复

```json
[
  {
    "failure": "PIL root screenshot X get_image failed under XWayland; observer exited",
    "log": "/home/he/output/ltv_stage7_tuning/all11_20261001/release/visualization.first_failure.log",
    "stage": "first observation"
  },
  {
    "failure": "XWD BPP24 initially rejected; screenshot recovered with BGR24 row-padding decoder",
    "log": "/home/he/output/ltv_stage7_tuning/all11_20261001/release/visualization.supplementary.log",
    "stage": "supplementary observation"
  }
]
```
一次补验收回放使用 W5 去重节省的预算，累计 78/80。参数和 estimator 二进制未改变。

完整配置：`/home/he/output/ltv_stage7_tuning/all11_20261001/release/euroc_stage7.yaml`。参数 SHA：`78879571631bc4b4d01139e25debacb3643d2711e16e452b36d7c8673a7406dd  stage7_final_parameters.yaml`。
