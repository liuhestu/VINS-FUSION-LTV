# EuRoC Loop Fusion：1 倍速回放与评估（2026-09-22）

三个序列均完整播放，bag、VINS、Loop Fusion 退出码均为 0，pose graph 已保存。三个序列的位置 ATE 均改善。但回环边残差存在明显离群值，不能仅凭单位四元数和 ATE 改善认定所有约束正确。

## 运行与评估方法

- 配置：`config/euroc/euroc_stereo_imu_config.yaml`，双目 + IMU，LTV 默认关闭，未修改 BRIEF/PnP 阈值或优化模型。
- 回放速度：1.0×；节点启动间隔 4 s；bag 结束后 drain 45 s；输入 `s` 保存并退出。
- 输出根目录：`/home/he/output/euroc_loop_final_1x_20260922`。之前中断的 0.2×、2×及旧错误链路输出均不参与评估。
- GT：`/home/he/datasets/euroc/ASL/<sequence>/mav0/state_groundtruth_estimate0/data.csv`。
- 比较同一份 `pose_graph.txt` 内相同关键帧的原始 VIO 位姿和最终优化位姿；non-loop 是同次运行的原始 VIO，不是独立重跑。
- 时间匹配：最近邻、最大误差 20 ms；两条轨迹使用相同 GT 样本，各自独立 SE(3) 刚体对齐，不估计尺度。单位为米。
- `evaluation.json` 保存完整精度指标、GT/pose graph 哈希和运行 manifest；每个序列保留 `vio.csv`、`vio_loop.csv`、图像/BRIEF/pose graph、节点日志及配置哈希。

## 位置误差

| 序列 | VIO ATE RMSE | Loop ATE RMSE | ATE 降幅 | VIO P95 | Loop P95 | VIO 最大误差 | Loop 最大误差 |
|---|---:|---:|---:|---:|---:|---:|---:|
| MH_01_easy | 0.240219 | 0.166610 | 30.64% | 0.362069 | 0.245279 | 0.372207 | 0.307867 |
| V1_01_easy | 0.108993 | 0.082019 | 24.75% | 0.169721 | 0.132959 | 0.193759 | 0.140903 |
| V1_02_medium | 0.101890 | 0.069196 | 32.09% | 0.176177 | 0.118694 | 0.203202 | 0.155848 |

## 数据完整性与回环诊断

| 序列 | VIO 行数 | Loop / 图关键帧数 | GT 匹配数 | 接受日志 / 图回环边 | 接受 PnP 内点 / 输入 | 加权内点率 |
|---|---:|---:|---:|---:|---:|---:|
| MH_01_easy | 1831 | 1091 / 1091 | 1085 | 183 / 183 | 9394 / 20650 | 45.49% |
| V1_01_easy | 1446 | 1100 / 1100 | 1100 | 128 / 128 | 5949 / 13155 | 45.22% |
| V1_02_medium | 845 | 768 / 768 | 768 | 45 / 45 | 1685 / 3478 | 48.45% |

全部保存回环参数有限，四元数范数误差小于 1e-3，无全零回环平移；保存日志报告 invalid_saved_loops=0。三组 `throw img0/img1` 日志计数均为 0。
但当前双目线程会无日志地丢弃积压帧，Loop 线程也会跳过积压关键帧，因此零条 throw 日志不能证明零丢帧；VIO 行数与图关键帧数也不应直接等同。未单独录制外参 topic，门控后能处理关键帧只证明接收过有效外参，不能代替持续到达频率统计。

当前 C++ 诊断会在中间阶段打印 rejection 标签，甚至随后接受同一候选。评估按 `(current,candidate)` 合并字段并取最后状态，PnP 统计不会重复累计。这也是历史“69 次 PnP”计数不能直接作为唯一候选数量的原因。

## 优化后回环约束残差

平移残差为 `R_old.T @ (p_current - p_old) - loop_translation` 的范数；yaw 残差为两帧 yaw 差减回环 yaw 后归一化至 ±180°。这是图内部约束残差，不是 GT 误差，未加鲁棒核权重。

| 序列 | 平移 RMSE / P95 / 最大（m） | yaw RMSE / P95 / 最大（°） |
|---|---:|---:|
| MH_01_easy | 3.071 / 8.779 / 15.783 | 8.691 / 20.617 / 59.553 |
| V1_01_easy | 1.306 / 3.258 / 5.688 | 6.804 / 13.123 / 44.105 |
| V1_02_medium | 1.418 / 3.120 / 3.945 | 9.972 / 30.126 / 37.187 |

残差表显示仍有异常约束需要逐边核查，尤其 MH_01 最大平移残差 15.78 m。此次保留完整诊断，不通过调阈值隐藏问题；结论仅为本次三个序列位置 ATE 改善，尚不能声称每条回环都可靠。

## 复现

从仓库根目录执行（输出目录必须是新的）：

```bash
source /opt/ros/humble/setup.bash
source ../../install/setup.bash
python3 vins/scripts/run_euroc_loop.py \
  --dataset-root /home/he/datasets/euroc \
  --output-root /home/he/output/euroc_loop_1x_NEW \
  --sequence MH_01_easy --sequence V1_01_easy --sequence V1_02_medium \
  --rate 1 --start-delay 4 --drain-delay 45 \
  --loop-timeout 240 --shutdown-timeout 30
python3 vins/scripts/evaluate_euroc_loop.py \
  /home/he/output/euroc_loop_1x_NEW \
  --sequence MH_01_easy --sequence V1_01_easy --sequence V1_02_medium
```
