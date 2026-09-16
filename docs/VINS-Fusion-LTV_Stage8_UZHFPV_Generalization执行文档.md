# VINS-Fusion + LTV Stage 8 执行文档：UZH-FPV 跨数据集泛化

## 目标与冻结边界

Stage 8 使用 Stage 7 唯一冻结参数，只比较原始 Baseline 与 Final LTV System。EuRoC
全量结果必须标记为 `tuning-set final score`；UZH-FPV 在任何参数选择中均未使用，是
跨数据集 held-out。看到 UZH-FPV 结果后不得修改参数、筛序列或改变评价口径。

UZH-FPV 面向高速、激烈飞行场景，数据集背景见[官方说明](https://rpg.ifi.uzh.ch/aggressive_flight.html)。

## 本地 held-out 清单

根目录固定为 `/home/he/datasets/UZHFPV`。只使用本地 16 条带官方 GT 的 Snapdragon
ROS1 bag：

```text
indoor_45_{2,4,9,12,13,14}_snapdragon_with_gt
indoor_forward_{3,5,6,7,9,10}_snapdragon_with_gt
outdoor_45_1_snapdragon_with_gt
outdoor_forward_{1,3,5}_snapdragon_with_gt
```

必须在运行前冻结清单以及 bag/GT SHA-256。没有官方 GT 的其它本地 bag 不进入正式
成功率分母，也不得作为替代序列。

## 四套标定、输入转换与 canonical cache

分别从官方 calibration archive 生成四套 VINS 配置：

```text
indoor_45  indoor_forward  outdoor_45  outdoor_forward
```

每套配置显式记录左右相机模型、内参、畸变、分辨率、`T_cam_imu`、IMU noise/random
walk、topic 名和来源文件 SHA-256。禁止在某条正式序列上重新标定或手调外参。

ROS1 bag 输入使用 stereo、IMU 与 `/groundtruth/odometry`。先实现只读转换器，按消息
header 时间戳生成与 EuRoC 语义一致的 canonical cache：严格递增 IMU、确定性双指针
stereo 配对、无损图像、pair/IMU/GT manifest 和 SHA-256。转换器必须验证消息类型、
坐标系、时间范围、左右覆盖率以及 GT pose/velocity 的有限性；原始 bag 不修改。

## evaluator 扩展

统一 evaluator 新增 UZH odometry reader，读取 pose quaternion、position 与 linear
velocity，并显式实现从 GT body frame 到 estimator IMU frame 的固定外参变换。通过合成
轨迹测试覆盖：时间匹配、SE(3) alignment、四元数顺序、速度旋转、body/IMU lever-arm
项和 P95/max。EuRoC 现有输出必须保持回归一致。

正式输出：

```text
ATE RMSE / P95 / max
Rotation RMSE / P95 / max
roll / pitch RMSE
Velocity RMSE / P95 / max
trajectory success / failure
reset / solver failure / NaN-Inf
wall time / realtime factor / peak RSS / factor counts
```

## 确定性运行与判定

每条序列先运行 Baseline，再运行 Final LTV System；两者共享同一 canonical cache，均为
单线程、loop closure OFF。所有 run 使用 partial/failed/final 事务目录，失败也必须保留
日志并计入成功率。不得只汇报成功配对的序列。

分别按 `indoor/outdoor` 与 `forward/45°` 汇总，并给出全 16 条 micro（按匹配样本）和
macro（按序列等权）结果。主结论以逐序列 ATE、Rotation、Velocity RMSE、最大误差和
成功率为依据，同时报告计算开销。Final LTV System 只有在成功率不低于 Baseline、无
新增 solver/NaN 失败，且多数分组的稳健主指标不出现明显退化时，才记为跨数据集泛化
通过；否则如实报告失败或局部有效，不回到 Stage 7 调参。
