# VINS-Fusion + LTV Observer 新对话提示词

我接下来只讨论一个新方向：**VINS-Fusion + LTV Observer**。请不要再把历史记忆中 HNO-VIO 当前的 `e → sigma_R → R` observer、RTAB-Map、OpenVINS 作为主线，除非我明确要求对比。

## 背景

我原来的 HNO-VIO 在 EuRoC 的 V1_01、V1_02、V2_01 等序列可以稳定，但在 MH_01/02/04/05、V1_03、V2_02/03 等更困难序列上容易发散。之前实验已经发现，姿态/重力方向漂移会进一步导致速度和位置爆炸。因此我准备暂停现有 HNO observer 路线，改成以成熟的 **VINS-Fusion ROS2** 为基线，再加入一个具有稳定性分析基础的 LTV observer。

LTV 方法来自论文：

**Pose, Velocity and Landmark Position Estimation Using IMU and Bearing Measurements**  
Miaomiao Wang, Abdelhamid Tayebi, 2025 ACC / arXiv:2407.18099

我目前只采用论文的**第一层 LTV observer**，不采用第二层需要已知 world-frame landmarks 的 nonlinear pose observer。

## LTV 第一层输入

- IMU gyroscope \(\omega\)
- IMU acceleration \(a\)
- monocular bearing \(z_i\)

其中：

\[
z_i=\frac{K^{-1}[u_i,v_i,1]^T}{\|K^{-1}[u_i,v_i,1]^T\|}
\]

如果 VINS 前端已经提供 normalized coordinates \((x_n,y_n)\)，则直接：

\[
z_i=\mathrm{normalize}([x_n,y_n,1]^T)
\]

## LTV 状态

\[
x=[{}^Bp_1,\dots,{}^Bp_N,\,v_B,\,\eta]
\]

其中：

\[
\eta = R^\top g
\]

LTV 动力学：

\[
{}^B\dot p_i=-\omega^\times {}^Bp_i-v_B
\]

\[
\dot v_B=-\omega^\times v_B+\eta+a
\]

\[
\dot\eta=-\omega^\times\eta
\]

视觉约束使用 bearing projection：

\[
\Pi_{z_i}=I-(R_c z_i)(R_c z_i)^T
\]

并构造对应的线性时变观测模型。论文使用 Riccati observer：

\[
\dot{\hat x}=A\hat x+Ba+K(y-C\hat x)
\]

\[
K=PC^\top Q
\]

\[
\dot P=AP+PA^\top-PC^\top QCP+V
\]

## 当前工程目标

```text
Camera features + IMU
        ↓
    LTV Observer
        ↓
  v̂_B , η̂
        ↓
VINS-Fusion sliding-window optimization
```

第一版只加入两个 factor，不修改原始 IMU preintegration：

\[
r_v=R_k^\top v_k-\hat v_{B,k}
\]

\[
r_g=R_k^\top g-\hat\eta_k
\]

也就是：

```text
原有：
IMU factor
+ reprojection factor

新增：
+ LTV velocity factor
+ LTV gravity factor
```

第一阶段暂时不要加入 LTV landmark factor。后续如果 gravity/velocity 有效果，再考虑：

\[
r_{f,i}=R_k^\top(P_i-p_k)-\hat{{}^Bp}_{i,k}
\]

需要特别注意：LTV 和 VINS-Fusion 使用的是同一批 IMU 和视觉 bearing，因此 LTV factor 不是独立传感器，存在 double counting 和 cross-correlation 问题。第一版以工程验证为主，factor 权重必须保守，并使用合理 covariance / robust loss。

## 当前开发环境

```text
Ubuntu 22.04
ROS2 Humble
C++17
EuRoC dataset
```

当前优先基线是 ROS2 Humble 版本的 VINS-Fusion： https://github.com/zinuok/VINS-Fusion-ROS2。

## 实验目标

实验目标不是单纯追求正常序列 ATE 大幅下降，而是验证 LTV 是否提升 aggressive motion 下的稳定性，重点测试：

```text
EuRoC:
MH_04_difficult
MH_05_difficult
V1_03_difficult
V2_02_medium
V2_03_difficult
```

建议消融：

```text
A. VINS-Fusion baseline

B. VINS-Fusion
   + LTV gravity factor

C. VINS-Fusion
   + LTV gravity factor
   + LTV velocity factor
```

重点指标：

- ATE
- rotation error
- roll/pitch error
- gravity-direction error
- velocity RMSE
- 最大瞬时误差
- tracking failure / divergence
- 高角速度、高加速度区间的误差增长率

## 当前阶段优先问题

1. VINS-Fusion ROS2 代码结构和修改入口
2. 如何从 feature tracker 获取 bearing
3. LTV observer 的离散化实现
4. LTV 状态和 feature 生命周期管理
5. gravity / velocity factor 的 Ceres residual 和 Jacobian
6. covariance / residual weight 的初始设置
7. 时间同步和坐标系
8. EuRoC 跑通和 baseline 对比
9. 后续是否加入 LTV landmark factor
10. 如何避免 double counting

回答时尽量简洁、技术化，不需要重复基础 VIO 概念。优先给出具体代码入口、公式、数据流和实验方案。
