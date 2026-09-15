# VINS-Fusion + LTV Observer Codex 执行文档

## 0. 任务目标

在现有 **VINS-Fusion ROS2** 基线上实现论文：

> Pose, Velocity and Landmark Position Estimation Using IMU and Bearing Measurements  
> Miaomiao Wang, Abdelhamid Tayebi, ACC 2025 / arXiv:2407.18099

中的**第一层 LTV Observer**，并将其输出作为弱约束加入 VINS-Fusion sliding-window optimization。

本阶段只实现：

```text
Camera bearings + IMU
        ↓
   LTV Observer
        ↓
  v_hat_B, eta_hat
        ↓
VINS-Fusion optimizer
   + gravity factor
   + velocity factor
```

明确不实现：

- 论文第二层 nonlinear pose observer；
- world-frame known landmark pose observer；
- LTV landmark factor；
- 修改 VINS 原始 IMU preintegration；
- 修改 VINS reprojection factor；
- RTAB-Map；
- HNO-VIO observer；
- OpenVINS；
- loop closure 相关修改；
- IMU bias observer。

第一目标不是提高所有 EuRoC 序列 ATE，而是验证：

> LTV velocity / gravity estimate 是否能够提高 VINS-Fusion 在 aggressive motion 下的稳定性。

---

# 1. Codex 第一件事：检查本地仓库

不要立即修改代码。

首先执行：

```bash
pwd
git status
git rev-parse HEAD
git branch --show-current
find . -maxdepth 3 -type d | sort | head -100
```

确认：

1. 当前使用的 VINS-Fusion ROS2 仓库来源；
2. 当前 commit；
3. Ubuntu 22.04 + ROS2 Humble 是否已经能够编译；
4. EuRoC baseline 是否已经能够运行；
5. 实际源码目录是否为：

```text
vins/
├── src/
│   ├── estimator/
│   ├── factor/
│   ├── featureTracker/
│   ├── initial/
│   └── utility/
```

不要假设网络版本和本地版本完全一致。

重点定位以下函数/文件：

```text
vins/src/estimator/estimator.cpp
vins/src/estimator/estimator.h

vins/src/estimator/feature_manager.cpp
vins/src/estimator/feature_manager.h

vins/src/featureTracker/feature_tracker.cpp
vins/src/featureTracker/feature_tracker.h

vins/src/estimator/parameters.cpp
vins/src/estimator/parameters.h

vins/src/factor/
```

重点查找：

```bash
rg "processIMU"
rg "processImage"
rg "optimization"
rg "slideWindow"
rg "para_Pose"
rg "para_SpeedBias"
rg "featureFrame"
rg "RIC"
rg "TIC"
rg "Eigen::Vector3d G"
```

在修改代码之前，先给出一份简短代码结构报告：

```text
1. IMU 数据进入 estimator 的位置
2. image/features 进入 estimator 的位置
3. optimization() 位置
4. sliding window 状态移动位置
5. feature id / normalized coordinates 数据格式
6. RIC / TIC 定义
7. V / bias / gravity 的坐标系定义
```

确认后再开始实现。

---

# 2. 坐标系和符号统一

这是整个实现中优先级最高的部分。

统一定义：

```text
W : VINS world frame
B : IMU / body frame
C : camera0 frame
```

论文：

\[
R=R_{WB}
\]

即 body 到 world 的 rotation。

因此：

\[
v_B=R_{WB}^{T}v_W
\]

论文 gravity state：

\[
\eta=R_{WB}^{T}g
\]

注意 VINS-Fusion 内部通常定义：

\[
G_{\mathrm{VINS}}=[0,0,+9.8]^T
\]

并使用类似：

\[
a_W=R(a_m-b_a)-G_{\mathrm{VINS}}
\]

而论文使用实际 gravity acceleration：

\[
g_{\mathrm{paper}}=[0,0,-9.81]^T
\]

因此必须统一为：

\[
\boxed{
g_{\mathrm{paper}}=-G_{\mathrm{VINS}}
}
\]

后续代码不要混用 `G` 和论文中的 `g`。

建议明确命名：

```cpp
Eigen::Vector3d gravity_paper = -G;
```

不要在不同函数中临时改变符号。

---

# 3. LTV Observer 严格数学定义

## 3.1 状态

对于 N 个当前 LTV landmarks：

\[
x=
[
{}^Bp_1^T,
\dots,
{}^Bp_N^T,
v_B^T,
\eta^T
]^T
\]

维度：

\[
n=3N+6
\]

其中：

\[
{}^Bp_i=R^T(p_i-p)
\]

\[
v_B=R^Tv_W
\]

\[
\eta=R^Tg
\]

---

## 3.2 IMU 输入

论文使用：

\[
\omega
\]

以及 body-frame apparent acceleration：

\[
a
\]

工程实现第一版使用 VINS 当前 bias estimate：

\[
\boxed{
\omega=\omega_m-\hat b_g
}
\]

\[
\boxed{
a=a_m-\hat b_a
}
\]

禁止将下面这个 world-frame acceleration 输入 LTV：

\[
R(a_m-b_a)-G
\]

LTV 输入始终保持 body-frame。

---

# 4. ACC 论文公式 (10)

实现以下连续动力学：

\[
{}^B\dot p_i
=
-\omega^\times{}^Bp_i-v_B
\]

\[
\dot v_B
=
-\omega^\times v_B+\eta+a
\]

\[
\dot\eta
=
-\omega^\times\eta
\]

其中：

\[
\omega^\times=
\begin{bmatrix}
0&-\omega_z&\omega_y\\
\omega_z&0&-\omega_x\\
-\omega_y&\omega_x&0
\end{bmatrix}
\]

必须写一个统一：

```cpp
Eigen::Matrix3d skew(const Eigen::Vector3d& v);
```

禁止项目中出现多个不同符号约定的 skew 实现。

---

# 5. Bearing Measurement

VINS feature tracker 已经产生去畸变 normalized coordinates：

```text
x_n
y_n
1
```

因此：

\[
z_i=
\frac{
[x_n,y_n,1]^T
}{
\|[x_n,y_n,1]^T\|
}
\]

不要再次乘：

\[
K^{-1}
\]

第一版只使用：

```text
camera_id == 0
```

即左目 / camera0。

即使 baseline 是 stereo VINS，也暂时只把 camera0 bearing 输入 LTV。

---

# 6. Camera-IMU 外参

论文定义：

\[
{}^Cp_i
=
R_c^T({}^Bp_i-p_c)
\]

其中：

```text
Rc : camera frame relative to body 的 rotation 定义
pc : camera optical center 在 body frame 中的位置
```

在使用 VINS `RIC[0] / TIC[0]` 前，必须通过本地代码确认其方向。

不能仅根据变量名判断。

要求 Codex：

1. 查 `readParameters()`；
2. 查 camera pose 输出公式；
3. 查 projection factor；
4. 明确写出：

```text
RIC[0] = ?
TIC[0] = ?
```

只有确认与论文定义一致后才能使用。

如果本地 VINS 确认：

\[
R_{WC}=R_{WB}R_{BC}
\]

\[
p_{WC}=p_{WB}+R_{WB}p_{BC}
\]

则：

```cpp
Rc = RIC[0];
pc = TIC[0];
```

---

# 7. Modified Bearing Output

这是实现中不能遗漏的一步。

定义：

\[
\Pi_i
=
I-(R_cz_i)(R_cz_i)^T
\]

论文的 observation 不是简单的 `Pi`，而是：

\[
\boxed{
y_i=\Pi_i p_c
}
\]

并满足：

\[
y_i=\Pi_i{}^Bp_i
\]

因此每个 feature 必须计算：

```cpp
Eigen::Vector3d b = Rc * z;
Eigen::Matrix3d Pi =
    Eigen::Matrix3d::Identity() - b * b.transpose();

Eigen::Vector3d yi = Pi * pc;
```

不能错误写成：

```cpp
y = z;
```

或者：

```cpp
y = 0;
```

除非实际标定满足 `pc == 0`。

---

# 8. LTV Matrix

构造：

\[
\dot x=Ax+Ba
\]

\[
y=Cx
\]

定义：

\[
A_\omega=
\operatorname{blkdiag}
(
\omega^\times,\dots,\omega^\times
)
\]

以及：

\[
\Gamma=
\begin{bmatrix}
I\\
I\\
\vdots\\
I
\end{bmatrix}
\]

因此：

\[
A=
\begin{bmatrix}
-A_\omega&-\Gamma&0\\
0&-\omega^\times&I\\
0&0&-\omega^\times
\end{bmatrix}
\]

\[
B=
\begin{bmatrix}
0\\
I\\
0
\end{bmatrix}
\]

视觉：

\[
\Lambda_z
=
\operatorname{blkdiag}
(
\Pi_1,\dots,\Pi_N
)
\]

\[
C=
\begin{bmatrix}
\Lambda_z&0&0
\end{bmatrix}
\]

\[
y=
[
y_1^T,\dots,y_N^T
]^T
\]

实现后必须检查矩阵维度：

```text
x : (3N+6) x 1

A : (3N+6) x (3N+6)

B : (3N+6) x 3

C : 3N x (3N+6)

y : 3N x 1

P : (3N+6) x (3N+6)

Q : 3N x 3N

V : (3N+6) x (3N+6)
```

所有 resize 后必须 assert。

---

# 9. Riccati Observer

实现论文：

\[
\dot{\hat x}
=
A\hat x+Ba+
K(y-C\hat x)
\]

\[
K=PC^TQ
\]

\[
\dot P
=
AP+PA^T
-PC^TQCP
+V
\]

这里的：

```text
P
Q
V
```

属于 Riccati observer design。

第一阶段禁止直接把：

```cpp
P_vv
P_eta_eta
```

当成 Ceres factor covariance。

LTV observer 的 `P` 与后续 factor 的 measurement covariance 分开处理。

---

# 10. 离散实现

论文给的是 continuous-time observer。

当前工程需要离散化。

第一版优先简单、透明、可调试，不追求复杂数值积分。

## IMU propagation

每个 IMU interval：

```cpp
propagateImu(dt, acc, gyro, ba, bg);
```

使用：

\[
\omega=\omega_m-b_g
\]

\[
a=a_m-b_a
\]

传播：

\[
\hat x_{k+1}
=
\hat x_k+
\Delta t(A\hat x_k+Ba)
\]

Riccati propagation：

\[
P_{k+1}
=
P_k+
\Delta t
(
AP+PA^T+V
)
\]

注意：

视觉 correction term 不在没有新 camera measurement 时重复使用。

---

## Camera update

新 camera frame 到达时：

1. 收集当前有效 camera0 features；
2. 构造 bearing；
3. 构造 \(\Pi_i\)；
4. 构造 \(C\)；
5. 构造 \(y\)；
6. 计算：

\[
r=y-C\hat x
\]

\[
K=PC^TQ
\]

第一版 correction 使用：

\[
\hat x
\leftarrow
\hat x+
\Delta t_{\mathrm{cam}}Kr
\]

以及：

\[
P
\leftarrow
P-
\Delta t_{\mathrm{cam}}
PC^TQCP
\]

这是对 continuous Riccati observer 的工程离散近似。

必须在代码注释注明：

```text
Engineering discretization of continuous-time observer.
Not a discrete equation given directly by the paper.
```

更新之后执行：

```cpp
P = 0.5 * (P + P.transpose());
```

并检查：

```text
NaN
Inf
negative diagonal
numerical explosion
```

必要时对最小 eigenvalue / diagonal 做非常小的 numerical floor。

禁止悄悄加入复杂 Kalman measurement update 而不说明数学变化。

---

# 11. 新建 LTV 模块

推荐目录：

```text
vins/src/ltv/
├── ltv_observer.h
├── ltv_observer.cpp
└── ltv_types.h
```

如本地工程结构不同，可适配，但不要把核心 LTV 代码全部塞入 `estimator.cpp`。

推荐类：

```cpp
class LtvObserver
{
public:
    LtvObserver();

    void reset();

    void propagateImu(
        double timestamp,
        const Eigen::Vector3d& acc_m,
        const Eigen::Vector3d& gyr_m,
        const Eigen::Vector3d& ba,
        const Eigen::Vector3d& bg);

    void updateFeatures(
        double timestamp,
        const FeatureObservations& observations,
        const Eigen::Matrix3d& R_bc,
        const Eigen::Vector3d& p_bc);

    bool valid() const;

    Eigen::Vector3d velocityBody() const;
    Eigen::Vector3d gravityBody() const;

    LtvSnapshot snapshot(double timestamp) const;

private:
    ...
};
```

推荐：

```cpp
struct LtvSnapshot
{
    double timestamp = 0.0;

    Eigen::Vector3d v_B =
        Eigen::Vector3d::Zero();

    Eigen::Vector3d eta =
        Eigen::Vector3d::Zero();

    bool velocity_valid = false;
    bool gravity_valid = false;

    int active_features = 0;
};
```

---

# 12. Feature 生命周期

论文理论中 N 个 landmarks 固定。

VINS 实际 feature track 会不断：

```text
create
track
lost
```

因此动态 feature 生命周期属于**工程扩展**，不能声称仍然严格满足论文固定 N 的理论条件。

建立：

```cpp
std::unordered_map<int, int> feature_id_to_slot;
```

其中：

```text
feature_id : VINS tracker feature id
slot       : LTV x 中 landmark block index
```

要求同一个 `feature_id` 跨 frame 始终对应同一个 LTV landmark state。

绝对禁止：

```text
每一帧重新按照 vector 顺序给 feature 编号
```

否则 landmark state 会错位。

---

## 新 feature

第一版：

\[
{}^B\hat p_i(0)=0
\]

与论文 simulation 初始化保持简单一致。

为新增 landmark 扩展：

```text
x
P
```

新增 landmark covariance 使用配置参数：

```yaml
ltv_new_landmark_p0: ...
```

不要硬编码散落在代码中。

---

## feature lost

不能在遍历矩阵时直接删除导致 index 混乱。

建议：

1. 收集 stale feature id；
2. 一次性 rebuild compact state；
3. 建立 old slot → new slot 映射；
4. 对 `x` 重排；
5. 对 `P` 同时重排行、列；
6. 更新 map。

可以设置：

```yaml
ltv_feature_max_missed_frames: 2
```

允许短暂丢失。

---

# 13. 第一阶段只运行 LTV，不接优化器

完成 LTV 类后，暂时禁止增加任何 Ceres residual。

运行：

```text
VINS baseline
+
passive LTV observer
```

LTV 仅输出日志。

建议保存：

```text
timestamp
active_feature_count

v_B_hat_x
v_B_hat_y
v_B_hat_z

eta_hat_x
eta_hat_y
eta_hat_z
eta_norm

innovation_norm
P_trace
P_min_diag
P_max_diag
valid
```

例如：

```text
results/ltv_debug.csv
```

---

# 14. LTV 独立验收

优先使用：

```text
EuRoC V1_01_easy
```

第一阶段不是立即跑 difficult。

检查：

## gravity

\[
\|\hat\eta\|\approx9.81
\]

并比较 GT：

\[
\eta_{GT}
=
R_{GT}^Tg
\]

重点检查 gravity direction：

\[
e_g
=
\arccos
\frac{
\hat\eta^T\eta_{GT}
}{
\|\hat\eta\|
\|\eta_{GT}\|
}
\]

---

## velocity

GT：

\[
v_{B,GT}
=
R_{GT}^Tv_{W,GT}
\]

比较：

\[
\hat v_B-v_{B,GT}
\]

输出：

```text
velocity RMSE
gravity-direction RMSE
eta norm
innovation norm
```

如果 passive LTV 本身明显发散：

> 停止，不允许继续实现 Ceres factor。

---

# 15. LTV Snapshot

LTV 通过 camera timestamp 产生：

```cpp
LtvSnapshot snapshot;
```

snapshot 必须冻结。

即：

```text
LTV → VINS
```

单向流动。

不能在一次 `optimization()` 内：

```text
VINS state
→ recompute LTV
→ LTV factor
→ optimize VINS
→ recompute LTV
```

避免明显 circular dependency。

LTV 可使用上一次已经完成优化得到的：

```text
Ba
Bg
```

作为下一段 IMU 的 bias correction。

---

# 16. Sliding Window Snapshot 管理

每个 window frame 需要对应：

```text
timestamp
ltv snapshot
```

建议维护：

```cpp
LtvSnapshot ltv_snapshots[WINDOW_SIZE + 1];
```

或等价容器。

必须与：

```text
Rs
Ps
Vs
Bas
Bgs
Headers
```

使用完全相同的 sliding-window 移动逻辑。

重点检查：

```text
slideWindowOld()
slideWindowNew()
```

或者本地对应函数。

禁止发生：

```text
VINS frame k
使用了 LTV frame k+1 snapshot
```

必须检查 timestamp。

建议允许：

```yaml
ltv_snapshot_max_time_error: 0.005
```

如果超过阈值，则本帧禁止加 factor。

---

# 17. Gravity Factor

passive LTV 验证通过后，第一项 factor 只实现 gravity。

定义：

\[
\boxed{
r_g
=
R_{WB,k}^{T}
(-G_{\mathrm{VINS}})
-
\hat\eta_k
}
\]

单位：

```text
m/s^2
```

新建：

```text
vins/src/factor/ltv_gravity_factor.h
vins/src/factor/ltv_gravity_factor.cpp
```

或者项目 factor 的同等位置。

不要修改原始 IMU factor。

---

## gravity validity gate

满足以下条件才允许加入：

```text
snapshot.gravity_valid == true
active_features >= minimum threshold
eta_hat finite
eta_norm within configured range
timestamp matched
LTV not recently reset
```

例如配置：

```yaml
ltv_min_features: 15
ltv_gravity_norm_min: 7.0
ltv_gravity_norm_max: 12.0
```

数值先作为工程配置，不要声称来自论文。

---

# 18. Gravity Factor 权重

不要使用：

```cpp
P_eta_eta.inverse()
```

作为第一版 information。

使用独立：

```yaml
ltv_gravity_sigma: ...
```

定义：

\[
r'_g
=
\frac{1}{\sigma_g}
r_g
\]

并增加 robust loss：

```cpp
ceres::HuberLoss
```

第一轮权重必须保守。

目标是：

```text
弱约束
```

而不是让 LTV 替代 IMU factor。

---

# 19. Velocity Factor

gravity-only 验证稳定后，再实现：

\[
\boxed{
r_v=
R_{WB,k}^{T}V_{W,k}
-\hat v_{B,k}
}
\]

其中 VINS：

```text
Vs[k]
```

应先通过本地代码确认是 world-frame velocity。

新建：

```text
vins/src/factor/ltv_velocity_factor.h
vins/src/factor/ltv_velocity_factor.cpp
```

factor 参数至少涉及：

```text
pose orientation
speed/velocity
```

不要重复创建新的 velocity state。

---

# 20. Ceres Jacobian

必须遵循 VINS 当前 quaternion parameterization。

先检查：

```text
pose_local_parameterization.*
```

确认是：

```text
left perturbation
```

还是：

```text
right perturbation
```

禁止凭印象写 Jacobian。

对于典型右扰动：

\[
R(\delta\theta)
=
R\exp(\delta\theta^\times)
\]

有：

\[
\delta(R^Tv)
\approx
[R^Tv]_\times\delta\theta
\]

因此候选 Jacobian：

\[
\frac{\partial R^Tv}
{\partial\delta\theta}
=
[R^Tv]_\times
\]

\[
\frac{\partial R^Tv}
{\partial v}
=
R^T
\]

gravity 同理：

\[
\frac{\partial R^Tg}
{\partial\delta\theta}
=
[R^Tg]_\times
\]

但最终符号必须根据本地 `PoseLocalParameterization` 验证。

要求加入 finite-difference Jacobian test。

如果 analytic Jacobian 与 numerical Jacobian 不一致：

> 不允许继续实验。

第一版允许优先使用 AutoDiff 验证逻辑，再决定是否改 analytic factor。

---

# 21. Ceres 接入位置

查找：

```cpp
Estimator::optimization()
```

现有结构通常类似：

```text
add marginalization prior
add IMU factors
add projection factors
solve
```

新增：

```text
add LTV gravity factors
add LTV velocity factors
```

保持：

```text
IMU factor       unchanged
projection factor unchanged
```

建议配置：

```yaml
ltv_enable: true
ltv_enable_gravity_factor: true
ltv_enable_velocity_factor: false
```

从而可以运行消融。

---

# 22. 第一版暂不修改 marginalization

初次验证阶段：

> LTV factor 只加入当前 sliding-window optimization。

暂时不要修改：

```text
marginalization_factor
marginalization_info
```

原因：

先验证：

```text
factor residual
Jacobian
weight
稳定性
```

是否合理。

如果结果明确有效，再进行第二阶段工作：

```text
LTV factor → marginalization prior
```

当前任务不要提前实现。

---

# 23. 配置参数

新增单独配置区域，例如：

```yaml
# -----------------------
# LTV observer
# -----------------------

ltv_enable: true

ltv_q_landmark: ...
ltv_q_velocity: ...
ltv_q_gravity: ...

ltv_v_landmark: ...
ltv_v_velocity: ...
ltv_v_gravity: ...

ltv_initial_p_landmark: ...
ltv_initial_p_velocity: ...
ltv_initial_p_gravity: ...

ltv_min_features: 15
ltv_feature_max_missed_frames: 2

ltv_snapshot_max_time_error: 0.005

ltv_enable_gravity_factor: false
ltv_enable_velocity_factor: false

ltv_gravity_sigma: ...
ltv_velocity_sigma: ...

ltv_huber_delta_gravity: ...
ltv_huber_delta_velocity: ...
```

第一版不要把论文 simulation：

\[
Q=10^{-4}I
\]

\[
V=10^6I
\]

机械地当成最终工程参数。

可以作为 reproduction 初始参考，但必须全部配置化。

---

# 24. Debug 开关

实现：

```yaml
ltv_log_debug: true
```

release 模式下必须能够关闭大量输出。

禁止每个 IMU callback：

```cpp
std::cout
```

刷屏。

使用节流日志或 CSV。

---

# 25. 编译测试

每完成一个阶段立即：

```bash
colcon build --symlink-install
```

禁止累计大量修改后一次性编译。

至少按下面顺序：

```text
Commit / Step 1
LTV types + empty class
→ build

Step 2
IMU propagation
→ build

Step 3
feature / bearing management
→ build

Step 4
Riccati observer
→ build

Step 5
passive logging
→ EuRoC test

Step 6
gravity factor
→ build + Jacobian test

Step 7
gravity experiment

Step 8
velocity factor
→ build + Jacobian test

Step 9
gravity + velocity experiment
```

---

# 26. 实验顺序

## Stage A：baseline

```text
VINS-Fusion
LTV completely disabled
```

确保与修改前结果一致。

必须确认：

```text
trajectory frame count
ATE
程序退出状态
```

没有发生 regression。

---

## Stage B：passive LTV

```text
VINS-Fusion
+
LTV observer
+
NO LTV factor
```

VINS trajectory 应基本等同 baseline。

验证 LTV 自身输出。

---

## Stage C：gravity

```text
VINS-Fusion
+
LTV observer
+
gravity factor
```

---

## Stage D：gravity + velocity

```text
VINS-Fusion
+
LTV observer
+
gravity factor
+
velocity factor
```

---

# 27. EuRoC 测试集合

开发调试首先：

```text
V1_01_easy
```

确认正常后测试：

```text
MH_04_difficult
MH_05_difficult
V1_03_difficult
V2_02_medium
V2_03_difficult
```

不要只报告最终 ATE。

记录：

```text
ATE
rotation error
roll error
pitch error
gravity direction error
velocity RMSE
maximum instantaneous position error
maximum rotation error
tracking failure
divergence
```

重点分析：

```text
high angular velocity
high acceleration
rapid viewpoint change
```

区间。

---

# 28. 消融矩阵

最终至少提供：

| Variant | LTV | Gravity factor | Velocity factor |
|---|---:|---:|---:|
| A | off | off | off |
| B | on | off | off |
| C | on | on | off |
| D | on | on | on |

其中 B 非常重要。

它可以判断：

> 只是运行 LTV 本身是否影响 VINS pipeline。

---

# 29. Double Counting

必须在代码 README / 注释中明确：

LTV observer 和 VINS-Fusion 使用相同：

```text
IMU
camera bearings
```

因此：

```text
LTV factor != independent sensor measurement
```

存在：

```text
cross correlation
double counting
```

本阶段只作为工程实验。

因此必须：

1. factor weight 保守；
2. 使用 robust loss；
3. 支持完全关闭；
4. 单独做消融；
5. 不将 LTV covariance 解释为独立 measurement covariance；
6. 不宣称整个 VINS+LTV 系统继承论文 GES/AGAS 理论保证。

论文稳定性结论针对论文完整 observer 及其假设，不自动适用于这里的 VINS hybrid architecture。

---

# 30. 当前禁止做的优化

当前不要自行扩展任务到：

```text
LTV landmark factor
known world landmarks
nonlinear pose observer
IMU bias state
online extrinsic calibration redesign
LTV-based initialization
loop closure
global optimization
RTAB-Map
SuperPoint
feature tracker replacement
GPU optimization
ROS architecture refactor
```

如果发现这些问题，只记录到：

```text
TODO_LATER.md
```

不在当前任务修改。

---

# 31. 必须加入的 sanity checks

运行时检查：

```text
dt > 0
dt < reasonable limit

active feature count

all state finite
all P finite

P symmetric

eta finite
v finite

bearing norm ≈ 1

Pi symmetric

Pi * Pi ≈ Pi

Pi * (Rc*z) ≈ 0

timestamp aligned
```

测试：

\[
\Pi^T=\Pi
\]

\[
\Pi^2=\Pi
\]

\[
\Pi(R_cz)=0
\]

这些可以写简单 unit test。

---

# 32. LTV reset

至少实现：

```cpp
void LtvObserver::reset();
```

以下情况 reset：

```text
VINS estimator reset
timestamp backward
large timestamp discontinuity
non-finite state
P numerical failure
```

reset 后设置：

```text
snapshot.valid = false
```

直到满足重新启用条件。

不能让损坏的 LTV state 继续进入 optimizer。

---

# 33. 第一阶段验收标准

第一阶段完成必须满足全部：

### Build

```text
Ubuntu 22.04
ROS2 Humble
colcon build passes
```

### Baseline

```text
LTV disabled 时结果与原 baseline 基本一致
```

### Formula

确认实现：

\[
{}^B\dot p_i
=
-\omega^\times{}^Bp_i-v_B
\]

\[
\dot v_B
=
-\omega^\times v_B+\eta+a
\]

\[
\dot\eta
=
-\omega^\times\eta
\]

以及：

\[
y_i=\Pi_ip_c
\]

### LTV

在 V1_01_easy：

```text
无 NaN
无 Inf
P 不爆炸
eta norm 合理
velocity 有界
innovation 可记录
```

### Factors

gravity / velocity factor：

```text
可以独立 enable/disable
Jacobian numerical check passes
snapshot timestamp matches window state
```

### Architecture

原始：

```text
IMU preintegration
reprojection factors
```

保持不变。

---

# 34. 每阶段 Codex 输出格式

不要只回答：

```text
Done
```

每阶段完成后输出：

```text
## Modified files

- path/file1
- path/file2

## What was implemented

...

## Formula/code mapping

paper equation → function / line

## Build result

...

## Runtime result

...

## Known limitations

...

## Next step

...
```

如果遇到数学或坐标系不确定：

> 停止该部分修改，先把相关原始代码和推导展示出来。

不要猜测后继续编码。

---

# 35. 推荐最终代码结构

目标结构大致为：

```text
VINS-Fusion-ROS2/
└── vins/
    └── src/
        ├── estimator/
        │   ├── estimator.cpp
        │   └── estimator.h
        │
        ├── featureTracker/
        │
        ├── factor/
        │   ├── ltv_gravity_factor.h
        │   ├── ltv_gravity_factor.cpp
        │   ├── ltv_velocity_factor.h
        │   └── ltv_velocity_factor.cpp
        │
        └── ltv/
            ├── ltv_types.h
            ├── ltv_observer.h
            └── ltv_observer.cpp
```

外加配置：

```text
config/...yaml
```

和 debug：

```text
results/ltv_debug.csv
```

---

# 36. 当前任务的最终数据流

最终第一版必须严格保持：

```text
                IMU
                 │
      acc-ba, gyro-bg
                 │
                 ▼
        ┌─────────────────┐
        │   LTV Observer  │
        └─────────────────┘
                 ▲
                 │
     normalized camera0
           bearings
                 │

                 ▼
       v_hat_B , eta_hat
                 │
          frozen snapshot
                 │
                 ▼
┌───────────────────────────────────┐
│        VINS-Fusion Window         │
│                                   │
│  original IMU factor              │
│  original reprojection factors    │
│                                   │
│  + LTV gravity factor             │
│  + LTV velocity factor            │
│                                   │
└───────────────────────────────────┘
```

禁止形成同一次优化中的反馈环：

```text
VINS optimization
      ↓
update LTV
      ↓
factor
      ↓
same optimization
```

---

# 37. 现在开始执行的顺序

Codex 当前只执行到下面阶段：

```text
STEP 0
检查本地 VINS-Fusion repo 和 baseline

STEP 1
创建独立 LtvObserver 模块

STEP 2
接入 IMU + camera0 normalized bearings

STEP 3
实现 continuous LTV 的工程离散版本

STEP 4
实现 feature lifecycle

STEP 5
输出 passive LTV debug CSV

STEP 6
V1_01_easy 独立验证
```

完成 STEP 6 后停止。

暂时不要实现：

```text
gravity factor
velocity factor
```

先汇报：

```text
代码修改
编译结果
LTV 输出
eta norm
gravity direction error
velocity error
innovation
P 数值状态
发现的问题
```

确认 passive LTV 正确后，再进入 Ceres factor 阶段。