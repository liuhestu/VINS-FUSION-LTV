# VINS-Fusion + LTV Observer 第二阶段 Codex 执行文档

## 0. 当前状态

Passive LTV Observer 的 STEP 0–6 已完成，目前状态如下：

```text
VINS-Fusion baseline
        +
Passive LTV Observer
```

已经实现：

- 独立 LTV observer；
- camera0 normalized bearing；
- `feature_id -> LTV landmark state` 生命周期；
- IMU bias correction；
- Riccati observer 工程离散；
- adaptive Euler 子步；
- passive CSV；
- EuRoC GT evaluation；
- timestamp/reset/PSD/finite sanity check。

当前未实现：

```text
LTV gravity factor
LTV velocity factor
sliding-window LTV snapshot
LTV marginalization
```

并且未修改：

```text
VINS IMU preintegration
reprojection factor
feature tracker
原 sliding-window 数学逻辑
```

---

# 1. Passive LTV 当前验证结果

EuRoC：

```text
V1_01_easy
```

完整运行结果：

```text
CSV frames:                   1445
valid frames:                 1232
gravity-valid frames:         1229

reset:                        0
NaN / Inf:                    0

max ||v_hat_B||:              2.603 m/s
max ||eta_hat||:             10.085 m/s²

camera update mean:           5.52 ms
camera update max:           13.09 ms
max adaptive Euler substeps: 20
```

与官方 EuRoC body-frame GT 对齐：

```text
matched frames:               1222

body velocity RMSE:           0.500 m/s
gravity direction RMSE:       2.69 deg
max timestamp error:          0.238 us
```

坐标系已经确认：

```text
Rs       = R_WB
Vs       = v_W
RIC[0]   = R_BC
TIC[0]   = p_BC

eta = R_WB^T * g_world
```

其中：

```text
g_world = [0, 0, -g]^T
```

之前直接使用 EuRoC Vicon sensor pose 评估得到的 110.9° 已确认属于 GT frame 使用错误，不再采用。

---

# 2. 第二阶段目标

第二阶段只完成：

```text
Passive LTV
    ↓
Frozen LtvSnapshot
    ↓
Sliding Window
    ↓
Gravity Direction Factor
    ↓
VINS-Fusion Optimization
```

本阶段明确不实现：

```text
velocity factor
landmark factor
marginalization integration
IMU preintegration modification
LTV bias state
nonlinear pose observer
```

只有 gravity-only 实验完成并验证有效后，才进入 velocity factor。

---

# 3. 为什么先实现 Gravity Factor

Passive LTV 当前：

\[
\text{gravity direction RMSE}=2.69^\circ
\]

而：

\[
\text{body velocity RMSE}=0.500\;m/s
\]

gravity direction 已经表现出比较明确的有效信息。

velocity 是否足够准确，暂时还需要进一步结合：

```text
GT velocity magnitude
instantaneous velocity error
aggressive-motion interval
```

判断。

因此第二阶段只增加 gravity factor，避免同时引入两个新的 optimization variable coupling。

---

# 4. Gravity Factor 不使用原始 eta 向量残差

不要直接实现：

\[
r_g
=
R_k^\top g-\hat\eta_k
\]

作为最终第一版 factor。

原因：

理论状态满足：

\[
\eta=R^\top g
\]

因此：

\[
\|\eta\|=\|g\|
\]

但是实际 LTV estimator 存在数值误差。

当前已经观测到：

\[
\|\hat\eta\|_{\max}=10.085\;m/s^2
\]

而 VINS 中：

\[
\|g\|\approx9.81\;m/s^2
\]

rotation optimization 只能改变 gravity vector 的方向，不能改变其模长。

因此使用 raw residual：

\[
R^\top g-\hat\eta
\]

会包含一个姿态无法消除的 radial residual。

这会污染 factor，并可能影响 robust loss 的实际权重。

---

# 5. Gravity Direction Factor

第一版只约束 gravity direction。

定义：

\[
g_W=
\begin{bmatrix}
0\\
0\\
-g
\end{bmatrix}
\]

或者根据当前 VINS gravity convention：

\[
g_W=-G_{\mathrm{VINS}}
\]

定义 VINS 当前 body-frame gravity direction：

\[
u_g
=
\frac{R_{WB}^\top g_W}
{\|g_W\|}
\]

LTV gravity direction：

\[
\hat u_g
=
\frac{\hat\eta}
{\|\hat\eta\|}
\]

定义 residual：

\[
\boxed{
r_g
=
u_g-\hat u_g
}
\]

即：

\[
\boxed{
r_g
=
\frac{R_{WB}^\top g_W}{\|g_W\|}
-
\frac{\hat\eta}{\|\hat\eta\|}
}
\]

residual dimension：

```text
3
```

单位：

```text
dimensionless
```

该 factor 主要约束：

```text
roll
pitch
```

不会提供 yaw observable information。

---

# 6. 不修改 LTV Observer

重要：

这里只修改 **LTV → VINS optimization interface**。

不要修改 LTV 内部状态定义。

LTV 仍然估计：

\[
\eta=R^\top g
\]

而不是把 LTV 状态改成 normalized gravity vector。

也就是说：

```text
LTV内部：
eta_hat

factor内部：
eta_hat.normalized()
```

不要把 normalization 加到 Riccati observer 状态传播中。

---

# 7. 新增 LtvSnapshot

首先完成 snapshot 层，不要立即写 Ceres factor。

推荐结构：

```cpp
struct LtvSnapshot
{
    double timestamp = 0.0;

    Eigen::Vector3d v_B =
        Eigen::Vector3d::Zero();

    Eigen::Vector3d eta =
        Eigen::Vector3d::Zero();

    bool valid = false;
    bool gravity_valid = false;
    bool velocity_valid = false;

    int active_features = 0;

    double innovation_norm = 0.0;
};
```

如果现有 `ltv_types.h` 已经存在类似结构，直接扩展，不重复建立另一套类型。

---

# 8. Snapshot 生成时机

在对应 camera frame 完成：

```text
IMU propagation
        ↓
camera bearing update
        ↓
LTV Riccati correction
```

之后冻结：

```cpp
LtvSnapshot snapshot =
    ltv_observer.snapshot(timestamp);
```

因此 snapshot 表示：

> 当前 camera timestamp 上、完成视觉 correction 后的 LTV estimate。

不要在 Ceres `optimization()` 内重新调用 LTV update。

---

# 9. 数据流必须保持单向

必须保持：

```text
IMU + bearing
      ↓
LTV Observer
      ↓
Frozen LTV Snapshot
      ↓
VINS optimization
```

禁止：

```text
optimization()
     ↓
修改 Rs/Vs/Bias
     ↓
重新计算当前 LTV
     ↓
重新生成 factor
     ↓
继续同一次 optimization
```

否则形成明显的同轮 circular feedback。

允许：

```text
上一轮 VINS bias estimate
          ↓
下一时间段 LTV IMU bias correction
```

即跨时刻的顺序依赖是可以接受的。

---

# 10. Sliding Window Snapshot

每一个 VINS window frame 保存一个对应 snapshot。

可以建立：

```cpp
LtvSnapshot ltv_snapshots[WINDOW_SIZE + 1];
```

也可以使用与项目代码风格一致的容器。

关键要求：

```text
ltv_snapshots[k]
```

必须严格对应：

```text
Headers[k]
Rs[k]
Ps[k]
Vs[k]
Bas[k]
Bgs[k]
```

不能仅按最近 timestamp 临时搜索一个 snapshot。

---

# 11. Snapshot Timestamp Check

每个 snapshot 保存 camera timestamp。

加入：

```yaml
ltv_snapshot_max_time_error: 0.005
```

即：

```text
5 ms
```

使用 factor 前检查：

\[
|t_{\mathrm{snapshot}}
-
t_{\mathrm{frame}}|
<
t_{\max}
\]

EuRoC 正常情况下应该远小于这个值。

当前 passive GT evaluation 已达到：

```text
0.238 us
```

因此 5 ms 只是防御性阈值。

如果时间不匹配：

```text
skip factor
```

禁止使用最近一帧 snapshot 强行代替。

---

# 12. Sliding Window 生命周期

重点检查本地：

```text
slideWindowOld()
slideWindowNew()
```

以及对应状态复制/移动逻辑。

Snapshot 必须和以下状态使用相同的 index permutation：

```text
Rs
Ps
Vs
Bas
Bgs
Headers
```

需要专门测试：

### MARGIN_OLD

```text
frame 0 removed
frame 1 → frame 0
...
```

LTV snapshot 必须同步移动。

### MARGIN_SECOND_NEW

如果项目使用 second-newest marginalization：

snapshot 必须使用和 `Rs/Vs/Headers` 相同的覆盖逻辑。

不要自己设计另一套 sliding policy。

---

# 13. Snapshot Unit Test

增加测试验证：

```text
timestamp
eta
valid flags
```

在 sliding window 前后没有错位。

推荐构造人为 timestamp：

```text
t0 = 0
t1 = 1
t2 = 2
...
```

每个 snapshot 的 eta 可以设成：

```text
eta.x = frame_id
```

执行 slide 后验证：

```text
Headers[k]
snapshot[k].timestamp
snapshot[k].eta.x
```

仍对应同一 frame。

---

# 14. 新增 Gravity Factor

推荐新建：

```text
vins/src/factor/ltv_gravity_factor.h
vins/src/factor/ltv_gravity_factor.cpp
```

如果当前项目 factor 目录层级不同，以本地结构为准。

Factor 输入：

```text
pose k
frozen eta_hat_k
```

不需要输入：

```text
position
velocity
Ba
Bg
feature state
```

实际 parameter block 可以继续使用 VINS pose block，但 residual 只依赖 quaternion。

---

# 15. Factor Residual

实现：

```cpp
Eigen::Vector3d g_world = -G;

Eigen::Vector3d gravity_body =
    R_WB.transpose() * g_world;

Eigen::Vector3d u_vins =
    gravity_body.normalized();

Eigen::Vector3d u_ltv =
    eta_hat.normalized();

Eigen::Vector3d residual =
    u_vins - u_ltv;
```

增加 sanity checks：

```text
eta_hat finite

||eta_hat|| > epsilon

G finite

||G|| > epsilon
```

---

# 16. Factor Validity Gate

只有同时满足以下条件才添加 factor：

```text
snapshot.valid
snapshot.gravity_valid

timestamp matched

eta_hat finite

eta norm valid

active feature number sufficient

LTV observer not just reset
```

新增配置：

```yaml
ltv_min_features: 15

ltv_gravity_norm_min: 7.0
ltv_gravity_norm_max: 12.0

ltv_snapshot_max_time_error: 0.005
```

这些值属于：

```text
engineering gate
```

不是论文理论阈值。

必须在代码注释中明确。

---

# 17. 不使用 Riccati P 直接作为 Factor Covariance

当前 passive LTV：

```text
P使用论文 simulation 量级参数
数值稳定
但尺度仍较大
```

因此禁止：

```cpp
sqrt_info =
    P_eta_eta.inverse().llt()...
```

第一阶段 factor covariance 与 observer P 分离。

使用独立：

```yaml
ltv_gravity_sigma_deg: 10.0
```

---

# 18. Gravity Factor Weight

因为 residual 是 unit vector difference：

对于较小夹角 \(\theta\)：

\[
\|u_1-u_2\|
\approx\theta
\]

其中 \(\theta\) 使用 rad。

因此可以把 factor sigma 用角度配置。

例如：

```yaml
ltv_gravity_sigma_deg: 10.0
```

代码转换：

```cpp
sigma_rad =
    sigma_deg * M_PI / 180.0;
```

然后：

\[
r'_g
=
\frac{1}{\sigma_{\mathrm{rad}}}
r_g
\]

---

# 19. 第一轮 Weight Sweep

Passive gravity direction RMSE：

\[
2.69^\circ
\]

但 LTV 与 VINS 共享：

```text
IMU
visual bearings
```

因此 LTV 不是独立传感器。

不能设置：

```text
sigma = 2.69 deg
```

然后把它当作真实 independent measurement noise。

第一轮建议测试：

```text
20 deg
10 deg
5 deg
3 deg
```

重点观察：

```text
10 deg
```

作为初始 conservative weight。

---

# 20. Robust Loss

增加：

```cpp
ceres::HuberLoss
```

配置：

```yaml
ltv_gravity_huber_delta: ...
```

注意：

HuberLoss 工作在经过 sqrt-information 加权后的 residual 上。

因此 delta 需要结合：

```text
sigma
```

解释，不要随意使用大数。

第一版可以采用比较宽松的 Huber gate，只用于防止极端 LTV outlier。

---

# 21. Quaternion Perturbation

在写 analytic Jacobian 前，必须重新检查本地：

```text
PoseLocalParameterization
```

确认 VINS 使用：

```text
q_new = q * dq
```

还是：

```text
q_new = dq * q
```

不要根据旧版 VINS-Mono 或网上代码直接假设。

必须以本地仓库为准。

---

# 22. 右扰动情况下的 Jacobian

如果确认本地使用：

\[
R'=
R\exp(\delta\theta^\times)
\]

则：

\[
R'^Tg
=
\exp(-\delta\theta^\times)R^Tg
\]

一阶近似：

\[
R'^Tg
\approx
R^Tg
+
[R^Tg]_\times\delta\theta
\]

对于 normalized gravity：

\[
u_g=
\frac{R^Tg}{\|g\|}
\]

由于 rotation 不改变 \(\|g\|\)，因此：

\[
\boxed{
\frac{\partial u_g}
{\partial\delta\theta}
=
[u_g]_\times
}
\]

所以：

\[
\boxed{
\frac{\partial r_g}
{\partial\delta\theta}
=
[u_g]_\times
}
\]

position Jacobian：

\[
\frac{\partial r_g}{\partial p}=0
\]

但最终实现符号必须通过 numerical Jacobian 验证。

---

# 23. 优先实现 AutoDiff / Independent Residual

不要第一步就直接写复杂 analytic Ceres Jacobian。

推荐顺序：

```text
Step 1
写纯数学 residual function

Step 2
AutoDiff / standalone evaluation

Step 3
finite difference

Step 4
analytic Jacobian

Step 5
analytic vs numeric comparison
```

这样可以分离：

```text
数学错误
Ceres参数布局错误
Quaternion local parameterization错误
```

---

# 24. Numerical Jacobian Test

增加 GTest。

测试：

```text
random orientation
near identity
large roll/pitch
EuRoC actual orientation samples
```

使用 central finite difference：

\[
J_i
=
\frac{
r(\delta_i+\epsilon)
-
r(\delta_i-\epsilon)
}{
2\epsilon
}
\]

建议：

```text
epsilon ≈ 1e-7 ~ 1e-6
```

检查：

```text
max abs error
relative error
```

目标至少：

```text
1e-5 ~ 1e-6
```

量级。

如果 analytic Jacobian 不通过：

> 不允许进入 EuRoC gravity-factor 实验。

---

# 25. Optimization 接入

定位：

```cpp
Estimator::optimization()
```

保持原有：

```text
marginalization prior
IMU factors
reprojection factors
```

不变。

新增：

```text
LTV gravity factors
```

第一版只对满足 validity gate 的 window frames 增加。

不要修改：

```text
IMUFactor
ProjectionFactor
```

---

# 26. Gravity Factor 不能影响 Position Block

虽然当前 pose parameter block 通常是：

```text
p + q
```

gravity factor 对 position 不敏感。

analytic Jacobian 中：

```text
translation columns = 0
rotation columns = J_rotation
```

必须通过测试确认。

---

# 27. 第一版不加入 Marginalization

当前只把 gravity factor 加入：

```text
current nonlinear optimization
```

不要添加到：

```text
MarginalizationInfo
ResidualBlockInfo
```

等 marginalization prior 逻辑中。

原因：

先独立验证：

```text
residual
Jacobian
weight
robust loss
causal effect
```

只有 gravity-only 明确有效后，再考虑 marginalization information retention。

---

# 28. 配置开关

增加：

```yaml
ltv_enable: true

ltv_enable_gravity_factor: false

ltv_gravity_sigma_deg: 10.0

ltv_gravity_huber_delta: ...

ltv_snapshot_max_time_error: 0.005

ltv_min_features: 15

ltv_gravity_norm_min: 7.0
ltv_gravity_norm_max: 12.0
```

要求：

```text
ltv_enable = false
```

时完全保持原 baseline 行为。

要求：

```text
ltv_enable = true
ltv_enable_gravity_factor = false
```

时保持 passive LTV 行为。

---

# 29. 增加 Gravity Debug CSV

在现有 passive CSV 基础上，可以增加：

```text
frame_index
timestamp

ltv_gravity_valid

eta_norm

gravity_angle_ltv_vs_vins_before_opt

gravity_factor_residual_norm

gravity_factor_weighted_residual_norm

gravity_factor_added
```

如方便，再输出：

```text
roll_before
pitch_before

roll_after
pitch_after
```

但不要为了日志大规模改 estimator architecture。

---

# 30. 第一轮实验只使用 V1_01_easy

首先运行：

## A. Baseline

```text
LTV disabled
gravity factor disabled
```

确认修改没有 regression。

---

## B. Passive LTV

```text
LTV enabled
gravity factor disabled
```

应与当前 STEP 6 结果一致。

---

## C. Gravity Factor

```text
LTV enabled
gravity factor enabled
sigma = 10 deg
```

检查：

```text
程序稳定
无 NaN
optimizer 正常收敛
ATE 不出现明显恶化
roll/pitch 不出现系统偏差
```

第一步不是要求 V1_01 ATE 一定下降。

---

# 31. 然后进入 Difficult Sequence

如果 V1_01 没有 regression，再测试：

```text
MH_04_difficult
MH_05_difficult
V1_03_difficult
V2_02_medium
V2_03_difficult
```

重点比较：

```text
Baseline
Passive LTV
Gravity factor
```

---

# 32. 重点指标

不要只报告 ATE。

至少记录：

```text
ATE

rotation RMSE

roll RMSE
pitch RMSE

gravity direction RMSE

maximum instantaneous attitude error

maximum instantaneous position error

tracking failure

divergence

trajectory completed frame count
```

如果能获得 IMU motion statistics，再增加：

```text
high angular velocity interval error

high acceleration interval error

error growth rate during aggressive motion
```

---

# 33. 需要分析 Gravity Factor 是否真的产生作用

如果：

```text
Baseline
和
Gravity Factor
```

结果几乎完全相同，不要立即把 sigma 从 10° 调到 1°。

先检查：

```text
factor 是否实际加入
每帧 residual 多大
Jacobian 是否非零
solver 中 parameter block 是否正确
factor weighted cost 占总 cost 比例
```

确认 factor 确实产生优化作用后，再调权重。

---

# 34. Double Counting 仍然是已知限制

LTV Observer 和 VINS-Fusion 共用：

```text
IMU
feature bearing
```

因此 gravity factor 并不是新的独立 measurement。

必须在 README 中保留说明：

```text
The LTV gravity factor is correlated with the original
VINS IMU/visual measurements.

Its covariance must not be interpreted as an independent
sensor covariance.

The factor is currently an engineering weak regularizer.
```

不要宣称：

```text
VINS + LTV 继承论文 GES
```

或者：

```text
新增了一个独立 gravity sensor
```

---

# 35. 当前不要修改 Passive LTV 参数

除非 gravity factor 调试明确发现问题，否则本阶段不要同时：

```text
调整 Q
调整 V
调整 P0
调整 feature lifecycle
调整 adaptive Euler
```

原因：

Passive LTV 当前已经有稳定基线：

```text
gravity direction RMSE = 2.69°
```

应该固定 observer，再研究 factor 的因果效果。

否则无法区分：

```text
observer tuning effect

vs

factor effect
```

---

# 36. 当前不要实现 Velocity Factor

即使 gravity factor 很快实现，也先停止。

不要顺手增加：

\[
r_v
=
R^\top V-\hat v_B
\]

原因：

当前：

\[
RMSE(v_B)=0.500\;m/s
\]

还需要分析其时间序列和 aggressive-motion interval 后再决定：

```text
是否加入
什么 gate
什么 sigma
是否只约束部分方向
```

因此：

```text
gravity-only
```

必须先独立完成消融。

---

# 37. 本阶段推荐修改文件

预计：

```text
vins/src/ltv/ltv_types.h

vins/src/estimator/estimator.h
vins/src/estimator/estimator.cpp

vins/src/factor/ltv_gravity_factor.h
vins/src/factor/ltv_gravity_factor.cpp

vins/test/test_ltv_gravity_factor.cpp
vins/test/test_ltv_snapshot.cpp

config/euroc/euroc_stereo_imu_ltv_config.yaml

vins/src/ltv/README.md
```

如果 CMakeLists 需要注册 factor/test：

```text
CMakeLists.txt
```

对应修改。

不要为了此次任务重构其他目录。

---

# 38. 编译执行顺序

严格分阶段：

```text
STEP 7.1
增加 LtvSnapshot
→ build

STEP 7.2
接入 sliding-window snapshot
→ build

STEP 7.3
snapshot lifecycle unit test
→ test

STEP 7.4
实现 gravity residual
→ build

STEP 7.5
finite-difference Jacobian test
→ test

STEP 7.6
接入 optimization
→ build

STEP 7.7
V1_01 baseline regression

STEP 7.8
V1_01 passive LTV regression

STEP 7.9
V1_01 gravity-only

STEP 7.10
gravity sigma sweep

STEP 7.11
difficult-sequence evaluation
```

不要一次修改完再统一编译。

---

# 39. 本阶段验收条件

必须全部满足：

## Build

```text
colcon build passes
```

## Test

```text
all existing tests pass

snapshot lifecycle test passes

gravity residual test passes

analytic/numerical Jacobian test passes
```

## Baseline

```text
LTV disabled
```

结果与原 baseline 无明显差异。

## Passive

```text
LTV enabled
gravity factor disabled
```

继续保持当前 passive LTV 结果。

## Gravity Factor

```text
无 NaN
无 Inf
无新增 reset
optimizer 正常收敛
timestamp 无错位
```

## Architecture

原：

```text
IMU preintegration
projection factors
sliding-window state definitions
feature tracker
```

不修改。

---

# 40. 完成本阶段后停止

完成 gravity-only difficult sequence 测试后停止。

不要继续：

```text
velocity factor
marginalization
landmark factor
```

先提交报告。

Codex 输出格式：

```text
## Modified files

...

## Snapshot implementation

...

## Gravity residual

...

## Jacobian validation

analytic vs numerical:
...

## Build / test

...

## V1_01

Baseline:
...

Passive:
...

Gravity:
...

## Difficult sequences

MH_04:
...

MH_05:
...

V1_03:
...

V2_02:
...

V2_03:
...

## Sigma sweep

20 deg:
10 deg:
5 deg:
3 deg:

## Failure / divergence analysis

...

## Known limitations

...

## Recommendation

whether gravity factor is useful
and whether velocity-factor stage should begin
```

---

# 41. 最终数据流

本阶段最终架构必须保持：

```text
          camera0 bearing
                 │
                 ▼
          LTV Observer
                 ▲
                 │
        bias-corrected IMU
                 │
                 ▼
          eta_hat, v_hat
                 │
          frozen snapshot
                 │
                 ▼
       sliding-window frame
                 │
                 ▼
      LTV gravity direction
              factor
                 │
                 ▼
┌──────────────────────────────────┐
│       VINS-Fusion optimizer      │
│                                  │
│ original marginalization prior   │
│ original IMU factors             │
│ original reprojection factors    │
│                                  │
│ + weak LTV gravity factor        │
└──────────────────────────────────┘
```

当前阶段只允许：

```text
LTV → VINS
```

作为单向辅助约束。

---

# 42. 当前立即执行

现在从：

```text
STEP 7.1
```

开始。

先完成：

```text
LtvSnapshot
+
sliding-window snapshot management
+
unit test
```

确认 timestamp/index 生命周期正确后，再实现 gravity factor。

不要直接跳到 Ceres。