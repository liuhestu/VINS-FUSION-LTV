# VINS-Fusion + LTV Observer 第三阶段 Codex 执行文档：Velocity Factor

本文档是 Stage 3 的直接执行规范。目标是在不改变 VINS-Fusion 原有状态定义、IMU 预积分、视觉重投影和边缘化逻辑的前提下，将 Passive LTV 输出的 body-frame velocity 作为可关闭、可诊断的弱约束加入当前滑窗优化。

本阶段完成 velocity-only 三序列快速测试后必须停止并提交报告。不要直接进入 Quality Gate、RL、marginalization 或 landmark factor。

---

# 1. 当前状态与边界

```text
Stage 1 Passive LTV：通过
Stage 2 Gravity factor：初步通过，只适合作为条件性弱约束
Stage 3 Velocity factor：允许开始
```

Stage 3 必须复用已有的 `LtvSnapshotWindow`、timestamp gate、factor diagnostics 和 EuRoC evaluator，不建立第二套 snapshot、同步或日志系统。

本阶段禁止修改：

```text
IMU preintegration / IMU factor
projection factors / feature tracker
VINS state layout
LTV observer equations
marginalization
loop_fusion / global_fusion
```

本阶段不实现自动 Quality Gate、RL policy/action/reward、adaptive weight、landmark factor 或同帧双向反馈。Gravity factor 默认继续关闭，但 Gravity 诊断数据必须保留。

---

# 2. 论文与代码坐标系

论文运动学为：

\[
\dot p=Rv,
\qquad
\dot v=-\omega^\times v+R^Tg+a.
\]

因此论文的 `v` 是 body-frame linear velocity。当前 LTV `velocity_body` 对应这一状态；VINS `Vs[k]` 是 world-frame velocity。比较前必须转换：

\[
{}^Bv_{\mathrm{VINS},k}=R_{WB,k}^{T}V_{W,k}.
\]

禁止直接计算 `Vs[k] - snapshot.velocity_body`。

若最终结果异常且已排除代码、数据与评估问题，返回原论文重点核对式 (3b)、(3c)、(13)–(19)、`R` 的方向、body velocity 和 apparent acceleration 定义。

---

# 3. Velocity residual

实现：

\[
\boxed{
r_v=\frac{R_{WB,k}^{T}V_{W,k}-\hat v_{B,k}}{\sigma_v}
}
\]

新建：

```text
vins/src/factor/ltv_velocity_factor.h
vins/src/factor/ltv_velocity_factor.cpp
```

Factor 类型固定为：

```cpp
ceres::SizedCostFunction<3, 7, 9>
```

参数块顺序：

```text
parameters[0] = para_Pose[k]
                [Px, Py, Pz, Qx, Qy, Qz, Qw]

parameters[1] = para_SpeedBias[k]
                [Vx, Vy, Vz, Bax, Bay, Baz, Bgx, Bgy, Bgz]
```

不得创建新 velocity state，也不得用 LTV velocity 覆盖 `Vs[k]` 初值。

Factor 构造参数：

```cpp
const Eigen::Vector3d &velocity_body
double sigma_mps
```

构造时拒绝非有限 `velocity_body`、非有限或非正 `sigma_mps`。运行时 quaternion 或 world velocity 非有限时返回失败，不得输出 NaN。

提供不依赖 Ceres 内存布局的 `computeResidual(...)` 静态函数，供单元测试、optimizer 前后 cost 统计和未来 Quality Gate 复用。

---

# 4. Analytic Jacobians

当前 `PoseLocalParameterization` 使用右扰动：

\[
R(\delta\theta)=R\exp(\delta\theta^\times).
\]

令 `u = R^T V`，则：

\[
R(\delta\theta)^TV
\approx u+[u]_\times\delta\theta.
\]

因此：

\[
\frac{\partial r_v}{\partial\delta\theta}
=\frac{[R^TV]_\times}{\sigma_v},
\qquad
\frac{\partial r_v}{\partial V_W}
=\frac{R^T}{\sigma_v}.
\]

当前项目的 `PlusJacobian` 把前六个 ambient 列直接映射为 `[delta position, delta theta]`，所以 `3 x 7` pose Jacobian 必须为：

```text
columns 0..2 = zero
columns 3..5 = skew(R^T V) / sigma_v
column  6    = zero
```

`3 x 9` speed/bias Jacobian 必须为：

```text
columns 0..2 = R^T / sigma_v
columns 3..8 = zero
```

禁止套用标准 Ceres quaternion ambient derivative 覆盖本项目的自定义布局；velocity factor 不直接约束 accelerometer bias 或 gyroscope bias。

---

# 5. Jacobian 测试门槛

新增 `vins/test/test_ltv_velocity_factor.cpp`，至少验证：

1. `R^T V == v_hat_B` 时 residual 为零；
2. 单位旋转与已知 90° 旋转的坐标变换正确；
3. position 变化不影响 residual；
4. accel/gyro bias 变化不影响 residual；
5. 无效 velocity、quaternion 或 sigma 被拒绝；
6. pose rotation Jacobian 与 numerical Jacobian 一致；
7. world velocity Jacobian 与 numerical Jacobian 一致。

Finite difference 要求：

```text
random states >= 20
epsilon = 1e-6
pose perturbation遵循 PoseLocalParameterization::Plus
```

解析与数值 Jacobian 不一致时立即停止，不允许进入 EuRoC 回放。

---

# 6. 配置与弱约束

在 `LtvConfig`、参数读取和 EuRoC LTV YAML 中增加：

```yaml
ltv_enable_velocity_factor: 0
ltv_velocity_sigma_mps: 1.0
ltv_velocity_huber_delta: 2.0
```

默认必须关闭。`1.0 m/s` 的第一轮依据是现有 Passive 数据的 LTV–VINS body velocity difference RMSE：

```text
V1_01 ≈ 0.48 m/s
V2_02 ≈ 0.82 m/s
V2_03 ≈ 0.98 m/s
```

这只是保守工程权重，不代表 LTV velocity 是统计独立、标准差为 `1.0 m/s` 的传感器观测。LTV 和 VINS 共享 IMU、camera bearings 和部分 bias estimate，因此必须使用独立 enable flag、独立 Huber loss、默认关闭且不进入 marginalization。

第一轮不自动 sweep。若三序列显示稳定但权重明显不合适，只在 V1_01 依次测试 `2.0 / 1.0 / 0.5 m/s`，不得对 11 序列做参数笛卡尔积。

---

# 7. Eligibility gate

新增：

```cpp
bool Estimator::ltvVelocityFactorEligible(int index) const;
```

必须同时满足：

```text
USE_IMU
ltv_config.enable
ltv_config.enable_velocity_factor
0 <= index <= frame_count <= WINDOW_SIZE
snapshot.valid && snapshot.velocity_valid
snapshot.last_reset_reason == None
snapshot.observed_features >= ltv_min_features
snapshot.frame_timestamp 与 Headers[index] 有限且误差不超过阈值
snapshot.velocity_body finite
velocity sigma finite and > 0
```

Velocity eligibility 不依赖 `gravity_valid` 或 `ltv_enable_gravity_factor`。

Stage 3 不增加经验速度上限、协方差阈值或 innovation 阈值；这些属于 Quality Gate。非有限输入仍必须拒绝。

---

# 8. Ceres 接入

在 `Estimator::optimization()` 中按下列顺序接入：

```text
marginalization prior
original IMU factors
eligible gravity factors
eligible velocity factors
original projection factors
solve
```

每个合格窗口帧调用：

```cpp
problem.AddResidualBlock(
    velocity_factor,
    velocity_huber_loss,
    para_Pose[i],
    para_SpeedBias[i]);
```

Gravity 和 Velocity 使用各自的 loss object。记录 velocity factor count，以及未经过 robust loss 的 quadratic cost before/after solve。

本阶段禁止把 velocity factor 加进 `MARGIN_OLD` 或 `MARGIN_SECOND_NEW` residual construction。

---

# 9. Snapshot 与 CSV 诊断

在 `LtvSnapshot` 追加：

```text
vins_velocity_body                     Vector3d
velocity_factor_residual               Vector3d
velocity_factor_residual_norm          double
velocity_factor_weighted_residual_norm double
velocity_factor_added                  bool
```

记录 optimization 前冻结状态：

\[
v_{B,\mathrm{VINS}}=R^TV_W,
\qquad
e_v=v_{B,\mathrm{VINS}}-\hat v_B.
\]

即使 velocity factor 关闭，只要 velocity snapshot 有效，也要计算未加权 difference，供后续分析。

CSV 保留全部已有列，并在末尾追加：

```text
vins_velocity_body_x/y/z
velocity_factor_residual_x/y/z
velocity_factor_residual_norm
velocity_factor_weighted_residual_norm
velocity_factor_added
```

不得删除或重命名已有 Gravity angle、residual、weighted residual 和 added 字段。未来 Quality Gate/RL 应能从同一行取得 feature counts、healthy updates、innovation、covariance、substeps、reset、gravity disagreement、velocity disagreement 和 factor state；Stage 3 只记录，不作策略决策。

---

# 10. EuRoC velocity evaluator

扩展 `vins/scripts/evaluate_vins_euroc.py`：

- `vio.csv` 有 quaternion 后的 `Vx,Vy,Vz` 时输出 velocity metrics；旧 pose-only 文件继续可用。
- Vicon bag 没有直接 velocity，使用世界系 GT 位置在 `t ± 0.05 s` 的线性插值做中心差分：
  \[
  V_{GT}(t)=\frac{p(t+0.05)-p(t-0.05)}{0.1}.
  \]
- 只使用差分窗口完整落在 GT 范围内的样本。
- 用位置 SE(3) alignment 的 rotation 变换 VINS world velocity；translation 不作用于 velocity。
- 新增 `--velocity-difference-window`，默认 `0.1` 秒。
- 输出 `velocity_samples`、`velocity_rmse_mps`、`velocity_p95_error_mps` 和 `maximum_velocity_error_mps`。
- Leica position-only 序列不输出 velocity/orientation metrics。

用匀速合成轨迹和已知旋转轨迹验证中心差分及 velocity alignment。若新增 Python test，必须注册进 ament test，不能留下无人执行的测试文件。

---

# 11. 分级实验

第一轮只测试 velocity-only：

```yaml
ltv_enable: 1
ltv_enable_gravity_factor: 0
ltv_enable_velocity_factor: 1
ltv_velocity_sigma_mps: 1.0
```

Gravity factor 关闭，但 Gravity diagnostics 继续记录。已有 Stage 2 Passive 结果作为主参考。

快速开发集固定为：

```text
正常基准：V1_01_easy
姿态困难：V2_02_medium
综合困难：V2_03_difficult
```

执行规则：

```text
loop closure OFF
rosbag playback rate 1.0
每个候选配置、每个序列只跑 1 遍
先 V1_01，再 V2_02，再 V2_03
V1_01 明显失败时立即停止
```

11 序列和多次重复只属于最终统计验证。

---

# 12. 初步通过条件

Build/test：

```text
colcon build passes
all existing tests pass
velocity residual/Jacobian tests pass
evaluator synthetic checks pass
```

Runtime：

```text
三个序列完整运行
no NaN / Inf
no Ceres residual or Jacobian failure
no new abnormal reset relative to Passive
snapshot timestamp aligned
factor count and gate coverage reported
```

数据集自身在相同位置发生的 `timestamp_gap` 单独说明，不算 factor 新增 reset。

V1_01 相对 Passive：

```text
ATE degradation <= about 3%
rotation RMSE degradation <= about 3%
velocity RMSE degradation <= about 3%
```

V2_02 或 V2_03 至少一个在以下指标中有一项改善约 `3%` 以上，且不是单个尖峰：

```text
velocity RMSE / P95 / max
ATE growth
attitude peak
roll/pitch RMSE
stability
```

另一个困难序列的 ATE、rotation RMSE 和 velocity RMSE 不应明显恶化，第一轮以约 `3%` 为判断线。

有效 factor 覆盖率低于 `20%` 时标记 `insufficient factor coverage`，该序列不能单独用于宣称成功或失败。

---

# 13. 时间序列分析

最终报告除整段 RMSE 外，至少分析：

```text
velocity error norm vs time
LTV–VINS velocity disagreement vs time
gravity disagreement angle vs time
factor enabled vs time
feature count / innovation / covariance vs time
reset events
```

重点比较 aggressive motion 前、中、后，判断 factor 是降低真实速度误差、只改变全局对齐，还是在 LTV 错误时拉坏 VINS；同时检查速度改善是否降低后续位置误差增长。

---

# 14. Gravity + Velocity 条件性消融

只有 velocity-only 满足第一轮验收，才允许增加一次：

```yaml
ltv_enable_gravity_factor: 1
ltv_enable_velocity_factor: 1
```

仍只运行 V1_01、V2_02、V2_03 各一遍。该实验只判断两个相关弱约束是否互补，不将固定同时开启设为默认。

若 combined 相对 velocity-only 没有清晰收益，保持两个 factor 独立关闭并进入 Quality Gate 数据分析；不要通过继续加大权重强行制造改善。

---

# 15. Failure 排查顺序

1. snapshot index 和 frame timestamp；
2. `Vs[k]` 是否为 world-frame velocity；
3. `R_WB`、quaternion 顺序和 `R^T V`；
4. velocity 单位与符号；
5. PoseLocalParameterization 右扰动；
6. `3x7`、`3x9` Jacobian 内存布局；
7. sigma、Huber、factor count；
8. observer reset、feature coverage、时间同步；
9. 排除代码与数据问题后，返回论文原式。

禁止在 Jacobian 未通过时用调权重掩盖实现错误。

---

# 16. Git 提交策略

```text
commit 1: add ltv velocity factor
          factor source/header + residual/Jacobian tests

commit 2: connect ltv velocity factor
          estimator gate + config + CSV + limitation note

commit 3: add euroc velocity evaluation
          evaluator extension + synthetic test
```

实验输出不提交。提交前恢复：

```yaml
output_path: "~/output/"
ltv_enable_gravity_factor: 0
ltv_enable_velocity_factor: 0
```

---

# 17. Codex 执行顺序

```text
STEP 8.1  确认 paper/code velocity frame convention
STEP 8.2  实现 residual → build
STEP 8.3  实现 Jacobians → finite-difference tests
STEP 8.4  config + eligibility → build/test
STEP 8.5  current optimization only → build/test
STEP 8.6  snapshot/CSV → disabled-path regression
STEP 8.7  evaluator velocity metrics → synthetic test
STEP 8.8  V1_01 velocity-only
STEP 8.9  V2_02 velocity-only
STEP 8.10 V2_03 velocity-only
STEP 8.11 汇总 → stop and report
```

不得一次修改完所有代码后才统一验证。

---

# 18. 最终报告格式

```text
## Modified files and commits
## Velocity frame convention
## Velocity residual
## Analytic vs numerical Jacobians
## Eligibility and optimizer integration
## Gravity/Velocity diagnostic data
## Build and unit tests
## V1_01: Passive vs Velocity-only
## V2_02: Passive vs Velocity-only
## V2_03: Passive vs Velocity-only
## Factor coverage and resets
## Velocity time-series analysis
## Failure/divergence analysis
## Known limitations
## Recommendation
```

报告需要明确：Velocity factor 是否初步通过、是否值得做 Gravity+Velocity 消融，以及是否可以进入 Quality Gate。

---

# 19. 完成后停止

完成 velocity-only 三序列快速测试后停止，不自动继续：

```text
full 11-sequence evaluation
five-run statistics
Quality Gate implementation
RL training
marginalization
landmark factor
```

只有报告确认 Velocity factor 至少初步有效，才讨论下一阶段。

---

# 20. 当前立即执行

从 `STEP 8.1` 开始，首先确认：

```text
paper v is body-frame velocity
VINS Vs[k] is world-frame velocity
residual uses R_WB^T * Vs[k]
PoseLocalParameterization uses right perturbation
```

确认后创建 factor 和 Jacobian test，不要直接跳到 Ceres 接入。
