VINS Fusion LTV 项目当前按 8 个阶段推进。

整体上，你这个项目最清晰的技术主线可以压缩成：

$$
\boxed{
\text{复现 LTV}
\rightarrow
\text{验证 LTV}
\rightarrow
\text{Gravity 辅助姿态}
\rightarrow
\text{Velocity 辅助速度}
\rightarrow
\text{质量门控}
\rightarrow
\text{联合结构验证}
\rightarrow
\text{联合调参}
\rightarrow
\text{跨数据集泛化}
}
$$

截至 Stage 6b 正式实验后的状态为：

```text
Stage 1  Passive LTV observer              ✅
Stage 2  Gravity factor                    ✅ 工程完成
Stage 3  Velocity factor                   ✅ 工程完成
Stage 4  Gravity Quality Gate              ✅ 工程通过 / 效果未通过
Stage 5  Velocity measurement Oracle       ✅ 负结果；只保留 V_fixed
Stage 6  G_gate + V_fixed joint structure  ❌ 效果验收失败
Stage 6b G_gate + online V_gate safety     ❌ 工程通过 / 安全性失败
Stage 7  Joint tuning                      ⏸ 仅文档；未执行
Stage 8  UZH-FPV held-out generalization   ⏸ 仅文档；未执行
```

Stage 6 的失败来自 MH_01_easy Rotation RMSE 相对 B 退化 10.65%；随后独立 Stage 6b
用在线 Heuristic Velocity Gate 替代 V_fixed，工程验收通过，但 MH_01_easy Rotation
仍退化 14.21%，超过 3% 硬门槛。因此按预先规则停止，不执行 Stage 7；Stage 8 也只
保留未来执行协议。详细数字与责任边界见 `stage_test_result.md`。


# 第一阶段：Passive LTV Observer —— 先证明 LTV 本身能工作。
核心不是改善 VINS，而是把论文第一层 LTV 独立复现出来：

$$
(\text{IMU}+\text{bearing})
\rightarrow
\text{LTV}
\rightarrow
\hat v_B,\hat\eta,\hat{{}^Bp_i}
$$

这一阶段解决的是公式、坐标系、外参、feature 生命周期、Riccati 数值稳定性等基础问题。LTV 完全旁路运行，不改变 VINS 的预积分、视觉重投影和优化结果。你目前这一阶段已经基本完成，仓库 README 也明确保持了这种 passive one-way branch 结构。

最终效果是：你拥有了一个独立于 VINS 优化器的“第二套运动状态估计”，可以评价它的 gravity direction 和 body velocity 是否真的有信息价值。

# 第二阶段：LTV Gravity → VINS —— 先让 LTV 真正帮助一次 VINS。

先把每个相机时刻的 LTV 输出冻结成：

$$
\text{LtvSnapshot}_k
$$

并和 VINS 滑窗中的

$$
R_k,P_k,V_k
$$

严格一一对应。你最新代码已经开始做这件事。

然后只把 gravity direction 加进 Ceres：

$$
\hat\eta_k
\rightarrow
\text{gravity factor}
\rightarrow
R_k
$$

直观上就是：

> 原来 VINS 的姿态完全由 IMU preintegration + vision 决定，现在又多了一条由 LTV 给出的“重力方向参考”。

这个阶段主要想改善的是：

$$
\boxed{\text{roll/pitch 稳定性}}
$$

尤其是在快速转动、视觉质量下降、IMU propagation 误差开始积累的时候。

它不会直接改善 yaw，也不会直接拉位置。

所以第二阶段真正需要回答的问题只有一个：

> LTV gravity 能不能作为一个弱约束，让 VINS 在 difficult/aggressive motion 下更不容易姿态漂掉？

# 第三阶段：LTV Velocity → VINS —— 开始影响速度和位置稳定性。
如果 gravity-only 确认有效，再加入：

$$
r_v
=
R_k^\top V_k-\hat v_{B,k}.
$$

这时候 LTV 不再只是辅助姿态，而开始直接约束 VINS 的 velocity state：

$$
\hat v_B
\rightarrow
V_k.
$$

整体变成：

$$
\text{IMU + Vision}
\rightarrow
\begin{cases}
\text{VINS optimization}\\
\text{LTV observer}
\end{cases}
$$

然后 LTV 再给 VINS 两个弱反馈：

$$
\hat\eta\rightarrow R,
$$

$$
\hat v_B\rightarrow V.
$$

这个阶段希望得到的效果是：

$$
\boxed{
\text{姿态稳定}
+
\text{速度不容易爆炸}
}
$$

而速度稳定以后，位置误差增长也可能间接受到抑制。

这实际上才是你最初做这个项目的核心目标：不是让 easy sequence 的 ATE 从 2 cm 变成 1.8 cm，而是减少 difficult sequence 中：

$$
R\rightarrow V\rightarrow P
$$

这一整条误差链的失控。

最终一个窗口优化求解的是：
$$
\min_{X} \left[ \|r_{\text{prior}}\|^2 + \sum \|r_{\text{IMU}}\|^2 + \sum \|r_{\text{reproj}}\|^2 + \sum \|r_{g}\|^2 + \sum \|r_{v}\|^2 \right]
$$

两个 factor 对滑窗的作用是：

                  LTV snapshot k
                 /              \
          eta_hat_k             v_hat_B_k
              │                     │
              ▼                     ▼
       Gravity Factor         Velocity Factor
              │                     │
              ▼                     ▼
         para_Pose[k]      para_Pose[k]
                           para_SpeedBias[k]
               \                 /
                \               /
                 Ceres optimizer
                       │
                       ▼
             optimized R_k, V_k


# 第四阶段：从“能用”变成“可靠使用” —— Quality Gate + Marginalization。
前两个 factor 如果有效，还不能说明系统已经完整，因为 LTV 本身有好的时候，也有差的时候。

例如：

* feature 太少；
* bearing 几何退化；
* aggressive motion 太强；
* observer 刚 reset；
* innovation 突然很大。

这时应该让：

$$
\text{LTV quality}
\rightarrow
\text{factor weight / enable}
$$

也就是：

> LTV 可信时帮助 VINS，不可信时自动减弱或关闭。

随后才考虑把 LTV factor 正式加入 marginalization。这样它提供的信息不仅影响当前一次窗口优化，还能随着 VINS prior 向未来传播。

这一阶段完成后，系统才从：

> “实验性地往 Ceres 塞两个 factor”

变成：

> “一个真正集成进 VINS sliding-window estimator 的 LTV 辅助观测分支”。

# 第五阶段：系统验证 —— 证明它到底有没有价值。
这一阶段代码反而不是重点，重点是消融：

| 方案                            | Gravity | Velocity | 用途             |
| ----------------------------- | ------: | -------: | -------------- |
| Baseline                      |       × |        × | 基准             |
| Baseline + Gravity            |       ✓ |        × | 单独验证 \(g\) 的作用 |
| Baseline + Velocity           |       × |        ✓ | 单独验证 \(v\) 的作用 |
| Baseline + Gravity + Velocity |       ✓ |        ✓ | 看两者是否互补        |


然后重点看 MH_04、MH_05、V1_03、V2_03 这类 aggressive sequence。

最后你要得到的不是一句：

> “ATE 平均降低了多少。”

而应该是更完整的结论，例如：

> LTV gravity 能显著降低高角速度区间的 roll/pitch error；velocity factor 能降低随后 velocity error 的增长速度，因此部分原本发散的序列可以保持跟踪。

如果最后能得到这种结果，这个项目就已经成立了。

# 第六阶段属于可选研究扩展：更深的 LTV–VINS 耦合。
如果前五阶段结果已经证明 LTV 有价值，才值得研究：

$$
\hat{{}^Bp_i}
$$

也就是 LTV landmark state。

例如进一步构造：

$$
R_k^\top(P_i-p_k)
-
\hat{{}^Bp}_{i,k}
$$

这样的 landmark factor。

但这一步复杂度会明显上升，因为它会和 VINS 自己的 feature/landmark state 产生更强相关性，double counting 也更加严重。

所以我不会把这一阶段视为当前项目必须完成的部分。





从项目定位上，我建议最终也一直保持这个思路：**不是“用 LTV 替代 VINS-Fusion”，而是利用 LTV 具有不同状态结构和稳定性设计的估计结果，给成熟的 VINS-Fusion 滑窗优化增加辅助约束，重点增强 aggressive motion 下的稳定性。**这比试图把论文完整 nonlinear pose observer 搬进 VINS 更合理。
