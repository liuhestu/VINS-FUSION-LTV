# VINS-Fusion + LTV Stage 5 执行文档：冻结 Velocity Oracle Mask

## 实验目标

Stage 5 只验证：按 Passive reference 上的**瞬时 body-velocity 误差**选择性加入现有
LTV velocity factor，是否能显示明显增益潜力。这里的 Oracle 是离线
measurement-accuracy hindsight gate，不是在线或可部署算法；GT 不进入正式 Oracle
run、residual、状态量、observer 或 marginalization。

该判据不是最终优化收益的理论 upper bound。滑窗中的 factor 会耦合 pose、velocity、
bias 和后续线性化点，因此“该时刻 LTV velocity 更接近 GT”不保证“加入 factor 后的
整条轨迹更好”。本实验能否定或支持的是当前 frozen measurement-accuracy gate，不能
单独否定所有可能基于最终优化收益定义的 hindsight/counterfactual gate。

参考运行中定义：

\[
v_{B,GT}=R_{WB,GT}^{T}V_{W,GT},\quad
e_{LTV}=\|\hat v_{B,LTV}-v_{B,GT}\|,
\]

\[
e_{VINS}=\|v_{B,VINS}-v_{B,GT}\|.
\]

冻结 mask 的两列为：

```text
oracle_0:   reference snapshot valid && e_LTV < e_VINS
oracle_002: reference snapshot valid && e_LTV + 0.02 < e_VINS
```

文中对正式模式采用语义化名称；括号内为 runner 保持兼容的内部 mode ID：

```text
V_oracle_any_advantage  (v_oracle_0):   A = e_VINS - e_LTV > 0
V_oracle_margin_002     (v_oracle_002): A > 0.02 m/s
```

## 正确实验模型

```text
canonical sensor input
        |
        v
Passive reference ── official GT
        |
        v
frozen oracle_mask.csv + SHA-256
        |
        +── V_fixed
        +── V_oracle_any_advantage
        `── V_oracle_margin_002
```

四种参考（Passive reference、V_fixed 与两个 Oracle）必须共享 canonical stereo
pair SHA、IMU SHA、base config SHA 和 GT SHA；
两个 Oracle 模式还必须共享 mask SHA。只要求输入日程相同，不要求 snapshot 数、轨迹行
数或 reset 时刻完全相同。

## 时序问题复盘与正确验收语义

Stage 5 必须固定的是 canonical sensor input schedule，而不是各模式的内部 estimator
轨迹。任一模式首次加入 factor 后，都可能改变后续的 pose、velocity、bias、LTV snapshot
validity 和 reset 时刻；这是相同输入下正常的算法响应，不能靠补帧、复制 snapshot 或修改
reset 来消除。

因此验收规则为：

- `consumed_pair_count == canonical_pair_count`；
- 每个模式自己的 `vio.csv`、`ltv_debug.csv` 可解析、timestamp 严格递增、无 NaN/Inf，
  且位于输入时间范围；
- `vio.csv` 行数、snapshot 数和不同模式之间的 timestamp grid 不要求相同；
- Oracle mask 锚定 Passive reference timestamp。Oracle run 的 timestamp 不在 mask 时
  fail-closed，不加 factor 并记录 mask miss；不得 nearest、插值或按当前运行状态补判定。

V2_03 的左右目数量不对称是独立的数据事实，不是算法时序不确定性：ASL 原始数据为
1,922 left、2,336 right，按 runtime 同步语义得到 1,921 pair、drop left/right 为 1/415。
在判定数据问题前必须审计 ASL 与 ROS2 bag 的 topic、顺序、重复、单调性、交集和首末
timestamp；审计结果见 `docs/stage5_v2_03_timestamp_audit.md`。

## 数据审计与 canonical input

V2_03 在生成 cache 前必须先运行 `audit_stage5_timestamps.py`。审计报告保存在
`docs/stage5_v2_03_timestamp_audit.md`。只有确认额外右目图像来自原始 ASL，而非 bag
转换、错误 topic、重复消息或 cache bug 后，才允许继续。

ROS runtime、cache pairer 和 replay 校验共同使用
`utility/stereo_synchronizer.h` 中的 3 ms 规则：

```text
|left-right| <= 3 ms -> pair
left earlier             -> drop left
right earlier            -> drop right
```

cache 中唯一正式图像输入清单为 `canonical_stereo_pairs.csv`，包含 pair timestamp、
左右 timestamp、来源 index 和无损 PNG 路径。`metadata.json` 必须分别记录：

```text
left_input_count / right_input_count / paired_count
dropped_left_count / dropped_right_count
left_coverage / right_coverage
pair_list_sha256 / imu_sha256 / ground_truth_sha256
```

## 冻结 mask 与运行时语义

先用 `reference` 模式生成 passive LTV CSV，再运行：

```bash
python3 vins/scripts/build_stage5_oracle_mask.py \
  --reference-ltv <reference>/ltv_debug.csv \
  --ground-truth <ASL>/mav0/state_groundtruth_estimate0/data.csv \
  --output <sequence>/oracle_mask.csv
```

mask 生成器可以在 reference 后处理中按 5 ms 容差匹配 GT。生成后记录 SHA 并冻结。
正式 Oracle run 只读取：

```yaml
ltv_enable_velocity_oracle_gate: 1
ltv_velocity_oracle_mask_path: "/absolute/path/oracle_mask.csv"
ltv_velocity_oracle_mask_column: "oracle_0"  # 或 oracle_002
```

运行时把 `snapshot.frame_timestamp` 转成 ns 后精确查表；禁止 nearest、插值、实时读
GT 或按当前模式重新计算 advantage。mask 缺失、timestamp miss、base eligibility 不成立
时全部 fail-closed。

## 模式

| 模式 | Velocity factor | Gate | 说明 |
|---|---:|---|---|
| Passive reference | OFF | 无 | 生成冻结 mask，同时作无 factor 参考 |
| V_fixed | ON | 无 | 固定开启对照 |
| V_oracle_any_advantage (`v_oracle_0`) | ON | frozen `oracle_0` | 主实验；只要求 LTV 瞬时误差更小 |
| V_oracle_margin_002 (`v_oracle_002`) | ON | frozen `oracle_002` | 敏感性实验；要求至少 `0.02 m/s` 优势 |

Gravity factor 和 loop closure 全部关闭。默认配置中 Velocity factor 与 Oracle Gate 仍
保持关闭。

## 回放与事务化输出

`stage5_replay` 强制单线程 estimator，逐一消费 canonical pairs，并单独记录
`canonical_pair_count` 与 `consumed_pair_count`。退出顺序固定为：

```text
drain -> Estimator/CSV 析构 -> unregisterPub -> node.reset -> rclcpp::shutdown
```

`run_stage5_velocity_oracle.py` 先写 `<mode>.partial-<uuid>`，正常退出并通过以下验证后
才原子 rename 为 `<mode>`；失败目录保留为 `<mode>.failed-<timestamp>-<uuid>`，不得
覆盖正式结果：

- consumed pair count 等于 canonical pair count；
- `vio.csv` 与 `ltv_debug.csv` 可解析、时间戳严格递增且无 NaN/Inf；
- 无 unusable solver、SIGSEGV 或 DDS 销毁错误；
- canonical pair、IMU、base config、GT、mask SHA 在运行前后不变；
- `factor_added => base_eligible && mask_loaded && mask_hit && oracle_pass`。

不得要求 `vio.csv rows == canonical pairs`。

## 执行顺序

1. 对 V2_03 完成 ASL/bag timestamp 审计；对每个序列使用共享 `StereoSynchronizer`
   生成一次 canonical pair list、IMU CSV 与 manifest。
2. 确认离线 replay 生命周期按 `Estimator/CSV 析构 -> unregisterPub -> node.reset ->
   rclcpp::shutdown` 退出，且无 DDS destruction error 或 SIGSEGV。
3. 以 Passive reference 运行 LTV、不加入 Velocity factor，生成 `ltv_debug.csv`。
4. 使用 Passive CSV + official GT 构建并冻结 `oracle_mask.csv`；记录 mask SHA。
5. 运行 V_fixed、V_oracle_any_advantage 和 V_oracle_margin_002；Oracle replay 只读
   frozen mask，不读取 GT。
6. 用 `verify_snapshot_timestamps.py` 进行每个输出自身的 timestamp 验证，并报告而非
   强制模式间 grid 是否一致；用官方 GT 计算指标。

先执行 `V1_01_easy`、`V1_03_difficult`、`V2_03_difficult` 作为工程门槛；通过后补齐
完整 EuRoC 11 序列。最终正式矩阵必须为 11 序列 × 4 模式，共 44 次回放。

`verify_snapshot_timestamps.py` 只验证每个模式自身的非零 timestamp 严格递增，并报告
模式间是否同 grid；不同 grid 是允许且需要报告的算法行为，不再作为时序失败。

## 指标与判定

每个序列报告 ATE、Rotation/Roll/Pitch RMSE、Velocity RMSE/P95/max、Oracle coverage、
正 advantage 分布、GT 最大匹配误差、factor coverage、mask hit/miss、reset、NaN/Inf
和 solver 状态。

- 若任一语义化 V_oracle 在至少两个困难序列上使 Velocity RMSE 相对 B 改善约 3% 以上，同时
  优于 V_fixed，且其他主要指标没有约 3% 以上退化，则值得继续研究不依赖 GT 的
  online gate。
- 若该 measurement-accuracy Oracle 仍没有跨困难序列的一致收益，则当前证据不支持
  沿此判据继续 Velocity Quality Gate、learned gate 和 RL Velocity 分支；这不是对
  所有优化收益型 Oracle 的普遍否定。
- V_oracle_margin_002 只判断小幅优势是否可能属于噪声，不替代
  V_oracle_any_advantage 的主结论。

## 范围禁止事项

本 Stage 5 不引入 Gravity factor、Gravity Quality Gate、Gravity + Velocity 联合、最终
2×2 ablation、marginalization 改动、RL、adaptive sigma 或联合调参；不得为追求模式间
snapshot 一致而补帧、复制图像或修改 reset 行为。

## 最终状态

全量 11 序列、44 次正式回放已经完成。measurement-accuracy Oracle 验证失败；最终
定性固定为瞬时 velocity accuracy 不适合作为非学习式 Velocity Gate 判据。停止手工
Velocity Quality Gate，后续联合结构只保留 `V_fixed`。完整结果见
`docs/stage_test_result.md`。
