# Stage 7 — ATE 优先、预算受控的 LTV 联合调参

**执行对象：本地 Codex。目标仓库：`liuhestu/VINS-FUSION-LTV`。**

> 本阶段固定 `B + G_gate + V_gate` 的算法结构，用传统的小规模参数搜索降低 ATE RMSE。先复用已有缓存和结果，再少量重放；不对每组参数执行 55 次全套实验，不实现 RL、Oracle 或 LTV marginalization。
>
> 本文中的搜索范围、运行预算及排序规则是本次新提出的实验协议，不是已经测得的最优参数或论文理论阈值。

## 1. 与旧计划的关系

用户现已明确授权进入调参，并指定 **ATE RMSE 为主指标**。因此本文件作为 Stage 7 的新执行规范，替代旧 Stage 7 中以下执行约束：

| 旧 Stage 7 | 本轮 Stage 7 |
|---|---|
| 固定 `G_gate + V_fixed` | 固定 `G_gate + V_gate` |
| 96 个候选 × 5 序列，再 8 个候选 × 11 序列 | 对照可复用时最多 16 个候选；不可复用时按 80 次回放上限缩减 |
| ATE、Rotation、Velocity 同时作为 3% 否决项 | ATE 主导选参；Rotation、Velocity 完整保留为辅助诊断 |
| 旧 Stage 6 的失败状态阻止启动调参 | 保留历史失败结论，按用户新授权单独启动本轮协议 |

**不要把旧 Stage 6/6b 的失败记录改成通过。** 在新报告中说明研究目标和验收口径发生了前瞻性修订，不追溯美化历史结果。

输入依据：

- `docs/euroc_notune_results.md`：最新同口径、未调参对照。
- `config/euroc/euroc_stereo_imu_ltv_config.yaml`：配置模板。
- `vins/scripts/run_stage6_joint.py`：实际运行配置的覆盖逻辑。
- `vins/scripts/evaluate_euroc_final.py`：最新统一评估口径。
- 旧 Stage 7/8 文档：当前工作树中已删除，仅能从 Git 历史读取；本轮不恢复或修改这些删除。

截至 2026-09-30，工作区已有 `/home/he/output/ltv_stage5_minimal_fix/cache/` 下 11 条 EuRoC canonical v2 cache，以及 `/home/he/output/ltv_euroc_final_unified/` 下 11 序列 × 5 模式的正式结果和原始 manifest。当前 replay 为 `/home/he/vins_fusion_ltv_ws/build/vins/stage5_replay`，SHA-256 是 `779da8043e36a5bed374fe28985158bd8827e29180abde78886d2f9fc54a3464`；正式 55-run 批次记录的是 `3c7ede1ae43be48305f11d8a33b034750ed651ca5e8b8e6c25fe5320aaa4afed`。**两者不同，历史轨迹当前不能直接当作同 binary 的调参对照。**正式批次的 evaluator SHA 也不同于当前脚本；旧轨迹若可复用，指标仍须按 Stage 7 统一口径重算。执行当天重新核验这些 SHA，不能把此处记录当作永久状态。

当前 No-tune ATE 分组结果如下，用于理解起点，不代替原始 JSON/CSV：

| 方法 | Easy/Medium 相对 B | Difficult 相对 B |
|---|---:|---:|
| G_gate | -5.12% | -1.57% |
| G_gate + V_fixed | -4.01% | +0.47% |
| G_gate + V_gate | -2.74% | -1.90% |

这说明 **联合结构尚未全面优于 G-only**。调参需要同时保留 G-only 对照，不能只要求超过 B，就预设 Velocity 的增量贡献成立。

命名统一：`V_gate` 是 Stage 6b 的在线 heuristic gate，不是 Stage 5 的 GT Oracle。最新 notune 文档中的 “Orcal gate” 描述需要单独注明为命名错误；不改动其数值。

## 2. 本轮只调什么

目标结构始终为：

```text
LTV observer ON
Gravity factor ON + Gravity Quality Gate ON
Velocity factor ON + Velocity Quality Gate ON
Velocity Oracle OFF，mask path 为空
loop closure OFF
LTV factors 不加入 marginalization prior
```

第一轮只开放下面 6 个数值参数，其余全部固定。

| 实际 YAML key | 当前值 | 本轮允许的离散取值 |
|---|---:|---|
| `ltv_gravity_sigma_deg` | 10.0 | 5.0 / 10.0 / 20.0 |
| `ltv_velocity_sigma_mps` | 1.0 | 0.5 / 1.0 / 2.0 |
| `ltv_gravity_gate_max_eta_norm_error`（重力模长绝对误差，m/s²） | 0.20 | 0.10 / 0.20 / 0.40 |
| `ltv_gravity_gate_max_normalized_innovation` | 0.05 | 0.03 / 0.05 / 0.08 |
| `ltv_velocity_gate_max_normalized_innovation` | 0.03 | 0.02 / 0.03 / 0.05 |
| `ltv_velocity_gate_max_disagreement_mps` | 0.50 | 0.25 / 0.50 / 0.75 |

禁止执行这 6 个参数的完整笛卡尔积。

第一轮固定：

```yaml
ltv_gravity_huber_delta: 2.0
ltv_velocity_huber_delta: 2.0
ltv_gravity_gate_min_features: 15
ltv_velocity_gate_min_features: 25
ltv_gravity_gate_reset_cooldown_frames: 0
ltv_velocity_gate_reset_cooldown_frames: 10
```

同时固定 LTV 的 Q/V/P0、初始化、特征生命周期、子步积分、所有原始 VINS IMU 噪声参数、视觉参数、相机标定、时间偏移、滑窗长度和 Ceres 配置。

不要重写 residual。Gravity 继续使用现有的**单位重力方向残差**，sigma 由度转换为弧度；Velocity 继续使用 `(R_WB^T * V_W - v_LTV) / sigma_v`。不能把角度 sigma 套到未归一化的重力加速度残差上。

为什么先固定 Huber：当前调 sigma 已改变加权残差的尺度和进入 Huber 线性区的条件；第一轮同时搜索 Huber 会扩大耦合和运行量。记录加权残差进入 robust 区间的比例，只有它确实频繁生效且第一轮收益不足时，才另开小预算搜索 Huber。**本轮不自动扩参。**

## 3. 先修“参数覆盖”，否则调参可能根本不生效

当前 `run_stage6_joint.py` 中的 `FROZEN_SETTINGS` 会覆盖基础 YAML，并把 `sigma_g=10`、`sigma_v=1` 和 Gate 阈值重新写回固定值。

**只改 YAML，然后原样调用旧 runner，不足以保证候选参数生效。**

建立 Stage 7 独立的候选覆盖入口，例如新脚本 `run_stage7_tuning.py`，复用旧 helper，但保持旧 Stage 6 默认行为不变。配置合并顺序固定为：

```text
基础数据集配置
→ 现有 FROZEN_SETTINGS
→ Stage 7 白名单候选参数
→ 固定 mode 开关、Oracle OFF、唯一输出路径
→ effective_config.yaml
→ 送入 replay
```

可复用现有 `effective_config(..., frozen_settings=...)`，但应传入完整合并后的 settings，不能只传 6 个候选键而丢失频率/单线程等设置。

EuRoC 配置里的 `cam0_calib`、`cam1_calib` 是相对路径。送入 replay 的临时 YAML 要放在原配置目录，或将这些路径显式改成等价绝对路径；记录 replay **实际读取的临时文件及其 SHA**，正常退出和失败时都清理临时文件。旧 runner 把 partial 输出路径写入 `effective_config.yaml`，目录发布后该路径不再存在；Stage 7 应保留运行时原件供审计，另生成指向最终目录的可复用配置。缓存键计算时排除/规范化这两个输出路径，不能因 UUID 变化而把同一算法配置误判为新 trial。

现有 EuRoC runner 的有效设置是 `freq=20`、`multiple_thread=0`、`show_track=0`、`save_image=0`，与基础 YAML 中的值不完全相同。本轮保持**实际已验证的运行设置**，不要退回模板默认值。

每个 trial 必须保存并核对：

```text
requested_parameters.json
resolved_parameters.json
effective_config.yaml
replay 实际接收的配置路径
配置 SHA、replay binary SHA、源码版本及未提交 diff 标识
```

增加测试：请求 sigma_g=5 后最终配置必须是 5，而不是被覆盖回 10；未知 key、修改原始 VINS 噪声参数、开启 Oracle 等请求必须被拒绝。无候选覆盖时，生成配置与原 No-tune 配置除输出路径外一致。

## 4. 评价目标：位置精度优先，但不靠漏帧获得好成绩

### 4.1 排序分数

对序列 s，在相同有效评价时刻上计算：

\[
 d_s(\theta)=\frac{ATE_s(\theta)}{ATE_s(B)}-1.
\]

候选主分数固定为：

\[
 J(\theta)=\frac{1}{|D|}\sum_{s\in D}d_s(\theta).
\]

这是逐序列相对误差的等权平均，越低越好。每条序列一票，不把所有位置样本混在一起，也不只优化绝对误差最大的 MH_04。

建议首轮晋级条件：工程检查通过，并且已完成的筛选序列中 `max(d_s) <= 0.03`。这只是**本轮候选筛选门槛**，不是统计显著性或安全保证。若没有候选满足，报告结果并保留原参数，不临时放宽到挑出赢家。

满足晋级条件后按以下固定顺序排序：

```text
1. J 更低
2. 最坏序列 d_s 更小
3. ATE P95 相对退化更小
4. 更接近原参数、参数 JSON 字典序
```

Rotation / Velocity 不再套用旧的逐序列 +3% 否决规则，也不进入 J。仍输出其数值和明显退化，不能因 ATE 有利而删除。NaN、求解失败、轨迹不完整等工程问题始终是硬失败。ATE P95/max 恶化超过 10% 时加显著警告、检查误差时序；不要仅用较好的 RMSE 掩盖尾部问题。

### 4.2 评估口径

复用 `evaluate_euroc_final.py` 的数学计算：

- ATE：同一批评价时刻，每个模式各自做刚体 SE(3) 位置对齐，**不估 scale**。
- Rotation / Velocity：维持当前文件约定的共同 B 对齐旋转，作为辅助结果；不得通过更换对齐规则提高分数。
- 官方 GT 文件、单位、时间容差与坐标约定保持一致。

当前脚本直接从官方 CSV reader 读取 velocity 列。本轮报告记录真实 `velocity_gt_source`；不得未经代码核查写成“本轮 evaluator 用位置差分生成速度”。ATE 优先是用户选择的研究目标，不需要以否定其余 GT 为理由。

当前 `evaluate_euroc_final.py` 的 `trajectory_metrics` 只输出 ATE、Rotation、Velocity 的 RMSE，且 `common_time_support` 和 `load_run` 固定要求五种模式。Stage 7 评估入口须从相同匹配样本计算位置误差 P95/max，并支持 B、G-only、W0 与少量候选的实际集合；不得把旧 evaluator 已有 P95/max 或可变模式接口写进运行记录。

当前跨模式/GT 容差分别为 1 ms / 20 ms，共同覆盖率门槛为 99%，首尾跨度差门槛为 0.1 s。本轮不通过放宽这些参数提高通过率。

不要求不同候选输出行数或所有内部 snapshot 完全相同，但必须消费同一 canonical 输入。保存每个序列固定的 B/GT 评价锚点；候选缺失时报告缺失位置及覆盖率，在该候选与 B 的共同有效时刻**同时重算 B 和候选**的 RMSE、P95/max，再计算 `d_s`，不可拿旧五模式表中的 B 数值与新支持集上的候选数值相除。**不得累计对所有 trial 求交集，逐步把难帧删光。**最后比较入围候选时，统一在两名入围者与 B 的同一共同评价集合重算后再选参数。大段缺失、覆盖率不足或异常 reset 的候选不能依靠剩余易帧获胜。

## 5. 数据分工与缓存复用

全部 EuRoC 已被反复查看，因此本轮按**开发/调参数据**处理，不能重新划一部分就宣称从未看过的 held-out。

| 用途 | 序列 | 说明 |
|---|---|---|
| 首轮筛选 D0 | V1_01_easy、MH_04_difficult、V2_03_difficult | 正常场景、可能受损场景、已有收益场景兼顾 |
| 入围验证 D1 | V1_02_medium、MH_03_medium、V2_02_medium | 检查风险序列及是否损失 G-only 的收益 |
| 参数冻结后的剩余确认 | MH_01_easy、MH_02_easy、MH_05_difficult、V1_03_difficult、V2_01_easy | 只作全量确认，不用于不断反调 |

缓存策略：

- canonical sensor cache 使用现有 `/home/he/output/ltv_stage5_minimal_fix/cache/` 的 11 条 v2 序列，逐条核对 metadata 中的 pair/IMU/GT SHA 与文件；不反复解 bag、重新配对或修改输入。
- 正式 B、G-only、W0 的首选来源是 `/home/he/output/ltv_euroc_final_unified/<sequence>/<mode>/`；逐条核对 `batch_manifest.json`、`input_manifest.json`、`validation.json` 和原始轨迹。**Markdown 表不能替代原始结果。**
- 当前 replay SHA 与正式批次不同。只有恢复并验证与正式批次相同的可执行文件及其依赖、输入、标定和非 LTV 有效配置时，才可直接复用旧轨迹；否则以当前二进制重建所需 B、W0 和同参数 G-only 对照。单条代表序列轨迹相似只是诊断证据，不足以把不同 SHA 的所有历史结果认定为同条件。
- 每个候选只跑目标 `joint_v_gate`，不是跑五种模式。
- estimator trial 缓存键包含：序列、canonical/IMU/标定 SHA、replay SHA、规范化后的有效算法配置；指标缓存另加 evaluator SHA 和评价范围。同键且验收文件齐全的成功运行才跳过，失败目录不能复用；输出路径或 runner 展示代码不同不能导致重复 replay。
- runner 与表格代码变更不必自动使所有 estimator 结果失效；是否重跑由实际输入与 estimator 行为依赖决定，是否重算指标由 evaluator 依赖决定。依赖闭包不清楚时不冒用旧数据。

先对一个代表序列核对新旧 B、W0 的时间支撑、数值差异和求解停止条件；审计和预算冻结完成前不启动批量候选。不同二进制的结果不得混合计算 Δ。

## 6. 省时间的执行方式

直接复用 `vins/stage5Replay.cpp` 的离线 feeder。它按传感器时间戳驱动算法，本轮不使用 `ros2 bag play`、不添加按原始时间间隔 sleep，也不通过乘缩 timestamp 来“加速”。

当前 replay 仍会初始化 ROS context、创建 node 和注册 publisher；它不依赖实时 topic 投喂。**本轮不为调参重新改成另一套无 ROS 架构**，继续使用已经修复生命周期的路径。

默认串行 `jobs=1`，关闭非必要可视化。先测一个序列的 wall time，输出剩余预算估计，不承诺固定运行时长。原始 Ceres 时间预算/迭代数保持不变：计算资源争抢可能改变基于时间的停止时机，不能为了并行而降低某些候选的实际求解量。

如之后确需并行，最多先试 2 个独立进程、独立输出目录，并验证同候选串行/并行轨迹和停止统计一致。未验证前保持串行。

第一轮使用 D0 的完整短序列，不直接从激烈运动中段冷启动。需要片段筛选时只能另做已定义初始化/预热协议的实验；不能用冷启动短片段排名替代全序列结果。

可以用已有 CSV 扫描阈值对 coverage 的影响，检查单位错误、参数未生效和近重复候选。**CSV 扫描只能估计 reference 路径下的开关行为，不能计算候选的真实 ATE。**参数会改变 VINS 状态，继而可能影响 LTV 的 bias 输入和后续 gate，所有晋级候选仍需真实 replay。

## 7. 分批候选：对照可复用时最多 16 组

### 7.0 预检与预算冻结

先完成：版本/文件清单、当前有效参数、缓存可用性、B/No-tune 复用审计、覆盖参数测试、目标函数和数据分工。

输出 `stage7_plan.json`，声明：

```text
max_unique_candidate_configs = 16     # 上限，包含 W0；不可复用对照时降低
max_new_replays = 80                  # 含筛选、控制组、复测及失败运行
initial_jobs = 1
baseline_reuse = verified_only
objective = mean_sequence_relative_ate
oracle_enabled = false
marginalization_enabled = false
```

第一次执行不允许直接启动 80 个运行；按以下批次推进，每批都有可独立读取的结果。

**预算分支在首批候选之前冻结并写入 `stage7_plan.json`：**

| 对照审计结果 | 首筛候选上限 | 保留的后续预算 |
|---|---:|---|
| 全部 B/W0 可复用 | W0 + 最多 15 个新候选（45 次 D0） | D1 最多 6、最终剩余 5、同参数 G-only 最多 11、复测 3，另留 10 次供缺失对照或失败运行 |
| 当前 SHA 不同且对照不可复用 | W0 + 最多 11 个新候选（33 次 D0） | 新 B 11、新 W0 11、同参数 G-only 11、D1 最多 6、最终剩余 5、复测 3，合计 80 次 |

第二行预留已覆盖全部 11 序列上的 B/W0，因而 D0 预检和 W0 回放包含在这 22 次内。若预检、失败或补缺已耗费额外次数，就在启动下一候选前相应扣减剩余 D0 名额；不得挪用 D1、最终确认或同参数控制的保留次数后仍声称完成全流程。若预算不足以完成规定对照和确认，停止并把阶段标为未完成，而不是扩大 80 次上限。

### 7.1 权重搜索：最多 6 组

Gate 参数固定当前值。候选先为：

| ID | sigma_g (deg) | sigma_v (m/s) |
|---|---:|---:|
| W0 | 10 | 1.0 |
| W1 | 5 | 1.0 |
| W2 | 20 | 1.0 |
| W3 | 10 | 0.5 |
| W4 | 10 | 2.0 |

W0 是当前 No-tune Joint，优先复用。其他候选各跑 D0 三序列。

随后增加 W5：将本批 G 轴和 V 轴各自排名最好的取值组合起来，做一次真正的联合 replay。如果组合重复已有候选就跳过，**不能把两个单参数收益相加当成联合收益**。

从本批选择满足晋级条件、J 最低的配置 W*。若没有新配置的 J 严格低于同口径 W0，就以 W0 进入下一批；不要因为权重搜索未赢就大范围扩大 sigma。差异大小和复测结果在报告中单列，不把一次较小差异称作统计显著。

### 7.2 Gate 阈值搜索：最多 8 组

固定 W* 的两个 sigma，其他值先仍为原始阈值。对以下四个维度分别测试两个替代值，一次只变一个阈值：

```text
G eta_norm_error:      0.10、0.40  （原值 0.20）
G norm_innovation:    0.03、0.08  （原值 0.05）
V norm_innovation:    0.02、0.05  （原值 0.03）
V disagreement:      0.25、0.75  （原值 0.50）
```

仍然每组只跑 D0。选参只依据已冻结的分数，不依据“某条曲线看起来适合我们的故事”。每个候选记录分序列通过率、reason-mask 和 factor 数。

对照不可复用的 80 次预算分支中，权重批次最多占 5 个新候选，Gate 批次最多占 6 个新候选，跳过 7.3。先依据新 W0 在 D0 的 CSV 和现有 Gate 判定公式，估计八种单阈值替代对通过帧数的绝对改变；每个 Gate 维度各取改变较大的一侧，再从剩余四项中取改变最大的两项，平局按本节列出顺序。此扫描只决定有限预算下测试哪些阈值，候选效果与排名仍须来自真实 replay。若某项缺少计算所需的诊断列，则该项排在可计算项之后，再按本节顺序补足名额。W5 与已有配置重复时不补位；失败运行消耗预算时依次删去排在末尾、尚未开始的 Gate 候选，并记录原因。

Coverage 为 0% 或 100% 可能是合法门控结果，不直接判成数值/回放错误。应标记 `degenerate_gate` 并报告：这条序列实际退化成哪个子结构。旧 runner 中强制 `0 < coverage < 1`、每条必须加入非零 Velocity factor 的规则，不应直接用于本轮候选验证。

但不得取消真正的不变量：`factor_added` 必须满足当前模式、基础 eligibility 和对应 gate；Oracle 必须为 0。也不能把所有 V 都关闭的候选解释为“Velocity 带来改善”。

### 7.3 局部联合候选：最多 2 组

在上述结果基础上，组合两个最有利的单阈值修改，再组合所有表现有利的阈值修改，最多新增 2 个未测试配置。与已存在配置重复就跳过。

两组都进行真实 replay；联合后可能更差，禁止假设单阈值收益可叠加。保存父候选及生成理由。仅在对照可复用且预算允许时执行，总候选数最多 `6 + 8 + 2 = 16`。

### 7.4 入围验证：最多 2 个候选

从全部候选中选最多 2 个合格配置，分别补跑 D1 的 3 条序列，总计最多 6 次新增运行。用 D0+D1 全 6 条共同、同口径结果重新排序。

同时展示：

```text
B（本轮同 binary）
G-only（本轮同 binary；最终增量控制另见 7.5）
W0：Joint No-tune 参数（本轮同 binary）
Joint candidate A
Joint candidate B
```

只用 D0 获益、到 D1 明显退化的参数不能提升为最终参数。若联合调参始终不及 G-only，应如实记录，保留 G-only 的部署选项，而不是必须选出双分支获胜者。

### 7.5 冻结一个参数集，再补全 11 序列

选定唯一配置后冻结 JSON/YAML 和 SHA，再跑剩余 5 条序列；已完成的 6 条直接复用，不重跑。

为了判断 Velocity 的增量作用，应取得**同一 replay、相同最终 Gravity sigma 与 Gate 阈值的 G-only 控制组**；其 Velocity factor 关闭，其他设置相同。最多 11 次，已有经审计等价的配置可复用。当前 binary 不同的分支即使 Gravity sigma 未变化，也须重跑 G-only；仅对比历史 G-only 与新联合不能严格归因 Velocity。

对代表序列进行少量重复运行，确认获益不是运行波动；不从多次运行中只报最好一次。如果重复运行逐值相同，仅说明固定环境下可复现，不构成跨场景的统计显著性证明。

预算账本按表中分支逐次扣减，包含失败和重复运行，**累计不得超过 80 次新 replay**。每次启动前同时检查当前余额与后续必须完成的保留次数；不足则停止并报告已完成范围，不能为了填满候选表牺牲对照或确认。

## 8. 候选成功、失败和停止规则

工程通过不等于参数更优。分别输出：

- `engineering_pass`：正常退出、完整消费输入、无非有限值、无新增求解失败/异常 reset、日志及 gate 不变量正确。
- `screening_pass`：满足本轮 ATE 晋级条件。
- `ate_improved_vs_notune`：相对 No-tune Joint 是否真正降低 J。
- `incremental_value_vs_g_only`：相对对应 G-only 的增量效果，允许为负或仅部分序列为正。

有数值失败的候选标记失败并保留，不用它进入排名；共用数据或配置覆盖存在 bug 时暂停整个 batch，先修实验基础设施。结果不佳不能通过改输入、GT、alignment、时间容差或原始 VINS 参数弥补。

本轮选参不是保证“所有序列都会改善”。若只有总体与部分场景改善，逐序列失败仍原样展示。若没有配置稳健优于 No-tune，则结论为“本轮预算内未找到更优全局参数”，继续保留原参数，不能称之为调参成功。

最后 5 条确认序列若暴露新退化，报告为调参结果的限制；不要立即针对它们修改参数后仍声称一次冻结验证。需要下一轮时新开协议和预算。

## 9. 推荐最小代码修改

新增薄脚本和协议文件即可，不重构 estimator：

```text
vins/scripts/run_stage7_tuning.py        # 候选、预算、断点续跑、调用已验证 replay
vins/scripts/evaluate_stage7_tuning.py   # 同口径的可变模式数量评价/排名
config/tuning/stage7_protocol.json      # 范围、候选规则、目标、序列和预算
vins/test/test_stage7_tuning.py         # 参数覆盖/缓存/排名/失败状态测试
```

以上是**建议新增文件**，不是声称仓库已有这些接口。若现有工具可直接扩展则复用，不重复建设。

复用入口：

```text
run_stage6_joint.py::effective_config
run_stage6_joint.py 的 CSV/输入/日志校验 helper
run_stage5_velocity_oracle.py::replace_setting / sha256
现有 stage5_replay 可执行文件
现有 evaluator 的 reader、matching、rigid_alignment 和误差定义
```

`evaluate_euroc_final.py` 目前按 5 个模式组织，且只计算三类 RMSE。本轮需要支持 B+候选、B+两个入围候选等小集合，并从相同的逐帧位置误差算出 ATE P95/max；可以提取通用函数或新增参数化 wrapper。**不要伪造其他模式目录、把同一轨迹复制五份来骗过验证**。历史正式批次的 evaluator SHA 与当前脚本也不同，重算结果要记录使用的 evaluator SHA 和评价支持集。

Stage 7 专用的 coverage/非零 factor 检查调整应与旧验证器分离。旧 notune 五模式回归必须不变。

测试至少包括：候选覆盖生效、非法 key 拒绝、无覆盖保持原行为、输出目录 UUID 不改变 estimator 缓存键、重复 trial 跳过、失败目录不可复用、二进制 SHA 不同拒绝复用、两种预算分支不会超过 80 次、分组均值与 Δ 计算、0/100% gate 覆盖的合法处理、相同评价支持集上的 ATE RMSE/P95/max、ATE 缺帧不能获益、未知参数不能静默忽略。

## 10. 输出与表格

所有输出写入新的 `/home/he/output/ltv_stage7_tuning/<batch_id>/`，不得覆盖 notune 或历史 Stage 目录；当前该 Stage7 根目录尚不存在，首次运行时创建。单次运行使用唯一 partial 目录，通过后才原子发布；失败另存，进程间禁止共享输出文件。trial 内同时保存运行时有效配置和发布后路径有效的配置，二者的 SHA 分别记录；缓存键采用去掉输出路径后的算法配置 SHA。

至少生成：

```text
stage7_plan.json
candidate_registry.jsonl
trial_results.jsonl
candidate_ranking.csv
stage7_final_parameters.yaml            # 仅选定后生成
stage7_final_parameters.sha256
stage7_report.md
```

建议报告写入新的 `docs/euroc_tuned_results.md`。保留 `docs/euroc_notune_results.md`，不就地覆盖为调参后数据。

主表建议为：

| Sequence | B ATE | G-only 同条件 | Δ vs B | W0 同条件 | Δ vs B | Joint Tuned | Δ vs B | Δ vs W0 |
|---|---:|---:|---:|---:|---:|---:|---:|---:|

主表各模式须来自同一 replay binary、输入和非 LTV 配置，并在该序列列出模式的共同评价时刻重算；历史 `docs/euroc_notune_results.md` 的数值单列为背景，不能混入新表计算 Δ。另附相同最终 Gravity 参数的 G-only 控制与 Joint Tuned 的比较。Rotation、Velocity 继续有对应结果表，但不驱动选参。

表格末尾始终追加：`Easy/Medium Mean`、`Difficult Mean`，可另加 `All Mean`。定义严格区分：

\[
\bar E_{group}=\frac1N\sum_s E_s,\qquad
\Delta_{group}=100\left(\frac{\bar E_{method}}{\bar E_B}-1\right).
\]

**主表分组 Δ 是均值之比，不是逐序列百分比的平均。**搜索 J 是另一个明确命名的 `mean_relative_ate`，两者不可混写，避免此前均值/百分比不一致问题。

报告同时包括：绝对 ATE、相对变化、由 Stage7 evaluator 新算出的 P95/max、运行耗时、覆盖率、被淘汰原因、重复运行差异。若 runner 增加可靠的进程峰值内存测量，再报告该值及测量方法；当前正式 EuRoC manifest 未保存峰值内存，不得补造历史数值。保留效果相反的序列，不只汇报收益项。

## 11. UZH-FPV 放在哪里

考虑用户的回放成本，本轮默认**只用 EuRoC 调参，不自动加入 UZH-FPV**。

工作区已有 `/home/he/output/ltv_uzhfpv_notune/uzhfpv_notune_results.md` 及对应运行目录：16 条中 15 条有效，未调参 `G_gate+V_gate` 的有效序列 Global Macro ATE 相对 B 为 `+3.10%`。这批结果已经被查看，应标记为**已见外部参考**，不能称为严格未见的 held-out。该报告还说明姿态 GT 指标排除、速度指标不可用；未来 UZH 比较不可照搬 EuRoC 三类指标的可用性假设。

冻结最终 LTV 参数后若另行执行 UZH-FPV 比较，须使用数据集专属的官方标定、topic 和噪声配置，并与现有结果分开标记；不得依据这些已见 GT 结果逐序列挑选 Gate/sigma。旧 Stage 8 文档目前仅在 Git 历史中，其“未见测试”前提已经不符合工作区状态。

## 12. 现在交给 Codex 的第一轮任务

**本次先执行到 7.1 权重搜索完成，不自动启动所有后续批次。**

```text
1. 读取本地最新 notune、当前 Stage7 文档、Git 历史中的旧 Stage7/8、runner、evaluator 和正式批次 manifest，确认版本及 SHA。
2. 建立 Stage 7 协议，明确新 ATE-first 目标和 G_gate+V_gate 结构。
3. 实现候选白名单覆盖与断点续跑；修正 Stage7 专用验证器的误拦截。
4. 通过参数覆盖/无覆盖回归测试，核验 B、W0 与 G-only 的复用条件；当前 replay SHA 不同，若不能恢复同条件旧 binary 则选定重建对照分支。
5. 输出 dry-run：新 B/W0、权重候选、后续保留次数、目录、序列和时间预算，保证总账不超过 80。
6. 执行 D0 上最多 6 组权重候选；只有审计通过的同条件结果才能复用。
7. 汇报实际有效参数、ATE 排名、逐序列退化、耗时和剩余预算，停止。
```

权重批次正确且可复现后，再按本文件执行 Gate 阈值与最终确认。不要一次同时修改结构、GT、Ceres、IMU 噪声和 Gate 参数。

---

## 核验来源与版本记录

2026-09-30 核验时仓库 HEAD 为 `0ba6cd9b63ca3ee397e0285f9847f5a16508482a`。以下 blob SHA 是当前 HEAD 的源码/文档版本；可执行文件及历史运行的 SHA 另列，三者不能互代。执行当天仍须记录实际 commit、工作树状态与文件 SHA。

| 文件 | 核验时 blob SHA / 作用 |
|---|---|
| `docs/euroc_notune_results.md` | `0f12f958fc62bea9e2f74bd3cade4505ce6be789`；No-tune 对照 |
| `config/euroc/euroc_stereo_imu_ltv_config.yaml` | `57340d311fe76d4257c0a280f151a6dec2798ca5`；配置默认值 |
| `vins/scripts/run_stage6_joint.py` | `b186f95fb5f752e2f3c5753b99eb3fca225e7560`；`FROZEN_SETTINGS`、`effective_config`、mode 与 coverage 校验 |
| `vins/scripts/evaluate_euroc_final.py` | `439fdead81c4e57c16685cbc6ca30b89384ebd10`；匹配与对齐口径 |
| `vins/scripts/run_euroc_final_experiment.py` | `e794754083f8804b2fbdcf49b3e218bc92ccc9de`；旧 55-run 总控 |
| `vins/stage5Replay.cpp` | `8621b64c017d609977ea2d174781c1540f2134c1`；离线投喂与生命周期 |
| 旧 Stage 7 执行文档 | `0e4a45f1e84b14389a658c51cb0cc378f387f886`；当前工作树已删除，可用 `git show HEAD:docs/VINS-Fusion-LTV_Stage7_Joint_Tuning执行文档.md` 读取 |
| 旧 Stage 8 执行文档 | `b1dd2d727e9759b1149a996e1df0971d4d0bc14d`；当前工作树已删除，可用 `git show HEAD:docs/VINS-Fusion-LTV_Stage8_UZHFPV_Generalization执行文档.md` 读取 |

当前 `stage5_replay` SHA-256 为 `779da8043e36a5bed374fe28985158bd8827e29180abde78886d2f9fc54a3464`，正式 55-run 的 `batch_manifest.json` 记录为 `3c7ede1ae43be48305f11d8a33b034750ed651ca5e8b8e6c25fe5320aaa4afed`。当前 `evaluate_euroc_final.py` 文件 SHA-256 为 `170f27ed8493b1e940c48ee93b7239e7ac0ca117883692f98ec888bafb9a5966`，正式批次记录为 `c210fb6c0d8e781cec4ef086dc050243fd5aea20d290e724f3c96e6cdec8ce48`。

数值求解说明参考 Ceres 官方文档的 `Solver::Options::max_solver_time_in_seconds`、`max_num_iterations` 与 LossFunction scaling。这里不要求升级本地 Ceres，也不将外部文档中的默认值替换为项目配置。

官方说明：[Ceres Solver options](https://ceres-solver.readthedocs.io/latest/nnls_solving.html#solver-options)、[Ceres LossFunction scaling](https://ceres-solver.readthedocs.io/latest/nnls_modeling.html#lossfunction)。
