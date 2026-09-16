# VINS-Fusion + LTV Stage 7 执行文档：联合参数调优

## 前置条件与范围

仅当 Stage 6 的结构、稳定性和 easy/medium 退化验收全部通过时执行。结构固定为
`G_gate + V_fixed`，不新增状态、factor、Oracle、learned gate、RL 或 marginalization
逻辑。EuRoC 11 条序列全部属于 tuning set；本阶段不声明 EuRoC 内部 held-out。

联合调优参数为 `sigma_g`、`sigma_v`、Gravity Gate thresholds 以及 gravity/velocity
Huber delta。Baseline 对每条序列只运行一次并冻结；所有候选复用相同 canonical cache、
evaluator 和单线程 replay。

## 固定离散域与确定性候选

```text
sigma_g_deg:                       [5, 10, 15, 20]
sigma_v_mps:                       [0.25, 0.5, 1.0, 2.0]
gravity_gate_min_features:         [12, 15, 18]
gravity_gate_max_eta_norm_error:   [0.10, 0.20, 0.30]
gravity_gate_max_normalized_innovation: [0.025, 0.05, 0.10]
gravity_huber_delta:               [1.0, 2.0, 3.0]
velocity_huber_delta:              [1.0, 2.0, 3.0]
```

候选生成规则必须写入 manifest：按上述列表顺序生成完整笛卡尔积，以规范 JSON
序列化参数后计算 SHA-256 并按 digest 字典序排序；固定取前 95 个，再无条件加入 Stage 6
当前参数作为第 96 个候选。这样无需随机状态也可在任何机器复现相同候选集。禁止看到
测试结果后改变离散域、候选数或次序。

## 两轮执行

第一轮筛选序列固定为：

```text
MH_01_easy  MH_03_medium  MH_04_difficult  V1_03_difficult  V2_03_difficult
```

对 96 个候选运行五条序列。先硬过滤：

- 任一运行失败、输入未完整消费、出现 solver/reset/NaN/Inf；
- MH_01 或 MH_03 的任一 ATE、Rotation、Velocity RMSE 相对 B 退化超过 3%；
- 任一候选的两类 factor 加入数为 0 或违反 Gravity Gate 不变量。

对剩余候选，以困难序列的稳健改善排序。每个主指标先计算相对 B 的改善率
`(B-candidate)/B`，在三条困难序列上取中位数；排序键依次为：三类主指标中最差的
中位改善、ATE 中位改善、Rotation 中位改善、Velocity 中位改善、参数 JSON。保留前
8 个候选进入第二轮。

第二轮让 8 个候选运行完整 EuRoC 11 序列。再次应用 solver/reset/NaN/Inf 和每条
easy/medium 主指标 3% 硬过滤，再用四条困难序列的同一稳健排序键选第一名。若全部被
过滤，Stage 7 失败，保持 Stage 6 参数，不得放宽门槛。

## 产物与最终冻结

每个 run 使用 partial/failed/final 事务目录；候选 manifest 记录参数、候选生成规则、
cache/GT/config SHA-256、代码 commit 和 evaluator 版本。输出至少包括：

- 96 个首轮候选的状态、硬过滤原因和排序分数；
- 8 个入围候选在全量 EuRoC 上的逐序列完整指标和诊断；
- 最终唯一参数集 `stage7_final_parameters.yaml` 及 SHA-256；
- Final LTV System 对 B 的全量 EuRoC 表，明确标记为 `tuning-set final score`。

最终参数一经发布即冻结。Stage 8 只能读取该文件，不得根据 UZH-FPV 结果回调任何参数。
