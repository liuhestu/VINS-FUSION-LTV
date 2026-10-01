# Stage7 全量选参与系统交付

本次用户授权连续完成 Stage7，替代旧执行文档中“7.1 后停止”的范围限制。
EuRoC 是调参集；“最佳”仅指本轮预算内全 11 序列已验证的最佳合格配置。

协议：`config/tuning/stage7_protocol.json`。当前 SHA 与历史正式批次不同，
重建全 11 序列 B/W0，最多 5 个新权重候选与 4 个 Gate 候选。
Gate 每维选择 W0 D0 日志中预计改变通过帧数最多的一侧，平局按协议取值顺序。
扫描只分配候选，效果与排名来自完整真实回放。

最多两个合格候选补跑全部剩余序列；统一 B、W0、两名入围者支持集，
按 J、最坏序列相对 ATE、P95、参数距离与 JSON 字典序排名。
新候选逐序列 ATE 相对 B 不超过 +3%，且全量 J 严格优于 W0，才发布。
否则保留 W0，记录预算内未找到更优参数。最后做同一 Gravity 参数的 G-only 全量控制。

80 次预算包含失败和复测：22 对照 + 27 搜索 + 6 D1 + 10 剩余序列 +
11 G-only + 3 代表序列复测 + 1 发布验收。重复组合不补位、不重复回放。
不自动重试失败运行；必需对照或控制失败则标记未完成。
中断后的启动仍计费，未完成的 partial 不作为成功缓存。
成功缓存需匹配算法配置/输入/二进制/动态依赖/标定，并复核文件 SHA 和工程校验。

```bash
source /opt/ros/humble/setup.bash
source /home/he/vins_fusion_ltv_ws/install/setup.bash
python3 vins/scripts/run_stage7_tuning.py \
  --output-root /home/he/output/ltv_stage7_tuning/all11_20261001 --dry-run
python3 vins/scripts/run_stage7_tuning.py \
  --output-root /home/he/output/ltv_stage7_tuning/all11_20261001
```

同一输出目录可续跑。不同实验用新目录，不改历史 notune。
runner 串行，保留既有求解时间与迭代限制。
临时 YAML 放原标定目录，正常/失败均清理；运行时与发布 YAML 分别保留。
完整发布配置与相机标定在 batch 的 `release/`；参数文件与 SHA 在 batch 根目录。

```bash
ros2 launch vins stage7.launch.py \
  config:=/absolute/batch/release/euroc_stage7.yaml \
  output_dir:=/absolute/independent/run rviz:=true
```

launch 在独立输出目录生成有效 YAML 和标定，启动 vins_node 与可选 RViz。
在线 topic 按完整 YAML；离线 runner 使用相同算法参数与 feeder。
RViz/topic 验收需实际观察并单独记录，不能从离线 ATE 推断实时性能。
全部复测、失败、工程状态、排名、覆盖率、Gate 数和耗时保留在 batch 中。

工程验证包括 Stage7 参数覆盖/白名单/缓存/预算/共同匹配/指标测试以及既有
Stage6、统一 evaluator 回归。默认 Stage6 验证器仍要求非退化 Gate；Stage7 通过
显式 `strict_stage6=False` 允许 0%/100%，保留 mode、eligibility、gate 和 Oracle 不变量。
评价文件记录 evaluator/reader 依赖 SHA；estimator 缓存不因展示代码修改而失效。
重复运行的全部结果保留，不挑最好一次。求解器停止条件仍为完整 YAML 中的
`max_solver_time` 与 `max_num_iterations`；现有日志未提供完整逐次 Ceres 停止统计。
