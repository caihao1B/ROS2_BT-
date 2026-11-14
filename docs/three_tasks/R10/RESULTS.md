# R10 修复与实际复验

**PARTIAL**。当前完整任务和专项故障的实测结果如下。

|案例|结果|仿真秒|实际完成与残余|终止原因|
|---|---|---:|---|---|
|spray_full_A1|未通过|1186.44|20/56 点（原 56）|TimeoutError('WAIT_DEADLINE')|
|spray_tool_jam|PASS|143.396|3/3 点（原 56）|COMPLETED|
|mowing_full_B1|未通过|327.93|6/34 段（原 34）；确认 26.489 m²；残余 161.903 m²|TimeoutError('ENDPOINT_RECOVERY_BUDGET')|
|mowing_startup_perception_loss|PASS|185.572|5/5 段（原 34）；确认 26.557 m²；残余 1.397 m²|COMPLETED|
|inspection_full_connector|PASS|343.642|原路径剩余 0.100 m|COMPLETED|
|inspection_follow_stop_resume|PASS|108.532|原路径剩余 73.584 m|COMPLETED|
|inspection_detour|PASS|100.074|原路径剩余 73.584 m|COMPLETED|
|inspection_persistent_block|PASS|79.298|原路径剩余 72.185 m|NO_VALIDATED_ROUTE_WITHIN_BOUNDED_SEARCH|
|mowing_persistent_block|PASS|62.728|0/5 段（原 34）；确认 2.598 m²；残余 25.356 m²|RuntimeError('ALTERNATE_ROUTE_REQUIRES_COVERAGE_RESEQUENCING_RETAIN_RESIDUAL')|

## 实现与原门限

- 全局修正采用独立 C++ 低通处理，原始 GNSS 创新、地图跳变、时间有效性和停止锁存先于滤波，精停门限保持原值。
- 割草 START_WORK 等待每次处理 BT 中断；取消优先于同次工作确认，确认车体和刀具停止后才依据新的 BT 指令恢复。同一条带的幂等键、残余和重试预算保留。
- 割草账本状态发布改用 STEADY_TIME，匹配工具服务器原 0.25 墙钟秒监督；仿真时间戳、反馈配对、事务和有效作业几何不变。
- 原地图、任务点和 34 段条带分母不变；六十次矩阵未执行，原 S17/S18 保持 PARTIAL。

独立 C++ 全局管理器探针：PASS；25 厘米 GNSS 冲突拒绝、20 厘米原始地图跳变触发停止锁存。它使用合成消息，只验证监督逻辑，不算真实传感器任务完成。

## 未完成项

- spray_full_A1 failed current evidence checks
- mowing_full_B1 failed current evidence checks
- Full original A1 not completed; partial point demo does not replace it
- Full original B1 traversal not completed; partial swaths do not replace it

## 当前完整 A1 的新增停止条件

本次完成 20/56 点，定位 RMSE 0.03538 米；1136.28 秒触发 `MAP_JUMP_REJECTED`，进入 `PLAN_REVALIDATION_REQUIRED`。前后有效地图观测相隔 3.80 秒，GNSS 当时仍通过独立监督。
近邻记录的二维重建表明修正角度变化接近原 0.035 弧度门限；缺少被拒绝候选的同步三维记录，不能把这次锁存完全归因于滤波滞后，也不能凭该近似证明滤波没有影响。下一步应记录精确候选与配对里程计，再验证观测恢复和生产计划重验；不直接清除锁存。
详细记录：`optimization_v2/three_tasks/results/R10/global_hold_diagnosis.json`。

## 当前完整 B1 的新增停止条件

当前版本完成 6/34 段，确认 26.489 m²、残余 161.903 m²；原 90 秒恢复预算耗尽后以 `HUMAN_REQUIRED` 停稳。首条连续恢复候选长度 1.895 米，但传感器定位显示最大偏离候选约 4.044 米。
这是已通过几何/代价检查的候选未被控制器正确执行，不是已证明无路可走。后续需要检查 MPPI 换向点处理、按挡位分段的实测停车边界及轨迹偏离监督。未放宽到达、曲率、整车边界或预算门限。

## START_WORK 当前版本实测

BT 中断后 0.564 仿真秒确认车体和刀具停止（原上限 2.0 秒）；恢复后完成 5 段，账本仅有原来的 2 个键，无不确定性。确认面积 26.557 m²、选择范围残余 1.397 m²。它是五段专项通过，不是完整 B1 通过。

## 账本时间基准复验

隔离 ROS 时钟探针：PASS。原版仿真定时发布与工具墙钟超时存在基准不一致；新版只切换心跳时钟。真实失败尝试、已确认量和不确定性保留在 `results/R10/repair_attempts.json`，该尝试不计作成功。

## 可视证据

本机诊断图：`optimization_v2/three_tasks/results/R10/repair_docking.png`、`repair_start_work.png`；GitHub 对应 `media/r10_repair_docking.png`、`media/r10_repair_start_work.png`。

图表只解释对应实测窗口，不能代替完整任务验收。

## 证据入口

- `spray_full_A1`：`optimization_v2/three_tasks/results/R12/spray_20261114_194849_404114`。
- `spray_tool_jam`：`optimization_v2/three_tasks/results/R12/spray_20261114_194217_549485`。
- `mowing_full_B1`：`optimization_v2/three_tasks/results/R12/mowing_20261114_205558_700516`。
- `mowing_startup_perception_loss`：`optimization_v2/three_tasks/results/R12/mowing_20261114_211518_676200`。
- `inspection_full_connector`：`optimization_v2/three_tasks/results/R08/inspection_20261114_122404_262658`。
- `inspection_follow_stop_resume`：`optimization_v2/three_tasks/results/R08/inspection_20261114_123912_581545`。
- `inspection_detour`：`optimization_v2/three_tasks/results/R08/inspection_20261114_124410_931009`。
- `inspection_persistent_block`：`optimization_v2/three_tasks/results/R08/inspection_20261114_124850_642479`。
- `mowing_persistent_block`：`optimization_v2/three_tasks/results/R09/mowing_20261114_140747_354583`。

上一轮定位抖动和启动超时诊断仍保留为历史输入；当前故障原因以本表对应新运行的事件、回执为准。学习说明见 TECHNICAL.md、CODE.md；启动和退出见 MANUAL.md。
