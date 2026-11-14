# R10 三任务分别验收

**PARTIAL**。尚不能按完整验收通过 R10；按本轮授权，R11 材质与 R12 独立演示可以交付，不能替代本阶段缺项。

原 A1 56 点、连接路 88.481 米、B1 34 段的要求保留；其余簇与六十次正式矩阵未由本轮推定通过。原 S17/S18 保持 PARTIAL。

|案例|结果|仿真秒|终态|确认量及残余|
|---|---|---:|---|---|
|spray_full_A1|未通过|509.824|INCOMPLETE|点 8/56，原任务 56|
|spray_tool_jam|PASS|153.178|COMPLETED|点 3/3，原任务 56|
|mowing_full_B1|未通过|529.17|HUMAN_REQUIRED|段 12/34，原任务 34；反馈 49.960 m²，残余 138.432 m²|
|inspection_full_connector|PASS|343.642|COMPLETED|原路径剩余 0.100 m|
|inspection_follow_stop_resume|PASS|108.532|COMPLETED|原路径剩余 73.584 m|
|inspection_detour|PASS|100.074|COMPLETED|原路径剩余 73.584 m|
|inspection_persistent_block|PASS|79.298|HUMAN_REQUIRED|原路径剩余 72.185 m|
|mowing_persistent_block|PASS|62.728|HUMAN_REQUIRED|段 0/5，原任务 34；反馈 2.598 m²，残余 25.356 m²|

受阻案例通过表示物理停车、有界处理与残余保留正确，不表示工作已经完成。条带抵达和反馈确认面积分别统计；不降低原始覆盖分母，不把模拟电机工作等同于真实药效或切割效果。

## 未完成项

- spray_full_A1 failed current evidence checks
- mowing_full_B1 failed current evidence checks
- Full original A1 not completed; partial point demo does not replace it
- Full original B1 traversal not completed; partial swaths do not replace it

## 完整点喷本次停止的解释

已记录 8 笔工作，其中 8 笔满足原独立物理反馈核对条件。完整案例的 `feedback_and_independent_physics_agree` 同时要求记录数等于选中的 56 点，因此未执行点也会导致该总检查失败；不能把它直接解释成已经完成的工作全部存在反馈冲突。未确认点仍保留，完整任务仍为未通过。
最后精停检查：点 `point_9_16`，瞬时目标距离 0.0166 m，位置样本 P95 散布 0.0244 m。瞬时距离很小并不保证定位稳定；原准入门限与重试上限不降低。终止原因：`RuntimeError('DOCKING_REALIGNMENT_EXHAUSTED')`。

最后停靠窗口的独立事后诊断：全局估计 P95 散布 0.024381 m，局部里程计 0.000427 m，真值位置最大位移 0.000000 m。该窗口主要是全局修正抖动，并非车辆仍在运动；此结果没有反向参与在线决策。下一轮先处理全局停靠精度、局部静止与定位不确定性的区别，保留独立 GNSS 监督与工具反馈门限。

## 完整割草的下一轮修复点

工具重新启动时，521.49 秒感知失效，521.516 秒已关闭工具请求，但任务仍等待工作确认直到 529.13 秒。BT 的取消确认提前超时。根据日志与代码，应修复 START_WORK 等待阶段的取消协同，并专门复验开刀时的感知中断；该修复尚未实施。证据见 `mowing_start_diagnosis.json`，最终实测停车已确认。

## 原始证据

- `spray_full_A1`：`optimization_v2/three_tasks/results/R12/spray_20261114_170714_616253`；终止原因 `RuntimeError('DOCKING_REALIGNMENT_EXHAUSTED')`。
- `spray_tool_jam`：`optimization_v2/three_tasks/results/R12/spray_20261114_170025_915535`；终止原因 `None`。
- `mowing_full_B1`：`optimization_v2/three_tasks/results/R12/mowing_20261114_174644_953306`；终止原因 `TimeoutError('WAIT_DEADLINE')`。
- `inspection_full_connector`：`optimization_v2/three_tasks/results/R08/inspection_20261114_122404_262658`；终止原因 `None`。
- `inspection_follow_stop_resume`：`optimization_v2/three_tasks/results/R08/inspection_20261114_123912_581545`；终止原因 `None`。
- `inspection_detour`：`optimization_v2/three_tasks/results/R08/inspection_20261114_124410_931009`；终止原因 `None`。
- `inspection_persistent_block`：`optimization_v2/three_tasks/results/R08/inspection_20261114_124850_642479`；终止原因 `NO_VALIDATED_ROUTE_WITHIN_BOUNDED_SEARCH`。
- `mowing_persistent_block`：`optimization_v2/three_tasks/results/R09/mowing_20261114_140747_354583`；终止原因 `RuntimeError('ALTERNATE_ROUTE_REQUIRES_COVERAGE_RESEQUENCING_RETAIN_RESIDUAL')`。

`spray_full_A1` 结束快照：`{"bt_reason": "NEXT_OBLIGATION", "localization_reason": "MAP_AND_GNSS_AGREE", "gate_reasons": ["NAV_COMMAND_STALE"], "scope": "Last snapshot for context, not an independent root-cause conclusion"}`。取消后的命令过期等状态不自动作为故障根因；需与先前事件和停止原因一起检查。

当前完整 B1 运行记录 2 次 `ENDPOINT_RECOVERY_SUCCESS`，1 次观测恢复事件。恢复成功仍需与最终段完成数分别阅读；原候选续行被拒绝后，重新生成候选并到达，也记录为有界恢复成功。

`mowing_full_B1` 结束快照：`{"bt_reason": "CANCEL_OR_STOP_CONFIRMATION_TIMEOUT", "localization_reason": "GNSS_FIXED_AND_INNOVATION_VALID", "gate_reasons": ["NAV_COMMAND_STALE", "TOOL_STATE_FOR_SEGMENT", "OBSTRUCTED"], "scope": "Last snapshot for context, not an independent root-cause conclusion"}`。取消后的命令过期等状态不自动作为故障根因；需与先前事件和停止原因一起检查。

命令见 MANUAL.md；技术选择见 TECHNICAL.md；Python/C++ 调用链、运动学及工具互锁见 CODE.md。
