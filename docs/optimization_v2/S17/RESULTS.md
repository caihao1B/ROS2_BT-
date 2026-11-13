# S17 当前执行结果

更新时间：2026-11-13T09:00:24.943963+08:00。本页只引用实际输出；未执行的矩阵项保持 NOT_RUN。

目前尚无完整六簇×十种子正式成绩，不能声明 S17 PASS，也不能进入 S18。地图资格、组件测试或短程演示不替代正式矩阵。

## 已完成的独立工作

- A/B 16 个测站资格通过；连接路 11 个建图站与 4 个独立站完成，独立 RMSE 0.01947 m。
- 新连续 atlas/plan 已建立，原数据库保持不变；B1/B2/B3 离线路由在测量占据图上重新验证，名义工时约 1040.70 / 1277.15 / 1086.41 秒。
- 54 组离线分簇对照、18 个同输入配对已实际执行。A 各组合可行；B 基础形状中 METIS 时间闭环为 3/3 可行；B 凹口/含洞形状尚未达到所有名义工时资格。
- 实际 A2 重启演示曾从 1/22 恢复至 2/22，未重复派发已确认目标；该演示使用此前 S17 区域版，不能冒充新连续版完整簇。
- 定位跳变几何、栅格时间连接、路径后缀恢复、检查点、教学例已有实际组件检查；最新统一检查详见 component_contracts。

## 实际运行开发记录

下表全部使用开发种子，完整道路与短程候选必须按各例 spec/inspection_contract 区分，均不计入 60 次正式成绩。

| 案例 | 结果 | 仿真秒 | 位置 P95 m | 障碍接触采样数 | 原因 |
|---|---|---:|---:|---:|---|
| [pilot_connector_ackermann_20261113_082424](/home/ceihao/robot_project/optimization_v2/results/S17/pilot_connector_ackermann_20261113_082424/metrics17.json) | PARTIAL | 191.179 | 0.01973 | 0 | BLOCK_RETRY_EXHAUSTED |
| [pilot_connector_ackermann_20261113_084204](/home/ceihao/robot_project/optimization_v2/results/S17/pilot_connector_ackermann_20261113_084204/metrics17.json) | PARTIAL | 196.121 | 0.01958 | 0 | BLOCK_RETRY_EXHAUSTED |
| [pilot_connector_diff_20261112_224110](/home/ceihao/robot_project/optimization_v2/results/S17/pilot_connector_diff_20261112_224110/metrics17.json) | PARTIAL | 236.948 | 0.01094 | 0 | BLOCK_RETRY_EXHAUSTED |
| [pilot_connector_diff_20261112_224819](/home/ceihao/robot_project/optimization_v2/results/S17/pilot_connector_diff_20261112_224819/metrics17.json) | PARTIAL | 282.210 | 0.01314 | 0 | BLOCK_RETRY_EXHAUSTED |
| [pilot_connector_diff_20261112_225649](/home/ceihao/robot_project/optimization_v2/results/S17/pilot_connector_diff_20261112_225649/metrics17.json) | PARTIAL | 286.487 | 0.01223 | 0 | BLOCK_RETRY_EXHAUSTED |
| [pilot_connector_diff_20261112_230627](/home/ceihao/robot_project/optimization_v2/results/S17/pilot_connector_diff_20261112_230627/metrics17.json) | PARTIAL | 371.333 | 0.01297 | 0 | BLOCK_RETRY_EXHAUSTED |
| [pilot_connector_diff_20261112_232123](/home/ceihao/robot_project/optimization_v2/results/S17/pilot_connector_diff_20261112_232123/metrics17.json) | FAIL | 847.714 | 0.01782 | 1087 | BLOCK_RETRY_EXHAUSTED |
| [pilot_connector_diff_20261112_235044](/home/ceihao/robot_project/optimization_v2/results/S17/pilot_connector_diff_20261112_235044/metrics17.json) | PARTIAL | 234.275 | 0.02366 | 0 | BLOCK_RETRY_EXHAUSTED |
| [pilot_connector_diff_20261112_235814](/home/ceihao/robot_project/optimization_v2/results/S17/pilot_connector_diff_20261112_235814/metrics17.json) | PARTIAL | 295.680 | 0.02175 | 0 | PREDICTION_WAIT_EXHAUSTED |
| [pilot_connector_diff_20261113_000717](/home/ceihao/robot_project/optimization_v2/results/S17/pilot_connector_diff_20261113_000717/metrics17.json) | FAIL | 303.122 | 0.25497 | 5654 | BLOCK_RETRY_EXHAUSTED |
| [pilot_connector_diff_20261113_074135](/home/ceihao/robot_project/optimization_v2/results/S17/pilot_connector_diff_20261113_074135/metrics17.json) | PARTIAL | 247.987 | 0.02262 | 0 | BUDGET_STOP |
| [pilot_connector_diff_20261113_075202](/home/ceihao/robot_project/optimization_v2/results/S17/pilot_connector_diff_20261113_075202/metrics17.json) | FAIL | 0.000 | nan | 0 | RuntimeError("essential node exited during startup: [('near_field', 1)]") |
| [pilot_connector_diff_20261113_075315](/home/ceihao/robot_project/optimization_v2/results/S17/pilot_connector_diff_20261113_075315/metrics17.json) | PARTIAL | 179.217 | 0.02411 | 0 | BLOCK_RETRY_EXHAUSTED |
| [pilot_connector_diff_20261113_080315](/home/ceihao/robot_project/optimization_v2/results/S17/pilot_connector_diff_20261113_080315/metrics17.json) | PARTIAL | 248.204 | 0.02107 | 0 | BUDGET_STOP |
| [pilot_connector_diff_20261113_081237](/home/ceihao/robot_project/optimization_v2/results/S17/pilot_connector_diff_20261113_081237/metrics17.json) | PASS | 338.987 | 0.02320 | 0 | FEEDBACK_COMPLETE |

完整道路试验 `pilot_connector_diff_20261112_232123` 已记录车体/工具与 `ped_1` 的实际接触，因此是 FAIL。不能只报告它的定位与停车门限通过。其后短程基线和 IMM 等候候选为安全 PARTIAL，尚不能证明完整通行。预测软代价候选 `pilot_connector_diff_20261113_000717` 同样记录实际接触，位置 P95 约 0.255 m，已拒绝启用；正式默认预测/缓存不变。

## 有界链路开发检查

- [study_probe_fault_sensor_delay_20261113_002314](/home/ceihao/robot_project/optimization_v2/results/S17/study_probe_fault_sensor_delay_20261113_002314/metrics17.json)：实验结果 FAIL；任务完成 True，安全 True。 延迟帧 78，故障暴露核实 True，预期响应核实 False。
- [study_probe_fault_sensor_delay_20261113_002811](/home/ceihao/robot_project/optimization_v2/results/S17/study_probe_fault_sensor_delay_20261113_002811/metrics17.json)：实验结果 PASS；任务完成 True，安全 True。 延迟帧 84，故障暴露核实 True，预期响应核实 True。
- [study_probe_localization_visual_20261113_001738](/home/ceihao/robot_project/optimization_v2/results/S17/study_probe_localization_visual_20261113_001738/metrics17.json)：实验结果 PARTIAL；任务完成 False，安全 True。 双目输出 425 帧、有效相对增量 345 帧；定位 P95 0.00828 m。

上述试验都是开发种子，不计正式矩阵。首次延迟故障试验不能证明注入与停车的因果响应，未计通过；新实现要求连续行驶后注入，并以入队时间检查响应窗口。定位消融共用速度上限和降级策略，GNSS 缺失会触发较低速度，因此不能把误差差异只归因于新增传感器。

多次运行记录到 Gazebo server 收尾退出码 `-11`；退出码已公开，原因仍需诊断。“没有残留进程”和“工具已关闭”不等于所有进程正常退出。

## 动态交互故障定位

失败候选首次接触发生于 215.102 仿真秒。独立真值记录显示，接触前 0.5 秒窗口内车辆位置保持不变；最后一条底盘门控记录为零速度且已确认静止。原 S02 ScriptedMotion 插件沿指定周期轨迹持续施加行人速度，没有对机器人避让的行为。由这些记录推断，仅停车不足以避免该场景的接触；需要进一步验证提前选择等待位置、时空绕行以及具有交互反应的行人场景。此解释不豁免接触，原例仍是 FAIL；不能把行人删掉或降低接触标准来通过。

## 仍未满足

- 六簇各十次完整任务及对应有效作业/残余：NOT_RUN。
- 两种底盘完整连接路：尚未全部通过。
- 凹口/含洞物理场景的独立可执行资格及闭环抽样：尚未完成；离线任务多边形对照不能替代。
- 控制器、定位输入、预测/记忆、在线覆盖全部配对闭环：尚未完成。
- 十类规定故障的完整暴露与响应验收、当前版重启合同：尚未完成。

这些缺项是自动验收与实际通行证据的缺口，不是新增的人工批准门槛。修复后仍须执行正式矩阵及规定消融，不能把已有开发测试换名记入成绩。
