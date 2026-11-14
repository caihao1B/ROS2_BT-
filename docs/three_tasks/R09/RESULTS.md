# R09 实测结果

**PARTIAL：暂不能进入 R10。** 最终准入以阶段回执哈希检查为准。原 S17/S18 保持 PARTIAL。

|案例|通过|抵达段/选择段/原 B1 段|反馈面积 m²|选择区域残余 m²|仿真 s|墙钟 s|定位 RMSE m|
|---|---|---|---|---|---|---|---|
|nominal|False|0/5/34|4.420|23.534|74.14|208.6|0.0535|

## nominal

会话：`mowing_20261114_131457_243866`。终态：`HUMAN_REQUIRED`；原因：`RuntimeError('ALTERNATE_ROUTE_REQUIRES_COVERAGE_RESEQUENCING_RETAIN_RESIDUAL')`。

实际倒车距离：0.000 m。原 B1 工作面积：195.994 m²，原分母不随本次选段缩小。

未完成选择段：`['B1:segment:0', 'B1:segment:1', 'B1:segment:2', 'B1:segment:3', 'B1:segment:4']`。非预期接触：`{}`。失败检查：`['selected_segments_traversed', 'two_work_strips', 'actual_reverse_travel', 'tools_off_during_turns']`。

规划查询次数 1。账本确认面积不超过隔离的实际工具作用面积，转场和倒车期间工具需物理停止。详细量见 `analysis_r09.json`、`coverage.geojson` 与 SQLite 账本。

组件案例取原 B1 前五段，包含两条工作条带及三段倒车连接；这不是完整 B1 通过。完整 B1 和完整 A1 由 R10 单列验收。路径段抵达不等于 95% 有效覆盖，保守工具足迹未确认的边缘仍是残余。

有界规划找到替代路径时仍须报告覆盖任务重排需求；未执行的替代路径不能计作绕行完成。两次规划没有可用候选也不能宣称全局物理无路。

手工命令见 MANUAL.md，算法与核心代码见 TECHNICAL.md、CODE.md。
