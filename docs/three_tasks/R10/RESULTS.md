# R10–R12 本轮实际结果

|案例|结论|完成情况|仿真秒|
|---|---|---|---:|
|spray_full_A1|PARTIAL/FAIL|28/56 点|1402.73|
|spray_tool_jam|PASS|3/56 点|92.79|
|mowing_full_B1|PARTIAL/FAIL|19/47 执行段（原参考 34 段）|879.27|
|mowing_startup_perception_loss|PASS|5/47 执行段（原参考 34 段）|246.91|
|inspection_full_connector|PASS|COMPLETED|343.64|
|inspection_follow_stop_resume|PASS|COMPLETED|108.53|
|inspection_detour|PASS|COMPLETED|100.07|
|inspection_persistent_block|PASS|HUMAN_REQUIRED|79.3|
|mowing_persistent_block|PASS|0/34 执行段（原参考 34 段）|62.73|

阶段：R08 PASS, R09 PASS, R10 PARTIAL, R11 PARTIAL, R12 PARTIAL。

B1 使用用户批准的新安全执行路径版本；原 34 段参考路径仍为 PARTIAL，原作业面积不变。定位、停车、曲率、接触、工具反馈和时限门限未放宽。原地图、原任务及 R08/R09 历史回执保留。S17/S18 正式六簇标准未补齐；不运行六十次矩阵。

[修复技术与核心代码说明](../R10末端定位与分段换向修复_20261115.md) · [手工检查](MANUAL.md)

当前未完成项：

- spray_full_A1 failed current evidence checks
- mowing_full_B1 failed current evidence checks
- Full original A1 not completed; partial point demo does not replace it
- Full approved B1 execution not completed; partial swaths do not replace it
