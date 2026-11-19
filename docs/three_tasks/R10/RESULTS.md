# R10 三项作业的实际结果

**PARTIAL。** 本轮在 1800 仿真秒的整簇任务上限内，A1 账本确认 22/56 个点，B1 逐段确认 25/47 段，工具实测确认 74.01 m²、残余 114.38 m²。11 月 15 日对照结果为 A1 28/56、B1 19/47。巡检和持续封堵沿用各自已封存的同版本记录。

|案例|任务结论|实际完成情况|仿真秒|
|---|---|---|---:|
|spray_full_A1|PARTIAL|22/56 点|944.30|
|spray_tool_jam|PASS|3/3 点（原簇 56 点）|94.11|
|mowing_full_B1|PARTIAL|25/47 个执行段（原参考 34 段）|1173.70|
|mowing_startup_perception_loss|PASS|5/5 个执行段（原参考 34 段）|280.16|
|inspection_full_connector|PASS|完成 88.48 m 原连接路|343.64|
|inspection_follow_stop_resume|PASS|完成跟随、停车与恢复|108.53|
|inspection_detour|PASS|完成观测绕行与回接|100.07|
|inspection_persistent_block|PASS|持续封堵，停车转人工|79.30|
|mowing_persistent_block|PASS|0/5 个执行段（持续封堵，按要求停车）|62.73|

点喷卡滞和割草启动感知中断均在新版本上重新运行。B1 的原始 34 段参考、原面积和原门限沿用，新获准的 47 个执行段有单独身份。路径数量、有效面积和停车分别校验。B1 本轮出现了短至 4 毫秒的恢复换挡区间，区间内缺少独立刀盘采样，该项仍未通过。

[11 月 19 日技术说明、回放和手工入口](../收尾复验_20261119.md) · [各例实际回执](../../../validation/R10/case_summary.json) · [原始版本与参考路径](SAFE_EXECUTION.md)
