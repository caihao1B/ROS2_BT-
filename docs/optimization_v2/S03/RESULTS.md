# S03 实测结果

运行 `test_20261109_143806_74637`：**PASS**。

**可以执行下一步 S04：两种底盘、模型参数与停止故障矩阵通过，S02 前置有效**。人工状态 NOT_CHECKED。

- S03-C01：PASS
- S03-C02：PASS
- S03-C03：PASS
- S03-C04：PASS

[交互报告](/home/ceihao/robot_project/optimization_v2/results/S03/test_20261109_143806_74637/index.html) · [本轮全部指标](/home/ceihao/robot_project/optimization_v2/results/S03/test_20261109_143806_74637/result.json)。本轮所有构建、教学和原停止门限回归命令见 commands.jsonl，各仿真案例中的 runtime.json 保存实际命令、域、PID、退出码和耗时。

## 本轮合格参数

| 底盘 | 平地转场 | 连接路 | 坡面 | 规划最小半径 | 1.2 m/s 停车时间/距离 |
|---|---|---|---|---|---|
| diff | 1.2 m/s | 0.8 m/s | 5°、0.4 m/s | 0 m | 1.542 s / 0.942 m |
| ackermann | 1.2 m/s | 0.8 m/s | 5°、0.4 m/s | 2.11 m | 1.522 s / 0.918 m |

动态对象在 S03 试验副本中停放在道路外，动态避障尚未认证。

交付检查：{"public_demos": "3/3 PASS", "owned_stop_and_mutex": "PASS", "browser_interaction": "PASS", "manual_status": "NOT_CHECKED"}

- diff_connector：PASS；cross_track_max_m=0.01564；endpoint_error_m=0.03998；min_footprint_road_margin_m=2.34223
- ackermann_connector：PASS；cross_track_max_m=0.00734；endpoint_error_m=0.05842；min_footprint_road_margin_m=1.64941
- diff_slope：PASS；cross_track_max_m=0.00009；endpoint_error_m=0.08214；max_pitch_deg=5.06716
- ackermann_slope：PASS；cross_track_max_m=0.00000；endpoint_error_m=0.08334；max_pitch_deg=5.00255
- diff_bench：PASS
- ackermann_bench：PASS
- diff_dock：PASS；cross_track_max_m=0.00000；endpoint_error_m=0.01750
- diff_command_loss：PASS
- diff_stale_timeout：PASS
- diff_cancel：PASS
- diff_actuator_loss：PASS
- diff_adapter_loss：PASS
- diff_feedback_loss：PASS
- diff_pause：PASS
- diff_reset：PASS
- diff_duplicate_owner：PASS
- ackermann_command_loss：PASS
- ackermann_stale_timeout：PASS
- ackermann_cancel：PASS
- ackermann_actuator_loss：PASS
- ackermann_adapter_loss：PASS
- ackermann_feedback_loss：PASS
- ackermann_pause：PASS
- ackermann_reset：PASS
- ackermann_duplicate_owner：PASS

开发试验发现：名义轮距直接驱动会产生接触模型偏差；S02 原坡面朝内绕序不能正确支持轮胎。已保留校准依据、修复坡面并重验 S02。最终资格只来自本轮通过的数据。5°以外坡面、更高速度、实机电机制动和工具作业效果不属于本次通过范围。
