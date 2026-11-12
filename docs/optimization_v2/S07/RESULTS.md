# S07 本轮实测结果

运行 `test_20261109_221109_708279`：**PASS**。可以执行下一步 S08：S07 四项自动验收通过且 S06 有效；不自动执行。用户人工验收 NOT_CHECKED。

[交互报告](/home/ceihao/robot_project/optimization_v2/results/S07/test_20261109_221109_708279/index.html) · [全部指标与命令](/home/ceihao/robot_project/optimization_v2/results/S07/test_20261109_221109_708279/result.json)

- S07-C01：PASS
- S07-C02：PASS
- S07-C03：PASS
- S07-C04：PASS

| 案例 | 结论 | 首获许可 / s | XY RMSE / m | 航向 RMSE / ° | 首获定位后准入比例 |
|---|---|---:|---:|---:|---:|
| zone_a | PASS | 8.900 | 0.041 | 0.216 | 55.1% |
| connector | PASS | 10.590 | 0.815 | 0.557 | 85.5% |
| zone_b | PASS | 9.932 | 1.322 | 0.240 | 72.0% |
| restart_occlusion | PASS | 9.966 | 0.056 | 0.137 | 30.7% |
| wrong_initial | PASS | 36.131 | 0.046 | 0.166 | 69.0% |

重启后重新获得许可：11.018 仿真秒；遮挡解除后重新获得许可：5.029 仿真秒。
- restart：许可关闭后 0.170 s 停至原速度门槛；从故障触发计 0.275 s；故障前确实在运动=True。
- occlusion：许可关闭后 0.147 s 停至原速度门槛；从故障触发计 1.170 s；故障前确实在运动=True。

[最后交付复核](/home/ceihao/robot_project/optimization_v2/results/S07/test_20261109_221109_708279/final_validation.json)：实际 GUI 截图检查、版本拒绝公共入口、浏览器交互与仿真时间加权统计已复核。用户人工验收仍单独记录。

准入比例按首获可靠定位后的全部时段计算，包含故障；低于 100% 说明当前观测有间歇失效并触发停车，不宣称已达到连续商用可用率。上述精度使用冻结 S06 坐标原点，不做真值轨迹拟合。传感器匹配需粗略操作员位置先验，不能解释为无先验任意位置定位。初始姿态仅用于仿真器；独立真值只供离线评分。

地图 `647148fb-975d-4efb-9c8d-8e5b5b8e42a9` 的实际数据库、点云、占据图和粗地形由 manifest 内容哈希绑定；在线 LOCALIZATION 不保存长期地图。每例 runtime.json 记录真实进程命令、退出码、DB 前后哈希、重启事件；metrics.json 记录停车时间、动作准入和 TF 所有权。

版本拒绝共 11 项检查：[contracts.json](/home/ceihao/robot_project/optimization_v2/results/S07/test_20261109_221109_708279/contracts/contracts.json)。它们使用诊断契约 fixture；S12 真实覆盖计划尚未实现，需再做端到端绑定测试。

自动检查 PASS 与用户人工验收分开记录。地图之外的区域、任意初始航向、重遮挡、长期场景变化不在本次成功范围。粗地形只是观测近似，S09 尚未判定通行性。

阶段命令、种子 41、实际依赖版本、学习例和浏览器测试退出码均在回执中；各案例完整输入哈希必须和当前文件一致，才允许断点复用。原 S01–S06 源码与回执未改动。
