# S13 本轮实测结果

状态：**PASS**；人工：NOT_CHECKED。

运行目录：`/home/ceihao/robot_project/optimization_v2/results/S13/test_20261111_214031_1075382`

命令：`/home/ceihao/robot_project/optimization_v2/ws/src/navigation_adapter/scripts/stage13.py test --sim-seconds 600 --wall-timeout 3600 --suite acceptance --seed 41`

现实耗时：2358.0 s。工具 OFF，整簇作业未执行。

| 案例 | 实际动作完成 | 跟踪 P95 / 最大 m | 方向最大 rad | 停车最大 s | 计算 P95 ms | 物理/安全 | 时延目标 |
|---|---:|---:|---:|---:|---:|---|---|
| action_semantics | True | 0.026911575295751156 / 0.02732239347839555 | 0.02712365321428134 | 0.49500000000000455 | 26.4804551 | PASS | PASS |
| diff_DWB | True | 0.025683100649119835 / 0.026782406755865358 | 0.04692443269546498 | 0.22600000000000087 | 26.033455149999995 | PASS | PASS |
| diff_MPPI | True | 0.016484871330874156 / 0.016732630708963282 | 0.039063015209264584 | 0.21000000000000085 | 29.5188788 | PASS | PASS |
| diff_MPPI_repeat_2 | True | 0.015239939881959756 / 0.015709859773672615 | 0.03972842253269926 | 0.2220000000000013 | 28.378712799999995 | PASS | PASS |
| diff_MPPI_repeat_3 | True | 0.009763513293818603 / 0.00981412808260376 | 0.03721465703333404 | 0.23100000000000165 | 31.5187355 | PASS | PASS |
| ackermann_RPP | True | 0.022698517590773258 / 0.02292344798247757 | 0.05849967976861592 | 0.21999999999999886 | 0.3490275000000001 | PASS | PASS |
| ackermann_MPPI | True | 0.02976795798465094 / 0.030173553103630887 | 0.06127718151812278 | 0.19999999999999574 | 7.6086025 | PASS | PASS |
| localization_loss | True | 0.02136028179385063 / 0.0214658423660079 | 0.021405983649684934 | 0.4870000000000019 | 30.810319 | PASS | PASS |
| inspection_diff | True | 0.010651232446866027 / 0.012935292582743545 | 0.07183762650329939 | 0.22099999999999653 | 30.7857459 | PASS | PASS |
| inspection_ackermann | True | 0.0030899654626465813 / 0.0033908288866271582 | 0.01380106163272199 | 0.22500000000000142 | 0.5746597999999993 | PASS | PASS |

近距低障碍独立检查：`{"confirmed_obstacle_snapshots": 6, "completed": true, "max_stationary_displacement_m": 1.9902332469576888e-13, "obstacle_contacts": 0, "near_samples": 206, "near_points_median": 4013.0, "pair_delta_max_s": 0.0, "low_obstacle_frames": 29, "scored_frames": 29, "height_m": 0.08, "distance_from_base_m": 1.1, "source": "actual Gazebo optical returns with noise/dropout/delay; independent stationary fixture scoring", "database_unchanged": true, "shutdown": {"state": "STOPPED", "tool_on_requested": false}, "passed": true}`

检查：`{"S13-C01": "PASS", "S13-C02": "PASS", "S13-C03": "PASS", "S13-C04": "PASS"}`

各正式检查通过；人工尚未查看。

可以执行下一步 S14

`metrics.json` 包含资源峰值、动作退出码、停车事件与检查点；`runtime.json` 保存命令/PID/退出码/数据库不变性；独立物理轨迹在 `tracking.json`。本报告不把过程中的诊断失败隐藏为成功。

依赖为 ROS 2 Jazzy/Nav2 1.3.13，Gazebo Harmonic；精确 deb 版本在 `optimization_v2/references/S13/dependency_lock.json`。
