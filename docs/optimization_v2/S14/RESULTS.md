# S14 本轮结果

状态：**PASS**；人工：NOT_CHECKED。

报告：[本轮离线交互报告](/home/ceihao/robot_project/optimization_v2/results/S14/test_20261112_143553_2062800/index.html)

实际调用：`/home/ceihao/robot_project/optimization_v2/ws/src/mission_bt/scripts/stage14.py test --sim-seconds 1800 --wall-timeout 10800 --suite acceptance --seed 41 --resume-from /home/ceihao/robot_project/optimization_v2/results/S14/demo_20261112_133927_1978303`

| 案例 | 作业状态 | 点完成 | 面积完成 m² | 仿真秒 | 停车确认 |
|---|---|---|---|---|---|
| decision_story | PARTIAL | 1/2 | 0.0000/0.0000 | 75.233 | True |
| mow_smoke | SUCCEEDED | 0/0 | 0.7183/0.7500 | 45.301 | True |
| nominal_spray_cluster | SUCCEEDED | 21/21 | 0.0000/0.0000 | 1283.063 | True |
| nominal_mow_cluster | SUCCEEDED | 0/0 | 16.2763/16.3595 | 1531.663 | True |
| budget_exhaustion | PARTIAL | 0/2 | 0.0000/0.0000 | 11.185 | True |
| persistent_block | PARTIAL | 1/2 | 0.0000/0.0000 | 76.923 | True |
| tool_loss | PARTIAL | 0/2 | 0.0000/0.0000 | 20.902 | True |

检查项：`{"S14-C01": "PASS", "S14-C02": "PASS", "S14-C03": "PASS", "S14-C04": "PASS"}`



可以执行下一步 S15

具体命令/返回码：当前报告目录 commands.jsonl；子进程退出码：各案例 runtime.json。代码/配置/输入哈希、直接前置 S10/S13 回执哈希见 S14/latest.json。报告目录保留原始决策、实际轨迹、SQLite账本及完整残余。

已知边界：派生区域地图只验证 A1/B1，尚未推广六簇和连接路；短段、故障正确停车和策略单测不能代替 C03。S13 已存在 Gazebo 关闭 SIGINT 后个别进程 -11 的问题，本轮实际退出码仍原样记录；它不赋予作业成功。

## 本机依赖与测试范围

种子41；ROS 2 Jazzy；Gazebo Harmonic。安装版本实测：

```text
ros-jazzy-behaviortree-cpp	4.10.0-1noble.20260911.134208
ros-jazzy-nav2-controller	1.3.13-1noble.20260921.031754
ros-jazzy-rtabmap-slam	0.23.13-1noble.20261003.101914
```

C++ 决策/工具联锁例、Python 去重与残余例、真实 BT 的异步取消握手、Point32面积传输以及中断时进程清理隔离均以本轮 commands.jsonl 为准。清理测试日志中的 KeyboardInterrupt 为主动注入；不能据此推断生产机器人异常。

SMOKE 与整簇必须分开：本轮短段即使达到目标，也不改变 A1 的21点或 B1 的16.3594578796 m²。本轮只验收表中实际执行的分支；未触发的补扫/局部绕行与六簇调度不得由单簇成功推定。人工状态仍是 NOT_CHECKED。

区域地图算法和独立测点指标见 TECHNICAL.md；实际整簇结果以本表为准。
