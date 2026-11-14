# R05 手工检查与复现

```bash
cd /home/ceihao/robot_project
bash optimization_v2/three_tasks/advance.sh R05 check
bash optimization_v2/three_tasks/advance.sh R05 open
```

`check` 核对验收回执和文件哈希；`open` 打开本地报告。检查三车均有传感器计数、真实往返距离、重启前后有效匹配和退出结果。误差图中的红线表示全局定位重启请求；定位误差只在离线评估中使用真值。报告不是完整路线验收。

重新执行单车（必须没有另一仿真运行；约 4 分钟墙钟、120 秒仿真）：

```bash
bash optimization_v2/three_tasks/advance.sh R05 test spray
bash optimization_v2/three_tasks/advance.sh R05 test inspection
bash optimization_v2/three_tasks/advance.sh R05 test mowing
```

依次测试，不同时运行。每次产生独立结果目录，失败也保留。构建、复测与重新封存：

```bash
bash optimization_v2/three_tasks/advance.sh R05 build
bash optimization_v2/three_tasks/advance.sh R05 test
bash optimization_v2/three_tasks/advance.sh R05 finalize
bash optimization_v2/three_tasks/advance.sh status
```

运行中另一个终端可观察在线数据（禁止将评估域接回控制）：

```bash
source /home/ceihao/robot_project/optimization_v2/env.sh
export ROS_DOMAIN_ID=119
ros2 topic echo /s07/localization/evidence --once
ros2 topic hz /s05/odometry
ros2 topic info /tf --verbose
```

`valid` 必须由当前观测配准和独立 GNSS 检查共同成立；`truth_used` 为 false。`odom->base_link` 只有一个公开发布节点。按 Ctrl+C 结束单独的 `topic hz` 观察，不会停止主仿真。主动中断主测试则由受控退出逻辑停止本轮进程。
