# S05 手工查验

工作目录 `/home/ceihao/robot_project`。公开命令自己加载环境；单仿真串行。一个终端运行演示，另一个终端查看主题或停止。人工未操作前记录 NOT_CHECKED，不冒充用户验收。

## 1. 先查真实回执和页面

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh check S05
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh open S05
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh next S05
```

check 核对源码/证据/前置哈希，不重新跑仿真。open 打开本轮实测页面：选择底盘/案例、拖动仿真时间、播放轨迹；比较三条估计与独立真值、误差曲线、坡道俯仰和健康状态。若显示 FAIL，先读失败检查和 RESULTS，不能因有画面就通过。无桌面用 `open S05 --print-path` 得到本地页面路径。

## 2. 正常闭环，两种车分别运行

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh demo S05 --case odometry_loop --seed 41 --sim-seconds 240 --wall-timeout 540
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh demo S05 --case odometry_loop --vehicle ackermann --seed 41 --sim-seconds 240 --wall-timeout 540
```

每次执行完成后再运行下一条。默认打开 RViz，固定坐标 odom，橙/蓝/绿色为轮速/轮速+IMU/多源融合路径；转弯由融合 odom 闭环驱动。执行器只用必要的 120 仿真秒，240 s 是上限，不故意延长等待。首次启动约 10–20 现实秒，前 8 仿真秒保持静止校准。RViz 自动结束且保存截图，演示结果另存在 `results/S05/last_demo.json`，不覆盖正式回执。

## 3. 轮滑与失联

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh demo S05 --case slip_and_icp_loss --seed 41 --sim-seconds 180 --wall-timeout 420
```

20–50 s 轮速失配；52–61 s 陀螺偏置；65–79 s 地面单平面造成 ICP 退化并降速/停车；86–94 s IMU 断流要求停止。用第二终端查看原因与融合输出：

```bash
source /home/ceihao/robot_project/optimization_v2/env.sh
ros2 topic echo /s05/safety_limit
# Ctrl+C 只停止 echo；以下逐条查看
ros2 topic echo /s05/diagnostics --once
ros2 topic echo /s05/odometry --once
ros2 topic echo /s05/planning_pose --once
ros2 topic info /tf --verbose
ros2 topic info /s03/actuator/request --verbose
```

/odometry 保留三维位姿；planning_pose 是单独的平面投影。/tf 唯一发布者应为 s05_local_odometry；执行出口唯一发布者应为 s03_drive_bridge。`ICP_DEGENERATE_OR_STALE` 和 `IMU_STALE` 可直接说明降速/停车原因。数据恢复后健康 OK 不意味着旧漂移消失。

可在演示中发起显式新 epoch（这是主动故障/恢复操作，会改变本次演示结果）：

```bash
ros2 service call /s05/reset_epoch std_srvs/srv/Empty '{}'
ros2 topic echo /s05/epoch --once --qos-durability transient_local
```

应看到新 epoch，停车初始化，再建立新的局部原点；不能把原点重置当定位误差被算法自动消除。正式矩阵用真实 Gazebo 时钟倒退检查这一机制。

## 4. 完整自动复验与停止

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh test S05 --suite acceptance --seed 41 --sim-seconds 300 --wall-timeout 1500
```

九个案例依次执行，总计约 13–18 分钟现实时间；不是一个 300 秒案例。每案例仿真预算≤300秒，墙钟1500秒为整轮保护。输出实际各项 PASS/FAIL、矩阵报告、回执。可加 `--gui` 为差速闭环保存 RViz 画面；默认复验无界面。

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh stop S05
```

只核验并停止本步持有的会话，先请求停车、等待物理制动、收尾子进程。不要使用 killall/pkill。Ctrl+C 也走同样收尾路径；中断正式矩阵记 PARTIAL。并行启动另一项目仿真应被共享锁拒绝。

## 5. 学习和复查

```bash
cat /home/ceihao/robot_project/docs/optimization_v2/S05/TECHNICAL.md
cat /home/ceihao/robot_project/docs/optimization_v2/S05/CODE_WALKTHROUGH.md
cat /home/ceihao/robot_project/docs/optimization_v2/S05/RESULTS.md
cat /home/ceihao/robot_project/optimization_v2/lessons/S05/README.md
```

若运行没有路径、节点崩溃、传感器超时、红色 STOP 不恢复或坡道姿态始终为零，都应按失败处理。输入速度一致性残差与真实 EKF 内部创新探针是两个不同证据，页面已标明。

本次已实际执行公开 `slip_and_icp_loss`（差速）和 `odometry_loop --vehicle ackermann`（阿克曼）演示，两者都完成 GUI 运行、停车收尾和浏览器交互检查。自动验收/演示通过不代表用户已经人工目视确认。
