# S08 手工验收

使用普通终端，命令负责装载环境；不要同时启动两套项目仿真。建议先打开报告，再运行演示。每次结束后观察终端实际 PASS/FAIL，不以 RViz 窗口出现为通过。

## 1. 核对正式回执并打开报告

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh check S08
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh open S08
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh open S08 --print-path
```

报告切换 normal、gnss_outage、false_fixed_and_loss、map_degradation；拖动时间条，查看最终轨迹、GNSS 天线位置、地图观测、创新距离、许可和理由。橙色点是天线原始 map 坐标，不是已扣杆臂的车体中心；固定安装偏差不等于定位错误。开关“真值”只影响离线评估画面，不影响在线算法。

位置 P95 应≤0.15m、航向 P95≤0.05rad。故障例同时展示误差和停车；双源失效期间不能显示可靠且允许运动。

## 2. GNSS 遮挡与浮点解（一个终端）

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh demo S08 --case gnss_outage --seed 41 --sim-seconds 240 --wall-timeout 540
```

实际诊断窗口75仿真秒，240是总预算上限，不是必须运行满240秒。一般需约2–3分钟现实时间，含启动与关闭。默认打开 RViz，结束后打开本轮交互报告。

- 25–43s GNSS 丢失：MAP_ONLY 降速至最多0.06m/s；如果地图也过期则停车。
- 50–58s float：不当 fixed 使用，仍降级；恢复需连续合格证据。
- RViz 车体附近状态文字与实际轨迹应同步。绿色表示当前准入，不保证整个场景始终有效。

## 3. 假 fixed 与双失效

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh demo S08 --case false_fixed_and_loss --seed 41 --sim-seconds 240 --wall-timeout 540
```

25–38s 注入 GNSS `[+1.2,−0.8,0]m` 偏差但仍报告 fixed；必须出现 INNOVATION_REJECTED_FIXED，不能继续全速。48–64s 两路全局信息失效，应取消运动许可并实际停稳；恢复后经过确认再运动。报告 physical_stops 给出故障发生至停稳的时间，保持原 |v|<0.01m/s、|w|<0.02rad/s 门限，目标<2s。

## 4. 地图短暂退化与错误匹配

在正式报告选择 `map_degradation`。该案例已经由第6节的正式自动命令运行；无需另外启动一个仿真。

25–29s 地图不可用，合格 GNSS 可暂管位置，航向仍有最长8s期限。40–54s 注入错误地图匹配位移 `[+1,−0.5,0]m`，应停车并锁止，保留被拒原始观测；结束后必须重新验证。运行器只对无真实任务的诊断 fixture 调用确认服务，不能把这项服务用于恢复旧割草计划。

## 5. 第二终端查看实时状态或停止

演示运行时可在第二终端执行：

```bash
source /home/ceihao/robot_project/optimization_v2/env.sh
ROS_DOMAIN_ID=119 timeout 8 ros2 topic echo /s08/status --once --full-length
ROS_DOMAIN_ID=119 timeout 8 ros2 topic echo /s08/pose --once
ROS_DOMAIN_ID=119 timeout 8 ros2 topic info /tf --verbose
```

应只有 S05 局部里程计与 S08 全局管理器两个动态 TF 发布者，分别负责 odom→base_link 和 map→odom；RTAB 不竞争广播。超时命令返回124表示在期限内没收到，不应算验收通过。

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh stop S08
```

也可在第一终端 Ctrl+C。只停止本阶段记录的会话；先发停车再结束节点，保留 PARTIAL 结果。没有会话时应显示 NOT_RUNNING。

## 6. 全套自动复验与学习

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh test S08 --suite acceptance --seed 41 --sim-seconds 300 --wall-timeout 1800
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh check S08
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh next S08
```

四例顺序运行，主动采集60+70+75+75=280仿真秒，另留20秒关闭余量。全套现实预算1800秒，预计约8–12分钟；全部用无界面仿真。坐标测试、生产 C++ 状态测试与教学程序没有仿真车辆，也不算车辆运行时间。

需要断点恢复时，传入上一轮 result.json 所在目录：

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh test S08 --suite acceptance --seed 41 --sim-seconds 300 --wall-timeout 1800 --resume /home/ceihao/robot_project/optimization_v2/results/S08/实际运行目录
```

上式最后路径需替换为真实目录；这是高级参数说明。仅复用地图、实际运行源码/配置/二进制、种子和案例预算均匹配的原始捕获，再重新评分。改变运行代码后必须重跑，不绕过版本门禁。

[技术原理](TECHNICAL.md) · [核心代码](CODE_WALKTHROUGH.md) · [实测结果](RESULTS.md) · [C++/Python 学习例](../../../optimization_v2/lessons/S08/README.md)

人工状态默认 NOT_CHECKED。你查看以上现象后可在后续消息中说明已验收；人工尚未查看不增加额外授权门槛。下一步判定以有效自动回执和 S09 前置为准。
