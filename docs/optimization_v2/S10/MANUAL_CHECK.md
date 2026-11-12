# S10 人工验收

以下命令在普通终端执行，不需要密码。每次只运行一个仿真。执行器加载原生 ROS 2 Jazzy 环境、验证 S08/S09，持有项目锁，结束时发 STOP_WORK、停车并保存 SQLite。自动通过不表示用户已经目视验收。

## 1. 先看已生成证据，无需重新仿真

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh check S10
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh open S10
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh next S10
```

check 应验证回执、实现和证据哈希及上游依赖链；open 打开离线交互网页。选择点喷洒/割草，拖动时间或播放，检查任务由待执行到运行，再进入完成/部分/取消/失联。绿色是在线确认足迹，蓝色是独立物理作用，虚线是目标。割草并集逐步增长，重复请求保持面积不变；失联任务保留不确定与残余。网页底部列出真实检查，不只是示意图。

无浏览器时读取路径：

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh open S10 --print-path
```

## 2. 终端 A：运行点喷洒演示

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh demo S10 --case point_service_states --seed 41 --sim-seconds 180 --wall-timeout 420
```

约等待数十秒启动和重定位，之后进行成功、同键重复、冲突、未到位、定位/速度/目标条件不满足、故障部分完成、重试、取消、失联和恢复拒绝。车辆保持驻停。桌面模式启动 RViz 展示当前车辆坐标系、三维观测及代价层；结束后自动打开本轮工具作业网页。工具生命周期以 Action 与账本/网页为准，RViz 未显示机械臂是正常现象。

180 是仿真秒上限，420 是现实秒保护，不是实测任务耗时。仿真实时倍率设为 0.5，为传感器计算留出时间。遇感知短时失效会关闭工具等待，不放宽安全门限。每例有独立时限；自然完成时提前收尾。

## 3. 终端 B：在演示进行时查看实时接口

```bash
source /home/ceihao/robot_project/optimization_v2/env.sh
export ROS_DOMAIN_ID=119
ros2 action list -t
ros2 interface show mission_interfaces/action/ToolTask
ros2 service call /s10/tool/capabilities mission_interfaces/srv/ToolCapabilities '{}'
ros2 topic echo --once /s10/action/state std_msgs/msg/String
ros2 topic echo --once /s10/ledger/state std_msgs/msg/String
ros2 topic info /s10/tool/command --verbose
```

应看到 action `/s10/tool_task`、map/plan/tool 版本和 S03 工具尺寸。状态中的 `phase` 不是完成量，只有 `confirmed_quantity` 是确认值。`HOLDING` 表示工具关闭等待感知；`STOPPING` 正等待实际停转。工具命令 publisher 只能是 `s10_tool_action_server`。没有仿真时这些命令会等待/找不到服务，不代表离线回执失效。

## 4. 点喷洒结束后：运行割草演示

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh demo S10 --case continuous_work_and_retry --seed 41 --sim-seconds 240 --wall-timeout 540
```

阿克曼车在诊断区域短距离直线移动，账本记录连续反馈足迹，执行取消、保留已确认面积、原键重试、已完成重复和 TRANSIT/TURN 拒绝。**不是 S13 控制器追踪完整覆盖路径。** 查看网页割草模式：确认面积按并集增长，不按消息条数或行驶距离估算；最终残余仍可查。无桌面可在 demo 命令后加 `--headless`，结束后手动 open 或访问 last_demo.json 指向的文件。

## 5. 查询持久账本，不依赖 ROS/Gazebo 运行

正式验收数据库：

```bash
python3 /home/ceihao/robot_project/optimization_v2/ws/src/work_ledger/scripts/inspect_ledger.py --case point_service_states
python3 /home/ceihao/robot_project/optimization_v2/ws/src/work_ledger/scripts/inspect_ledger.py --case continuous_work_and_retry
```

应看到 integrity_check=ok、每个幂等键唯一一条任务、确认量/目标量/终态/不确定标志。完成任务不会出现第二笔重复工作；部分/取消保留非零确认量。`LOST` 不能解释成“肯定没喷”。若查看刚执行的演示，在对应 case 后加 `--demo`；只读查询，不清除账本。

## 6. 全量自动复验与停止

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh test S10 --suite acceptance --seed 41 --sim-seconds 300 --wall-timeout 1500
```

串行执行 Python 账本回归、原工具交接回归、真实 ROS Action 进程故障测试、C++/Python 教学例、两个 Gazebo 案例和浏览器交互检查。组件合成观测与真实 Gazebo 物理效果分开记录。

第二终端停止当前 S10 自有会话：

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh stop S10
```

停止会发关闭动作并停车；若无法确认工具停止，账本不会伪造完成，未完成任务在重启后保持待核查。不要 killall 或直接改数据库状态。中途终止的正式复验会记录 PARTIAL，不能继续引用旧通过状态冒充本轮结果。没有正在运行的 S10 时应返回 NOT_RUNNING。

若任一条件失败，查本轮 `result.json`、`metrics.json`、每例日志和账本。只运行 demo 不覆盖正式测试回执。人工检查记录另存，避免擅改已哈希的自动证据。
