# S06 手工验收

所有仿真串行运行。一个终端启动演示；若要操作暂停／继续，使用第二个终端。仿真默认差速底盘，S03/S05 两种底盘的既有验收不因此扩大成“两车都已完成本步建图”。

## 1. 核对完成状态和打开实测报告

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh check S06
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh open S06
```

`check` 检查本轮回执、输入哈希和 S05 依赖，不重新仿真。只有 `PASS` 才表示完整自动矩阵通过；`STALE` 表示代码／配置或前置已变化。人工状态保持 `NOT_CHECKED`，不会代你确认看过界面。

浏览器选择 `mapping_full` 查看两区与弯曲通路，选择 `mapping_loop` 检查短程回访。拖动旋转、滚轮缩放，拉动采集时间轴看真实扫描快照逐步加入；切换“局部里程计累积／回环优化后”，比较轨迹和重影，红线表示真实回环边。表格显示 RMSE、点云 P95 和动态残留。原始弱纹理案例是失败诊断对照。

没有桌面时只打印报告路径：

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh open S06 --print-path
```

## 2. 实时演示：地图增长和回访

终端 1：

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh demo S06 --case mapping_loop --seed 41 --sim-seconds 600 --wall-timeout 1200
```

启动器负责环境，默认打开 RViz。案例内部实际采集 140 仿真秒，600 秒是上限。通常约数分钟结束；主机负载会改变现实时间。启动约 8–10 仿真秒后行驶；约 25 秒自动暂停、32 秒继续、45 秒保存、55 秒继续，用于自动演示状态链。回访在路线后段发生，不能拿某一帧 `loop_closure_id=0` 推断整次没有回环。

RViz 固定坐标为 `map`。应看到实际三维点云、树干／树冠及建筑高度，绿色轨迹随车辆生长。辅助 `Filtered mapping lidar` 显示 `/s06/mapping/cloud`，已变换到 base_link；不使用旧的传感器帧话题。回环后全局轨迹可修正，局部里程计仍连续。GUI 启动成功不等于画面正确；若没有点云，检查下面的输入和状态，并查看本轮 `slam.log`。前向距离或局部观测退化时应停车，不要求机器人穿过障碍。

## 3. 检查话题、TF 和手工生命周期

演示运行时，终端 2：

```bash
cd /home/ceihao/robot_project
source optimization_v2/env.sh
ros2 topic echo /s06/manager/status std_msgs/msg/String --once --full-length
ros2 topic hz /s06/mapping/cloud
```

第一条状态中的 `mode` 应为 `MAPPING`。`hz` 持续运行，按 Ctrl+C 结束；它统计建图筛选后的频率，不是雷达原始 10 Hz，约 1–2 Hz 合理，丢弃和时间差有实际日志。

```bash
ros2 node info /s06/rtabmap/s06_slam --no-daemon --spin-time 3
ros2 topic info /tf --verbose --no-daemon --spin-time 3
ros2 run tf2_ros tf2_echo map base_link
```

这里显式指定状态消息类型，并让节点／TF 查询直接发现在线节点，避免已有 ROS CLI daemon 缓存导致“节点不存在”的误报。

RTAB-Map 应订阅 `/s06/mapping/stereo`、`cloud`、`odom`。动态 `/tf` 有两个合法节点：`s05_local_odometry` 发布 odom→base_link，`s06_slam` 发布 map→odom；不是两个节点同时抢同一条边。`tf2_echo` 持续运行，Ctrl+C 结束。

自动生命周期演示结束后（约 60 仿真秒），可逐条执行：

```bash
ros2 service call /s06/manager/pause std_srvs/srv/Trigger '{}'
ros2 topic echo /s06/manager/status std_msgs/msg/String --once --full-length
ros2 service call /s06/manager/resume std_srvs/srv/Trigger '{}'
ros2 topic echo /s06/manager/status std_msgs/msg/String --once --full-length
ros2 service call /s06/manager/save std_srvs/srv/Trigger '{}'
ros2 topic echo /s06/manager/status std_msgs/msg/String --once --full-length
ros2 service call /s06/manager/resume std_srvs/srv/Trigger '{}'
```

每次请求后等待约 1–3 秒再看状态。`ACCEPTED` 仅表示受理，最终应分别看到 `PAUSED`、`COLLECTING`、`SAVED`、`COLLECTING`；BUSY 表示上一动作尚未完成。保存先停车，再刷新真实数据库；若在 140 秒总预算结束后操作，服务会消失，此时应查看已经保存的文件。手工额外暂停可能影响这一轮演示路线能否在预算内完成，演示不会覆盖正式 PASS 回执。

## 4. 打开已采集数据的前后对照

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh demo S06 --case loop_comparison --seed 41 --sim-seconds 120 --wall-timeout 300
```

这是离线查看，仿真时钟 N/A，不重启 Gazebo。页面上的 DB、PLY、位姿图和 manifest 都来自当前正式验收数据；导出前后使用同一批观测。

## 5. 重新执行完整自动验收

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh test S06 --suite acceptance --seed 41 --sim-seconds 900 --wall-timeout 3600
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh check S06
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh next S06
```

测试依次完成跨两区长路线、短回访、原场景弱纹理对照，再检查真实 DB 重开、错误候选、点云高度／误差、动态残留、停车、暂停／继续、来源哈希和报告交互。900 秒是每个案例预算，不是三个案例总预算；最长现实上限 3600 秒。失败或中断会产生 FAIL/PARTIAL 回执，不用旧 PASS 冒充本轮结果。

执行器另支持 `--resume /绝对路径/已完成的mapping_full案例`，仅复用已正常完成、种子／预算相符且**采集时记录的全部运行输入哈希仍匹配**的长采集。它还会重开／重新导出／重新评分，并执行剩余案例。不是跳过失败或强制通过的选项。

## 6. 停止

在第二终端执行：

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh stop S06
```

只对本步骤记录且 PID／启动时间／UID／命令均匹配的会话发送中断。启动器先停车、保存数据库，再收尾自己启动的进程；不会全局 kill ROS/Gazebo。中断的整套验收显示 PARTIAL，已采集数据库仍保存在该轮目录，可用于诊断；正式 PASS 仍须跑完矩阵。

## 7. 学习核心代码

```bash
cat /home/ceihao/robot_project/docs/optimization_v2/S06/TECHNICAL.md
cat /home/ceihao/robot_project/docs/optimization_v2/S06/CODE_WALKTHROUGH.md
cat /home/ceihao/robot_project/optimization_v2/lessons/S06/README.md
```

教学 README 提供 C++ 编译和 Python 执行命令。教学小图的误差不是实际机器人的误差；实际数值以 RESULTS 和交互报告为准。
