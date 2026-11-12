# S13 手工验收

所有命令从普通终端运行即可，入口自动加载隔离环境。两车顺序运行，不能并行占用 Gazebo。默认打开 RViz；浏览器报告包含真实物理轨迹、速度、角速度、转角、计算时延和观测图片。GUI 能打开不等于验收通过。

## 终端 1：差速车到点和道路巡检

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh demo S13 --case diff_point_and_inspect --seed 41 --sim-seconds 420 --wall-timeout 900
```

先等重定位完成，通常几十个仿真秒。应看到先 NavigateToPose 到第一个服务位姿，再 FollowPath 沿原连接路前 6 m 巡检。报告中检查点应含当前相机画面、点云点数、估计位姿与时间；工具 OFF。若定位保持 HOLD、路线碰撞、动作取消后无法恢复，命令返回非零，不能当作“只是画面问题”。该演示不声称巡检完整连接路。

## 终端 1：阿克曼路径与停车转向

前一个命令结束后执行：

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh demo S13 --case ackermann_follow_path --seed 41 --sim-seconds 420 --wall-timeout 900
```

先在零车速下调整前轮，再跟踪注册到调试场地的 S12 短曲线。查看曲率与真实转角；后续取消动作应停车并留下剩余路径，不能出现零速原地掉头。注册短段是控制器组件验收，不是执行原作业簇。

## 终端 2：随时停止本会话

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh stop S13
```

只停止本步匹配 PID/进程创建标识的会话。应关工具、停止驱动并保存未完成进度；不终止其他 ROS 会话。主终端 Ctrl+C 也会进入收尾。

## 完整自动复验

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh test S13 --suite acceptance --seed 41 --sim-seconds 600 --wall-timeout 3600
```

顺序测试差速 DWB、三次独立冷启动的差速 MPPI、阿克曼 RPP/MPPI、动作语义、定位丢失及两车道路巡检。每个案例有自己的仿真上限，整体现实上限 3600 s。演示通过不替代此正式矩阵。若失败，本轮回执必须显示 FAIL/PARTIAL。

## 查看证据、学习材料与下一步判定

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh check S13
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh open S13
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh open S13 --print-path
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh next S13
cat /home/ceihao/robot_project/docs/optimization_v2/S13/RESULTS.md
cat /home/ceihao/robot_project/docs/optimization_v2/S13/TECHNICAL.md
cat /home/ceihao/robot_project/docs/optimization_v2/S13/CODE_WALKTHROUGH.md
cat /home/ceihao/robot_project/optimization_v2/lessons/S13/README.md
```

浏览器切换案例和时间曲线，查看实际轨迹是否贴合计划、是否出现停顿及其原因。`check` 只核对回执和文件版本，`test` 才重新执行。人工未查看始终记 NOT_CHECKED；是否可进入 S14 由完整自动验收与前置决定，不额外要求一次授权。

补充检查：正式矩阵先运行距基座 1.1 m、实体 8 cm 低障碍感知测试。下视雷达仅接入感知；原定位支路不变。Gazebo 实时时率上限 0.4，任务仿真秒与现实秒分别记录。动作语义用例在已经验证的短段范围内折返，工具关闭；不借此宣称长距离整簇定位已通过。


## 已知环境退出异常

本轮四项导航验收通过，但不能据此宣称所有进程都正常退出。11 个物理案例的 Gazebo 服务进程收尾退出码为 -11（SIGSEGV），其中一次 ROS-Gazebo bridge 也记录 -11；真实退出码保存在各 `runtime.json`，没有改写为 0。每例均已确认工具 STOPPED、冻结数据库哈希不变，且按 PID 与进程创建标识核查没有遗留本轮进程。

使用 GDB 的有界复现显示，发送 SIGINT 后，在 `__nptl_deallocate_tsd` 工作线程退出阶段发生崩溃。仅保留 Gazebo 官方 Physics/UserCommands/SceneBroadcaster/Sensors(ogre) 和一个相机的最小场景，也复现相同现象；该场景没有项目插件。故障地址对应退出前加载的 `libboost_thread.so.1.83.0`，提示库卸载与线程清理的生命周期问题，但根因尚未完全定位，尚未修复。ROS-Gazebo bridge 的异常原因未单独确认。没有修改系统驱动、安全策略或忽略运行中的必要节点退出。

证据：[退出码与进程清理核查](/home/ceihao/robot_project/optimization_v2/results/S13/test_20261111_214031_1075382/cleanup_probe/exit_summary.json)、[最小场景诊断](/home/ceihao/robot_project/optimization_v2/results/S13/test_20261111_214031_1075382/cleanup_probe/diagnosis.json)、[GDB 栈](/home/ceihao/robot_project/optimization_v2/results/S13/test_20261111_214031_1075382/cleanup_probe/minimal_official_gdb.log)。这是环境收尾限制，单列保留；S13 的导航、停车、重启及巡检检查和进入 S14 开发的判定，按有效回执的既定门限执行。不能由此推定环境已达到无异常退出的发布标准。

## 已实际执行的可视回归

两个公开 demo 命令已在本轮源码版本上实际运行，均返回 0；源码哈希与最终正式矩阵完全一致。保存了 RViz 窗口截图、真实动作事件和浏览器交互检查，清单见 [GUI 实测证据](/home/ceihao/robot_project/optimization_v2/results/S13/test_20261111_214031_1075382/gui_evidence/manifest.json)。画面检查由助手完成；用户人工验收字段仍为 NOT_CHECKED。

本轮演示：差速 `demo_20261111_213108_1049549`，阿克曼 `demo_20261111_213718_1065652`。可查看 [差速 RViz](/home/ceihao/robot_project/optimization_v2/results/S13/test_20261111_214031_1075382/gui_evidence/diff_point_and_inspect/diff_point_and_inspect/rviz.png) 和 [阿克曼 RViz](/home/ceihao/robot_project/optimization_v2/results/S13/test_20261111_214031_1075382/gui_evidence/ackermann_follow_path/ackermann_follow_path/rviz.png)。实际物理轨迹与控制耗时请查看本轮浏览器报告；RViz 中的绿色轨迹是在线估计值，报告中的红色轨迹才是独立物理评分值。
