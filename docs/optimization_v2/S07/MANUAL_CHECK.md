# S07 手工验收

使用一个普通终端，逐条运行，不并行开两份仿真。以下执行器自动 source 项目环境。自动验收与用户手工查看分开记录；实际数值及状态见 RESULTS。

## 1. 检查已有证据与进入下一步的条件

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh check S07
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh next S07
```

`check` 校验实际文件与回执版本，不重跑仿真。PASS 表示自动证据一致；STALE 表示代码、配置、地图或前置变化，需复验。`next` 给出 S08 的前置判断，不实施 S08。

## 2. 查看交互报告

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh open S07
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh open S07 --print-path
```

切换 zone_a / connector / zone_b；拖动时间条，观察定位轨迹与独立评分真值。取消“放大机器人附近”看整图，切换三维斜视检查高度结构。restart_occlusion 中检查停止定位进程、重启、70–85 秒传感器遮挡时的状态与动作准入。真实来源及误差统计在页面下半部分。

## 3. 重新演示重启和三个位置

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh demo S07 --case restart_relocalize --seed 41 --sim-seconds 300 --wall-timeout 660
```

顺序启动作业区 A 的实际定位进程重启/遮挡案例，再从连接路和作业区 B 的预先声明位置重新开系统。共 285 仿真秒，预算 300 秒；现实预算另计。默认打开 RViz，完成后打开该次独立演示报告。应该看到固定地图保持不变，INIT/重定位到可靠定位后才移动；故障期间停止。RViz 窗口能打开不等于定位通过，必须结合报告的实际匹配、状态和误差。演示不会覆盖正式自动验收回执。

## 4. 错误版本拒绝

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh demo S07 --case wrong_map --seed 41 --sim-seconds 90 --wall-timeout 240
```

这是离线准入测试，无仿真时钟，也不启动底盘。报告应明确列出 REQUEST_BINDING_MISMATCH、MAP_CONTENT_CHANGED 等拒绝原因。正确诊断契约能通过，真正工作计划类型在 S12 前被明确拒绝。

## 5. 完整自动复验

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh test S07 --suite acceptance --seed 41 --sim-seconds 600 --wall-timeout 1800
```

三位置各 80 秒，重启+遮挡 125 秒，错误初始位置 90 秒，共 455 仿真秒。测试默认无 GUI，输出真实轨迹、物理停车、源码版本和新回执。中断或失败会写 PARTIAL/FAIL，不保留旧 PASS 冒充本轮结果。

## 6. 停止与恢复

仿真运行时在第二个终端执行：

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh stop S07
```

只向本阶段有匹配 PID、UID、进程创建标识的会话发中断；驱动先发 STOP，等待制动，再关闭本次进程。它不会全局 kill ROS/Gazebo。也可在启动终端按 Ctrl+C。恢复使用上面的 demo/test 命令建立新会话，不手工给旧会话续时钟。

## 7. 学习材料

```bash
cat /home/ceihao/robot_project/docs/optimization_v2/S07/TECHNICAL.md
cat /home/ceihao/robot_project/docs/optimization_v2/S07/CODE_WALKTHROUGH.md
cat /home/ceihao/robot_project/docs/optimization_v2/S07/RESULTS.md
cat /home/ceihao/robot_project/optimization_v2/lessons/S07/README.md
```

不要手工修改 frozen 下的只读载荷或通过放宽协方差/停车门槛消除告警。需要修改地图时生成新版本并重验绑定；地图失效或观测丢失时停下来是预期行为。
