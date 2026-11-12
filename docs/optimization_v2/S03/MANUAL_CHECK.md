# S03 手工验收

使用一个普通终端，命令自动加载新环境。不要与原项目或其他阶段并行运行仿真。演示默认打开 Gazebo 并跟随车辆；车辆到终点后自动停止并退出，`--sim-seconds` 是上限，不是强制运行满时长。

## 1. 差速车通过连接路

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh demo S03 --case diff_connector --seed 41 --sim-seconds 240 --wall-timeout 480
```

应看到蓝色车身、黄色喷洒接口、两只驱动轮，车辆沿曲线连接路到 B 区。本步行人停放在道路外；这项演示验证路线净空，不认证动态避障。终点应保持静止。此时没有喷洒动作，喷头外形只表示接口包络。

## 2. 阿克曼车通过同一路线

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh demo S03 --case ackermann_connector --seed 41 --sim-seconds 240 --wall-timeout 480
```

应看到绿色车身、黄色刀盘包络，前轮随弯道转向，后轮驱动。它不能在原地旋转车体。可拖动视角查看前轮；报告的左右转角曲线是实际关节读数。终点停车后关闭窗口，不需要手动发额外速度。

## 3. 失联停车演示

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh demo S03 --case watchdog_stop --seed 41 --sim-seconds 90 --wall-timeout 240
```

约 2 秒开始使能并行驶，约 7.5 仿真秒主动停止发送上层命令。阿克曼车应在保留门限内停车；数据记录实际停车延迟、距离和原因。此案例会提前正常结束，不必等满 90 秒。

## 4. 查看证据与有效状态

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh open S03
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh open S03 --print-path
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh check S03
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh next S03
```

报告下拉框可选车辆和工况；蓝线是实际物理轨迹，虚线是目标路线，红框含工具包络。时间滑块/播放按钮使用记录的实际时刻。曲线包括轮速、转角、速度和停止距离。展开各故障案例可看到真实线速度/角速度随时间降到停车门限内的曲线。`check` 核对版本与文件哈希，不会重新运行仿真。人工状态默认 NOT_CHECKED，表示没有替你记录目视确认，不是新增的继续授权门槛。

## 5. 完整复验

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh test S03 --suite acceptance --seed 41 --sim-seconds 300 --wall-timeout 1800
```

默认无界面，串行执行两车通路、5°爬坡、扫速、左右圆弧/S 弯、差速短程停靠，以及两车各 9 种故障：命令失联、过期时戳、取消、执行桥失联、仲裁进程失联、反馈失联、仿真暂停、复位、重复发布者。包含 C++/Python 教材和原停止门限回归。需观察两次连接路 GUI 并留下本轮截图时，在命令末尾加 `--gui`。测试目标实时率为 2，任务时间和评分仍使用仿真钟；正式演示目标实时率为 1。

## 6. 停止与失败定位

在第二个终端运行：

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh stop S03
```

也可在当前终端 Ctrl+C。执行器先向客户端发停止信号，客户端发送 STOP，保留制动时间后关闭本会话；不杀其他 ROS/Gazebo 进程。中断测试状态应为 PARTIAL。若启动提示共享会话锁忙，应先结束先前的仿真。

错误日志位于终端给出的案例目录：`drive.log` 看仲裁器，`driver.log` 看客户端，`server.log` 看物理/插件，`metrics.json` 看哪项数值失败，`runtime.json` 看实际命令和退出码。不能因为场景可见或客户端返回 0，就认定运动/制动通过。

## 7. 参数和学习

```bash
cat /home/ceihao/robot_project/optimization_v2/config/vehicles/qualified_profiles.json
cat /home/ceihao/robot_project/docs/optimization_v2/S03/TECHNICAL.md
cat /home/ceihao/robot_project/docs/optimization_v2/S03/CODE_WALKTHROUGH.md
cat /home/ceihao/robot_project/optimization_v2/lessons/S03/README.md
```

若本轮未通过，以 FAIL/PARTIAL 回执为准，不能使用旧 qualified_profiles 文件宣称当前配置已合格。

S03 的底盘验收副本将行人与移动设备停放在道路外，地图的静态障碍与坡面保留。这里的无碰撞通过只验证底盘道路净空，不代表动态避障通过；动态对象在原 S02 场景仍正常运动。

整套正式测试通常需数分钟至十余分钟，不能将这段现实耗时与未来单簇 30 分钟任务预算混为一谈。
