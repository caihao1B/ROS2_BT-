# S09 手工验收

一个主终端执行即可，第二终端可停止或检查话题。命令默认打开 RViz 和本轮离线浏览器报告，按顺序运行，不并行仿真。执行器自己加载环境、检查 S08、上锁、设置 domain 和保存进程身份；不需要手动启动其他 ROS 节点。

先核对自动证据：

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh check S09
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh open S09
```

实际入口复核使用了 75 s / 85 s 的短演示（分别运行 65 s / 75 s，留收尾裕度）；以下较长预算命令使用相同执行器和场景。

低障碍、3°/9° 坡面，差速车：

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh demo S09 --case low_obstacles_and_slope --seed 41 --sim-seconds 240 --wall-timeout 540
```

只替换底盘，场景相同；RViz 中也可切换两张成本图：

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh demo S09 --case low_obstacles_and_slope --vehicle ackermann --seed 41 --sim-seconds 240 --wall-timeout 540
```

等约 20–45 现实秒：RViz 以 odom 显示观测地形、Nav2 代价和目标框。车辆本阶段驻停，位于单独声明的诊断通行区。作业区 A/B 的原始边界没有改变。代价图应同时包含可观测地面与红色障碍，不能是一整张禁行图。三个低障碍高 0.08/0.15/0.30 m；36 s 左右移走 8 cm 测试物，随后红色栅格应显著减少。未知空洞不会自动变绿。缓坡中心有渐进代价，陡坡为不可通行。显示面板 `Raw observed lidar (toggle)` 可开启原始感知云，与彩色地形切换对照；`Ackermann Nav2 costmap (toggle)` 可启用并关闭 `Diff Nav2 costmap` 层，避免两层重叠。

行人遮挡和突然恢复运动：

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh demo S09 --case occluded_pedestrian --seed 41 --sim-seconds 180 --wall-timeout 420
```

看相机真实检测框、点云目标框、ID 和 OBSERVED/COASTING 状态。行人 20–24 s 停住后继续移动；遮挡超过短时关联寿命后新 ID 正常。树木误识别可能出现，几何风险仍应保留。没有 person 类别不等于没有障碍。

相机处理失效 25–45 s：

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh demo S09 --case camera_loss --seed 41 --sim-seconds 100 --wall-timeout 300
```

失效时 camera_valid=false，类别退为 unknown_obstacle；彩色点云、地形和 Nav2 继续刷新。视频窗口可能保留最后一帧，请以消息时间戳判定新鲜度。

第二终端可读取实际消息（每条看一次后退出）：

```bash
cd /home/ceihao/robot_project
source optimization_v2/env.sh
export ROS_DOMAIN_ID=119
ros2 topic echo /s09/terrain/health --once
ros2 topic echo /s09/objects --once
ros2 topic echo /s09/safety --once
ros2 lifecycle get /s09_nav2/diff/costmap
```

应看到有效时戳、几何点数/计算时延、目标位置/速度/尺寸/类别/协方差/ID，以及 active 代价节点。时间戳不前进、frame 错误、GEOMETRY_UNVERIFIED 或全局定位未准入均不能当作成功。观看 GUI 是独立人工记录，自动回执始终不冒充用户确认。

停止、正式复验与下一步判断：

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh stop S09
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh test S09 --suite acceptance --seed 41 --sim-seconds 300 --wall-timeout 1800
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh check S09
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh next S09
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh open S09 --print-path
```

正式矩阵为 60+60+75+75=270 仿真秒，300 秒总预算包含收尾裕度；现实运行含启动/组件/报表，多于 270 秒正常。演示使用请求时限减去 10 秒的运行时长，给停车/保存留余量。中断返回 PARTIAL，按相同输入哈希可用 `--resume <已完成部分的正式运行目录>` 复验；源/模型/运行配置变化则拒绝复用。stop 只收尾本次身份匹配的进程。
