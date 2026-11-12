# S02 手工验收

普通终端一个即可；停止演示时可用第二终端。全部命令自带环境与会话互斥，不要同时启动原项目和新场景。Gazebo 初次出现通常需 5–15 秒；演示会按 `/clock` 结束，并有现实时间保护。

## 1. 看完整地图

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh demo S02 --case world_overview --seed 41 --sim-seconds 180 --wall-timeout 360
```

可见浅绿色 A 区、深绿色 B 区、曲线道路；两区内部都没有预先划定三块田。用鼠标拖动/滚轮调整 Gazebo 视角。应看到四个行人和一个设备持续运动。机器人留 S03 加入，本步不会出现车辆。

## 2. 切换地面视角和变体

上个演示结束后逐条运行：

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh demo S02 --case world_overview --view ground --seed 41 --sim-seconds 60 --wall-timeout 180
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh demo S02 --case world_overview --variant notch --seed 41 --sim-seconds 60 --wall-timeout 180
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh demo S02 --case world_overview --variant holes --seed 41 --sim-seconds 60 --wall-timeout 180
```

地面视角可以分辨三种橙色低障碍、灰色设施、蓝色行人与黄色设备。凹口变体边缘有凹入，含洞变体每区有一个设施岛，围绕它的区域仍连通。三种坡地的角度为 5°/10°/18°。坡度能否通过要等 S03/S09 的车辆和通行代价检查。

## 3. 先验与实际观测对照

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh demo S02 --case prior_vs_unknown --seed 41 --sim-seconds 120 --wall-timeout 300
```

此命令实际采集 120 仿真秒观测，然后打开本次离线报告。无需等待新演示，也可直接打开正式验收报告：

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh open S02
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh open S02 --print-path
```

报告的三个勾选框分别控制规划先验、实际传感器回波、仅评估真值；真值默认关闭。关闭真值后，先验不应包含橙色临时障碍和动态人物位置。蓝点是固定探针的真实观测，无法覆盖整张大地图属于正常视距限制。播放滑块显示保存的实际轨迹，不是在线预测输出。`open` 打开最新正式验收报告；演示报告路径另在终端输出和 `results/S02/last_demo.json` 保存。

## 4. 自动复验与下一步判断

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh test S02 --suite acceptance --seed 41 --sim-seconds 300 --wall-timeout 1200
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh check S02
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh next S02
```

验收顺序运行几何/错误输入、两轮同种子运动、凹口/含洞、接触夹具、俯视/地面图和 C++/Python 教学程序。每案例使用 8–24 仿真秒，合计 100 仿真秒；300 是允许的单案例上限，不强行占满。测试结束应得到四项 PASS。`check` 只校验文件/前置回执，不会重新跑仿真。手工状态仍为 NOT_CHECKED，表示未替用户记录目视确认。

## 5. 停止与异常

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh stop S02
```

只停止 S02 记录的会话。也可在运行终端 Ctrl+C；中断测试应为 PARTIAL，不应遗留 PASS 冒充完整验收。若提示共享会话锁忙，应先关闭原先自己启动的仿真。若窗口全白、无回波、节点崩溃、时钟停止，检查终端给出的案例目录中的 `server.log`、`online.log`、`evaluation.log` 和 `runtime.json`。不要用全局 `killall` 处理。恢复后重跑正式验收。

## 6. 学习示例

```bash
cat /home/ceihao/robot_project/optimization_v2/lessons/S02/README.md
cat /home/ceihao/robot_project/docs/optimization_v2/S02/TECHNICAL.md
cat /home/ceihao/robot_project/docs/optimization_v2/S02/CODE_WALKTHROUGH.md
```
