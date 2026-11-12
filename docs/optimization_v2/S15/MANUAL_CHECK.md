# S15 手工查验

一次只运行一个仿真。演示会打开 RViz；停止命令在第二终端执行。每个案例最多 100 仿真秒，命令中的更大上限不会改变固定短任务；外层现实超时独立保护。

## 1. 已有结果，无需重跑

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh check S14
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh check S15
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh open S15
```

页面可选择四个场景、三种方法并拖动观测时间。应看到历史轨迹、多个未来分支及概率、真实未来观测标签、误差与闭环表格。N/A 表示没有足够样本，disabled 表示未满足启用门槛，不是把失败隐藏成成功。

首次启动前会校验完整前置证据、增量编译及离线评分，本机通常需等待约 1–2 分钟。终端出现 `S15 RUN` 后进入具体仿真；请不要因此并发重复启动。

## 2. 实际转弯演示

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh demo S15 --case prediction_turning --seed 41 --sim-seconds 240 --wall-timeout 600
```

RViz 中开启“S15 actual future modes and probabilities”，彩色分支随实际行人观测改变；行人采用刚体行走姿态网格，不宣称关节步态动画。车辆可能等待，报告必须说明预测等待还是当前障碍停车。两个点的目标数量始终为 2，每点反馈确认量为 2。

## 3. 推理超时降级

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh demo S15 --case prediction_timeout --seed 41 --sim-seconds 180 --wall-timeout 420
```

18–23 仿真秒丢弃实际推理响应，模拟超时。应出现 INFERENCE_TIMEOUT / FALLBACK；当前感知停车和工具反馈门仍有效，不能把回退解释成放开安全限制。正式矩阵另外包含真实预测进程 SIGSTOP 失联和协方差异常。

## 4. 自动复验和断点

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh test S15 --suite acceptance --seed 41 --sim-seconds 300 --wall-timeout 4200
```

运行 12 个正常对照和 3 个故障实验。机器较慢导致现实预算不足时会记 PARTIAL，已完成实验按源码、输入和证据哈希保存。保持源码不变后，将本轮目录替换进下面命令继续：

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh test S15 --suite acceptance --seed 41 --sim-seconds 300 --wall-timeout 3600 --resume-from /home/ceihao/robot_project/optimization_v2/results/S15/实际未完成运行目录
```

断点复用的是已完成的实验，不把未完成的任务改成 SUCCEEDED。demo 不覆盖正式验收回执。数据集已冻结时自动复验校验其哈希；新环境首次采集需先执行 `source optimization_v2/env.sh`，再运行 `optimization_v2/venvs/scene/bin/python optimization_v2/ws/src/motion_prediction/scripts/dataset15.py --wall-timeout 2400`，整个 episode 分组不会跨训练/测试混用。

## 5. 查看实时接口、停止与学习

第二终端加载环境后可以查看真实风险消息：

```bash
source /home/ceihao/robot_project/optimization_v2/env.sh
ros2 topic echo /s15/risk std_msgs/msg/String --once
ros2 topic echo /s14/directive std_msgs/msg/String --once
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh stop S15
```

stop 只处理本阶段记录且身份匹配的进程，先停导航和工具、保存账本。GUI 退出码不是人工验收；手动观察尚未进行时始终保留 NOT_CHECKED。

学习入口：TECHNICAL.md、CODE_WALKTHROUGH.md、RESULTS.md，以及 `/home/ceihao/robot_project/optimization_v2/lessons/S15/README.md`。依赖安装脚本为 `optimization_v2/scripts/install_s15_dependencies.sh`，不使用 sudo pip，不能与仿真并行解压安装。
