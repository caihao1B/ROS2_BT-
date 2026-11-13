# 农田自主作业项目 · 当前开发机入口

S01–S16 有通过回执，S17 与 S18 均为 PARTIAL。S18 本轮先交付启动、诊断、学习资料、存储组件和隔离外观预览，不代表全地图正式验收已经完成。

[快速开始](/home/ceihao/robot_project/docs/optimization_v2/快速开始.md) · [18 步学习导航](/home/ceihao/robot_project/docs/optimization_v2/学习导航.md) · [S18 手工检查](/home/ceihao/robot_project/docs/optimization_v2/S18/MANUAL_CHECK.md) · [任务 Prompt 合集](/home/ceihao/robot_project/docs/optimization_v2/命令Prompt合集.md)

```bash
bash /home/ceihao/robot_project/optimization_v2/run.sh doctor
bash /home/ceihao/robot_project/optimization_v2/run.sh status
bash /home/ceihao/robot_project/optimization_v2/run.sh spray --cluster A1 --gui
bash /home/ceihao/robot_project/optimization_v2/run.sh mow --cluster B1 --gui
bash /home/ceihao/robot_project/optimization_v2/run.sh inspect --vehicle diff --gui
# 每次只启动一项，另一个终端可运行：
bash /home/ceihao/robot_project/optimization_v2/run.sh stop
```

默认短测 180 仿真秒、540 现实秒。原地图和正式任务保持不变；短测未完成不计作全簇通过。每次运行均记录地图/计划版本、反馈账本、残余、定位与安全评分。`status` 只快速读取回执，`audit` 执行文件级新审计。新电脑从零安装和完整工作区重建尚未验收。

前方障碍距离跟随目前仅离线核心通过检查，ROS/BT 闭环待实现。外观预览未替换正式模型。完整 S18 前置仍要求 S17 通过，不能用独立部分交付覆盖这个条件。
