# 11 月 15 日人工复现与故障记录

11 月 19 日已针对下述中断开展实测修复。当前结果、手工入口和剩余工作见[三任务收尾复验](../收尾复验_20261119.md)。下文的计数对应 11 月 15 日的运行。

先使用已经过实际检查的 GUI 和较短演示学习，再处理完整任务的剩余问题。当前完整 A1 为 28/56 点，批准的 B1 安全执行版本为 19/47 段；小规模演示通过不替代完整任务。

## 先打开 GUI

```bash
cd /home/ceihao/robot_project
bash optimization_v2/three_tasks/gui.sh
```

浏览器打开 `http://127.0.0.1:8088`。另一终端从以下案例中选择一条执行，同一时间只运行一个：

```bash
cd /home/ceihao/robot_project
# 点喷：三点、真实卡滞、残余恢复和幂等反馈
bash optimization_v2/three_tasks/advance.sh R12 record spray
# 巡检：跟随前方移动物体及停车恢复
bash optimization_v2/three_tasks/advance.sh R12 record inspection_leader
# 割草：五段、启动阶段感知中断、取消及恢复
bash optimization_v2/three_tasks/advance.sh R12 record mowing_startup_perception_loss
```

任务总览看当前执行位置；决策页看原因码、候选行为和重试次数；传感器页看新鲜度与定位创新；账本页看有效作业量和剩余量。关闭 GUI 不会停止任务。需要结束任务时，在任务终端按 Ctrl+C 并等待停车和退出，不要只关闭窗口。

## 可直接复制的下轮提示词

> 基于当前 GitHub 同步版本继续修复 R10。先阅读 R10 的 RESULTS.md、GUI_AND_BOUNDED_REPAIR.md、validation/R10/bounded_A1_result.json、bounded_A1_heading_transitions.json、bounded_B1_result.json 及 terminal_live_evidence.json；本机证据原件位于 delivery/r10_completion_20261115。保持 GUI、通过的三点喷洒/五段割草录像、原地图、原任务面积、旧参考路径状态和所有原精度/安全门限。先离线核查 A1 point_11_21 附近三次旋转跳变与静止观测/里程计时刻是否一致，再核查 B1 safe_turn:5:3 停车重定位时 GNSS 被当前修正拒绝、恢复候选无法积累的条件；喷洒的独立候选 GNSS 检查可作为对照，但不要直接假定移植即可解决。用针对性的组件测试和短复验确认修复，再按新版本逐项复验受影响的完整 A1/B1。不要靠扩大阈值、增加无界重试、缩小面积分母或修改旧回执取得 PASS；不运行六十次矩阵。运行时用 GUI 展示数据、决策、控制和工具反馈，最后更新实际结果和 GitHub。

完整任务的手工运行仍有原 1800 仿真秒上限；实际现实时间取决于机器负载和实时因子。
