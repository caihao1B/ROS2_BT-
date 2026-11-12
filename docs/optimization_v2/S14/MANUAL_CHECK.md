# S14 手工验收

逐条执行，不并行开启两套仿真。执行器会加载环境并检查 S10/S13。demo 默认打开 RViz，结束后打开离线报告。自动结果不代表人工已验收。

一个终端：

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh check S14
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh open S14
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh open S14 --print-path
```

先查看本轮状态、reason_code 时间轴、实际 BT 高亮、固定分母和剩余点/区域。PARTIAL 的 check 返回非零是未满足验收条件，不是该命令要求你放宽安全门限。

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh demo S14 --case decision_story --seed 41 --sim-seconds 300 --wall-timeout 720
```

这是短任务 SMOKE：固定两个点的到达、服务反馈，移动中暂停真实定位观测进程5秒，检查停车、等待、恢复/重规划及账本。自然环境的动态障碍也可能导致保护停车。重点检查原因与实际输入相符，不能把本案例当成21点整簇成功。

早期版本曾运行公开 GUI 命令及无界面短测；这些历史记录不代表当前地图和执行路线已经验收。当前结果应以报告中的 run_id、地图/计划版本及实际指标为准。demo 的 PASS 仅表示该演示的故障/恢复/停止断言成立，实际作业状态仍看 mission.state 和固定分母。运行负载与观测时序会影响完成情况，不能把成功启动窗口当作任务通过。

报告滑条同步决策证据、真实 BT 节点状态和最近的已记录估计位姿；地图的覆盖和残余明确显示最终账本状态。RViz 中部分显示项沿用 S13 名称，因为 S14 复用了已有导航可视化话题。

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh demo S14 --case nominal_spray_cluster --seed 41 --sim-seconds 1800 --wall-timeout 5400
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh demo S14 --case nominal_mow_cluster --seed 41 --sim-seconds 1800 --wall-timeout 5400
```

A1 固定21点，B1固定16.3594578796 m²。检查定位拒绝与准入、工具启停、有效反馈、原始分母。区域配准已有独立留出测站验证，整簇运行仍须持续获得在线定位准入；起点在90仿真秒内未满足准入会停车并保存PARTIAL，不得手工解锁或把粗初值当作定位。现实超时不代表完成1800仿真秒，完整耗时以报告为准。

割草的每条带结束时主动发送 STOP_WORK，持续作业的 ToolTask 因而可能记录 `state=CANCELLED, reason=STOP_WORK`；这是分段关工具，不等于整簇失败。应同时检查 `uncertain=false`、停止确认、真实反馈面积并集和最终 `mission.state`。喷洒点则必须有去重后的成功数量确认，不能套用割草的判断方式。

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh test S14 --suite acceptance --seed 41 --sim-seconds 1800 --wall-timeout 10800
```

自动测试串行运行真实 BT 协议、正常短任务、两整簇尝试，以及预算不足、持续定位观测失联、工具反馈丢失。故障测试通过仅表示正确停车、保存剩余和不确定项，不表示作业完成。整个矩阵可能需要较长现实时间。

第二终端的停止命令：

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh stop S14
```

只停止本次匹配 PID/创建时间/UID 的会话。先撤销导航和工具，保存 SQLite/残余，再关闭进程。若工具记录 uncertainty_pending=true，不可盲目重试喷洒；先对照物理效果和反馈记录。

```bash
cat /home/ceihao/robot_project/docs/optimization_v2/S14/TECHNICAL.md
cat /home/ceihao/robot_project/docs/optimization_v2/S14/CODE_WALKTHROUGH.md
cat /home/ceihao/robot_project/docs/optimization_v2/S14/RESULTS.md
cat /home/ceihao/robot_project/optimization_v2/lessons/S14/README.md
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh next S14
```

## 中断后的已完成案例复用

自动验收可追加 `--resume-from`，指向本步已有的完整运行目录。仅当完整簇完成、固定分母正确、源码/配置/地图/计划/前置回执完全一致，且逐个证据文件哈希校验通过时才复用；残缺案例或变更版本会重新运行。复用保留原始仿真时间与来源，报告不把它写成本次新跑的仿真。
# 阿克曼转场诊断（不代替完整簇）

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh demo S14 --case ackermann_navigation_probe --seed 41 --sim-seconds 300 --wall-timeout 1000
```

在原 B1 第四转场的起终点间运行 Hybrid-A* + RPP，工具全程关闭。检查该演示目录的 `client/planned_paths.jsonl`、`client/events.jsonl` 和 `planner.log`：应看到 SmacPlannerHybrid、NavigateToPose/RPP 以及实际路径；不能出现 INVALID_CONTROLLER。此诊断范围为 SMOKE，覆盖分母为零，不代表 B1 16.3594578796 m² 已完成。


## 新版实测路线转场检查

单独执行，不与其他仿真并行。该命令从新版最长转场起点出发，工具保持关闭；查看报告的实际轨迹、FollowPath 分段结果、停止转向对齐和取消后恢复记录。它不产生作业面积。

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh demo S14 --case ackermann_registered_route_probe --seed 41 --sim-seconds 300 --wall-timeout 1000 --headless
```

地图重建后，须依次重新生成执行路线并复验，不能手改版本号：

```bash
cd /home/ceihao/robot_project
source optimization_v2/env.sh
optimization_v2/venvs/scene/bin/python optimization_v2/ws/src/mission_bt/tests/build_atlas14.py
optimization_v2/venvs/scene/bin/python optimization_v2/ws/src/mission_bt/tests/build_routes14.py
optimization_v2/venvs/scene/bin/python optimization_v2/ws/src/mission_bt/tests/routing_contract14.py
```

如果原观测的地图身份不再匹配，路线构建器会拒绝；此时需要重新采集已准入的传感器代价图，不能绕过身份校验。

## 陈旧地形风险的真实样本回放

下面是独立组件回放，不启动Gazebo，也不替代完整簇验收：

```bash
/home/ceihao/robot_project/optimization_v2/ws/install/mission_bt/lib/mission_bt/risk_evidence_test /home/ceihao/robot_project/optimization_v2/ws/src/mission_bt/tests/fixtures/stale_risk_evidence.json
```

应输出`ACTUAL_STALE_RISK_REPLAY_PASS`。它检查真实5次退化观测不能刷新旧风险时间，原2秒后输出未知；持续正负障碍仍否决，三帧有效空闲才清除内部致命状态。运行中的`/s09/terrain/cells`额外包含风险年龄与内部存储值，可以区分当前有效风险和已过期记录。

### 树干邻域与停靠回归

先运行生产几何函数的真实观测重放（不启动仿真）：

```bash
/home/ceihao/robot_project/optimization_v2/ws/install/mission_bt/lib/mission_bt/local_surface_test /home/ceihao/robot_project/optimization_v2/ws/src/mission_bt/tests/fixtures/local_surface.json
```

输出应包含 `ACTUAL_TREE_NEIGHBOR_REPLAY_PASS: 26 cells` 和正负 6 cm、5° 拒绝验证。随后可以单独运行曾经受阻位置的两点短任务：

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh demo S14 --case spray_registered_docking_probe --seed 41 --sim-seconds 180 --wall-timeout 720
```

查看车从 A1_47 到 A1_51，经实际地图定位与 DWB 导航，到位后工具反馈入账。它只验证两个点，报告必须显示 SMOKE；完整喷洒验收仍要求 21/21。停止方法仍是 `stepctl.sh stop S14`，不要并行开第二个 Gazebo。

### 观测恢复专项回放

```bash
/usr/bin/python3 /home/ceihao/robot_project/optimization_v2/ws/src/mission_bt/tests/admission_recovery14.py
```

应看到两段实际失联证据恢复时间及 `NO_GLOBAL_REPLAN`。边界测试还检查闪断重置、45秒失联、时钟倒退、12次恢复总上限和真实障碍不被忽略。它是日志/状态机回放；真实动作链使用上面的 `decision_story` 演示，观察 `OBSERVATION_RECOVERY_WAIT` 与工具反馈。
