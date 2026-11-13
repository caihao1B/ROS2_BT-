# S17 暂停与恢复检查点（2026-11-13）

用户为切换系统要求本轮在十分钟内保存、上传并暂停。S17 为 **PARTIAL，正式六簇成绩 0/60**；S18 未执行。当前项目位于 `/home/ceihao/robot_project`，不是已不存在的新加卷 RobotShared。

## 本轮可保留成果

- 连续区域/连接路测量地图和六簇计划已绑定：atlas `optimization_v2/maps/s17_full/manifest.json`，map_id `4e4b85d7-ccf3-58ed-925b-1efba3fa6262`；plan `optimization_v2/plans/s17_full/mission_plan.json`，revision `463a8579db9c153339eeef840c88c5e07651f180449104238a56c11894d1b205`。原数据库不改动。
- 修复授权边界重复计算车宽、巡检剩余工时重复计入已完成子段、障碍记忆随 odom 漂移。增加实际终点顺序与全车身越界审计，真实路段终点及作业分母不变。
- 差速短程 `pilot_connector_diff_20261113_081237`：PASS，338.987 仿真秒、P95 0.02320149 m、零接触和越界、最大停车延迟 0.386 秒。这一例早于 map 记忆修复，修改后的差速版本尚待复验；更不能替代完整连接路。
- 最新阿克曼短程 `pilot_connector_ackermann_20261113_084204`：PARTIAL，196.121 仿真秒、P95 0.01957667 m、零接触和越界、最大停车延迟 0.235 秒。第二段 `BLOCK_RETRY_EXHAUSTED`。
- A 区凹口 v3、含洞 v2 离线均为 GEOMETRY_VALID，三个连通簇、全部 66 点保留，名义工时符合要求；物理变体闭环 NOT_RUN。B 变体规划器已写，尚待执行与核验。
- 已有 54 组离线分簇和 18 个配对；尚无全部规定的闭环消融/故障正式成绩。Gazebo 收尾多次退出 -11，已停净进程仍不代表该问题修复。

## 断点的诊断证据

最新阿克曼案例内保留 `planner_snapshot17.json`、`actual_navmap.png`、`offline_planner_diagnostic/`、`envelope_diagnostic.json`。实际格网中圆形 0.6 m 清障半径有连通路线，0.7 m 不连通；原车身 + Dubins 最小转弯半径 2.11 m 的安装版 Hybrid-A* 搜索失败。72/144/288 角度格、插值及更换临时目标未解开阻塞。缩小诊断足迹也未找到路径（部分撞上百万迭代上限），因此 **不能断言只是车身太宽或几何绝对不可行**。诊断中修改足迹只用于离线分类，未投入车辆。

动态历史的半径约 0.529–0.586 m，额外格子量化裕量约 0.106 m；45 秒历史涵盖了行人整个往返周期，可能使通道长期保守封闭，需要进一步分离真实障碍、历史占用和运动学可行性。不得删行人、缩小真实足迹、放宽碰撞或停车标准来通过。原非交互脚本行人曾主动撞上已停车辆，原失败仍保留。

还发现现场代价地图快照仅有 0/254/255；独立 Nav2 两插件复验能产生膨胀梯度、标记/清除/未知/过期/错误坐标系均通过。应继续核查实际快照/运行层次，尚不能认定系统膨胀实现失效。

## 恢复顺序与命令

先读 AGENTS.md、本文件、S17 RESULTS/TECHNICAL 与 prompts/S17.md。新终端：

```bash
cd /home/ceihao/robot_project
source optimization_v2/env.sh
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
bash optimization_v2/stepctl.sh status
optimization_v2/venvs/scene/bin/python optimization_v2/ws/src/acceptance_runner/scripts/audit17.py
```

1. 确认当前没有残留仿真进程、检查有效地图和前置哈希。用最新保留的实际地图诊断阿克曼阻塞；先离线检查碰撞器/路径搜索和分层占用，不盲目重跑六簇。必要时采集真正的安装版全局/局部代价图与参数。
2. 修复后跑组件检查，再分别复验两底盘动态短段，然后完整连接路。当前开发命令：

```bash
optimization_v2/venvs/scene/bin/python optimization_v2/ws/src/acceptance_runner/scripts/verify17.py
optimization_v2/venvs/scene/bin/python optimization_v2/ws/src/acceptance_runner/scripts/pilot17.py connector_ackermann --dynamic-prefix --seconds 600 --wall 1500 --rtf 1
optimization_v2/venvs/scene/bin/python optimization_v2/ws/src/acceptance_runner/scripts/pilot17.py connector_diff --dynamic-prefix --seconds 600 --wall 1500 --rtf 1
```

3. 运行稳定后冻结输入，六簇每簇十个固定种子，单例含收尾≤1800 仿真秒；完成全部故障、变体资格和配对消融，失败保留且先诊断，不跳失败凑成绩。正式分片命令见 MANUAL_CHECK.md。
4. 只有全部条件满足后才发布 S17 PASS、同步完整项目及必要资产。**不执行 S18**。当前 GitHub 上传是源码/文档/小型诊断证据检查点，不代表完整 S17 资产移植或验收通过。原始大日志、测站点云和地图资格原始证据保留在本机。

续做提示词：

> 从 `/home/ceihao/robot_project/docs/optimization_v2/S17/PAUSE_RESUME_20261113.md` 的检查点继续。先核查项目进程、回执及地图版本，修复阿克曼动态段无可行路径并复验改动后的差速车；再补齐完整连接路、六簇六十次正式验收、故障和配对消融。保持原始作业分母、真值隔离和安全门限，不把短程 PASS 当作 S17 PASS。完成 S17 后同步 GitHub；不要执行 S18。
