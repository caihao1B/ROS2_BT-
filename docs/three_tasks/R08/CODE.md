# R08 核心代码阅读

代码根目录：`optimization_v2/ws/src/task_platform`。

- `r08_backend/surface_statistics.hpp`：射线支持的地表分位数、空间支持的高低表面模式与强危险保留。`terrain_r08.py` 将其接入独立的生产地形可执行程序，不覆盖 R07。
- `r08_backend/global_consistency.hpp`：在机器人位置计算改正位移，在天线位置计算 GNSS/地图冲突。`terrain_r08.py` 生成独立 `r08_global_manager`。
- `r08_backend/policy.hpp` 和 `common.xml`：真实 BehaviorTree.CPP 节点。移动跟随不累计静止阻塞时间；观测前缀有效的规划补丁有独立执行分支。取消和物理停车确认仍由监督进程检查。
- `r08_nav/observed_prior.cpp`：把当前 odom 风险栅格按原采集时刻 TF 转进全局规划代价，已观测自由不能擦掉先验禁区，旧移动障碍不永久残留。
- `accumulation_r08.py`、`map_inputs_r08.py`：短时静态表面窗口，使用局部观测位姿作运动补偿，限制年龄并去重。
- `motion_consistency_r08.py`：同时间窗的 GNSS、激光和轮速位移一致性；`overlay_r08.py` 接入真实 ICP 增量门控及轮速协方差管理。
- `regional_r08.py`：捕获时间匹配、多初值重启、点面跟踪、GNSS 交叉验证和启动稳定性准入。
- `tracker_r08.py`：当前点云轨迹关联、路径投影、间距/TTC；不读取演员脚本。`prefix_r08.py` 在实际执行候选上检查完整足迹。
- `mission_r08.py`：发送真实 FollowPath/ComputePathToPose 动作，确认取消，等待新的 BT 上下文，检验补丁、重接、到达与残余。
- `control_r08.py`：唯一底盘出口，核对消息年龄、版本、定位、感知、BT 和规划前缀；限速时同步缩放角速度以保留曲率。
- `actors_r08.py`、`analyze_r08.py`：独立真值记录及离线评分。不能导入在线控制链。

`detour_r08.py` 是有界局部 A* 组件研究实现；当前在线绕行使用原生 Smac 加观测前缀验证，不把这个组件单测当成在线绕行证据。

`map_inputs_r08.py` / `accumulation_r08.py`：先对齐、去重实际多帧射线，再做数量准入；窗口不是重复复制点来增加数量。

`crossing_r08.py` / `tracker_r08.py`：用 A P Aᵀ 传播平面跟踪协方差，比较未来 0.2–2.0 s 的障碍包络和路径包络；输出冲突时刻与最小预测裕量，`control_r08.py` 把它作为独立停车条件。

`mission_r08.py`：依据当前障碍距离选择其后方回接点；维护路径时间戳，等待与该候选一致的观测验证，发送真实 FollowPath，测量回接后恢复原义务。`analyze_r08.py` 在隔离真值域检查真实距离、接触和终点，不参与车辆控制。

`shutdown_r08.py`：将 ROS 输入生产节点与估计器分组停止，避免订阅数据仍持续进入时并行拆除估计器。`ros_shutdown.json` 逐进程记录退出阶段、耗时、退出码和是否强制终止。任务抵达、实际停车、Nav2 关闭、进程干净退出是分开的检查。

`rig_r08.py` 将实际射线配置与近地传感器外参纳入新版本，验证旧标定字段没有被隐式修改。场景、噪声产生器、地图分支、BT 与定位证据读取同一有效配置。`transport_r08.py` 生成当前会话专用 DDS 缓冲配置。`map_inputs_r08.py` 日志记录收到的真实点云宽高，防止仅修改说明却没有增加实际采样。`probe_r08.py` 只做原地定位诊断，不发布导航动作或作业信用。

`r08_backend/cooperative_shutdown.hpp` 把异步停止请求与 ROS context 销毁分开。C++ 信号处理函数只设置 `sig_atomic_t`；执行器返回后才在主线程清理。`shutdown_probe.cpp` 与 `test_shutdown_r08.py` 通过真实进程信号验证“回调尚在执行时 context 不被销毁”，不是只检查字符串或模拟退出码。

`crossing_r08.MotionMemory` 保存 `track_id -> last_observed_motion_time`。`tracker_r08.py` 每帧先更新这份有界记忆，再传入 `crossing(..., recent_motion=...)`。它只改变是否需要继续评估动态风险，不伪造速度，也不把历史观测作为当前地面可通行证据。

`activity_r08.ActivityMemory` 用最新可观测轨迹更新有限时长的位置样本；`apply_grid` 仅增加避让代价，永远不把未知格清成可通行。`route_conflict` 检查沿实际当前路径的 1.5 米车身扫掠，并给出越过活动区域的接回距离。Python 跟踪节点发布 `/r08/dynamic_risk`；C++ `ObservedPriorLayer` 把该实时网格叠入 Nav2，Smac 生成候选路径，独立实时前缀验证与 RPP 执行保持分工。

`tracking_error_r08.errors` 只用于离线评分，按时间切换实际发布的路径参考。它同时拒绝未准入候选上的运动，避免只换评分参考就获得通过。原连接路偏离量与有效路径跟踪误差分别保留。
