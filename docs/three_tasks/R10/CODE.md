# R10 核心代码

源码目录：`optimization_v2/ws/src/task_platform/`。

|文件|执行逻辑|
|---|---|
|`scripts/mission_spray_r10.py`|真实 NavigateToPose → 车辆/工具停车 → 全局静止位置窗口 → 必要时局部 FollowPath 精停 → ToolTask；失败只补残余，取消和人工锁存有界处理|
|`scripts/progress_spray_r10.py`|接近目标的有界进展判据；`stopped_decision` 让停车等待响应后到的定位重验证锁存，人工停止优先|
|`scripts/approach_handoff_r10.py`|对新鲜、已准入且有平移命令的近目标停滞提出取消接近动作；必须实测停车并通过原精停检查，不产生导航成功或作业信用|
|`scripts/docking_r10.py`|保留原喷洒精停样本数、时间跨度、距离和散布门限|
|`scripts/local_dock_r10.py`|静止全局观测中位数形成最多 0.15 m 的候选修正；将整个路径一次转换至局部 odom，包含旋转扫掠；本函数没有控制或作业计量权限|
|`scripts/revalidation_spray_r10.py`|原先验区域内整车扫掠检查与路径摘要；运行时另须实际 Nav2 IsPathValid|
|`r10_backend/correction_filter.hpp`|C++ SE(3) 修正低通：平移插值、四元数 slerp；原始异常监督在调用前执行|
|`r10_backend/stopped_revalidation.inc`|停车后候选确认、保持锁存的 prepare、带地图/计划/epoch/路径证据的 commit，再经过原确认窗口恢复|
|`scripts/spray_global_r10.py`|点喷独立原生管理器：用原始合格固定解及其采集时刻的里程计检验停车候选，不依赖旧位置已经接受的 GNSS 计数；行驶监督门限不变|
|`scripts/near_mowing_r10.py`、`self_capture_mowing_r10.py`|各雷达在自己的采集车体坐标中按原车体遮罩剔除自回波，再做原有运动补偿；保留低矮地面例外，不扩大遮罩|
|`scripts/mission_mowing_r10.py`|逐个当前版本执行段执行；START_WORK 持续响应 BT；恢复按挡位执行并在换向前实测停车|
|`scripts/arrival_mowing_r10.py`|取消导航并停车后，用连续观测确认原 0.10 m / 0.12 rad 段到达标准，避免对已实际到达的目标反复停车调整|
|`scripts/recovery_mowing_r10.py`|有界关刀退让候选、曲率/整车轮廓、挡位分段、局部进度索引、跟踪偏差、换挡停车准入|
|`scripts/start_work_r10.py`|启动工具过程中每次 tick 都优先检查新鲜 BT 指令；取消先于工作确认|
|`scripts/planning_envelope_r10.py`|解析车身角点在朝向设计扰动区间内的极值，加入 Nav2 膨胀、位置余量及栅格半对角线，生成独立规划包络；物理车身和到达门限不变|
|`scripts/prepare_safe_plan_r10.py`、`safe_plan_r10.py`|裁剪原条带、显式保留遗漏原义务、验证完整包络与版本，原作业多边形不因裁剪变小|
|`r10_backend/safe_route.cpp`|OMPL Reeds–Shepp / 有界 RRTConnect 用扩展包络检查连续连接，保留精确换向尖点，C++ 只输出候选，不发布控制|
|`scripts/diagnostics_mowing_r10.py`|只读派生原 R09 诊断器，每 5 仿真秒保留一份完整风险/代价图，上限 400 份，避免只记录开头而丢失后半程失败现场|
|`scripts/ledger_mowing_r10.py`、`ledger_spray_r10.py`|仅把心跳调度改成单调墙钟；原反馈准入、仿真时间戳、面积/剂量和幂等键保留|
|`r10_backend/tool_bridge.cpp`|使用现有 ros_gz_bridge 转换器；先取消/释放 Gazebo 传输资源，后关闭 ROS context，修复退出崩溃|
|`scripts/analyze_spray_r10.py`、`analyze_mowing_r10.py`|只在会话结束后读取隔离真值、接触、物理效果和账本；禁止在线控制导入|
|`scripts/stage_r10.py`|重新分析同版本实际任务、原生故障探针、完整原分母与历史封存案例；缺少任一必要项保持 PARTIAL|

Python 任务层只发 Nav2/ToolTask 动作，不直接发布底盘或工具执行指令。唯一底盘出口独立检查时钟、观测、定位、BT、挡位、速度/曲率和工具状态；唯一工具出口独立检查工作准入、反馈与停止确认。原始工具账本数据库不能用路径进度覆盖。

阅读顺序：先看 mission 的 `main` 状态机，再看纯判据函数，最后对照运行目录 `mission_events.jsonl`、`control/gate.jsonl` 与 `ledger/ledger.sqlite`。纯函数测试证明局部判据，真实任务和故障录像才证明整条链路。
## 条带受阻后的接回

`coverage_rejoin_r10.py` 沿原条带弧长提出前方候选；候选没有控制权和作业信用。`mission_mowing_r10.coverage_rejoin` 先确认停车、关刀，再用当前 Nav2 代价地图和 SmacHybrid 验证候选及接回后的原路径。连接由 `recover_endpoint` 按挡位实际执行；随后恢复同一原任务键和原面积。`analyze_mowing_r10.py` 用独立物理刀具效果核对反馈面积，跳过部分仍为残余。诊断的非零 `segment_start` 无法通过 R12 完整任务范围检查。

`local_dock_r10.ready_for_stopped_observation` 只判断是否值得提前结束局部运动、转入实际停车和原全局精停检查。连续局部距离≤10 mm 不等于喷洒准入；事件明确 `navigation_success=false`、`work_admitted=false`。这样不会为了继续追逐 Nav2 内部 8 mm 目标而把微小动作拖到通用无进展保护，原 BT 门限和人工锁存仍完整生效。

`mission_spray_r10.settle_and_redock` 只有在仍有物理修正次数时，才允许“稳定偏差”提前结束观察并申请下一次移动。用完两次修正后的最终检查仍使用原 8 秒窗口；窗口内必须满足原稳定性判据，否则报错。`test_final_dock_observation_r10.py` 直接执行这一生产函数，验证延迟收敛和持续偏差两种情况，且两种情况都不得多发第三次移动。



## 用户批准的新安全路径版本

B1 已新增独立安全执行版本，原 34 段参考和 PARTIAL 记录保留；原面积、原精度、安全及时间门限不变。算法、C++/Python 逻辑及手工检查见 [安全执行版本说明](SAFE_EXECUTION.md)。仅以实际运行回执判断新版本是否通过。
