# S10 → 后续喷洒/割草机械臂项目的接口合同

版本 `S10-TOOL/1`，能力范围 `SIMULATION_DIAGNOSTIC_ONLY`。ROS action `/s10/tool_task`，类型 `mission_interfaces/action/ToolTask`；能力查询 `/s10/tool/capabilities`，类型 `mission_interfaces/srv/ToolCapabilities`。外部机械臂可替换硬件适配器；任务编排继续使用相同 action。

- `POINT_SERVICE=1`：在 map 目标处驻停，目标条件通过，反馈确认的归一化剂量达到请求值后关闭；默认 S03 剂量时长 2 s、作用半径 0.20 m、到位误差 0.08 m。
- `START_WORK=2`：在 COVERAGE 段、0.02–0.12 m/s、角速度 ≤0.097 rad/s 下累计观测足迹并集；有效刀幅 0.90 m，包络沿用 S03。达到目标面积的 95% 后关闭，保留残余多边形。
- `STOP_WORK=3`：抢占正在执行的工作并等待实际停止。停车/紧急停止不能等待工具慢速停机；正常转场应等 `STOPPED`。取消与 `STOP_WORK` 的活动任务结果为 CANCELLED。
- action goal 接收仅表示受理。feedback 公开 phase/progress/confirmed_quantity/health/elapsed_sim_s；result 公开终态、剩余量、残余 GeoJSON、错误码、replayed、uncertainty_pending。
- 同一幂等键必须逐字段相同（包括任务、操作、map_id/map_revision/frame_epoch、plan/tool 版本、目标、段、量、时限）。SUCCEEDED 重复只返回存档，不发 ON；更改 payload 返回 IDEMPOTENCY_CONFLICT。要改变目标/地图须新计划和新键，不能覆盖旧工作。
- 已确认停止且部分完成的任务可用原键重试，累计值和几何并集保留；失联或重启中断的任务设 RECOVERY_REQUIRED，需独立核查真实工具/作业结果后由后续任务管理器决定新计划。当前没有自动“清除不确定”后门，尤其不自动重复喷洒。
- 动作终态：SUCCEEDED / PARTIAL / FAILED / CANCELLED / LOST；STOPPED 是关闭动作成功；RECOVERY_REQUIRED 是恢复准入拒绝。典型错误：VERSION_*、IDEMPOTENCY_CONFLICT、NOT_DOCKED、NOT_STOPPED、LOCALIZATION、TARGET_CONDITION、WORK_SPEED、NON_WORK_SEGMENT、TOOL_HEALTH、FEEDBACK_LOST、CLOCK_WATCHDOG、DEADLINE。
- 适配器必须提供独立于命令的工具转速/阀流量观测，带 epoch、序号、观测时刻、tool_revision、boot、attempt、key、健康度。不能把控制器期望转速当反馈。流量应来自流量计或具有标定误差的估计，并声明单位。当前 flow 为无量纲额定流量，quantity 单位剂量秒，不能解释为升。
- 一个 action server 是唯一工具命令出口；C++ Gazebo 插件是仿真物理执行出口。工具节点不发车辆速度；机械臂扩展也不能抢占底盘唯一出口或自行发布 map→odom。机械臂碰撞包络、标定、工具版本变化必须重新准入。
- 近景相机/检测/VLA 后续只提供有时戳和置信度的目标条件及建议，受确定性联锁约束；当前 target_ok 为显式诊断条件，不是假称视觉识别。无机械臂依赖，无 MoveIt/VLA 安装。
- SQLite 单写入节点使用 WAL + synchronous=FULL；反馈事件去重和几何/计量更新在同一 BEGIN IMMEDIATE 事务。数据库不是多机器分布式事务；当前依靠主机进程所有权与 ROS domain 隔离，不宣称 SROS2 安全认证或工业 PL/SIL 等级。

独立评分仅在 domain 120 消费 `/s10/evaluation/effect` 的物理关节/位姿；在线 domain 119 绝不能订阅真值。仿真原始传感器 domain 121，误差注入仍保留。

割草启动与计量分开：原地启动转子可获得 `start_valid`，但 `valid`（计量）仍要求至少 0.02 m/s。车辆收到实际连续工作确认 `working_confirmed` 后才进入 COVERAGE 行驶。后续硬件适配必须保留这一就绪交接，不用“ON 命令已发”替代实测确认。

`requested_quantity` 在 S10 表示最低确认量，不是精准剂量上限。确认量可能低于实际物理作用量；当前没有最大施药量/流量闭环的验收承诺。后续真实喷洒适配必须增加经过标定的流量控制与过量约束，不能将本步通过解释为精准施药通过。
