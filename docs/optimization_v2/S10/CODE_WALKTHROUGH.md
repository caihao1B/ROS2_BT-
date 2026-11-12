# S10 核心代码导读

先阅读 TECHNICAL.md 理解请求、反馈、确认和真值评分的分工。下面函数行号对应本轮源码，点击可直接定位。

## C++ 动作状态机

- [void accept](../../../optimization_v2/ws/src/tool_sim/src/action_server.cpp#L34)
- [void tick](../../../optimization_v2/ws/src/tool_sim/src/action_server.cpp#L43)
- [void begin_stop](../../../optimization_v2/ws/src/tool_sim/src/action_server.cpp#L33)
- [void complete](../../../optimization_v2/ws/src/tool_sim/src/action_server.cpp#L29)
- [void request](../../../optimization_v2/ws/src/tool_sim/src/action_server.cpp#L28)
- [ToolServer](../../../optimization_v2/ws/src/tool_sim/src/action_server.cpp#L79)

## C++ 仿真物理工具

- [void Configure](../../../optimization_v2/ws/src/tool_sim/src/ToolActuator.cc#L30)
- [void PreUpdate](../../../optimization_v2/ws/src/tool_sim/src/ToolActuator.cc#L38)
- [void PostUpdate](../../../optimization_v2/ws/src/tool_sim/src/ToolActuator.cc#L67)

## Python 事务与几何

- [def footprint](../../../optimization_v2/ws/src/work_ledger/scripts/ledger_core.py#L11)
- [def reserve](../../../optimization_v2/ws/src/work_ledger/scripts/ledger_core.py#L33)
- [def guard](../../../optimization_v2/ws/src/work_ledger/scripts/ledger_core.py#L54)
- [def observe](../../../optimization_v2/ws/src/work_ledger/scripts/ledger_core.py#L69)
- [def finish](../../../optimization_v2/ws/src/work_ledger/scripts/ledger_core.py#L101)
- [def snapshot](../../../optimization_v2/ws/src/work_ledger/scripts/ledger_core.py#L24)

## Python 在线配对与服务

- [def observe](../../../optimization_v2/ws/src/work_ledger/scripts/ledger_node.py#L22)
- [def service](../../../optimization_v2/ws/src/work_ledger/scripts/ledger_node.py#L26)
- [def publish](../../../optimization_v2/ws/src/work_ledger/scripts/ledger_node.py#L35)

## Python 定位/传感器适配

- [def loc](../../../optimization_v2/ws/src/tool_sim/scripts/online_adapter.py#L26)
- [def raw](../../../optimization_v2/ws/src/tool_sim/scripts/online_adapter.py#L36)

## Python 场景客户端

- [def goal](../../../optimization_v2/ws/src/tool_sim/scripts/client10.py#L36)
- [def execute](../../../optimization_v2/ws/src/tool_sim/scripts/client10.py#L44)
- [def ensure_success](../../../optimization_v2/ws/src/tool_sim/scripts/client10.py#L69)

## Python 独立评分

- [def observe](../../../optimization_v2/ws/src/tool_sim/scripts/evaluator10.py#L11)

## 怎样沿一次喷洒追踪数据

1. client10.goal 生成带版本与幂等键的目标，execute 发送真实 ROS action。它不会把 accepted 当成功；等待 feedback 与 terminal result。
2. C++ accept 异步请求账本 reserve。reserve 的 payload 哈希必须匹配已有键；已完成直接回放。新工作先事务预留，确认不等于执行。
3. C++ tick 检查反馈时效、定位/目标/速度/工作段、实际停转，再发 ON。PreUpdate 只操作独立工具关节，并读取物理 JointVelocity；延迟反馈经过 online_adapter，未带工具真值位姿。
4. ledger_node 将工具时刻与最近估计姿态配对；ledger_core.observe 再验时间、epoch、seq、版本、attempt、准入与连续工作。有效才在一个事务中插入事件并更新剂量/并集。
5. C++ 收到确认量达到目标后发 OFF，等待实际停转后 finish 写终态，再完成 ROS action。取消也经过停转确认；失联不能伪造停止。
6. 独立 evaluator 在 domain 120 从物理工具位姿计算作用量；analysis10 在会话结束后交叉对照。在线系统不会加载这份评分结果。

## 割草和异常的关键不同

割草调用 footprint，把有效刀幅的观测矩形内缩，与任务多边形相交并求 union；使用 difference 给出残余。掉帧不插补路径，TURN/TRANSIT 不计入。重试复用原键和原目标，完成回放不启动转子。部分任务继续时边缘可有必要的物理重叠，但面积并集不会重复累计。

SQLite 在启动时把遗留 RUNNING 变为 RECOVERY_REQUIRED；这是“可能已经发生未确认作业”的保守解释。失联与恢复过程保留剂量和几何，不清零、不盲目重喷。

## 教学与生产的界线

[lesson.cpp](../../../optimization_v2/lessons/S10/lesson.cpp) 是不依赖 ROS 的停转状态机教学缩减；[lesson.py](../../../optimization_v2/lessons/S10/lesson.py) 直接运行生产账本引擎，但输入为明确的合成观测。完整 ROS 服务、Gazebo 物理效果与启动管理分别在生产代码中，不能由教学例通过替代。

`ledger_core.latest_at` 只取不晚于权威时钟的上下文；`online_adapter.flush_poses/process_pose` 缓存并按事件时刻配对状态、地形和轮速，解决 DDS 到达顺序造成的假过期。原定位输出未被改写。

最终高频路径：`snapshot(..., geometry=False)` 只读标量，不计算/发布完整多边形；`observe` 的割草几何使用 5 mm 固定精度和额外内缩，抑制数值细碎顶点。`reserve(..., now)` 在同一事务记录真实准入时刻；报告使用数据库 transitions，重复或冲突查询不会覆盖已完成任务的终态。

割草交接：`guard(..., for_start=True)` 允许健康工具在原地启动，默认 `guard` 仍要求合法行驶速度才能计量。C++ 由真实转速维护 `working_confirmed`；`motion10.py` 在 COVERAGE 段只有收到该确认才允许发 DRIVE。转场必须已经停刀。
