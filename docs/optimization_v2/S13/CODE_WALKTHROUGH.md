# S13 核心代码导读

生产链：S04/S05/S07/S08/S09 → Nav2 → C++ Gate → S03 Drive → Gazebo 轮关节。评估器在运行结束后读取独立域物理真值。

| 文件 / 当前行 | 函数 | 职责 |
|---|---|---|
| [src/measured_controller.cpp](../../../optimization_v2/ws/src/navigation_adapter/src/measured_controller.cpp#L31) | `computeVelocityCommands` | 按 TF 最新共同时刻同步位姿，拒绝过期；调用上游控制器并统计含适配开销的耗时。 |
| [src/command_gate.cpp](../../../optimization_v2/ws/src/navigation_adapter/src/command_gate.cpp#L22) | `Gate::tick` | 定位、感知、工具、版本和租约共同准入，唯一底盘输出、加速度与曲率限幅；原门限下连续实测停稳 0.25 s 后恢复使能。 |
| [src/WheelDrive.cc](../../../optimization_v2/ws/src/navigation_adapter/src/WheelDrive.cc#L83) | `WheelDrive STEER` | 实测停止时控制真实转向关节，零驱动速度。 |
| [src/clock_relay.cpp](../../../optimization_v2/ws/src/navigation_adapter/src/clock_relay.cpp#L14) | `clock callback` | 保留真实时钟值和倒退事件，仅减少时钟广播。 |
| [scripts/client13.py](../../../optimization_v2/ws/src/navigation_adapter/scripts/client13.py#L106) | `execute` | 真实动作、取消、抢占、进度与终态。 |
| [scripts/client13.py](../../../optimization_v2/ws/src/navigation_adapter/scripts/client13.py#L83) | `follow_resuming / revalidate` | 保留 map 坐标剩余路径，重新准入后恢复。 |
| [scripts/client13.py](../../../optimization_v2/ws/src/navigation_adapter/scripts/client13.py#L70) | `align` | 等待停车与转角编码器确认。 |
| [scripts/prior_cost13.py](../../../optimization_v2/ws/src/navigation_adapter/scripts/prior_cost13.py#L18) | `in_map / fuse` | 静态测量图与实时风险合并，保守转换 map 栅格。 |
| [scripts/calibrate13.py](../../../optimization_v2/ws/src/navigation_adapter/scripts/calibrate13.py#L19) | `fit` | 独立训练/留出观测的局部地图配准。 |
| [scripts/registered_map13.py](../../../optimization_v2/ws/src/navigation_adapter/scripts/registered_map13.py#L10) | `validate_registration` | 核查配准观测哈希、地图/GNSS版本和留出误差。 |
| [scripts/runtime13.py](../../../optimization_v2/ws/src/navigation_adapter/scripts/runtime13.py#L29) | `run_case` | 物理仿真、三域隔离、真实进程重启和观测中断。 |
| [scripts/analysis13.py](../../../optimization_v2/ws/src/navigation_adapter/scripts/analysis13.py#L11) | `evaluate` | 事后独立物理真值评分，不进入在线控制。 |
| [scripts/stage13.py](../../../optimization_v2/ws/src/navigation_adapter/scripts/stage13.py#L30) | `assess` | 聚合完整矩阵与时延门限，生成真实回执。 |

上游控制器位于 `/opt/ros/jazzy`：DWB 负责采样积分和代价评价；RPP 计算前视曲率；MPPI 用对应底盘模型优化采样速度序列。适配层不重写这些算法。参数由 `scripts/configure13.py` 生成，参考源码链接与精确版本在 `optimization_v2/references/S13/README.md`、`dependency_lock.json`。

Python `checkpoint()` 保存实时图像、点云点数、时效和位姿；`progress.json` 仅记导航进度，不给 S10 账本增加作业量。`make_path()` 以 map/零时间戳表达静态路线；TF 时间同步使用观测的共同历史时刻，不用真值和未来外推。

教学例在 `optimization_v2/lessons/S13`。C++ 演示门控/限幅/停车，Python 演示单调进度与剩余路径；它们的断言不代替完整 Gazebo 验收。

补充入口：`src/static_map_planner.cpp::createPlan` 包装上游 Smac 并保留静态 map 语义；`scripts/near_field13.py::cloud/tick` 对齐主雷达和下视观测、丢点与延时，输出仅供感知的点云；`scripts/sensor_check13.py::evaluate_sensor` 在运行结束后评分实体低障碍，不能参与在线决策。

| 补充生产代码 | 函数 |
|---|---|
| [src/static_map_planner.cpp](../../../optimization_v2/ws/src/navigation_adapter/src/static_map_planner.cpp#L30) | `nav_msgs::msg::Path createPlan` |
| [scripts/near_field13.py](../../../optimization_v2/ws/src/navigation_adapter/scripts/near_field13.py#L29) | `def cloud` |
| [scripts/sensor_check13.py](../../../optimization_v2/ws/src/navigation_adapter/scripts/sensor_check13.py#L3) | `def evaluate_sensor` |

停车锁存生产实现：[stop_latch.hpp](../../../optimization_v2/ws/src/navigation_adapter/src/stop_latch.hpp#L8) 的 `StopLatch::update`。输入是停止请求、当前反馈是否新鲜且停稳、仿真时间；输出为是否继续强制 STOP。门控日志保存 `brake_latched`、`brake_cause` 和 `measured_standstill`，便于区分原故障和等待停车。

车体过滤：[self_mask.hpp](../../../optimization_v2/ws/src/navigation_adapter/src/self_mask.hpp#L6) 的 `selfPoint` 使用当前车型 footprint 外扩边界；[CMakeLists.txt](../../../optimization_v2/ws/src/navigation_adapter/CMakeLists.txt) 在编译目录生成 `terrain_s13.cpp`，替换原 S09 硬编码自体屏蔽，并附加确认网格，复用全部地形风险计算。原始代码与公式仍可在 `terrain_perception/src/terrain_node.cpp`、`include/terrain_perception/risk.hpp` 学习。

[observation_confirmation.hpp](../../../optimization_v2/ws/src/navigation_adapter/src/observation_confirmation.hpp#L6) 的 `confirmLethal` 只决定是否写入额外长期记忆，不改变即时风险。新鲜原始候选与已经滞留在风险栅格中的旧值分别处理，避免把同一条旧观测重复计为三次确认。

通信发现由 [middleware13.py](../../../optimization_v2/ws/src/navigation_adapter/scripts/middleware13.py#L3) 的 `local_dds` 为本轮子进程设置，配置在 `config/fastdds_local.xml`；`tests/discovery_check.py` 验证 40 个实际参与者的完整互通，`observer13.py` 发布真实所有权审计确认供启动检查。

热重启时钟屏障：[startup_clock.hpp](../../../optimization_v2/ws/src/navigation_adapter/src/startup_clock.hpp#L13) 的 `waitForRestartClock` 读取新节点及其代价图的实际时钟。`runtime13.py` 将重启前已观测时间传入新进程；`analysis13.py` 核对两个进程的 READY 记录。组件回归在 [startup_check.py](../../../optimization_v2/ws/src/navigation_adapter/tests/startup_check.py#L1)，使用合成时钟明确测试迟到和缺失，不冒充真实车辆闭环。

热重启通信握手：[nav_lifecycle13.py](../../../optimization_v2/ws/src/navigation_adapter/scripts/nav_lifecycle13.py#L8) 的 `bringup_after_restart` 先进行真实状态查询，再请求生命周期 STARTUP；查询超时允许重建客户端，转换请求不盲目重复。生产客户端在此期间保持停止租约，并重建两个 ActionClient。

精确终点处理：[exact_goal.hpp](../../../optimization_v2/ws/src/navigation_adapter/src/exact_goal.hpp#L10) 的 `canUseExactGoal` 限制修正范围并逐点调用碰撞判定；[static_map_planner.cpp](../../../optimization_v2/ws/src/navigation_adapter/src/static_map_planner.cpp#L38) 提供当前代价图中心格及 Nav2 车体轮廓碰撞检查。教学例直接复用该函数，受阻或较远的容差终点不能被盲目替换。
