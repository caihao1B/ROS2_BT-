# S17 核心代码导读

本目录实现实验执行和独立评分，并包含为本轮暴露问题增加的隔离修正。ROS 2、Nav2、robot_localization、RTAB-Map 和 Gazebo 的上游实现没有被冒称为本阶段原创。候选或实验脚本存在不代表对应试验通过。

## 本版本的实际入口与行号

- [include/acceptance_monitor.hpp:9](../../../optimization_v2/ws/src/acceptance_runner/include/acceptance_monitor.hpp#L9)：`inline json monitor`。C++ 独立审计：仿真时钟、原始接触计数、1800 秒和工具停止反馈。
- [include/pose_jump17.hpp:7](../../../optimization_v2/ws/src/acceptance_runner/include/pose_jump17.hpp#L7)：`inline bool pose_jump`。C++ 定位监督：在同一时刻车体位置比较新旧全局变换，避免原点距离放大角度修正。
- [src/supervisor17.cpp:33](../../../optimization_v2/ws/src/acceptance_runner/src/supervisor17.cpp#L33)：`void mapEvidence`。C++ 地图匹配观测、GNSS 交叉检查、拒绝真实跳变。
- [src/supervisor17.cpp:54](../../../optimization_v2/ws/src/acceptance_runner/src/supervisor17.cpp#L54)：`void tick`。C++ 全局来源、新鲜度、静止恢复和唯一 map→odom 发布。
- [scripts/matrix17.py:5](../../../optimization_v2/ws/src/acceptance_runner/scripts/matrix17.py#L5)：`def make`。Python 冻结六簇×十种子，以及连接路、故障和消融要求。
- [scripts/checkpoint17.py:18](../../../optimization_v2/ws/src/acceptance_runner/scripts/checkpoint17.py#L18)：`def commit`。Python 哈希绑定与原子提交；指标不一致、重复成绩或证据修改均拒绝。
- [scripts/stage17.py:37](../../../optimization_v2/ws/src/acceptance_runner/scripts/stage17.py#L37)：`def main`。Python 串行仿真、分片、恢复、前置验证、超时和本会话收尾。
- [scripts/overlay17.py:8](../../../optimization_v2/ws/src/acceptance_runner/scripts/overlay17.py#L8)：`def build`。Python 对原 S14 做精确匹配的独立适配，并保存每次执行源码快照。
- [scripts/temporal_join17.py:17](../../../optimization_v2/ws/src/acceptance_runner/scripts/temporal_join17.py#L17)：`def take`。Python 栅格与健康状态按相同时间戳连接，拒绝过期/未来观测。
- [scripts/inspection17.py:60](../../../optimization_v2/ws/src/acceptance_runner/scripts/inspection17.py#L60)：`def execute`。Python 长路分段、原路径后缀恢复、参考路径与在线规划动作选择。
- [scripts/inspection_budget17.py:10](../../../optimization_v2/ws/src/acceptance_runner/scripts/inspection_budget17.py#L10)：`def estimate`。Python 按已完成子段与当前 Nav2 路径估计剩余工时；已流逝时间不算完成进度。
- [scripts/inspection_audit17.py:14](../../../optimization_v2/ws/src/acceptance_runner/scripts/inspection_audit17.py#L14)：`def audit`。Python 离线核对真实路段终点顺序和整个车身的授权区域约束。
- [scripts/map_memory17.py:10](../../../optimization_v2/ws/src/acceptance_runner/scripts/map_memory17.py#L10)：`class MapMemory`。Python 固定地图坐标系保存已确认障碍，避免里程计漂移拖出虚假占用；未知不能清障。
- [scripts/dynamic_envelope17.py:21](../../../optimization_v2/ws/src/acceptance_runner/scripts/dynamic_envelope17.py#L21)：`class Envelope`。Python 保存已观测运动轨迹的有界占用，45 秒过期；无仿真对象身份或未来真值输入。
- [scripts/geometry_plan17.py:52](../../../optimization_v2/ws/src/acceptance_runner/scripts/geometry_plan17.py#L52)：`def qualify`。Python 独立几何变体资格；洞同时从运动域扣除，不能把仅作业面积挖洞当成物理可通行。
- [scripts/resume17.py:24](../../../optimization_v2/ws/src/acceptance_runner/scripts/resume17.py#L24)：`def restore_mission`。Python 验证已确认工具账本，恢复已完成任务与残余，不重复计信用。
- [scripts/evaluate17.py:6](../../../optimization_v2/ws/src/acceptance_runner/scripts/evaluate17.py#L6)：`def evaluate`。Python 原始分母、独立域真值插值、定位分位数、唯一出口和停止审计。
- [scripts/statistics17.py:13](../../../optimization_v2/ws/src/acceptance_runner/scripts/statistics17.py#L13)：`def paired`。Python 只有同场景/种子/任务/限制才配对；缺项保留。
- [scripts/audit17.py:4](../../../optimization_v2/ws/src/acceptance_runner/scripts/audit17.py#L4)：`def audit`。即使回执为 PARTIAL，仍核查所有证据、源码和前置依赖哈希；不改变验收状态。
- [scripts/report17.py:6](../../../optimization_v2/ws/src/acceptance_runner/scripts/report17.py#L6)：`def aggregate`。Python 聚合 60 个固定案例及 R01–R08，缺证据不能成为 PASS。
- [scripts/partition17.py:7](../../../optimization_v2/ws/src/acceptance_runner/scripts/partition17.py#L7)：`def run`。离线分簇实际对照；同输入生成路线，区分几何合格与名义工时合格。
- [scripts/faults17.py:78](../../../optimization_v2/ws/src/acceptance_runner/scripts/faults17.py#L78)：`def assess`。故障实验：独立核实故障暴露、预期响应和安全；仍需完整运行矩阵。
- [scripts/visual_gate17.py:7](../../../optimization_v2/ws/src/acceptance_runner/scripts/visual_gate17.py#L7)：`class Window`。视觉候选：有效跟踪的相对增量、时钟/丢失清空、协方差门控，无额外 TF。
- [scripts/coverage_compare17.py:23](../../../optimization_v2/ws/src/acceptance_runner/scripts/coverage_compare17.py#L23)：`def choose`。在线点服务候选：从当前估计位置选最近未服务点，重新等待匹配的 BT 指令。
- [scripts/risk_adapter17.py:10](../../../optimization_v2/ws/src/acceptance_runner/scripts/risk_adapter17.py#L10)：`def mission`。仅开发候选：复用 S15 风险恢复，明确排除它的两点测试任务替换。
- [scripts/prediction_cost17.py:4](../../../optimization_v2/ws/src/acceptance_runner/scripts/prediction_cost17.py#L4)：`def apply`。仅开发候选：将预测占用加入软代价，保留未知格和真实障碍。

## 数据链与责任边界

```text
带误差传感器 → 轮速/IMU/激光局部里程计 → 冻结地图定位与 GNSS 监督
               └→ 三维地形/目标跟踪 → 时间戳连接 → 二维代价
固定作业清单 → 可解释 BT → 唯一导航动作 → 唯一底盘/工具出口
实际工具反馈 → 幂等账本 → 完成与原始残余
隔离真值域 120 → 独立审计 → 指标/检查点 → 60 例矩阵与 R01–R08
```

`FollowPath` 不会自动替长路径重新选择全局绕行；`NavigateToPose` 使用在线全局规划器。S17 巡检子段使用 `NavigateToPose`，当前代价地图决定绕行；真实路段终点保持不变。割草工作段仍必须沿已验证的作业路径，并由有效工具足迹结算。

C++ 学习例直接调用生产审计与跳变函数；Python 学习例调用生产统计与时间连接函数。教学输入是合成的，不能替代仿真成绩。编译运行命令和用途见 [教学说明](../../../optimization_v2/lessons/S17/README.md)。
