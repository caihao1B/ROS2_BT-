# S03 核心代码导读

调用链：stepctl → stages/S03/run.sh → stage_runner → 模型生成与 runtime → ROS 仲裁器/试验客户端/Gazebo 关节插件 → 独立评分 → 报告与资格文件。

| 核心入口 | 实际源码及本版本行号 |
|---|---|
| `config` | [robot_description/scripts/generate.py:13](../../../optimization_v2/ws/src/robot_description/scripts/generate.py#L13) |
| `inertial` | [robot_description/scripts/generate.py:16](../../../optimization_v2/ws/src/robot_description/scripts/generate.py#L16) |
| `vehicle` | [robot_description/scripts/generate.py:33](../../../optimization_v2/ws/src/robot_description/scripts/generate.py#L33) |
| `world` | [robot_description/scripts/generate.py:66](../../../optimization_v2/ws/src/robot_description/scripts/generate.py#L66) |
| `Configure` | [chassis_bridge/src/WheelDrive.cc:46](../../../optimization_v2/ws/src/chassis_bridge/src/WheelDrive.cc#L46) |
| `Reset` | [chassis_bridge/src/WheelDrive.cc:56](../../../optimization_v2/ws/src/chassis_bridge/src/WheelDrive.cc#L56) |
| `PreUpdate` | [chassis_bridge/src/WheelDrive.cc:60](../../../optimization_v2/ws/src/chassis_bridge/src/WheelDrive.cc#L60) |
| `PostUpdate` | [chassis_bridge/src/WheelDrive.cc:119](../../../optimization_v2/ws/src/chassis_bridge/src/WheelDrive.cc#L119) |
| `encoders` | [chassis_bridge/scripts/drive_bridge.py:25](../../../optimization_v2/ws/src/chassis_bridge/scripts/drive_bridge.py#L25) |
| `command` | [chassis_bridge/scripts/drive_bridge.py:47](../../../optimization_v2/ws/src/chassis_bridge/scripts/drive_bridge.py#L47) |
| `tick` | [chassis_bridge/scripts/drive_bridge.py:58](../../../optimization_v2/ws/src/chassis_bridge/scripts/drive_bridge.py#L58) |
| `limit` | [chassis_bridge/scripts/kinematics.py:3](../../../optimization_v2/ws/src/chassis_bridge/scripts/kinematics.py#L3) |
| `steering` | [chassis_bridge/scripts/kinematics.py:10](../../../optimization_v2/ws/src/chassis_bridge/scripts/kinematics.py#L10) |
| `curvature` | [chassis_bridge/scripts/kinematics.py:12](../../../optimization_v2/ws/src/chassis_bridge/scripts/kinematics.py#L12) |
| `encoder_delta` | [chassis_bridge/scripts/kinematics.py:22](../../../optimization_v2/ws/src/chassis_bridge/scripts/kinematics.py#L22) |
| `phases` | [chassis_bridge/scripts/test_driver.py:16](../../../optimization_v2/ws/src/chassis_bridge/scripts/test_driver.py#L16) |
| `main` | [chassis_bridge/scripts/test_driver.py:23](../../../optimization_v2/ws/src/chassis_bridge/scripts/test_driver.py#L23) |
| `physical` | [chassis_bridge/scripts/analyze.py:10](../../../optimization_v2/ws/src/chassis_bridge/scripts/analyze.py#L10) |
| `stop_score` | [chassis_bridge/scripts/analyze.py:19](../../../optimization_v2/ws/src/chassis_bridge/scripts/analyze.py#L19) |
| `analyze` | [chassis_bridge/scripts/analyze.py:29](../../../optimization_v2/ws/src/chassis_bridge/scripts/analyze.py#L29) |
| `stop_all` | [chassis_bridge/scripts/runtime.py:7](../../../optimization_v2/ws/src/chassis_bridge/scripts/runtime.py#L7) |
| `run_case` | [chassis_bridge/scripts/runtime.py:28](../../../optimization_v2/ws/src/chassis_bridge/scripts/runtime.py#L28) |
| `qualify` | [chassis_bridge/scripts/stage_runner.py:25](../../../optimization_v2/ws/src/chassis_bridge/scripts/stage_runner.py#L25) |
| `receipt` | [chassis_bridge/scripts/stage_runner.py:33](../../../optimization_v2/ws/src/chassis_bridge/scripts/stage_runner.py#L33) |
| `main` | [chassis_bridge/scripts/stage_runner.py:59](../../../optimization_v2/ws/src/chassis_bridge/scripts/stage_runner.py#L59) |

## C++：唯一关节执行器

`Configure` 绑定两只驱动轮；阿克曼额外绑定两个转向关节，启用关节位置/速度反馈。读取车辆配置和会话，建立一条执行请求订阅，不订阅任何车体位置真值。Gazebo 原生物理系统负责轮地接触、重力和约束。

`PreUpdate` 先处理时钟/暂停、租约版本/epoch/序号与反馈，再计算限幅后的目标。差速映射左右轮速；阿克曼用真实转角推算当前曲率，驱动后轮，并对转角做限速比例控制。它仅写 JointVelocityCmd，不调用车体 SetWorldPose/SetLinearVelocity/SetAngularVelocity，因此车体必须通过实际关节与接触运动。

`Reset` 增加 epoch、清空速度和旧租约，显式要求重新 ARM。`PostUpdate` 发布实际车体位姿到评估专用话题；该话题只桥接到 ROS 120。在线反馈 `/s03/encoders` 只有实际关节、使能/暂停/epoch、控制器状态，没有实际世界位置。

## Python：观测、仲裁和试验

`generate.py` 从 YAML 同时生成质量、惯量、轮子、转向节、工具包络、接触传感器及驱动配置。它读取冻结 S02 世界生成派生场景，替换 S02 的固定探针，保留原道路、坡面和静态障碍，将动态物体停放在试验区外并关闭脚本运动。此隔离夹具只验证底盘，不能推导动态避障能力。没有第二份被静默改动的基础地图。

`Drive.encoders` 将角度量化到 4096 ticks/rev、转角 0.001 rad，利用编码器增量积分平面里程计；没有读取 Gazebo 世界位姿。`Drive.command/tick` 检查时戳、序号、epoch、单一合法发布者、反馈与时钟新鲜度；拒绝旧命令自动恢复。状态使用具体原因字符串，便于后续 BT 判断停车原因。

`test_driver` 是试验客户端：公开路线、已知试验起点、编码器里程计与前向 LaserScan 是全部运动输入。路线只用于 Pure Pursuit 底盘能力验证，后续 Nav2 可替换该客户端，但不能绕过唯一仲裁器。故障注入只允许操作 runtime 提供的本会话 PID，并核对 /proc 创建标识。

`analyze.physical` 对独立记录的实际位姿差分得到速度和角速度，复位前后分段处理，不把世界复位瞬移当成高速运动。`stop_score` 要求实际线/角速度满足门限并持续至少 0.3 s，不能依据 commanded_v=0 判停车。`analyze` 同时验证关节反馈、半径、侧滑、路径偏差、完整 footprint 净空和接触对。

## 参数变更原则

更换轮胎、质量、工具、摩擦或轮距必须重新生成并验收；最终 `qualified_profiles.json` 由通过的案例生成并绑定配置哈希。教学例仅说明映射，不应把本机有效接触轮距当成真实硬件标定。模型中的工具反馈/作业效果尚未实现，S10 承接；S04 将替换临时测距探针并建立完整传感器模型。
