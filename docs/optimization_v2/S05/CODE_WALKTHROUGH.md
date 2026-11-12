# S05 核心代码学习路线

真实实现复用 S03/S04，新增 localization_guard；没有另写一套 EKF/ICP。按“观测→质量→增量→滤波→控制许可→独立评分”阅读。

## Python 生产调用链

- [sensors.py：FaultAdapter.enqueue](../../../optimization_v2/ws/src/localization_guard/scripts/sensors.py#L15)：在可信传感器生成域对 S04 带误差观测继续施加地面单平面与 IMU 断流；不发送未来轨迹或真值标签给估计器。
- [cloud_filter.py：geometry](../../../optimization_v2/ws/src/localization_guard/scripts/cloud_filter.py#L11)：cKDTree 邻域法向与未中心化法向矩阵最小特征值；main 中有限值/车体/距离/体素筛选，仅影响定位支路。
- [icp_gate.py：main](../../../optimization_v2/ws/src/localization_guard/scripts/icp_gate.py#L12)：配对 RTAB-Map OdomInfo 与里程计；拒绝 lost/零四元数/退化。内嵌 emit 用合格历史位姿求短窗口速度与协方差，绝对 pose 仅作诊断。
- [input_adapter.py：Inputs](../../../optimization_v2/ws/src/localization_guard/scripts/input_adapter.py#L12)：reset_state/reset 管理 epoch；imu 估计静止零偏与加速度倾角；wheel 生成原始轮速对照并通过与独立 ICP 的差值调度协方差。
- [motion.py：main](../../../optimization_v2/ws/src/localization_guard/scripts/motion.py#L10)：本步测试用闭环圆线/坡道跟随，使用融合 odom，不读取 truth；健康限速、前向距离与原 S03 ARM/DRIVE/STOP 仲裁同时生效。不是 Nav2 或完整覆盖执行器。
- [observe.py：main](../../../optimization_v2/ws/src/localization_guard/scripts/observe.py#L12)：记录在线三条路径、协方差、TF/订阅图；另发布平面规划 PoseStamped，不生成第二个 TF。
- [analyze.py：trajectory_score](../../../optimization_v2/ws/src/localization_guard/scripts/analyze.py#L8)：只在仿真结束后读取独立真值；共同初始原点对齐、插值、ATE 和 1 s RPE，不逐段拟合掉漂移。analyze 汇总真实物理停车、接触、隔离与资源证据。
- [runtime.py：run_case](../../../optimization_v2/ws/src/localization_guard/scripts/runtime.py#L22)：建立三个 ROS domain 和独立 Gazebo partition；保存原始进程命令/退出码；有界收尾。compose 仅在私有仿真域设置出生位置。
- [stage_runner.py：main](../../../optimization_v2/ws/src/localization_guard/scripts/stage_runner.py#L76)：检查 S04 回执、共享 flock、顺序正式矩阵、失败仍记结果、报告与版本哈希；stop 核验 PID 的 uid/创建时间/命令。
- [support.py：validate_config](../../../optimization_v2/ws/src/localization_guard/scripts/support.py#L18)：拒绝额外/重复源、绝对 ICP 位姿反馈、ICP 外部初值/IMU、双 TF 所有者、抹去坡地姿态配置。

- [odom_output.py：Integrator / update](../../../optimization_v2/ws/src/localization_guard/scripts/odom_output.py#L16)：主融合/对照均积分 EKF 估计速度与转率，倾角保留，唯一 TF 发布者为 s05_local_odometry；不采用原始滤波 pose 突变或真值修正。

## C++ 生产核心与上游边界

- [HealthGuard::tick](../../../optimization_v2/ws/src/localization_guard/src/health_guard.cpp#L31)：根据最新 IMU、有效 ICP 增量、融合 odom 的时间与质量，发布 OK/DEGRADED/STOP、限速和 DiagnosticArray。仅申请控制限制，实际唯一执行出口仍在 S03。新 epoch 先初始化停车；拒绝来自旧时钟的未来数据。
- [上游 void Ekf::correct](/home/ceihao/robot_project/optimization_v2/references/S05/ekf.cpp:54)：按 update_vector 提取观测子集，计算创新和马氏距离、Kalman 增益、状态及 Joseph 协方差更新。这是锁定版本参考源码，运行使用系统已安装库，不在项目中复制编译另一套。
- [上游 void Ekf::predict](/home/ceihao/robot_project/optimization_v2/references/S05/ekf.cpp:222)：三维运动模型传播状态、构造 Jacobian、预测协方差。这是锁定版本参考源码，运行使用系统已安装库，不在项目中复制编译另一套。
- [RTAB-Map ROS 数据接入](/home/ceihao/robot_project/optimization_v2/references/S05/OdometryROS.cpp:1)：本步配置不启用 IMU 初始化，ICP 只接收点云；质量由 OdomInfo 报告，lost 的无效四元数由项目门控拒绝。点到平面残差最优化本身在上游 RTAB-Map/libpointmatcher，不在本项目伪造。

## 可运行教学例

- [C++ lesson.cpp](../../../optimization_v2/lessons/S05/lesson.cpp#L7)：直接调用实际 robot_localization::Ekf::correct。初始速度 0.5、观测 0.8；正常方差 0.0025 更新到约 0.782353，方差 4.0 只更新到约 0.502970，严格门控拒绝后仍为 0.5。trace 是这些有限更新的真正内部创新，不是整车 EKF 日志。
- [Python lesson.py](../../../optimization_v2/lessons/S05/lesson.py#L1)：固定帧偏移轨迹 ATE=0.573 m、1 s RPE=0.100 m；对比观测方差权重。它是教学数值例，正式成绩由实际 Gazebo 观测计算。
- [编译和运行命令](../../../optimization_v2/lessons/S05/README.md)。依赖库名为本机 librl_lib.so，Eigen3 仅用于矩阵表达；不修改系统库。

## 自查阅读问题

1. 为什么轮速 pose、同一轮速 Twist、激光上游 Twist 不能全部无条件融入？先检查输入来源的相关性，再看 validate_config。
2. 为什么只有地面时高匹配率仍会漂移？用 math_checks.py 的平面和三正交面法向矩阵比较。
3. 为什么轮滑时提高轮速方差？看 Python 的 Kalman 权重式，再看 C++ 真实引擎输出。
4. 为什么最终轨迹没有被真值强行拉回？在线域无 truth，评分器才读 evaluation，且只做固定起点对齐。
5. 为什么“健康 OK”不代表累计误差为零？健康检查关注输入新鲜度和配准几何，绝对位置漂移需要后续全局参照监督。
