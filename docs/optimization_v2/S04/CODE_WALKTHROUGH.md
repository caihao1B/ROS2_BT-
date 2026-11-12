# S04 核心代码导读

以下为本版本生产实现的函数和行号，代码位于独立 optimization_v2 工作区。C++ 与 Python 教学例另见 [README](../../../optimization_v2/lessons/S04/README.md)，不能把教学片段当成替代 ROS/Gazebo 的生产节点。

## 从物理量到标准消息

1. [generate_world.py:7](../../../optimization_v2/ws/src/sensor_sim_adapter/scripts/generate_world.py#L7) 的 `generate` 调用已验收 S03 世界组合器，给复制的车体添加 GPU lidar、两个 camera、IMU、NavSat 以及关节观察插件；恢复 S02 运动障碍。只改本案副本，不覆盖冻结原资产。质量假设为传感器质量已经包含在底盘总质量中，未增设精细传感器碰撞结构。
2. [JointObserver.cc:20](../../../optimization_v2/ws/src/sensor_sim_adapter/src/JointObserver.cc#L20) 的 C++ `Configure` 查找真实车轮/转向关节，为缺失的 JointPosition/JointVelocity 建立读数组件；`PostUpdate`（36 行）在每个物理更新后按 0.01 s 节拍读取组件，发布采集时间、轮角与角速度。没有 SetWorldPose、车体位姿求导或执行器写入。
3. [runtime.py:18](../../../optimization_v2/ws/src/sensor_sim_adapter/scripts/runtime.py#L18) 的 `run_case` 启动 Gazebo、显式 ros_gz_bridge 白名单和三个 ROS 域。Gazebo 原生插件实现射线/渲染/惯性与地理坐标；上游 C++ ros_gz_bridge 转换为 ROS 标准消息。本项目自写 C++ 只负责额外关节采样，不重复实现图像或点云桥接。
4. [adapter.py:23](../../../optimization_v2/ws/src/sensor_sim_adapter/scripts/adapter.py#L23) 的 `Adapter.__init__` 创建 121/119 两个显式 Context 和各传感器独立随机流。视觉进程只发布图像/点云，运动进程只发布 IMU/轮速/GNSS，防止重复发布与高频惯性被图像计算阻塞。

## Python 误差适配逻辑

| 函数与源码 | 输入 → 计算 → 输出 | 学习重点 |
|---|---|---|
| [cloud:60](../../../optimization_v2/ws/src/sensor_sim_adapter/scripts/adapter.py#L60) | native xyz → 沿射线加距离噪声 → 32×1024 PointCloud2 | 高度来自真实返回，不是人工拉伸；无效点保留；ring 来自栅格 |
| [camera:72](../../../optimization_v2/ws/src/sensor_sim_adapter/scripts/adapter.py#L72) | Gazebo RGB/BGR → 有种子像素噪声/整帧丢弃 → rgb8 | Image.step 是每行字节数，不是像素数；尊重行步长 |
| [imu:80](../../../optimization_v2/ws/src/sensor_sim_adapter/scripts/adapter.py#L80) | gyro/accel → 初偏+√Δt 随机游走+白噪声 → Imu | 协方差是方差，不是标准差；无姿态用 covariance[0]=-1 |
| [wheel:89](../../../optimization_v2/ws/src/sensor_sim_adapter/scripts/adapter.py#L89) | q/ω → tick 量化+S03运动学+比例故障 → JointState 与 Twist | 不把 GNSS 或世界位姿差分当轮速；未观测维度用大协方差 |
| [gnss:96](../../../optimization_v2/ws/src/sensor_sim_adapter/scripts/adapter.py#L96) | 原生天线地理坐标 → ENU 白噪声/相关项/偏移 → NavSatFix | false_fixed 的观测真偏移且仍小协方差；不能只改标签 |
| [enqueue/flush:48](../../../optimization_v2/ws/src/sensor_sim_adapter/scripts/adapter.py#L48) | 采集时刻 t → 最小堆到期 t+delay → 发布或丢过期包 | 时间戳不改成到达时间；有界内存与恢复后乱序处理 |
| [ecef/geodetic:14](../../../optimization_v2/ws/src/sensor_sim_adapter/scripts/common.py#L14) | WGS84 ↔ ECEF ↔ ENU | 在米制局部空间加误差，再转换回地理坐标 |

PointCloud2 本步 point_step=18：offset 0/4/8/12 分别是 float32 x/y/z/intensity，16 是 uint16 ring；row_step=width×18。没有额外 point-time 字段。Image 的 data 是 height×step 个字节，rgb8 每像素三字节。Imu 中 header.stamp/frame_id、角速度、加速度及 3×3 协方差分别承担时空关联和质量含义；单位不匹配时数值“看起来有变化”也不能算有效传感器。

## 同步、分流和记录

[fanout.py:12](../../../optimization_v2/ws/src/sensor_sim_adapter/scripts/fanout.py#L12) 建立唯一外参树和分流订阅。`cloud`（29 行）保留完整定位帧，感知每两帧取一帧并隔列取样，保持垂直环；`image`（34 行）按投影公式构造 CameraInfo，用 `(sec,nanosec)` 精确匹配左右图，不能用“最近到达的另一张”冒充同步。

[observe.py:65](../../../optimization_v2/ws/src/sensor_sim_adapter/scripts/observe.py#L65) 的 `receive` 在 119 采集实际接收计数、采集时间、到达仿真时间与墙钟。PNG 编码交给 `media_worker`（31 行）的有界线程队列，避免压缩图像阻塞 200 Hz IMU。`record`（58 行）只在 10–12 s 写入实际 ROS CDR，超过空间预算立即关闭。`health`（101 行）只看最新观测时间与 GNSS 相邻位置突跳，无私有故障时序输入。

[analyze.py:10](../../../optimization_v2/ws/src/sensor_sim_adapter/scripts/analyze.py#L10) 的离线 `analyze` 才允许读生成器私有基准与评分真值，用于证明噪声/漂移/假固定解确实改变了数据。频率 `rate` 使用 `(接收数量−1)/(末采集时刻−首采集时刻)`；RTF 使用仿真时间差/实际接收墙钟差。双目同步误差比较同对采集时间，不比较 CPU 回调先后。短录包检查重新读取 SQLite/CDR 并解码真实消息，而不是只信写入计数。

[stage_runner.py:47](../../../optimization_v2/ws/src/sensor_sim_adapter/scripts/stage_runner.py#L47) 的 `main` 验证 S03 回执、取得跨阶段共享锁，编译插件与教学例，再顺序运行四个案例。所有案例达到要求后才写 PASS；中断写 PARTIAL、失败写 FAIL。`receipt`（25 行）绑定源码、配置、证据、文档与前置回执哈希。`demo` 使用独立记录，避免演示替代完整测试。

## 怎样学习和改动

先运行两种教学例，再打开实际报告看时延、轮速和 GNSS 误差曲线，最后沿生成器→标准消息→分流器→评分器阅读。改变噪声、频率、外参、底盘或地图后，必须运行正式测试重建证据；仅 check 旧回执会检测到不一致，不会自动证明改动正确。

后续 S05 可直接订阅 `/s04/localization/{points,imu,wheel,gnss}` 中适合该估计器的输入，但不能读生成器日志或评价域。标准消息与清晰时空合同是后续替换成真实驱动时的衔接点；仿真瞬时扫描、理想相机、等效轮滑等差异仍需在实车迁移时重新验证。
