# 第 1 步技术介绍与实现依据

本步建立可重复检查的原项目基线和隔离开发环境。新环境使用 Ubuntu 24.04、ROS 2 Jazzy、Gazebo Harmonic，独立 ROS domain 119；原农田与模型仍通过原生入口运行在 domain 118。新功能只在 `optimization_v2/ws/src` 开发，原生源码保留为已知基线。

## 为什么先固定基线

项目有两份 ros2_ws 源码，逐文件哈希比较得到 13 个差异文件，涉及路径、传感器配置、启动文件与模型合同。原入口实际加载 `native_20261108/ros2_ws/install`，因此修改根目录下另一份源码不会自然影响当前仿真。`baseline_manifest.json` 明确记录运行绑定、源码差异、模型资源、硬件和文件哈希。

运行所需的模型保存在主项目中，本步不重复复制约 41 MB 的原机器人资产。原先位于项目外的 `sensor_smoke.py` 纳入新包，原入口已改为调用它；原 BT 完整任务入口引用缺失的 dynamic_avoidance，现改为明确拒绝并给出替代入口。其 C++ 状态机及不依赖缺失动态任务栈的回归已迁入新包，完整任务层在 S14 实施。

## 环境隔离原理

ROS 环境叠加会改变包查找、Python 模块和动态库解析顺序。本步先清理 `AMENT_PREFIX_PATH/CMAKE_PREFIX_PATH/COLCON_PREFIX_PATH/PYTHONPATH/LD_LIBRARY_PATH`，再装载系统 Jazzy 与新工作区的 `local_setup.bash`。没有全局修改用户 shell 配置。

ROS_DOMAIN_ID 119 与 118 用于隔离 DDS 发现；这不是仿真资源的互斥锁。Gazebo partition 隔离 Gazebo Transport；原生入口和新阶段演示仍通过同一个 flock 文件防止两个仿真实例同时运行。验收包含故意污染环境变量后重新装载，以及 C++ 发布器/Python 订阅器在 119 通信、118 收不到该消息的检查。

## 数据与控制边界

静态检查器只订阅 `/clock`、前后原始与过滤雷达、IMU、轮关节、GNSS、融合里程计、TF 和驱动请求。检查器不发布速度命令；原 drive 节点继续作为唯一命令出口。它验证的是原项目的双二维雷达等现有观测，不是 S04 的三维雷达、双目和正式 RTK 模拟。

静态判据保留：位移跨度小于 0.03 m、最大前向速度小于 0.03 m/s、观察到的执行请求全部为零。时钟必须递增，关键消息达到最低计数，里程计和 TF 发布拓扑符合现有合同，日志不能存在资源加载错误。

消息接收频率使用 `(N−1)/(最后接收现实时间−首次接收现实时间)`；RTF 使用 `Δ仿真时间/Δ现实时间`。频率图明确以现实秒计，因此不能把它直接与仿真配置频率比较而忽略 RTF。静态检查只证明数据链、资源和静止条件，不证明传感器精度、运动控制质量或完整覆盖。

## 依赖兼容样例

系统新增 Fields2Cover 2.1.1、METIS 5.1.0 及四个必要依赖，没有安装深度学习栈。C++ 样例使用实际安装的 `Fields2Cover::Fields2Cover` CMake 目标，计算 20×12 m 矩形、1 m 地头、2 m 割幅的条带并排序。METIS 对六节点链图生成三个候选分区。这只是接口/链接/运行检查，不代替 S11 的连通几何验证和 S12 的整条路径可行性验证。

S01 只接受固定 `--seed 41`，对应 METIS 样例和固定回归输入；旧静态场景没有因此新增 Gazebo 随机种子注入，不能声称旧传感器噪声逐样本可重复。S02/S04 分别建立场景与误差注入的可配置种子。

上游 [opennav_coverage jazzy-v2 分支](https://github.com/open-navigation/opennav_coverage/blob/jazzy-v2/README.md) 说明其 v2 适配以 Fields2Cover 2.0.0 验证；本机实际安装为 2.1.1。普通 jazzy 分支仍有旧 v1 说明，不能混用。本步只锁定已编译运行的直接 C++ API，完整 Nav2 coverage server 尚未集成。RTAB-Map 按阶段需要留到 S05/S06，故它未安装不阻塞 S02 场景建设。

## 故障、所有权与证据

进程会话保存 PID、进程创建时间、UID、命令标识与 run_id。停止前同时核对，防止 PID 复用时误杀其他进程。正常退出先通知 ROS launch 清理自身子进程；外层现实超时作为兜底。S01 为静止检查，没有开始工具作业，也没有隐藏的导航动作。

自动回执记录本轮命令退出码、检查项、源码/模型/配置/输出的 SHA-256。只哈希 manifest 不足以防止模型文件改变，因此回执同时列出实际资源文件。任何被绑定文件变化后，`stepctl check S01` 应返回 STALE，要求重新验证；`stepctl next 01` 依据当前有效回执判断能否进入 S02。

## 优缺点与保留限制

本机实际 GUI 检查发现 Ogre2 材质发白，日志出现材质/固定功能管线错误。S01 演示使用 `--render-engine-gui ogre` 只替换界面渲染，农田颜色恢复，服务端传感器仍使用 Ogre2。引擎切换是 Gazebo 提供的运行选项，见 [官方故障排查说明](https://gazebosim.org/docs/garden/troubleshooting/)；本机 `gz sim --force-version 8 --help` 也列出了单独设置 GUI 引擎的参数。没有更改驱动或原模型。S04 的相机图像必须另测材质与成像，不能用 GUI 兼容结果替代传感器图像验收。

隔离工作区和显式失败减少“改错源码”及依赖偶然可用的问题，代价是需要维护清晰的运行资产清单。复用原模型节省空间，但旧模型较重，且历史派生模型用了 300 Nm 临时转向力矩；原始 10 Nm 模型没有因此变成可用的实机标定。轻量双底盘与新的动力学验证在 S03 完成。

本步的静态烟雾检查、状态机测试和 API 样例分别覆盖不同问题。任何一项成功都不能推出完整自主作业成功；人工观察和自动验收也分别记录。


## 中断保存的集成证据

真实启动静态仿真后调用 stop S01，观察到阶段结果为 PARTIAL、检查器写出 interrupted=true、ROS launch 退出码为 0，且会话记录已移除。证据见 [停止集成检查](/home/ceihao/robot_project/optimization_v2/setup_logs/stop_integration.json)。对应修复是在 rclpy 初始化后显式接管 SIGINT，并使用 try_shutdown，避免中断时重复关闭上下文而漏写结果。
