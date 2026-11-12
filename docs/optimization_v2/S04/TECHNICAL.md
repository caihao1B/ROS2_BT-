# S04 技术说明：多传感器模拟与真实误差注入

本步在 S02 连续不规则三维场地和 S03 两种固定底盘上，建立可供后续定位、建图和感知使用的观测接口。Gazebo 实际渲染双目图像和三维激光，物理引擎提供 IMU、天线位置和关节运动；误差适配器输出带噪观测。两种底盘分次运行，原 Dino、原停车门限及 S03 唯一执行器保持原状。自动状态和实测值以 [RESULTS.md](RESULTS.md) 及有效回执为准。

## 数据流与职责

```text
S02 世界 + S03 车辆 + 原生 Gazebo Sensors/Imu/NavSat + C++ JointObserver
    │ 显式桥接白名单
    ▼ ROS domain 121（无噪原始量，仅传感器生成器）
视觉适配进程                 惯性/轮速/GNSS 适配进程
    └──────── 带噪且保留采集时间的观测 ────────┘
                         ▼ ROS domain 119
        标定/同源分流 ── 定位支路 / 感知支路
        在线观测审计 ── RViz / 曲线面板 / 有界短录包

精确车体位姿与接触量 → ROS domain 120 → 独立评分器 → 离线报告
```

传感器生成器是唯一跨 121/119 的受信适配器；在线分流器和观测审计器只读公开标定与观测，不读 world.sdf、私有生成日志或评分真值。运行时保存实际 ROS 图、订阅者、发布者和进程域，检查在线图无 raw/evaluation/world 话题。该边界是 ROS 图和程序输入白名单，**不是操作系统访问控制或 DDS 安全沙箱**。同一用户有意读取磁盘或切换 domain 仍有能力越界。仿真初始车位是测试夹具，不是定位观测；后续算法不得读取它来代替定位。

S03 仍独占 `/tf` 与 `/s03/actuator/request`；S04 仅新增一个静态外参发布者，避免重复 TF/底盘出口。S04 的运动脚本只在安全短测试道上前后行驶，使观测包含真实车体运动；它不是覆盖规划、避障或定位系统。

## 传感器、坐标和数据合同

配置分别在 [sensors.yaml](/home/ceihao/robot_project/optimization_v2/config/sensors.yaml)、[data_contract.yaml](/home/ceihao/robot_project/optimization_v2/config/data_contract.yaml)、[fault_profiles.yaml](/home/ceihao/robot_project/optimization_v2/config/fault_profiles.yaml)。每轮另存有效配置、世界副本及 SHA-256，不能仅修改参数后沿用旧 PASS。

| 传感器 | 冻结初始规格 | 在线输出与含义 |
|---|---|---|
| 三维激光 | 原生 GPU lidar，32×1024，10 Hz，垂直 −0.35～0.35 rad，0.15～50 m | PointCloud2；x/y/z 为米；intensity=0 表示未建模反射强度，ring 来自原生垂直光束编号 |
| 双目 | 两个实际相机，640×360 RGB，15 Hz，水平视角 1.2 rad，基线 0.12 m | Image rgb8 + 左右 CameraInfo；理想已校正针孔、全局快门 |
| IMU | 200 Hz | Imu；角速度 rad/s、含重力加速度 m/s²；不提供理想姿态，orientation_covariance[0]=-1 |
| 轮速 | 原生关节读取 100 Hz，位置 4096 tick/rev 量化 | JointState 的角度/角速度，以及 TwistWithCovarianceStamped 的车体 v/ω；无世界位姿求导 |
| GNSS | 5 Hz，WGS84 地理坐标 | NavSatFix；经纬度为度、高度为椭球高米，协方差在局部 ENU；单天线不提供航向 |

外参相对 S03 的 base_link（地面投影参考点），单位米：lidar=(0.10,0,0.78)，IMU=(0,0,0.34)，GNSS=(−0.10,0,0.80)，左/右相机=(0.25,±0.06,0.66)。雷达、IMU、GNSS 与车体同向；相机光学轴为右、下、前，其旋转相对车体为 roll=−π/2、pitch=0、yaw=−π/2。后续融合必须补偿 GNSS/IMU 杆臂，不能把各传感器位置直接当作 base_link。

相机 fx=fy=640/(2tan(0.6))，cx=320、cy=180，畸变系数为零，右相机投影矩阵 P[3]=−fx×0.12。这里的理想标定由仿真几何确定，尚未模拟焦距漂移、畸变残差、曝光、滚动快门。实际 CameraInfo 和 TF 被保存到各案例 `online/result.json` 与短录包。标定协议依据 [ROS Jazzy CameraInfo 定义](https://github.com/ros2/common_interfaces/blob/jazzy/sensor_msgs/msg/CameraInfo.msg)。

消息采集时间统一来自 `/clock`，时延只改变投递时刻，**不重写 header.stamp**。双目按采集时间精确配对，队列最多 6 帧；单侧丢帧会造成整对减少，不以重用旧帧伪造同步。定位点云保留 10 Hz/完整栅格；感知支路每两帧取一帧并隔列取点，目标 5 Hz、32×512；两路保留同一传感器 frame 与采集时间。IMU/轮速/GNSS 同源转发到定位支路。本步没有计算视觉特征、语义识别或融合结果。

| 接口 | QoS / 有界队列 | 原因 |
|---|---|---|
| 生成器输出 | best_effort、volatile；图像 keep_last=3，其余=10 | 短队列偏向新观测；实际缺包需要记录 |
| 原始输入 | 运动/点云 depth=20，图像=3 | 吸收短时调度波动 |
| 分流节点 | best_effort depth=20 | 保留短突发，不无限堆积 |
| CameraInfo、TF static | reliable、transient_local depth=1 | 后加入者能获得标定 |
| 在线审计订阅 | 运动 depth=100、点云=30、其余=10 | 测试采集缓冲，不是控制器队列；年龄另行统计 |
| 诊断 | reliable depth=10 | 保留近期质量状态 |

QoS 的取舍遵循 [ROS 2 Jazzy QoS 官方说明](https://github.com/ros2/ros2_documentation/blob/jazzy/source/Concepts/Intermediate/About-Quality-of-Service-Settings.rst)。best_effort 不保证零丢失。报告的“频率缺口”是配置频率与实收频率之差，不等于精确区分源端漏采、传输丢包和审计丢包。故障前基线单独统计，故意掉帧的窗口不会被误报为正常达标。

## 误差模型及选择理由

共同形式是 z=h(x)+b+n。h(x) 来自真实仿真测量，n 为短时随机误差，b 为慢变偏置或受控故障。只加独立白噪声无法体现连续定位漂移；直接给所有输出添加巨大随机值也不便分析。

- 激光：沿实际射线方向加入 σ=0.015 m 距离噪声。保留真实三维碰撞/遮挡几何；不是把二维 scan 复制到多层。当前无逐点时间和扫描运动畸变，因此后续算法必须支持瞬时帧，不能宣称已验证点云去畸变。
- 图像：真实渲染像素上加入 σ=0.8 灰度级的噪声，故障时左右独立 Bernoulli 丢帧。环境纹理、照明与成像仍比实车理想，不能据此断言视觉算法已具备田间泛化能力。
- IMU：白噪声 + 初始偏置 + 随机游走，b(k+1)=b(k)+σ_walk√Δt·N(0,I)。陀螺白噪声 0.003 rad/s，加速度白噪声 0.03 m/s²。消息协方差表达白噪声，不把未知漂移当成已知校正量；偏置需在 S05 等步骤估计。
- 轮速：C++ 插件读取真实轮轴角速度；Python 使用 S03 差速/阿克曼运动学换算 v/ω，然后加入噪声。位置做编码器 tick 量化，JointState 速度保留关节读数。轮滑故障在轮式里程计 v 上加 30% 比例失配；这验证融合算法能否抵抗错误轮式里程计，**不是改变地面摩擦的轮胎物理滑移实验**。
- GNSS：原生天线坐标转 ENU 后添加白噪声与一阶相关误差。相关项 b(k+1)=αb(k)+σ_c√(1−α²)N，α=exp(−Δt/τ)，τ=2 s。fixed 每轴标准差为 0.02/0.02/0.04 m，float 为 0.3/0.3/0.6 m。随后转回 WGS84，输出含杆臂的天线观测。并未实现卫星星历、载波相位、真实 RTK 解算或遮挡多路径物理模型。
- 假固定解：坐标真实增加 [3,−2,0.5] m，但 reported_mode 仍为 fixed、协方差仍小。在线模块看不到“false_fixed”真相标签；只能从观测异常或后续独立定位残差识别。NavSatStatus 不是 RTK fixed/float 的完整标准，故 companion quality 字段明确约定语义。
- 时延：使用仿真时间优先队列。恢复零时延时，新消息可能超过队列中的旧消息；适配器丢弃被超越的旧时间戳并记录 late_discard，避免定位系统收到倒序时间。时钟倒退清空队列和发布历史，另记 epoch。

各传感器使用 seed=41 派生的独立随机流；固定输入和随机调用序列可重复。Gazebo 渲染与 DDS 调度受系统负载影响，端到端包序列不是逐位确定性复现，必须保留每轮真实计数。

## 故障时序与可见现象

区间均为采集仿真时间的左闭右开区间；所有故障都改变数值、到达时刻或消息数量。

| 仿真秒 | 注入 | 手工可见现象 |
|---|---|---|
| 12–20 | 180 ms 投递时延 | 年龄曲线抬高；IMU/轮速新鲜度诊断变化 |
| 22–30 | 左右图像各 50% 掉帧 | 帧率降低，配对数降低 |
| 32–40 | 轮式里程计比例 +30% | 离线曲线中观测速度与轮轴基准分离 |
| 42–50 | GNSS float | GNSS 轨迹散布扩大 |
| 52–60 | GNSS dropout | 位置停止更新；quality=no_fix，诊断 GNSS stale |
| 62–72 | 错误 fixed | 位置偏移约 3.6 m，标签仍 fixed；突跳提示来自观测 |
| 74–82 | 陀螺 z 偏置 +0.04 rad/s | IMU 曲线整体抬高 |

诊断只做时效和相邻 GNSS 突跳启发式检查，并不等于 S08 的完整定位完整性监督；平滑偏移可能不会触发相邻点突跳告警。

## 实时性能、资源与验证

早期“所有生成 + 同步 PNG 压缩”方案被数据否定：IMU/轮速接收频率曾约 136/83 Hz。当前拆分视觉与惯性生成进程，PNG 使用最多 4 项后台队列；短时缓冲有上限。高频数值审计仍按真实消息记录，不通过补点来凑频率。

单案例短录包只记录采集 10–12 s，序列化数据设 64 MiB 限额并预留 1 MiB 元数据空间；验收回读 SQLite/CDR 内容、时间范围和实际文件大小。每案例最多 3 组完整观测快照。延时队列每个适配进程 32 MiB；拥有进程 RSS 总量监控阈值 8 GiB；案例磁盘阈值 256 MiB，每现实秒检查并停止。RSS/磁盘是轮询监视阈值，停止前可有采样间隔内的小幅超出，不是 cgroup 或文件系统硬配额。公开执行器限制案例时长及总现实时间，不提供无限录制入口。

CPU 报告同时区分全机平均负载和进程单核百分比（800% 约等于八个核）；进程 RSS 相加会重复计算共享页，作为保守指标。GPU 利用率及显存来自设备级 nvidia-smi，包含桌面等其他 GPU 客户端，不伪称仿真独占用量。RTF 用稳定段采集仿真秒/实际接收现实秒测量，目标 ≥0.9。

正式四案串行运行：差速/阿克曼 nominal 各 50 s，sensor_faults 各 90 s。C01 检查五类观测、标定和同步；C02 检查实收频率 ±10%、RTF/资源/短录包；C03 检查每项故障的实际数据效果；C04 检查真值边界与唯一发布者。演示独立记录，不覆盖正式回执。旧项目几何/坐标处理测试另行回归；C++/Python 教学例实际编译执行。

原生传感器配置参考 [Gazebo Sim 8 sensors 示例](https://github.com/gazebosim/gz-sim/blob/gz-sim8/examples/worlds/sensors.sdf) 和 [WGS84 spherical_coordinates 示例](https://github.com/gazebosim/gz-sim/blob/gz-sim8/examples/worlds/spherical_coordinates.sdf)。本步只完成多传感器输入基础；S05 再用这些带误差数据实现局部里程计。

回放补充：当前最小录包器不保存 DDS discovery 的完整 QoS 配置。手工回放使用 `sensor_replay_qos.yaml` 显式恢复 TF static 与 CameraInfo 的 reliable/transient_local，避免标准 TF 监听者与默认 volatile 回放不匹配。该配置与源码一起绑定回执。
