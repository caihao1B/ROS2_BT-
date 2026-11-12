# 从传感器到可信位姿：本项目定位与融合学习手册

这份资料解释当前项目实际使用的算法，重点是 IMU、相机、激光、轮速如何分工，以及 SLAM 的结果如何成为车辆可以使用的位姿。它是独立的学习资料；没有修改生产节点、配置、地图、阶段回执，也不启动仿真。源码核查日期：2026-11-12。主对话可能继续更新项目，请结合本目录的源码快照判断版本。

## 01 · 先看结论与数据流

当前系统是分层的多传感器定位系统。不能概括为一个“相机＋IMU 紧耦合 SLAM”。

| 问题 | 当前实现 | 主要用途 |
| --- | --- | --- |
| 刚才到现在移动了多少？ | S05：轮速、IMU、独立 LiDAR ICP 速度增量 → robot_localization 普通 EKF → 连续里程计积分 | 高频局部运动估计 |
| 怎样建立三维地图、认出旧地点？ | S06：RTAB-Map，双目视觉地点识别、点到平面 ICP、GTSAM 位姿图优化 | 建图与回环 |
| 重启后如何在已知地图中定位？ | S07：冻结 RTAB-Map 数据库、只读定位、几何质量门控 | 冻结地图重定位基线 |
| 当前 S14 完整簇运行采用什么？ | 注册区域分支：冻结区域点云＋实时激光＋相对里程计，SE(2) 匹配 | A/B 区域定位；并非在线重建 RTAB-Map |
| 定位结果能否用于行驶？ | S08：GNSS 坐标配准、杆臂补偿、创新门限、地图/GNSS 来源监督 | 唯一 map→odom 修正与停车/恢复依据 |
| 相机和雷达还融合在哪里？ | S09：图像语义、激光几何、双目深度一致性、目标跟踪 | 障碍物感知；不是机器人自身位姿滤波 |

<!-- widget:flow -->

**相机与 IMU 的联系是分层、间接的。** IMU 在 S05 帮助估计角速度和重力方向；相机在 S06/S07 帮助识别地点和验证回环，RTAB-Map 又使用 S05 里程计提供的倾斜姿态作为重力约束。这不等于把像素重投影误差、IMU 预积分误差、陀螺仪/加速度计偏置一起放入同一个优化器。

当前没有实现 VINS 式紧耦合视觉惯性里程计，也没有 FAST-LIO 式误差状态激光惯性滤波。请把“已经使用多种传感器”和“实现了某一种紧耦合算法”分开理解。

## 02 · 每种传感器提供什么信息

| 数据 | ROS 消息 / 入口 | 本项目怎么用 | 单靠它缺少什么 |
| --- | --- | --- | --- |
| 轮速 | TwistWithCovarianceStamped；/s04/localization/wheel | 前向速度；同时加入车体侧向/竖向零速滚动约束 | 打滑时不等于实际车速；不是地图绝对位置 |
| IMU | Imu；/s04/localization/imu | 陀螺仪角速度；准静态加速度方向估计 roll/pitch | 重力不提供绝对 yaw；加速度会混入车辆运动 |
| 三维激光 | PointCloud2；/s04/localization/points | S05 局部 ICP；S06 建图；S14 区域匹配 | 平地、长直墙、重复结构可能约束不足 |
| 双目相机 | Image + CameraInfo；/s04/stereo/left、right 下的 image_rect | 地点识别、视觉几何；S09 语义与视差深度 | 弱纹理、重复纹理、光照变化会影响可靠性 |
| GNSS/RTK 仿真观测 | NavSatFix 进入适配层，监督器接收 /s08/gnss 结构化观测 | ENU 配准、绝对位置一致性检查、有限降级 | 单天线静止时没有可靠绝对航向；fixed 标签不是正确性保证 |

仿真观测、算法输出、评估真值有不同用途。在线算法应读取带噪观测；仿真真值只用于评估。项目用 ROS domain 119/120/121 区分在线、评估和原始传感器链路；这属于工程隔离，不能理解为访问控制安全边界。

融合之前先解决四件事：**时间、坐标、外参、协方差**。时间戳错误会把“旧位置的观测”当成“现在的观测”；相机光学系和车体系不能直接相加；GNSS 天线位置不等于底盘中心；协方差控制算法如何看待误差大小。

S07 预处理器将左右图像严格配对，限制图像与点云时间差不超过 40 ms；用里程计位置插值与四元数 SLERP，把整帧点云变换到原始相机时刻。它没有伪造新的相机采集时间，也没有实现带逐点时间的激光扫描去畸变。

```text
p_base(t_camera) = inverse(T_odom_base(t_camera))
                   × T_odom_base(t_lidar) × p_base(t_lidar)
```

注意一个运行依赖：S14 区域匹配器自身不订阅图像，但是它当前使用的 /s07/inputs/cloud 由上述双目同步预处理器输出。因此相机丢失仍可能使这条点云输入停更。这是输入链路依赖，不是相机像素进入了区域匹配的目标函数。

## 03 · S05：普通 EKF 怎样融合轮速、IMU 和激光

核心配置在 `optimization_v2/config/local_estimator.yaml`。当前使用 robot_localization 3.8.3 的 15 状态 EKF，`two_d_mode: false`，频率 30 Hz。状态顺序如下：

```text
x = [x, y, z, roll, pitch, yaw,
     vx, vy, vz, wx, wy, wz, ax, ay, az]
```

| 输入配置 | 实际选择的分量 | 未选择的关键分量 |
| --- | --- | --- |
| twist0：轮速 | vx、vy、vz；后两项是车体零速约束 | 轮速积分得到的位置、轮速 yaw |
| imu0：IMU | roll、pitch、wx、wy、wz | yaw 绝对值、ax/ay/az |
| twist1：激光增量 | vx、vy、vz、wx、wy、wz | 激光累计绝对位姿 |

`true/false` 掩码比话题里“有哪些字段”更重要。IMU 消息有加速度，不代表 EKF 使用了加速度；有 `/s05/imu/gravity_removed` 诊断话题，也不代表它参与了本滤波器。滤波状态含 ax/ay/az，但这里没有直接融合加速度计测量。状态中没有陀螺仪偏置、加速度计偏置项，因此不应称为包含 IMU 偏置状态的 ESKF。

EKF 每轮先预测，再根据测量与预测的差异更新。Q 描述过程模型的不确定性，R 描述观测的不确定性，P 描述状态估计的不确定性。

```text
预测：x⁻ = f(x, dt)              P⁻ = F P Fᵀ + Q
创新：r = z - h(x⁻)             S = H P⁻ Hᵀ + R
增益：K = P⁻ Hᵀ S⁻¹
更新：x⁺ = x⁻ + K r
协方差：P⁺ = (I-KH) P⁻ (I-KH)ᵀ + K R Kᵀ
```

这是教学符号；上游实现还涉及角度归一化、异常测量拒绝和时间处理。先读本项目输入掩码，再读 `references/S05/ekf.cpp` 的 `predict()`、`correct()`，会比直接从矩阵代码开始更容易。

**IMU 预处理。** `input_adapter.py → Inputs.imu()` 在启动后 1–4 秒、轮速近零时估计陀螺仪静止偏置。加速度模长接近重力且车辆加速度较小时，才使用加速度方向更新 roll/pitch，并采用时间常数 0.25 秒的低通。剧烈加速时不把加速度方向直接当重力方向；不可信倾斜观测用 IMU 协议标记为不可用。这里是启动静止标定，不是持续联合估计偏置。

**激光预处理。** `cloud_filter.py` 去除非有限点、车体点、超范围点并做体素降采样；用邻域 PCA 法向量的方向分布检查几何约束。`icp_odometry` 进行独立的点到平面局部配准；`icp_gate.py` 再审核匹配状态、内点率、时间和协方差，将一段时间内的位姿差转换成速度增量，送入 EKF。它没有把由同一 EKF 预测出的速度再次伪装成独立激光观测。

**轮滑降权。** 对齐时间窗口后比较轮速与独立激光速度。差异超过 0.08 m/s 且持续 0.3 秒时，将前向轮速方差从 0.0025 提高到 4.0；观测还在，但权重下降。不要把这种残差比较理解成轮胎物理模型。

> 源码核对提示：配置和旧 S05 说明写有恢复阈值 0.04 m/s、恢复保持 1.5 秒；当前 `Inputs.wheel()` 的 `elif fresh` 分支会直接清除 slipping，并未执行该恢复迟滞。这里按实际源码说明，不修改原对话正在验收的实现。阅读配置时必须追到变量是否真正被使用。

<!-- widget:ekf -->

**为什么还要 `odom_output.py`？** 公开的 `/s05/odometry` 不是直接转发 EKF 的位置字段。`Integrator.update()` 将滤波后的车体速度、角速度与 roll/pitch 积分为连续位置和 yaw，避免未锚定的位置交叉协方差更新造成公开 odom 突跳。原始滤波输出仍在 `/s05/raw/fused` 供分析。此积分器的协方差传播是工程近似，没有完整处理跨时刻相关性。

这一设计适合当前非完整约束的差速/阿克曼车辆。如果后续使用麦克纳姆全向底盘，不能原样保留“车体 vy 恒为零”的滚动约束。

## 04 · S06：RTAB-Map 怎样建图与回环

定位前端回答相邻时刻怎样移动；回环回答是否再次看到了很久以前的地点。两者共同为地图提供约束。

本机锁定 RTAB-Map / rtabmap_ros 0.23.13。`slam_mapping.yaml` 选择 GFTT/ORB 视觉特征、ICP 配准、libpointmatcher 点到平面实现，以及 GTSAM 位姿图后端。

| 阶段 | 实际技术选择 | 理解重点 |
| --- | --- | --- |
| 输入整理 | mapping_inputs.py；2 Hz 建图采样；体素、车体屏蔽、时间一致性 | 减少负载，保留可用三维结构 |
| 动态点抑制 | 在局部里程计坐标对齐后检查历史邻近支持 | 几何时序过滤，不是语义分割网络 |
| 地点识别 | GFTT 检测、ORB 描述子、视觉词袋与地点候选评分 | “外观看起来像”只产生候选 |
| 几何验证 | 无初值全局候选需视觉几何初值，再由 ICP 验证；近邻候选可用里程计初值 | 不能把每一种约束都写成同一条固定链 |
| 地图优化 | SE(3) 位姿图，GTSAM；加入回环和相邻约束 | 修正历史关键帧之间的不一致 |
| 重力方向 | Mem/UseOdomGravity=true；GravitySigma=0.032 | 来自 S05 姿态的 roll/pitch 约束，不提供绝对位置和 yaw |

核心配准残差是点到平面的距离：把当前点 p 变换到参考坐标系，检查它相对于对应面上点 q、法向 n 的距离。平面法向约束不全时，算法必须识别退化，而不是因为点很多就宣布可信。

```text
ICP 残差：r_i = n_iᵀ (R p_i + t - q_i)
目标：min Σ ρ(r_i²)

位姿图：min Σ || Log(Z_ij⁻¹ T_i⁻¹ T_j) ||²_(Ω_ij)
T_i：关键帧位姿；Z_ij：观测到的相对运动；Ω_ij：信息矩阵
```

相机在这里有实际价值：平地、相似墙面可能让纯激光产生错误的地点关联，视觉外观和几何能提供另一类约束。但弱纹理或重复纹理也会削弱视觉；回环不是只要检测到相似图像就立刻“拉回去”。

RTAB-Map 通过 `Mem/UseOdomGravity` 使用的重力姿态已经来自融合链路。它不是第二份统计独立的原始 IMU 证据，也不等于 IMU 预积分因子。GNSS 没有同时灌进这条 RTAB-Map 图优化后再被当作独立监督证据。

<!-- widget:graph -->

这个图优化实验只展示一维带锚点的最小二乘。生产后端优化三维位姿、旋转及多个约束，不能用教学实验的数值推断项目精度。

## 05 · S07 与当前 S14：重定位不等于继续建图

S07 冻结数据库、点云以及 manifest 中的地图/坐标版本，启用 RTAB-Map 的只读定位模式。在线观测在既有地图上求位姿，不应悄悄修改用于规划和验收的参考地图。`localization_preprocessor.py` 对齐输入，质量层审核匹配，再交给全局位姿管理器。

**当前 S14 的注册区域完整簇分支发生了实现变化。** `mission_bt/scripts/runtime14.py` 在该模式下用 `regional_localizer14.py` 替换 RTAB-Map 在线定位进程，读取 `maps/s14_regions/manifest.json` 绑定的冻结区域点云。旧 RTAB-Map 建图/重定位基线仍然存在。请先看运行入口选择了哪条分支，再判断某次实验使用的算法。

`regional_math14.py → SurfaceMap.align()` 的实际流程：

1. 过滤当前点云，使用高于 0.2 m 的结构点；需要足够点数。
2. 从人工粗先验启动；之后用相对 S05 里程计预测匹配初值。
3. 初始化时进行三个 yaw 初值尝试，比较合格结果是否彼此一致。
4. 用点到点对应和 SVD 作粗配准，修正反射解；再用点到平面残差作精配准。
5. 用鲁棒权重 `min(1, 0.08 / abs(residual))` 降低大残差影响；检查法方程最小特征值，拒绝几何退化。
6. 偶数序号点用于优化，奇数序号点用于留出验证；检验覆盖率和对应距离。
7. 继续检查初值捕获范围、跟踪修正幅度、区域边界和地图版本；输出观测，不直接发布 map→odom。

| 当前约束 | 作用 |
| --- | --- |
| 留出对应距离 < 0.35 m、重叠率 ≥ 0.55、内点距离 P95 ≤ 0.30 m | 拒绝覆盖不足或几何不一致的候选 |
| 初始化捕获偏移 < 3 m、角度 < 0.35 rad | 限制局部匹配的可用范围；不是任意位置全局定位 |
| 多初值结果差异 ≤ 0.08 m、≤ 0.015 rad | 拒绝初始化歧义 |
| 跟踪修正 ≤ 0.15 m、≤ 0.035 rad | 防止突然换到另一个结构 |

这里估计的是 **SE(2)：x、y、yaw**。输入虽是三维激光，输出的区域地图观测把 z、roll、pitch 固定为零；S05 自身仍保留三维局部姿态。不能据此宣称 S14 区域定位已解决任意坡面上的完整六自由度定位。

留出点没有参与这次优化，能减少“只在拟合点上看起来很好”的问题；但它们仍来自同一帧传感器和同一份地图，不能当作第二个独立传感器。GNSS 检查提供另一来源的一致性证据，也仍受外参、配准与共同偏差影响。

为什么增加区域地图？全局刚体配准只能整体旋转、平移；无法消除大地图内部的局部形变。当前注册区域路径针对 A/B 工作区进行约束与检查。已有完整簇运行证据不能自动推广到所有簇、跨区连接路或任意地形。

## 06 · 位姿矫正与 GNSS 监督：修哪里、何时允许走

按照 ROS 坐标约定，odom 用于连续的短时运动，map 用于长期参考。项目把这两种职责分开。

```text
map ──全局修正──> odom ──连续局部运动──> base_link ──外参──> sensors

T_map_base = T_map_odom × T_odom_base
T_map_odom = T_map_base(观测时刻) × inverse(T_odom_base(同一时刻))
```

S14 当前链路中，`s05_local_odometry` 发布 odom→base_link；`s08_global_pose_manager` 发布 map→odom。S06 建图模式下由 SLAM 发布全局变换，各模式不能同时争抢同一个 TF。修正 map→odom 并不意味着向局部积分器强行写入一组新位置。

<!-- widget:tf -->

GNSS 并不是再加一个“把所有位置平均”的 EKF。当前是带质量门控的全局参考选择器，核心在 `gnss_supervisor/src/gnss_supervisor.cpp`。

1. 将经纬高转换到 ECEF、局部 ENU；通过已声明控制点把坐标参考配准。S08 基线采用二维刚体 SVD 配准；S14 注册区域使用自己的区域参考与校准结果，不能把两套 manifest 混用。
2. 把 GNSS 协方差旋转到一致坐标，并处理配准不确定性。
3. 利用底盘姿态旋转天线杆臂，再从天线位置扣除杆臂，得到底盘参考位置。
4. 将 GNSS 与同一时刻的地图/里程计预测比较，检查观测新鲜度、fixed 模式、协方差、距离和 NIS。
5. 地图可信时优先使用地图修正；地图暂失、GNSS 合格且航向尚有效时，只作有限的 GNSS 位置降级。
6. 来源丢失、版本不一致、突跳或航向过期时停车；恢复要求连续确认和实际静止。

```text
天线杆臂补偿：p_base = p_antenna - R_map_base × lever_arm_base
创新：r = p_base_GNSS - p_base_predicted
S = C_GNSS_xy + reference_variance × I
NIS = rᵀ S⁻¹ r
当前门控：||r|| ≤ 0.15 m 且 NIS ≤ 9.21
```

9.21 对应二维卡方分布约 99% 的门限；只有在创新近似零均值高斯、S 能代表误差时，才有该统计含义。本项目的 S 使用观测协方差加工程参考方差，不是完整联合滤波器计算出的全部创新协方差，所以应称为一致性门控，不应宣称获得了严格标定的 99% 可靠性。

<!-- widget:gnss -->

关键状态有 `TRUSTED_MAP`、`MAP_ONLY`、`GNSS_ONLY`、`CONFIRMING`、`STOPPED`。单天线 GNSS 只提供位置，地图丢失时借用局部航向的时间有限；基线 `heading_hold_s` 为 8 秒。GNSS 降级只平滑修正 XY 平移，不凭空制造绝对航向。

一次大修正可能改变机器人与已规划路径的关系，所以 `plan_revalidation_required` 会锁住行驶，要求重新确认计划。**TF 仍存在不代表 navigation_allowed 为真；定位允许也只是整车允许行驶的必要条件之一。** 监督器输出协方差还带有工程设定值，不能直接当作精度证明。

## 07 · 相机融合、紧耦合算法与当前选择的取舍

S09 的 `object_fusion/scripts/fusion_node.py` 对真实仿真图像运行 NanoDet，把激光几何投影到相机图像中进行关联，再以 StereoSGBM 视差深度进行一致性检查。`tracker.py` 维护 `[x,y,vx,vy]` 目标状态，使用线性卡尔曼滤波和匈牙利匹配。这些状态描述障碍物，不是机器人自身的地图位姿。

| 方案 | 优点 | 代价或边界 | 与当前项目的关系 |
| --- | --- | --- | --- |
| 当前模块化 EKF＋RTAB-Map/区域匹配 | 接口清楚、能逐模块检查；轮速适合地面车辆；部署和诊断较直接 | 模块间共享信息的相关性难完全处理；前端丢失后后端不能凭空恢复 | 已实现，先理解并验证它 |
| 视觉惯性紧耦合，如 VINS 类 | 联合利用像素运动和惯性约束；偏置可进状态 | 对时钟、标定、曝光、纹理、激励要求高；接入后需重新验收 | 可作为未来对照，当前未实现 |
| 激光惯性紧耦合，如 FAST-LIO/LIO-SAM 类 | 高频惯性传播，支持运动补偿；适合复杂三维运动 | 需要真实逐点时间、IMU 外参、偏置/噪声模型；开阔退化仍存在 | 不能只换包名就认为完成升级 |
| RTK 与地图互相监督 | 可暴露长期偏移；位置参考直观 | 单天线无静止航向；多径、配准误差仍需处理 | 当前使用参考选择＋门限，不是完整联合最优估计 |

建议先学习现有链路的时间、坐标与质量检查，再比较更复杂的算法。要接近实机，最先补齐的是可观测的数据契约、标定、时间与误差模型，而不是只增加滤波器数量。后续算法替换应安排在独立实验中，保留相同路线、观测和指标做对比。

## 08 · 源码在哪里，怎么打开

实际项目根目录是 `/home/ceihao/robot_project`。不要使用原来新加卷上的旧入口。

| 文件夹 | 应该读什么 |
| --- | --- |
| optimization_v2/config | 输入掩码、噪声、配准与监督门限 |
| optimization_v2/ws/src/localization_guard | S05 预处理、局部里程计、质量门控；Python 为主，健康监督为 C++ |
| optimization_v2/ws/src/map_manager | S06 建图输入与 RTAB-Map 启动 |
| optimization_v2/ws/src/global_pose_manager | S07 时间对齐、冻结地图定位与质量检查 |
| optimization_v2/ws/src/gnss_supervisor | 坐标转换的 Python 与全局监督 C++ |
| optimization_v2/ws/src/mission_bt/scripts | 当前 S14 区域匹配、数据适配和运行分支 |
| optimization_v2/ws/src/object_fusion | 相机/激光障碍感知与跟踪；与自身定位区分阅读 |
| optimization_v2/references/S05、S06 | 上游实现的学习副本与来源说明；不是另一个生产实现 |
| optimization_v2/maps/frozen、maps/s14_regions | 冻结地图及版本契约；阅读 manifest，不直接编辑生产地图 |

细到函数名和行号的入口见 [源码导航](SOURCE_INDEX.md)，对应内容的 SHA-256 在 [源码快照](SOURCE_SNAPSHOT.json)。文件更新后行号可能变化，可重新搜索函数名。源码快照用于说明这份教程读过哪版文件，不替代阶段验收回执。

下列命令只打开文件或打印文本：

```bash
cd /home/ceihao/robot_project
xdg-open docs/optimization_v2/learning_localization/index.html

# 从最有价值的输入配置开始
less optimization_v2/config/local_estimator.yaml
# less 中按 / 搜索、n 下一个、q 退出
less +35 optimization_v2/ws/src/localization_guard/scripts/input_adapter.py
less +19 optimization_v2/ws/src/localization_guard/scripts/odom_output.py
less +20 optimization_v2/ws/src/mission_bt/scripts/regional_math14.py
less +31 optimization_v2/ws/src/gnss_supervisor/src/gnss_supervisor.cpp

# 如本机安装了 VS Code，可跳到具体行
command -v code && code --goto optimization_v2/ws/src/localization_guard/scripts/input_adapter.py:35
```

搜索“生产运行时到底启用了谁”，优先看运行入口而非文件名：

```bash
cd /home/ceihao/robot_project
rg -n 's05_fused|ekf_node|icp_odometry' optimization_v2/ws/src/localization_guard/scripts/runtime.py
rg -n 'regional_localizer|start_slam|REGISTERED_REGION' optimization_v2/ws/src/mission_bt/scripts/runtime14.py
rg -n 'twist0|twist1|imu0|two_d_mode' optimization_v2/config/local_estimator.yaml
rg -n 'mapEvidence|gpsEvidence|PLAN_REVALIDATION_REQUIRED|GNSS_ONLY' optimization_v2/ws/src/gnss_supervisor/src/gnss_supervisor.cpp
```

## 09 · 不打扰主任务的实验与观察命令

先做脱离 ROS 的小实验，只需要系统 Python 和已有 C++ 编译器。它们不会连接传感器、发速度、修改地图或调用恢复服务。

```bash
cd /home/ceihao/robot_project
python3 docs/optimization_v2/learning_localization/examples/learning_lab.py

# C++ 二进制放入一次性临时目录；退出这个子 shell 时自动清理
(
  set -e
  learning_build_dir=$(mktemp -d /tmp/robot-learning.XXXXXX)
  trap 'rm -rf -- "$learning_build_dir"' EXIT
  g++ -std=c++17 -O2 -Wall -Wextra -pedantic \
    docs/optimization_v2/learning_localization/examples/pose_correction.cpp \
    -o "$learning_build_dir/pose_correction"
  "$learning_build_dir/pose_correction"
)
```

预期：Python 给出五项教学检查通过；正常轮速方差时估计速度更接近轮速，滑移降权后更接近激光；16 cm 偏移即使 NIS 通过，也会被距离门限拒绝；图优化后误差重新分配，但不会神奇地消失。C++ 验证 TF 复合与门控逻辑。教学程序通过只证明教学例子正确，不代表重新验收了机器人。

如果主任务当前已经启动对应节点，可在另一个终端进行一次短暂只读观察。不为学习额外启动第二套仿真：

```bash
source /home/ceihao/robot_project/optimization_v2/env.sh

# 明确查看融合器到底选了哪些测量分量
timeout 6s ros2 param get /s05/fused/s05_fused twist0_config
timeout 6s ros2 param get /s05/fused/s05_fused imu0_config
timeout 6s ros2 param get /s05/fused/s05_fused twist1_config

# 连续局部位姿、区域匹配证据、全局监督状态
timeout 6s ros2 topic echo /s05/odometry --once
timeout 6s ros2 topic echo /s07/localization/evidence --once
timeout 6s ros2 topic echo /s08/status --once

# 观察变换；6 秒后主动结束
timeout 6s ros2 run tf2_ros tf2_echo map odom
```

`env.sh` 会重建当前 shell 的 ROS 环境。若暂未运行对应阶段，命令超时或没有节点属于正常情况；timeout 的 124 退出码不是项目 FAIL。这里不执行复位、不发布 initialpose、不解除停车，也不重跑会更新版本回执的阶段脚本。

观察时重点记录：`state` 为什么变化；`source` 选择哪个来源；`map_reason`、`gnss_reason` 为什么拒绝；`innovation_m` 与 `nis` 是否同时满足；`plan_revalidation_required` 是否锁住任务。数据连续更新不等于位置正确，还需要独立误差与任务证据。

## 10 · 六次学习，每次都有可查验的产出

| 次序 / 建议时长 | 阅读与练习 | 完成标准 |
| --- | --- | --- |
| 1 / 30 分钟 | 第 1–2 节；手画实际数据流和坐标树 | 能说清自身定位与障碍跟踪的区别；标出两个 TF 发布者 |
| 2 / 45 分钟 | 第 3 节；local_estimator.yaml、Inputs.imu/wheel；运行 EKF 实验 | 能解释 IMU 加速度为什么没有直接融合，R 增大为何降权 |
| 3 / 45 分钟 | cloud_filter、icp_gate、Integrator.update | 能解释平地退化、速度增量与累计位姿的差别；找到公开 odom 的真实来源 |
| 4 / 60 分钟 | 第 4 节；slam_mapping.yaml；图优化实验 | 能区分地点候选、几何验证、回环约束、全局优化；解释重力不能确定 yaw |
| 5 / 60 分钟 | 第 5–6 节；SurfaceMap.align、mapEvidence、gpsEvidence、tick | 能写出 map→odom 公式；解释为什么 16 cm 偏差可被拒绝；找到 S14 分支切换位置 |
| 6 / 45 分钟 | 做一次可用节点的只读观察；完成下方自测 | 给出一页“数据来源—算法—拒绝原因—输出”的读码笔记，引用文件与函数 |

建议笔记模板：本次节点输入是什么？哪些字段真的参与计算？用了哪个坐标和时刻？关键误差模型是什么？何时拒绝？谁消费输出？失效后车辆会怎样？最后写一条可复现的观察或教学实验。

<!-- widget:quiz -->

**自测答案。** 相机没有进入 S05 EKF；IMU 绝对 yaw 没有被融合。重力只约束两个倾斜自由度。当前 S14 区域匹配是 SE(2)，不能因传感器是三维就称它为六自由度 SLAM。GNSS fixed、TF 存在都不足以允许行驶。图像停止可能影响区域匹配输入，是因为共享预处理器的同步依赖。轮滑恢复迟滞配置存在但当前分支未使用。掌握这些区别，比背算法名称更有用。

## 11 · 一手资料与适用范围

下面链接用于核对概念和上游实现；本项目的接线、掩码和门限以本地源码为准。

- [ROS REP-105：坐标系约定](https://github.com/ros-infrastructure/rep/blob/master/rep-0105.rst)：读 odom、map 和 Frame Authorities。
- [robot_localization 3.8.3 配置说明](https://github.com/cra-ros-pkg/robot_localization/blob/3.8.3/doc/configuring_robot_localization.rst)：读 15 维掩码、避免重复使用同源测量的讨论。
- [RTAB-Map Memory.cpp 上游参考](https://github.com/introlab/rtabmap/blob/master/corelib/src/Memory.cpp)：搜索 computeTransform、guess.isNull，理解无初值时的视觉分支。master 是可变参考，不能称为本机 0.23.13 的逐字源码。
- 算法对照的一手说明：[VINS-Fusion](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion/blob/master/README.md) 介绍相机/IMU 空间与时间标定；[FAST-LIO](https://github.com/hku-mars/FAST_LIO/blob/main/README.md) 说明紧耦合迭代 EKF 与逐点时间需求；[LIO-SAM](https://github.com/TixiaoShan/LIO-SAM/blob/master/README.md) 说明 IMU 预积分、偏置估计与扫描去畸变。这三者是比较对象，不是本项目已接入模块。
- [本项目 S06 参考版本说明](../../../optimization_v2/references/S06/README.md)：主库同名 0.23.13 GitHub 标签不可用；实际参数含义核对安装头文件，ROS 包装层有自己的版本来源。
- [S05 技术记录](../S05/TECHNICAL.md)、[S06 技术记录](../S06/TECHNICAL.md)、[S07 技术记录](../S07/TECHNICAL.md)、[S08 技术记录](../S08/TECHNICAL.md)：结合当前代码阅读；旧结论不能覆盖后续 S14 分支变化。

本手册解释已读源码与数学原理。网页中的曲线、速度、门限演示均为明确标注的合成教学数据；它们不是新的仿真实测。已有 S14 验收、S15 进行中的工作以及 GitHub 交付继续由主对话处理，本资料不改变任何阶段结论。
