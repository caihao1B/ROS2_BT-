# 源码导航

本表由实际文件生成。链接打开源码；函数后的数字是快照时的行号。
在终端使用 `less +行号 文件路径` 或编辑器跳转查看。完整 SHA-256 见 SOURCE_SNAPSHOT.json。

| 文件（相对项目根目录） | 作用与阅读入口 |
| --- | --- |
| [optimization_v2/config/local_estimator.yaml](../../../optimization_v2/config/local_estimator.yaml) | S05 输入掩码与 EKF 状态选择 |
| [optimization_v2/config/localization_policy.yaml](../../../optimization_v2/config/localization_policy.yaml) | S05 噪声、轮滑、几何门限 |
| [optimization_v2/ws/src/localization_guard/scripts/runtime.py](../../../optimization_v2/ws/src/localization_guard/scripts/runtime.py) | 实际启动的 EKF 与 ICP 节点；`compose:11`；`run_case:22`；`launch:24`；`gz:26` |
| [optimization_v2/ws/src/localization_guard/scripts/input_adapter.py](../../../optimization_v2/ws/src/localization_guard/scripts/input_adapter.py) | IMU 标定、倾斜估计、轮滑降权；`Inputs:12`；`reset_state:20`；`reset:22`；`request_reset:29`；`clock:30`；`icp:34`；`imu:35`；`wheel:50` |
| [optimization_v2/ws/src/localization_guard/scripts/cloud_filter.py](../../../optimization_v2/ws/src/localization_guard/scripts/cloud_filter.py) | 点云过滤与法向几何退化检测；`geometry:11`；`cloud:18` |
| [optimization_v2/ws/src/localization_guard/scripts/icp_gate.py](../../../optimization_v2/ws/src/localization_guard/scripts/icp_gate.py) | 独立激光配准的时间与质量门控；`clock:16`；`geom:21`；`emit:24`；`drain:55`；`odom:60`；`info:61`；`epoch:62` |
| [optimization_v2/ws/src/localization_guard/scripts/odom_output.py](../../../optimization_v2/ws/src/localization_guard/scripts/odom_output.py) | 公开连续里程计：积分估计速度；`Integrator:16`；`reset:18`；`update:19`；`reset:36`；`epoch:39`；`clock:40`；`receive:44` |
| [optimization_v2/ws/src/localization_guard/src/health_guard.cpp](../../../optimization_v2/ws/src/localization_guard/src/health_guard.cpp) | C++ 局部定位健康监督；`tick:31` |
| [optimization_v2/references/S05/ekf.cpp](../../../optimization_v2/references/S05/ekf.cpp) | 上游 EKF 学习副本；predict 与 correct；`Ekf::correct:54`；`Ekf::predict:222` |
| [optimization_v2/config/slam_mapping.yaml](../../../optimization_v2/config/slam_mapping.yaml) | RTAB-Map 建图、ICP、重力、GTSAM 配置 |
| [optimization_v2/ws/src/map_manager/scripts/mapping_inputs.py](../../../optimization_v2/ws/src/map_manager/scripts/mapping_inputs.py) | 建图输入同步与时序几何过滤；`temporal_filter:15`；`process:34` |
| [optimization_v2/ws/src/map_manager/scripts/mapping_runtime.py](../../../optimization_v2/ws/src/map_manager/scripts/mapping_runtime.py) | 建图模式启动与 TF 职责；`run_case:16`；`launch:19`；`gz:21` |
| [optimization_v2/config/slam_localization.yaml](../../../optimization_v2/config/slam_localization.yaml) | 冻结地图只读定位配置 |
| [optimization_v2/ws/src/global_pose_manager/scripts/localization_preprocessor.py](../../../optimization_v2/ws/src/global_pose_manager/scripts/localization_preprocessor.py) | 位置插值、SLERP、整帧时间对齐；`interpolate_odom:18`；`forward_local_tf:35`；`process:44` |
| [optimization_v2/ws/src/global_pose_manager/scripts/localization_quality.py](../../../optimization_v2/ws/src/global_pose_manager/scripts/localization_quality.py) | S07 几何观测审核；`clock:24` |
| [optimization_v2/ws/src/global_pose_manager/scripts/map_refinement.py](../../../optimization_v2/ws/src/global_pose_manager/scripts/map_refinement.py) | S07 有界匹配修正；`refine:8` |
| [optimization_v2/config/gnss_alignment.yaml](../../../optimization_v2/config/gnss_alignment.yaml) | S08 控制点与 ENU 地图配准 |
| [optimization_v2/config/localization_supervisor.yaml](../../../optimization_v2/config/localization_supervisor.yaml) | S08 GNSS/地图监督门限 |
| [optimization_v2/ws/src/gnss_supervisor/scripts/alignment.py](../../../optimization_v2/ws/src/gnss_supervisor/scripts/alignment.py) | 坐标转换、刚体 SVD、协方差传播；`solve:5`；`convert:14` |
| [optimization_v2/ws/src/gnss_supervisor/scripts/source_adapter.py](../../../optimization_v2/ws/src/gnss_supervisor/scripts/source_adapter.py) | GNSS 与地图证据适配；`record:15`；`pub:16`；`fault:17`；`match:18`；`contract:34`；`gnss:41`；`quality:42`；`map_obs:43` |
| [optimization_v2/ws/src/gnss_supervisor/src/gnss_supervisor.cpp](../../../optimization_v2/ws/src/gnss_supervisor/src/gnss_supervisor.cpp) | C++ 创新检查、来源选择与 map→odom；`mapEvidence:31`；`gpsEvidence:42`；`tick:52` |
| [optimization_v2/ws/src/mission_bt/scripts/runtime14.py](../../../optimization_v2/ws/src/mission_bt/scripts/runtime14.py) | 当前区域定位分支的实际选择；`runtime:7`；`run_case:50` |
| [optimization_v2/ws/src/mission_bt/scripts/regional_localizer14.py](../../../optimization_v2/ws/src/mission_bt/scripts/regional_localizer14.py) | 区域定位：初值、歧义、修正与版本门控；`hint:33` |
| [optimization_v2/ws/src/mission_bt/scripts/regional_math14.py](../../../optimization_v2/ws/src/mission_bt/scripts/regional_math14.py) | SE(2) 粗配准、鲁棒点到平面、留出验证；`rotation:8`；`SurfaceMap:11`；`prepare_normals:15`；`align:20`；`rigid_fit:50` |
| [optimization_v2/ws/src/mission_bt/scripts/regional_source14.py](../../../optimization_v2/ws/src/mission_bt/scripts/regional_source14.py) | S14 注册区域 GNSS 输入适配；`convert:13` |
| [optimization_v2/maps/frozen/manifest.json](../../../optimization_v2/maps/frozen/manifest.json) | S07 冻结地图版本契约；仅读取 manifest |
| [optimization_v2/maps/s14_regions/manifest.json](../../../optimization_v2/maps/s14_regions/manifest.json) | S14 冻结区域地图版本契约 |
| [optimization_v2/ws/src/object_fusion/scripts/fusion_node.py](../../../optimization_v2/ws/src/object_fusion/scripts/fusion_node.py) | 相机语义、激光与双目深度关联；`info:20`；`process:24` |
| [optimization_v2/ws/src/object_fusion/scripts/tracker.py](../../../optimization_v2/ws/src/object_fusion/scripts/tracker.py) | 障碍物 KF 与匈牙利匹配；不是自身定位；`Tracker:4`；`update:6` |
| [optimization_v2/config/s05_dependency_lock.json](../../../optimization_v2/config/s05_dependency_lock.json) | S05 已安装依赖版本记录 |
| [optimization_v2/config/s06_dependency_lock.json](../../../optimization_v2/config/s06_dependency_lock.json) | S06 已安装依赖版本记录 |
| [optimization_v2/references/S06/README.md](../../../optimization_v2/references/S06/README.md) | 上游来源与版本差异说明 |
