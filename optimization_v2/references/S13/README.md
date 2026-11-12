# 上游与版本

复用安装的 ROS 2 Jazzy Nav2，不重写控制器：DWBLocalPlanner、RegulatedPurePursuitController、MPPIController、SmacPlanner2D、NavigateToPoseNavigator。版本由 `dependency_lock.json` 的 dpkg-query 实测记录。

- https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/controller_plugins/configuring_regulated_pp/
- https://api.nav2.org/nav2-jazzy/html/motion__models_8hpp_source.html
- https://docs.nav2.org/jazzy/configuration_and_development/tuning_guide/

自写 `measured_controller.cpp` 包装 Nav2 Controller 接口，记录实际计算时间；`command_gate.cpp` 实现项目准入与唯一出口。`WheelDrive.cc` 来源于本项目 S03 的物理驱动，保留原实现，仅新增显式停车转向模式；S03 文件未修改。源文件哈希见回执。本项目原模块的许可/来源说明继续适用。

上游包许可证以 `/opt/ros/jazzy/share/<package>/package.xml` 为准；Nav2 主要采用 Apache-2.0，DWB 各模块以各自包声明为准，不重新分发修改后的上游库。

针对本机 Jazzy 的参数语义还核对了以下上游源码：

- https://raw.githubusercontent.com/ros-navigation/navigation2/jazzy/nav2_dwb_controller/dwb_critics/src/rotate_to_goal.cpp
- https://raw.githubusercontent.com/ros-navigation/navigation2/jazzy/nav2_mppi_controller/src/optimizer.cpp
- https://raw.githubusercontent.com/ros-navigation/navigation2/jazzy/nav2_mppi_controller/src/critics/path_follow_critic.cpp
- https://raw.githubusercontent.com/ros-navigation/navigation2/jazzy/nav2_mppi_controller/src/critics/path_align_critic.cpp

MPPI 的正则项包含 gamma/采样标准差²，降低采样尺度后继续沿用原 gamma 会改变代价平衡；本步通过实测重新匹配低速尺度，速度上限与安全门限不因此改变。

- [Jazzy PathAlignCritic](https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_mppi_controller/src/critics/path_align_critic.cpp)：比较预测轨迹与可行路径朝向。
- [Jazzy ConstraintCritic](https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_mppi_controller/src/critics/constraint_critic.cpp)：阿克曼半径的软代价；最终输出仍由运动模型和独立门控施加硬约束。
- [Nav2 Jazzy 坐标与里程计](https://docs.nav2.org/jazzy/configuration_and_development/first_time_robot_setup_guide/odom/setup_odom_gz/)：map/odom 各自职责。

- [Jazzy PathAngleCritic](https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_mppi_controller/src/critics/path_angle_critic.cpp)：目标点夹角与路径切线角不是同一量，方向跟踪主要由启用 `use_path_orientations` 的 PathAlignCritic 约束。

S13 的地形二进制由本项目 S09 源码在编译目录生成：风险计算和即时保护保持不变，只适配车型自体屏蔽并追加独立的新鲜观测确认网格。原 S09 源码/头文件由前置回执及运行绑定核验，S13 CMake、适配头文件和最终二进制归本步哈希。Nav2 到点树参考本机 `/opt/ros/jazzy/share/nav2_bt_navigator/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml` 的 PipelineSequence/RateController；不使用其中的代价图清除恢复。

- [ROS 2 Jazzy Fast DDS participant.cpp](https://github.com/ros2/rmw_fastrtps/blob/jazzy/rmw_fastrtps_shared_cpp/src/participant.cpp)：LOCALHOST 分支设置 maxInitialPeersRange=32。
- [ROS 2 官方 DDS 配置示例](https://github.com/ros2/ros2_dds_profiles_examples/blob/main/fastdds/unicast_discovery_from_another_host.xml)：自定义 UDP 发现范围；S13 仅保留 127.0.0.1，范围 100，并保留共享内存传输。

- [Jazzy Costmap2DROS::on_activate](https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_costmap_2d/src/costmap_2d_ros.cpp)：初始 TF 等待期限由节点当前 ROS 时钟计算。S13 热重启先确认实际时钟已接收，不修改上游期限。

- [Jazzy NoiseGenerator](https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_mppi_controller/src/noise_generator.cpp)：`regenerate_noises` 控制每轮是否重新采样；false 时重置仍会生成新样本。差速配置启用持续采样，并匹配 optimizer 的序列移位条件。
