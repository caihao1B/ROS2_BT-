# S07 核心代码导读

真实调用链：`stepctl.sh → stages/S07/run.sh → localization_stage.main → run_case`。生产实现与上游定位算法分开；两个 lessons 文件只用于最小例子学习。

## 自写 C++：唯一全局校正与准入

- [fromPose](../../../optimization_v2/ws/src/global_pose_manager/src/global_pose_manager.cpp#L17)：将 ROS 位姿转换成 SE(3)；四元数归一化。
- [evidence](../../../optimization_v2/ws/src/global_pose_manager/src/global_pose_manager.cpp#L32)：检查版本/时间/质量，将全局观测乘局部里程计逆得到 map→odom；多帧一致且停车后才准入。跳变不论发生在可靠还是退化状态，都锁存重新验证。
- [tick](../../../optimization_v2/ws/src/global_pose_manager/src/global_pose_manager.cpp#L52)：独立墙钟和仿真时钟检查、合同/里程计/观测时效；发布唯一全局 TF、协方差位姿和带原因的状态。
- [Manager](../../../optimization_v2/ws/src/global_pose_manager/src/global_pose_manager.cpp#L73)：建立订阅、唯一发布者和诊断恢复服务。恢复服务不代表真实作业计划已经验证。

核心公式：`T_map_odom = T_map_base(t) × inverse(T_odom_base(t))`。实际 TF 的时间戳来自最新里程计，全局匹配年龄另行公布；局部积分不被回环修正打断。

## 自写 Python：观测、版本和实际试验

**s07_common.py**
- [safe](../../../optimization_v2/ws/src/global_pose_manager/scripts/s07_common.py#L11)：拒绝绝对路径、上级目录和逃逸符号链接。
- [verify_bundle](../../../optimization_v2/ws/src/global_pose_manager/scripts/s07_common.py#L17)：逐个校验真实载荷、标定、S06 回执及诊断请求版本。
- [capture_inputs](../../../optimization_v2/ws/src/global_pose_manager/scripts/s07_common.py#L30)：记录实际在线模块、配置、二进制的内容哈希，断点复用前逐项比较。

**freeze_map.py**
- [freeze](../../../optimization_v2/ws/src/global_pose_manager/scripts/freeze_map.py#L6)：复制真实 S06 DB/点云，导出占据层与粗地形，生成版本并设为只读。

**localization_preprocessor.py**
- [interpolate_odom](../../../optimization_v2/ws/src/global_pose_manager/scripts/localization_preprocessor.py#L18)：位置线性插值、四元数 SLERP；只用时间两侧真实里程计，不外推。
- [main](../../../optimization_v2/ws/src/global_pose_manager/scripts/localization_preprocessor.py#L29)：直接接收带误差相机和点云；复用 S06 稳定点过滤；整帧运动补偿后严格同步；转发局部 TF 到隔离的上游输入。

**map_refinement.py**
- [refine](../../../optimization_v2/ws/src/global_pose_manager/scripts/map_refinement.py#L8)：缓存 KD-tree 对应搜索与 SVD 刚体精修；交替点划分拟合/验证，修正越界或退化即拒绝。

**localization_quality.py**
- [main](../../../optimization_v2/ws/src/global_pose_manager/scripts/localization_quality.py#L18)：配对同时间的 RTAB Info/位姿/点云；精配准并检查留出点残差和协方差；文件变化锁存失效，发布质量与合同心跳。

**localization_motion.py**
- [main](../../../optimization_v2/ws/src/global_pose_manager/scripts/localization_motion.py#L10)：全局准入与 S05 健康状态取交集，沿用 S03 前方制动保护；只发送诊断低速请求，唯一底盘桥实际执行。

**localization_runtime.py**
- [compose](../../../optimization_v2/ws/src/global_pose_manager/scripts/localization_runtime.py#L11)：只在仿真侧改变三种出生位置，不把其写进在线估计器。
- [run_case](../../../optimization_v2/ws/src/global_pose_manager/scripts/localization_runtime.py#L17)：真实关闭/启动定位进程、发送粗先验、遮挡注入、资源与时间 watchdog、仅收尾自身进程、比较数据库前后哈希。

**localization_analysis.py**
- [analyze](../../../optimization_v2/ws/src/global_pose_manager/scripts/localization_analysis.py#L5)：唯一读取独立真值的离线评分，使用固定原点、不拟合轨迹；检查误差、物理停车、多帧恢复、唯一所有权和版本。

**localization_stage.py**
- [aggregate](../../../optimization_v2/ws/src/global_pose_manager/scripts/localization_stage.py#L22)：四个检查项必须覆盖完整正常/失败矩阵，不以单个演示代替整步 PASS。
- [receipt](../../../optimization_v2/ws/src/global_pose_manager/scripts/localization_stage.py#L28)：写四份材料和真实文件的哈希回执；失败也更新状态。
- [main](../../../optimization_v2/ws/src/global_pose_manager/scripts/localization_stage.py#L53)：共享锁、会话身份、编译和组件测试、五案例矩阵、报告与独立 demo 记录。

## 生产算法与教学例边界

上游 RTAB-Map 负责特征、图优化与粗定位，不是本项目重写。Python 精配准只做已经匹配候选的小修正；C++ 管状态和 TF 所有权。教学例展示矩阵关系和确认/拒绝，不替代生产质量检测、时序检查或真实物理制动。

正常/边界验证还包括 `tests/refinement.py` 的已知刚体误差、越界、少点和退化直线，以及 `tests/manager_contracts.py` 在独立 ROS domain 123 启动真实 C++ 节点，对单帧拒绝、多帧恢复、质量失败、跳变锁存、合同撤销和时钟倒退进行组件验证。它们是明确标记的合成单元输入，不能冒充 Gazebo 重定位数据。

[两种语言运行命令](../../../optimization_v2/lessons/S07/README.md) · [实际数据与边界](RESULTS.md)
