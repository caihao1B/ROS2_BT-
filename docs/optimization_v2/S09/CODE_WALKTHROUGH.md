# S09 核心代码导读

调用链：stepctl → stages/S09/run.sh → stage.main → runtime.run_case → 原 S04/S05/S07/S08 节点 + 新 terrain / fusion / costmap_adapter / 两个 Nav2 节点 → 独立 analyze → 报告和哈希回执。

- [terrain_node.cpp · void cloud](../../../optimization_v2/ws/src/terrain_perception/src/terrain_node.cpp#L32)：C++ 生产：时间/TF 校验、PCL 体素及地面、局部 PCA 高程风险、聚类和栅格发布。
- [risk.hpp · inline int risk](../../../optimization_v2/ws/src/terrain_perception/include/terrain_perception/risk.hpp#L7)：C++ 生产：车型能力与不确定度裕度；clear 控制三次观测清除。
- [risk_layer.cpp · void updateCosts](../../../optimization_v2/ws/src/terrain_perception/src/risk_layer.cpp#L11)：C++ 生产 Nav2 插件：实际主代价地图写入、过期/错误坐标退未知、覆盖旧值避免幽灵。
- [detector.py · class Detector](../../../optimization_v2/ws/src/object_fusion/scripts/detector.py#L7)：Python 生产：核验 ONNX 哈希、OpenCV DNN 与 letterbox；project_box 执行标定投影。
- [fusion_node.py · def process](../../../optimization_v2/ws/src/object_fusion/scripts/fusion_node.py#L24)：Python 生产：按曝光时刻取双目/里程计、推理和视差互证、点云框关联、目标发布。
- [tracker.py · def update](../../../optimization_v2/ws/src/object_fusion/scripts/tracker.py#L6)：Python 生产：状态预测、NIS/尺寸门控、Hungarian 分配、歧义 coast 与 ID 生命周期。
- [costmap_adapter.py · def grid](../../../optimization_v2/ws/src/terrain_perception/scripts/costmap_adapter.py#L22)：Python 生产：授权边界变换、带协方差目标包络、实时风险与停止状态。
- [analysis09.py · def analyze](../../../optimization_v2/ws/src/terrain_perception/scripts/analysis09.py#L7)：仅离线评估：可见 ROI、真实目标匹配、低障碍清除、实际 Nav2 输出与真值隔离检查。
- [runtime.py · def run_case](../../../optimization_v2/ws/src/terrain_perception/scripts/runtime.py#L25)：执行器：ROS119/121/120、生命周期、故障时段、预算、资源和仅本会话收尾。
- [Actor.cc · void PreUpdate](../../../optimization_v2/ws/src/terrain_perception/src/Actor.cc#L38)：仅仿真：行人停止后恢复运动，真值仅发评分域；不进入在线算法。

生产函数读取带噪传感器，未读取 scene/fixtures.json。fixtures、Gazebo pose 和脚本只在仿真编排与离线评分出现。C++ PCL/ROS2/Nav2 为上游库调用；风险判定、健康门控、自定义层是本项目实现。OpenCV Zoo nanodet.py 为保留许可的上游类，letterbox/时间匹配/几何互证和关联由本项目实现。

消息协议使用 JSON String，并带 frame、t 和完整字段，便于本阶段审查。不是 ROS 类型检查过的专用目标消息；后续跨项目接口稳定后可迁移 vision_msgs/自定义消息，但必须保持时戳、协方差、分类来源和 ID 生命周期语义。

学习顺序：先运行 lessons/S09 两个例子，再查看生产 risk/tracker；随后阅读 PCL 提取和标定投影；最后对照实际 runtime 命令和 analysis 判据。不要把数学例子当成传感器准确率测试。

已核查原项目 scan_filter.py 的本车过滤/TF 失效逻辑和 scan_quality.py 的质量统计；S09 使用三维点云而非替换雷达成二维扫描。原入口未改动。

## 把 C++ 点云回调逐步展开

下面是生产回调的阅读顺序，不是另一套实现；可运行例子见 lessons。

1. `cloud()` 先比较扫描时间、系统时间、上次扫描时间与邻近 odom。拒绝发生在计算几何之前，失败状态 `max_speed=0`；不能先把一帧陈旧点云画出来再宣称可用。
2. `pcl::fromROSMsg` 读点，剔除非有限值、量程外点和车体包络，先加雷达外参再乘扫描时刻的 odom 变换。
3. `VoxelGrid` 以 4 cm 叶大小合并邻近点，`SACSegmentation` 用带方向限制的地面模型拟合。主地面提供高度参考，不负责直接决定哪些格可走。
4. 以 15 cm 格保存点高程，邻近格组成协方差矩阵，Eigen 最小特征向量给局部法向。平面残差大的格不能按光滑斜坡放行；较高的水平面也可能是台阶。
5. `s09::risk()` 分别带入差速/阿克曼参数；`clear()` 要求连续三次新地面证据。旧占用到期退回未知，必须由新观测才能恢复空闲。
6. `ObservedRiskLayer::updateCosts()` 在 Nav2 真正的主栅格上逐格写入值。若改成只做 max 合并且没有清除，会留下永久障碍；若超时写 0，则会把失效误当安全。当前覆盖旧值并对超时写 UNKNOWN。

读风险代码时可以先在教学例中比较下面两种输入：

```cpp
// 坡度 2°、残差台阶 3.5 cm，其他项较小。
// 加 1.2 cm 裕度后：差速超过 4 cm 候选限值，阿克曼未超过 6 cm。
s09::risk(2., .035, .01, .03, true, diff);
s09::risk(2., .035, .01, .03, true, ack);
```

这只解释阈值与裕度的作用，不意味着阿克曼已通过实体台阶爬越试验。两个车型都受 5° 已验证坡度能力约束，工具/车体 footprint 决定后续二维空间包络。

## 把 Python 融合与跟踪逐步展开

`fusion_node.process()` 对每份几何消息选择匹配图像；左右图必须同戳，图像位姿还必须匹配曝光时刻。`Detector.infer()` 做 RGB letterbox、上游 NanoDet 解码和 NMS，再恢复原图坐标。`project_box()` 将点簇八角从 odom 转回对应时刻的相机；框的 IoU、几何尺寸和双目深度通过后才赋类别。

`unique_semantics()` 约束同一视觉框最多标记一个点簇。没有可用类别的簇仍保留为障碍，视觉失效也不阻塞独立的 terrain → risk → Nav2 支路。

`Tracker.update()` 的关键矩阵含义：

```python
# x = [px, py, vx, vy]；F 把速度推进到位置。
F = np.eye(4)
F[0, 2] = F[1, 3] = dt
x = F @ x
P = F @ P @ F.T + Q
# 位置观测创新用于 NIS 门控，然后做一对一分配。
innovation = z - H @ x
S = H @ P @ H.T + R
nis = innovation @ np.linalg.solve(S, innovation)
# 已关联目标用 Joseph 形式更新，避免简化减法破坏数值正定性。
K = P @ H.T @ np.linalg.inv(S)
I_KH = np.eye(4) - K @ H
P = I_KH @ P @ I_KH.T + K @ R @ K.T
```

以上片段解释生产公式，不可单独运行。真实 `update()` 同时处理尺寸门限、歧义、多目标分配、生命周期和时间倒退。遮挡时协方差增长；超过 1.2 s 后删除旧轨迹，所以重现后的新 ID 不代表识别失败，也不代表识别到了同一个真实身份。

`analysis09.analyze()` 最后读取 domain 120 真值，核对可见低障碍、行人位置、突然运动速度和原始格清除。它从不向在线算法发布真值修正。需要同时阅读 `checks` 和原始指标：示例通过、消息有字段、画面出现框，都不能独自证明避障正确。
