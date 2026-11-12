# S02 核心代码导读

真实调用链：`stepctl.sh → stages/S02/run.sh → stage_runner.py → generate_scene.py / runtime.py → Gazebo + 两个独立 ROS 域 → validate_scene.py → report.py → latest.json`。

## 生产实现索引

| 函数 | 文件与本版行号 |
|---|---|
| `geometries` | [scripts/generate_scene.py:18](../../../optimization_v2/ws/src/scenario_world/scripts/generate_scene.py#L18) |
| `write_mesh` | [scripts/generate_scene.py:34](../../../optimization_v2/ws/src/scenario_world/scripts/generate_scene.py#L34) |
| `mesh_polygon` | [scripts/generate_scene.py:42](../../../optimization_v2/ws/src/scenario_world/scripts/generate_scene.py#L42) |
| `part` | [scripts/generate_scene.py:58](../../../optimization_v2/ws/src/scenario_world/scripts/generate_scene.py#L58) |
| `generate` | [scripts/generate_scene.py:74](../../../optimization_v2/ws/src/scenario_world/scripts/generate_scene.py#L74) |
| `void Configure` | [src/ScriptedMotion.cc:25](../../../optimization_v2/ws/src/scenario_world/src/ScriptedMotion.cc#L25) |
| `void PreUpdate` | [src/ScriptedMotion.cc:38](../../../optimization_v2/ws/src/scenario_world/src/ScriptedMotion.cc#L38) |
| `void PostUpdate` | [src/ScriptedMotion.cc:58](../../../optimization_v2/ws/src/scenario_world/src/ScriptedMotion.cc#L58) |
| `validate_config` | [scripts/online_probe.py:15](../../../optimization_v2/ws/src/scenario_world/scripts/online_probe.py#L15) |
| `main` | [scripts/online_probe.py:19](../../../optimization_v2/ws/src/scenario_world/scripts/online_probe.py#L19) |
| `def points` | [scripts/online_probe.py:34](../../../optimization_v2/ws/src/scenario_world/scripts/online_probe.py#L34) |
| `def camera` | [scripts/online_probe.py:47](../../../optimization_v2/ws/src/scenario_world/scripts/online_probe.py#L47) |
| `main` | [scripts/evaluation_probe.py:9](../../../optimization_v2/ws/src/scenario_world/scripts/evaluation_probe.py#L9) |
| `def poses` | [scripts/evaluation_probe.py:18](../../../optimization_v2/ws/src/scenario_world/scripts/evaluation_probe.py#L18) |
| `def contact` | [scripts/evaluation_probe.py:22](../../../optimization_v2/ws/src/scenario_world/scripts/evaluation_probe.py#L22) |
| `offline` | [scripts/validate_scene.py:37](../../../optimization_v2/ws/src/scenario_world/scripts/validate_scene.py#L37) |
| `observed` | [scripts/validate_scene.py:91](../../../optimization_v2/ws/src/scenario_world/scripts/validate_scene.py#L91) |
| `reproducibility` | [scripts/validate_scene.py:122](../../../optimization_v2/ws/src/scenario_world/scripts/validate_scene.py#L122) |
| `run_case` | [scripts/runtime.py:26](../../../optimization_v2/ws/src/scenario_world/scripts/runtime.py#L26) |
| `screenshot` | [scripts/runtime.py:14](../../../optimization_v2/ws/src/scenario_world/scripts/runtime.py#L14) |
| `main` | [scripts/stage_runner.py:37](../../../optimization_v2/ws/src/scenario_world/scripts/stage_runner.py#L37) |
| `write_receipt` | [scripts/stage_runner.py:27](../../../optimization_v2/ws/src/scenario_world/scripts/stage_runner.py#L27) |

## Python 数据流

`geometries` 生成连续作业区和 Bézier 道路；`generate` 先写公共先验，再初始化隐藏物体随机数，保证随机种子变化不污染先验。`mesh_polygon` 约束三角化保留孔洞；`write_mesh` 计算法向，避免 DART 导入不完整网格。`part` 用同一几何/局部位姿生成 visual 与 collision，几何验收再反向读取 SDF 对比，避免只验证生成器内部变量。

`runtime.run_case` 启动暂停的世界、两个白名单桥和观测器，等订阅就绪后通过 world control 解暂停。在线域只桥接 clock、PointCloud2、Image；评估域桥接位姿和接触。两边各按仿真钟停止，同时有现实 watchdog。退出时只发信号给自己创建的进程。

`online_probe` 仅读公共地图与探针标定。回波用已知固定外参转成世界坐标，保存压缩快照；RGB 按实际消息编码保存。它不读取精确障碍物位置、不对物体分类。`evaluation_probe` 是另一进程，读取私有 manifest 和实测真值；最后的 `observed` 离线比较雷达命中、相机 ROI、实际轨迹和接触对。不能把 `camera_evaluated.png` 中的红框解释为真实检测模型输出。

`offline` 反向读取 GeoJSON、SDF 与 OBJ，检查面积、连通、模型坐标、资源和碰撞体。错误输入必须触发拒绝，否则测试失败。`reproducibility` 对两次实际轨迹按公共仿真时刻插值，比较各物体三维位置，容差 0.03 m；不是把两次解析轨迹公式相等当成实测重复性。

## C++ 数据流

`Configure` 读取每个动态物体的中心、振幅、周期和相位，仅发布评估用位置。`PreUpdate` 读取 Gazebo 实际 link 世界位姿，计算周期轨迹及其解析速度，再做比例位置反馈和限速。`SetLinearVelocity` 接受 link 坐标，所以先对世界速度做逆旋转。`PostUpdate` 以约 10 Hz 发布实际世界位置，而非理想目标；被动碰撞夹具也记录实际位置。

物理求解、传感器渲染和消息桥分别由上游 Gazebo Physics/Sensors 和 ros_gz_bridge 完成；本项目没有自行实现刚体碰撞求解器或相机光栅化。C++ 理想速度执行器不适合直接用于真实机械臂/底盘。

## 审查和修改顺序

先读 scenes.yaml 的单位、原点和初始参数，再看公共 GeoJSON；改几何后必须重生成并复验 S02，旧哈希回执会失效。未来在线算法只能依赖公共文件和传感器接口。S03 加入车辆时不要把评估真值桥合并到在线域，也不要把固定探针当成车载传感器。教学例见 lessons/S02/README.md，二者均有正常与错误输入并实际运行。
