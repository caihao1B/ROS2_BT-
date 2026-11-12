# S06 核心代码导读

下面行号由当前源码生成。Python 负责采集编排、数据适配、导出评分和呈现；自写 C++ 负责建图生命周期与直接调用 RTAB-Map 库检查真实数据库。RTAB-Map 的注册和 GTSAM 的优化是上游 C++，不冒充本项目自研。

## Python 生产调用链

### scripts/mapping_inputs.py

真实消息同步、观测坐标变换和时序动态过滤。process 保留传感器时间戳，显式拒绝配对误差与预热不足。

- `temporal_filter`：[第 15 行](../../../optimization_v2/ws/src/map_manager/scripts/mapping_inputs.py#L15)。
- `main`：[第 24 行](../../../optimization_v2/ws/src/map_manager/scripts/mapping_inputs.py#L24)。
- `process`：[第 34 行](../../../optimization_v2/ws/src/map_manager/scripts/mapping_inputs.py#L34)。

### scripts/mapping_motion.py

从公开采集航点和估计位姿计算 v/ω；前向测距、局部健康许可和管理器状态共同门控，不使用评分真值。

- `main`：[第 11 行](../../../optimization_v2/ws/src/map_manager/scripts/mapping_motion.py#L11)。
- `graph`：[第 23 行](../../../optimization_v2/ws/src/map_manager/scripts/mapping_motion.py#L23)。
- `scan`：[第 37 行](../../../optimization_v2/ws/src/map_manager/scripts/mapping_motion.py#L37)。
- `send`：[第 43 行](../../../optimization_v2/ws/src/map_manager/scripts/mapping_motion.py#L43)。

### scripts/mapping_runtime.py

复用冻结的 S03/S04/S05 可执行节点，隔离 raw/online/evaluation domain，限定资源、时钟和关闭时间。

- `run_case`：[第 16 行](../../../optimization_v2/ws/src/map_manager/scripts/mapping_runtime.py#L16)。
- `launch`：[第 19 行](../../../optimization_v2/ws/src/map_manager/scripts/mapping_runtime.py#L19)。

### scripts/mapping_scene.py

仅仿真侧派生场景：树干、树冠和纹理；保留 S02 原场景并检查新增树干净空。此函数不被在线控制器调用。

- `compose`：[第 9 行](../../../optimization_v2/ws/src/map_manager/scripts/mapping_scene.py#L9)。

### scripts/mapping_observer.py

实际节点／约束／TF 发布者和生命周期观测，记录数据时戳与接收时仿真钟以区别暂停期间的缓存消息。

- `graph_data`：[第 10 行](../../../optimization_v2/ws/src/map_manager/scripts/mapping_observer.py#L10)。
- `main`：[第 14 行](../../../optimization_v2/ws/src/map_manager/scripts/mapping_observer.py#L14)。
- `info`：[第 22 行](../../../optimization_v2/ws/src/map_manager/scripts/mapping_observer.py#L22)。

### scripts/mapping_export.py

关闭后重开真实 DB，导出扫描点云，校验数据库未被读工具改写，并绑定采集开始时的源码／版本。

- `safe`：[第 10 行](../../../optimization_v2/ws/src/map_manager/scripts/mapping_export.py#L10)。
- `verify_manifest`：[第 16 行](../../../optimization_v2/ws/src/map_manager/scripts/mapping_export.py#L16)。
- `export`：[第 22 行](../../../optimization_v2/ws/src/map_manager/scripts/mapping_export.py#L22)。

### scripts/mapping_analysis.py

唯一允许读取独立评分真值的模块；固定原点评分、同一回访点云对比较、离线动态残留标注和逐项门槛。

- `score_poses`：[第 18 行](../../../optimization_v2/ws/src/map_manager/scripts/mapping_analysis.py#L18)。
- `ghosting`：[第 22 行](../../../optimization_v2/ws/src/map_manager/scripts/mapping_analysis.py#L22)。
- `dynamic_residue`：[第 48 行](../../../optimization_v2/ws/src/map_manager/scripts/mapping_analysis.py#L48)。
- `analyze`：[第 70 行](../../../optimization_v2/ws/src/map_manager/scripts/mapping_analysis.py#L70)。

- `revisit_alignment`：[第 32 行](../../../optimization_v2/ws/src/map_manager/scripts/mapping_analysis.py#L32)。
- `loop_event_comparison`：[第 59 行](../../../optimization_v2/ws/src/map_manager/scripts/mapping_analysis.py#L59)。

`revisit_alignment` 排除保存后短时重复边，使用真实回访附近的稳定点云快照；`loop_event_comparison` 只比较共同历史节点。

### scripts/mapping_report.py

将保存的真实扫描快照投影到可旋转三维视图；用原始和优化姿态形成前后对照，不展示人工制作的假轨迹。

- `visual_data`：[第 4 行](../../../optimization_v2/ws/src/map_manager/scripts/mapping_report.py#L4)。
- `render`：[第 16 行](../../../optimization_v2/ws/src/map_manager/scripts/mapping_report.py#L16)。

### scripts/mapping_stage.py

公开 demo/test/stop、前置检查、完整矩阵、真实状态回执和续跑哈希准入。

- `aggregate`：[第 18 行](../../../optimization_v2/ws/src/map_manager/scripts/mapping_stage.py#L18)。
- `receipt`：[第 26 行](../../../optimization_v2/ws/src/map_manager/scripts/mapping_stage.py#L26)。
- `main`：[第 53 行](../../../optimization_v2/ws/src/map_manager/scripts/mapping_stage.py#L53)。

## 自写 C++ 与上游引擎

### src/mapping_manager.cpp

管理器的服务受理与最终完成分开；WAIT_STOP 检查反馈速度后才调用 pause/backup。RESUMING 期间仍不允许运动；超时或时钟倒退进入 FAULT。

- `void request`：[第 22 行](../../../optimization_v2/ws/src/map_manager/src/mapping_manager.cpp#L22)。
- `void call`：[第 30 行](../../../optimization_v2/ws/src/map_manager/src/mapping_manager.cpp#L30)。
- `void tick`：[第 38 行](../../../optimization_v2/ws/src/map_manager/src/mapping_manager.cpp#L38)。
- `Manager():`：[第 54 行](../../../optimization_v2/ws/src/map_manager/src/mapping_manager.cpp#L54)。

### src/graph_probe.cpp

以只读模式重开实际 DB。getGraph 读取原始／优化节点和信息矩阵；RegistrationIcp 复核实际回环扫描对；另用视觉→ICP 注册检查错误候选，并保留纯 ICP 误接受证据。

- `int main`：[第 12 行](../../../optimization_v2/ws/src/map_manager/src/graph_probe.cpp#L12)。
- `map.getGraph`：[第 21 行](../../../optimization_v2/ws/src/map_manager/src/graph_probe.cpp#L21)。
- `auto test=`：[第 29 行](../../../optimization_v2/ws/src/map_manager/src/graph_probe.cpp#L29)。
- `visual_params=`：[第 51 行](../../../optimization_v2/ws/src/map_manager/src/graph_probe.cpp#L51)。

### 上游 CoreWrapper.cpp

- `CoreWrapper::commonMultiCameraCallbackImpl`：[锁定版本第 1429 行](/home/ceihao/robot_project/optimization_v2/references/S06/CoreWrapper.cpp:1429)。
- `void CoreWrapper::backupDatabaseCallback`：[锁定版本第 3541 行](/home/ceihao/robot_project/optimization_v2/references/S06/CoreWrapper.cpp:3541)。

### 上游 CommonDataSubscriber.cpp

- `void CommonDataSubscriber::setupCallbacks`：[锁定版本第 415 行](/home/ceihao/robot_project/optimization_v2/references/S06/CommonDataSubscriber.cpp:415)。
- `setupStereoCallbacks(`：[锁定版本第 574 行](/home/ceihao/robot_project/optimization_v2/references/S06/CommonDataSubscriber.cpp:574)。
- `setupRGBDCallbacks(`：[锁定版本第 685 行](/home/ceihao/robot_project/optimization_v2/references/S06/CommonDataSubscriber.cpp:685)。

完整注册链：CoreWrapper 同步回调构造 `SensorData` → `rtabmap::Rtabmap::process` → `Memory` 的地点检索／变换验证 → `RegistrationVis` 生成全局候选初值 → `RegistrationIcp` 几何细化 → `OptimizerGTSAM` 对 SE(3) 因子图优化。邻接边细化在有可信局部初值时可直接用 ICP；不是每条边都重新做全局视觉搜索。

接口声明可本地查看 `/opt/ros/jazzy/include/rtabmap-0.23/rtabmap/core/Registration.h`、`RegistrationInfo.h`、`Rtabmap.h` 和 `Parameters.h`。安装版实际启用 GTSAM 和 libpointmatcher；未启用 OpenCV xfeatures2d，因此选择 GFTT/ORB，不依赖 BRIEF/FREAK。

## 关键逻辑的读法

1. 先读 mapping_inputs.process：找到一对有相同时间戳的双目图，再选相邻的雷达／里程计；不符合 40 ms 上限就丢弃。
2. temporal_filter 将当前和旧扫描投到局部 odom，验证跨帧稳定性；它不查 actor ID。
3. mapping_manager.tick 把“请求保存”变成“禁止行驶→确认停车→暂停计算→真实数据库写盘和重开”。
4. graph_probe 说明“函数返回了一个变换”与“可接受的全局回环”有区别；检查 false_candidate 的两个 accepted 字段。
5. mapping_analysis 的真值只影响报告，绝不能导入上述在线模块。

## C++ / Python 教学例

两者实现固定航向的二维加权最小二乘小图，展示回环边如何分配累积误差，并测试负信息权重和断开图。生产实现是三维 SE(3)，不把二维教学函数直接用作机器人 SLAM。

[教学命令和解释](../../../optimization_v2/lessons/S06/README.md) · [C++ 源码](../../../optimization_v2/lessons/S06/lesson.cpp) · [Python 源码](../../../optimization_v2/lessons/S06/lesson.py)
