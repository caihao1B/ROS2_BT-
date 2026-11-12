# ROS 2 农田自主作业机器人

在三维仿真场景中，用同一套任务与安全接口运行两种机器人：差速车逐点喷洒，阿克曼车连续割草。两个连续不规则作业区通过曲线道路连接；工作区由算法划分为三个连通簇，再生成点服务或条带路径。

项目重点是把 **传感器 → 定位与感知 → 规划 → 可解释行为树 → 导航控制 → 工具反馈账本** 连成可检验的闭环。在线算法接收带误差的模拟观测；仿真真值仅用于独立评分。喷洒与割草执行器抽象为动作接口，后续可独立接入机械臂、近景视觉或 VLA。

> 开发机正式验收：S14 PASS、S15 PASS。S15 的默认策略为 `no_prediction`；未达收益门槛的预测候选保持禁用。新机器需安装、构建与重新验收。

![场地与当前任务分簇示意](media/project_overview.svg)

<table>
<tr><th>差速点服务底盘</th><th>阿克曼连续覆盖底盘</th></tr>
<tr><td><img src="media/chassis_diff.png" alt="Gazebo 差速底盘实际截图" width="520"></td><td><img src="media/chassis_ackermann.png" alt="Gazebo 阿克曼底盘实际截图" width="520"></td></tr>
</table>

底盘图来自 S03 的真实 Gazebo 验收截图；后续阶段增加传感器、定位和导航接口。上方总览图展示公开先验和算法分簇，不计作实际完成量。

## 系统结构

```mermaid
flowchart TD
  G[Gazebo Harmonic 三维世界与轻量底盘] --> S[带误差的雷达 / 双目 / IMU / 编码器 / GNSS]
  S --> O[连续局部里程计]
  S --> M[三维建图与冻结地图重定位]
  O --> L[全局定位与 GNSS 一致性监督]
  M --> L
  S --> L
  S --> P[地形几何 / 视觉检测 / 目标跟踪]
  P --> C[二维通行代价与独立停车保护]
  P --> F[IMM / Trajectron++ 多模态预测]
  F --> R[时空冲突风险监督]
  R --> T
  R --> V
  B[授权先验与工具几何] --> Q[工作量三分簇 / 点序 / 条带与工时]
  Q --> T[可解释行为树与任务调度]
  L --> T
  C --> T
  T --> N[Nav2 NavigateToPose / FollowPath / 巡检]
  N --> V[唯一底盘速度与转向出口]
  T --> A[ToolTask 抽象工具动作]
  A --> K[实际反馈 / 幂等账本 / 残余]
  K --> T
  V --> G
  G -. 独立评分域 .-> E[误差 / 完成率 / 停车 / 算力报告]
```

## 算法与技术栈

| 模块 | 当前实现 | 选择理由与边界 |
|---|---|---|
| 仿真 | Ubuntu 24.04、ROS 2 Jazzy、Gazebo Harmonic；差速与阿克曼轻量模型 | 同图分别运行，降低算力需求；无真实机械臂 |
| 传感器 | 32×1024 主三维雷达、128×32 近场补盲雷达、640×360 双目、IMU、编码器、GNSS；误差与失效注入 | 定位与避障使用独立处理分支；瞬时扫描不宣称扫描内去畸变 |
| 局部定位 | 轮速/IMU 与激光里程计、robot_localization | 连续 odom；不把全局修正重复当作独立测量融合 |
| 建图 | RTAB-Map、视觉地点检索、点到面 ICP、GTSAM 位姿图 | 保存真实观测数据库；回环须通过几何质量检查 |
| 全局定位 | 冻结图重定位、区域激光地图、GNSS 坐标与杆臂修正、一致性门控 | 唯一 map→odom；版本失配或参考失效时拒绝任务 |
| 三维感知 | PCL 地面与局部平面、坡度/台阶/可观测性、NanoDet 与三维关联 | 三维观测投影为二维通行代价；未知不自动当空闲 |
| 分簇 | METIS 加权图划分、连通修复、有界边界交换；K-means 对照 | 固定三簇，保留边角和孔洞，不保证全局最优 |
| 点服务 | 可见图/Dijkstra 距离、OR-Tools TSP、NN+2-opt 对照 | 按真实运动与停靠工时选择候选；不直接采用欧氏穿障距离 |
| 割草覆盖 | Fields2Cover 条带、Dubins 转场、车体扫掠与曲率验证 | 阿克曼不能原地旋转；曲率切换处停车并确认转角 |
| 控制 | Nav2 DWB、RPP、MPPI；Smac Hybrid-A* | 当前点服务使用 DWB、条带使用 RPP，MPPI 保留对照；计划路径不等于实际作业完成 |
| 动态预测 | IMM CV/CA/左右转弯；固定 ETH 权重 Trajectron++；CPU 隔离推理 | 在线观测历史与邻居，3.2 秒多模态输出；近 2 秒风险只能收紧速度，默认选择由实测决定 |
| 决策 | BehaviorTree.CPP + Python 任务执行器 | reason_code、证据、候选成本、迟滞、取消握手和有界重试 |
| 作业账本 | ROS 2 Action、SQLite、几何并集与残余导出 | 按新鲜且身份匹配的实际工具反馈记账；取消前的已确认作业保留，失联和重复请求不能虚增完成量 |

完整簇当前使用经实际观测测站配准的区域激光地图；RTAB-Map 建图与冻结图重定位基线保留。近场补盲雷达是额外模拟测距通道，并非从主雷达凭空补出的观测。

C++ 负责关键 ROS 节点、几何与控制接口；Python 负责仿真编排、任务调度、离线规划、账本和报告。具体依赖版本由 `optimization_v2/config/*lock*` 记录。

## 任务与验收方式

- 喷洒：到达目标停车点 → 工具准入 → 驻留 → 反馈确认 → 下一点。
- 割草：进入作业带 → 开工具并等待反馈 → FollowPath → 停工具 → 转场；按有效工具足迹并集计面积。
- 巡检：工具关闭，执行连接路导航与检查点记录。
- 每簇正式任务含停车收尾最多 **1800 仿真秒**，离线名义计划目标 **1440 秒**；这不是整套测试的现实耗时上限。
- 当前导航准入速度上限为 **0.12 m/s**，割草命令保留裕量至 **0.11 m/s**；车辆配置中的更高候选速度尚不能据此声明通过完整作业验收。
- 当前固定完整簇验收分母为 A1 的 **21 个点**、B1 的 **16.3594578796 m²**。原大地图仍保留；当前任务规模由工时闭环显式选定，不表示整张农田已完成覆盖。
- 短测标记 `SMOKE`；未完成、超时和安全拒绝保留 `PARTIAL` 与原始残余。

## 已完成的真实验收

S14 的冻结地图、GNSS 区域配准、规划版本和完整簇已复验。以下为固定任务分母的结果，包含返回与停车收尾：

| 完整簇 | 实际结果 | 仿真用时 | 独立定位误差 P95 |
|---|---|---|---|
| A1 差速点服务 | 21/21 点，返回完成 | 1283.063 s（21.38 分钟） | 0.0114 m |
| B1 阿克曼割草 | 5 条作业带与全部 11 个子任务完成；16.2763 / 16.3595 m²，覆盖率 99.4915% | 1531.663 s（25.53 分钟） | 0.00636 m |

![A1 实际轨迹和定位误差](media/s14_spray_actual.png)
![B1 实际轨迹和定位误差](media/s14_mow_actual.png)

故障验收包括定位参考失效、低障碍、预算耗尽、持续封堵与工具失联。预期拒绝、保存残余和安全停车也是故障实验结果；不计作任务完成。`decision_story` 的故障演示在完成 1/2 点后因重试预算耗尽结束，未将它改写为全部成功。

恢复资产后，打开 [S14 交互回放](validation/s14/index.html) 可查看实际 BT 理由、轨迹、反馈账本和残余。GitHub 文件页不运行 HTML，请在本地浏览器打开。Gazebo 在确认停车、工具关闭和暂停后的清理阶段仍可能返回 −11；清理退出码与任务结果分别记录。

## 动态预测的真实对照与演示

S15 完成了 IMM（匀速、加速、左右转弯）和官方固定权重 Trajectron++ 的实际推理适配。输入只有在线观测历史和邻居，输出未来 3.2 秒的多模态轨迹；未来标签仅用于离线评分。

数据集按整段 episode 划分：训练诊断、验证、测试各 4 段，分别包含转弯、停走、脚本让行和遮挡。网络未重新训练；这些相近场景变体不能作为开放世界泛化证明。

| 测试方法 | 加权 ADE / FDE（米） | 95% 矩椭圆覆盖率 | 离线推理 P95（毫秒） |
|---|---|---|---|
| 因果 CV 评分参考 | 0.083 / 0.157 | 100.0% | 0.14 |
| IMM | 0.132 / 0.268 | 100.0% | 1.65 |
| Trajectron++ | 0.563 / 1.117 | 69.9% | 6.81 |

每种方法在同一批 **128 个测试窗口**上评分。CV 只用于离线误差参考；闭环的 `no_prediction` 不向控制器发送未来预测。表中推理延迟不等于完整 ROS 链路延迟，后者见交互报告。

| 闭环方法 | 实际完成 | 障碍物碰撞 | 平均任务时间（仿真秒，含停车） |
|---|---|---|---|
| `no_prediction` | 4/4 | 0 个案例 | 53.73 |
| `imm` | 3/4 | 0 个案例 | 64.92 |
| `learned` | 4/4 | 0 个案例 | 63.39 |

正式矩阵包含 **12 个正常闭环对照 + 3 个故障实验**，固定相同的场景与两点任务。未完成任务的耗时仍包含在均值中，不能把它看作成功任务效率。超时、真实进程失联和大协方差故障均验证回退到当前观测安全基线。

**本轮默认策略：`no_prediction`。** 这是预注册门槛根据真实结果作出的选择；阶段实现验收通过，不等于预测模型有性能提升。候选状态如下：

- `imm`：disabled；原因：`NEW_FAILED_TASK, INCOMPLETE_PAIRED_SUCCESS_EVIDENCE, INSUFFICIENT_MEASURED_CONFLICT_EXPOSURE`。
- `learned`：disabled；原因：`NO_EFFICIENCY_QUALIFICATION, CALIBRATION_NOT_QUALIFIED, INSUFFICIENT_MEASURED_CONFLICT_EXPOSURE, NO_LEARNED_ACCURACY_GAIN`。

风险量是模式包络相交的代理，不是已校准的碰撞概率。实测冲突分母不足时记 N/A，不能声明零漏报。两点差速基准不替代完整六簇，也不替代阿克曼动态条带中断验收。

![实际观测和学习模型未来分支回放](media/s15_prediction_actual.gif)

动画来自冻结测试观测和真实模型输出，是离线窗口回放；深色为历史，紫色为未来分支，橙色为仅在评分时使用的未来观测。坐标以当前观测为原点，显示范围 ±3 米。模型预测不准的情况原样保留。

![真实 RViz 预测图层](media/s15_rviz_actual.png)

实际 RViz 演示显示点云、跟踪目标、未来分支和概率。显示参考系的单独修正及复验证明见 [UI_VALIDATION](docs/optimization_v2/S15/UI_VALIDATION.md)；原 15 项矩阵保留原始版本记录。

![仿真相机真实检测画面](media/s15_camera_actual.jpg)

相机图来自实际 S15 采集；刚体行人网格没有关节步态动画，右侧树木存在误检，未为展示目的删除。当前感知与预训练模型均仍有域差异。

恢复资产后打开 [S15 交互报告](validation/s15/index.html)，可切换场景、模型和观测时间，检查概率、风险、BT 理由、任务完成和算力。模型来源及许可见 [模型锁](optimization_v2/config/model_lock.yaml)；媒体来源与哈希见 [MANIFEST](media/MANIFEST.json)。

## 项目目录

```text
optimization_v2/
  ws/src/          ROS 2 包、算法与真实测试
  config/          传感器、车辆、定位、规划、行为与依赖锁
  datasets/        按整段场景拆分的预测观测数据集
  maps/            授权先验、冻结地图与区域地图
  plans/           三分簇、离线路径及版本绑定
  behavior_trees/  任务行为树
  contracts/       数据、动作与理由码合同
  stages/          每步运行入口
  lessons/         C++ / Python 教学示例
  stepctl.sh       list / status / demo / test / check / open / stop
 docs/optimization_v2/
  prompts/         S01–S18 分步执行要求
  SNN/             原理、核心代码、手工验收和实际结果
```

## 运行入口

先阅读 [构建与资产恢复](BUILDING.md)。以下接口已经在开发机使用；新机器需安装依赖、恢复资产、构建并重新验收，不能复制开发机 PASS 回执。独立场景预览入口可在没有开发机回执时查看实际世界和静止底盘。

```bash
bash optimization_v2/stepctl.sh list
bash optimization_v2/stepctl.sh status
bash optimization_v2/stepctl.sh check S14
bash optimization_v2/stepctl.sh open S14
```

`test` 执行真实测试；`check` 只校验现有回执、源文件和输入哈希；`demo` 不自动替代正式验收。仿真实验串行运行，`stop` 仅结束本步骤拥有的进程。

## 学习与扩展

已实施阶段提供 `TECHNICAL.md`、`CODE_WALKTHROUGH.md`、`MANUAL_CHECK.md`、`RESULTS.md`，以及可实际编译/运行的 C++ 和 Python 示例。阶段执行要求见 [18 步命令合集](docs/optimization_v2/命令Prompt合集.md)。

学习入口：[S14 技术原理](docs/optimization_v2/S14/TECHNICAL.md)、[S14 核心代码](docs/optimization_v2/S14/CODE_WALKTHROUGH.md)、[S15 技术原理](docs/optimization_v2/S15/TECHNICAL.md)、[S15 核心代码](docs/optimization_v2/S15/CODE_WALKTHROUGH.md)、[S15 手工查验](docs/optimization_v2/S15/MANUAL_CHECK.md)。

动态多模态预测由 S15 管理；经验缓存、六簇综合验收和完整移植交付分别属于尚未实施的 S16、S17、S18。项目面向仿真研究和工程学习，未作实车商业认证。
