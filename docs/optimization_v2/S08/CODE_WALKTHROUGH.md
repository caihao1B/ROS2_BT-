# S08 核心代码阅读顺序

统一入口是 [stage.py](../../../optimization_v2/ws/src/gnss_supervisor/scripts/stage.py#L1)，负责前置检查、会话锁、组件测试、逐例运行、图表及回执。runtime.py 是 S07 启动编排的隔离派生，复用已有传感器、局部 EKF、RTAB、质量节点；没有复制一套估计算法。S08 会话启动新管理器，不启动 S07 管理器。

| 位置 | 函数及学习重点 |
|---|---|
| [alignment.py](../../../optimization_v2/ws/src/gnss_supervisor/scripts/alignment.py#L5) | `solve`：控制点去中心化、SVD、反射修正、残差与退化检查 |
| [alignment.py](../../../optimization_v2/ws/src/gnss_supervisor/scripts/alignment.py#L14) | `convert`：WGS84→ENU→map；NavSatFix 当地 ENU 协方差旋转，加入配准预算 |
| [source_adapter.py](../../../optimization_v2/ws/src/gnss_supervisor/scripts/source_adapter.py#L18) | `match`：按原时戳配对质量标签与 fix；固定种子故障时序在独立配置中；错误 fixed 仍对外报 fixed，防止算法偷看故障名称 |
| [source_adapter.py](../../../optimization_v2/ws/src/gnss_supervisor/scripts/source_adapter.py#L34) | `contract`：将 S07 地图合同与配准文件不可变条件合并 |
| [gnss_supervisor.cpp](../../../optimization_v2/ws/src/gnss_supervisor/src/gnss_supervisor.cpp#L28) | `lookup`、`fresh`、`jump`：时间匹配、新鲜度和原跳变门限 |
| [gnss_supervisor.cpp](../../../optimization_v2/ws/src/gnss_supervisor/src/gnss_supervisor.cpp#L31) | `mapEvidence`：地图原始位姿转 correction，拒绝大跳变，初始三帧/1.5s一致性与 GNSS 验证 |
| [gnss_supervisor.cpp](../../../optimization_v2/ws/src/gnss_supervisor/src/gnss_supervisor.cpp#L42) | `gpsEvidence`：协方差正定、杆臂补偿、二维创新距离与 NIS 双门控，不信任 fixed 标签本身 |
| [gnss_supervisor.cpp](../../../optimization_v2/ws/src/gnss_supervisor/src/gnss_supervisor.cpp#L52) | `tick`：20Hz 现实定时器同时监测仿真时钟，选择主参考、停车迟滞、唯一 TF 与解释状态 |
| [gnss_supervisor.cpp](../../../optimization_v2/ws/src/gnss_supervisor/src/gnss_supervisor.cpp#L77) | 构造函数接线与诊断锁止确认服务；不允许绕过实际静止与重验证条件 |
| [analysis08.py](../../../optimization_v2/ws/src/gnss_supervisor/scripts/analysis08.py#L6) | `analyze`：只在离线读取 domain120 评分文件；位置/航向 P95、时间加权准入、从故障发生起的物理停车延迟 |
| [report08.py](../../../optimization_v2/ws/src/gnss_supervisor/scripts/report08.py#L3) | `render`：把实际观测/状态/误差嵌入离线 HTML；同一时间滑块，真值仅评估开关 |

上游 C++ 的职责：robot_localization 负责 S05 局部融合；RTAB-Map 负责只读冻结图匹配；本项目 C++ 只做全局选择/质量监督/唯一 TF。Python 生产代码处理坐标转换、测量边界、运行编排、评分和可视化，不是简化版 EKF。

阅读时重点看三个量：`map_t_` 是最近合格地图观测原时刻；`heading_t_` 表示绝对航向最后被验证的时刻；`gps_t_` 是最近合格 GNSS 原时刻。它们不同，不能用一个“当前定位时间”掩盖过期航向。`correction_` 是实际采用的变换，异常观测保存在日志但不应用；`hold_` 使未来真实计划必须重新准入。

教学代码及已执行命令见 [lessons README](../../../optimization_v2/lessons/S08/README.md)。生产逻辑测试见 [supervisor.py](../../../optimization_v2/ws/src/gnss_supervisor/tests/supervisor.py#L1)，在 domain123 启动真正的 C++ 可执行文件，测试19项边界，包括初始冲突、负协方差、过期观测、高度/航向跳变、无观测、false fixed、float、GNSS 接管、航向失效、双失效、恢复、跳变、合同撤销、暂停及倒退。坐标样例见 [coordinates.py](../../../optimization_v2/ws/src/gnss_supervisor/tests/coordinates.py#L1)。合成输入组件测试与真实 Gazebo 精度测试分别记录。

生产逻辑可按下面的顺序理解（伪代码，真实实现以上方 C++ 为准）：

```text
校验地图/配准版本与时钟 → 取得同时间局部里程计
地图：计算候选 map→odom → 跳变或冲突则拒绝/锁止
GNSS：转换坐标 → 扣杆臂 → 查协方差 → 检查距离与 NIS
选源：地图+一致GNSS / 仅地图降速 / GNSS短时接管 / 停车
恢复：连续证据 + 时间跨度 + 实际静止 + 必需的计划重验证
最后：唯一 TF + 带理由状态；状态许可之外还要经过底盘保护
```
