# S15 核心代码导读

生产调用链：S09 objects → history/IMM 或 Trajectron++ → 异步 ROS forecasts → C++ 风险监督 → S15 Policy → 原 Nav2 和反馈工具动作。所有权出口沿用 S14。

- [history15.py:27](../../../optimization_v2/ws/src/motion_prediction/scripts/history15.py#L27)，`prepare`：只用已收到的历史构造 8 个格点；未来标签不进入此函数。
- [imm15.py:67](../../../optimization_v2/ws/src/motion_prediction/scripts/imm15.py#L67)，`update`：混合四个模式、分别传播、Joseph 更新和似然归一。
- [trajectron15.py:43](../../../optimization_v2/ws/src/motion_prediction/scripts/trajectron15.py#L43)，`predict`：调用官方编码器和真实 25 模态解码器；labels=None。
- [engine15.py:32](../../../optimization_v2/ws/src/motion_prediction/scripts/engine15.py#L32)，`predict`：行人进入学习候选，其他运动目标明确回退 IMM。
- [prediction_node15.py:11](../../../optimization_v2/ws/src/motion_prediction/scripts/prediction_node15.py#L11)，`main`：ROS 观测与隔离推理进程之间的有界异步协议，记录延迟与故障。
- [evaluate15.py:44](../../../optimization_v2/ws/src/motion_prediction/scripts/evaluate15.py#L44)，`evaluate`：只在离线评分读取未来观测，按整段 episode 分组。
- [analysis15.py:75](../../../optimization_v2/ws/src/motion_prediction/scripts/analysis15.py#L75)，`choose_default`：按测试前冻结的门槛选默认模型，证据不足则禁用。
- [runtime15.py:8](../../../optimization_v2/ws/src/motion_prediction/scripts/runtime15.py#L8)，`adapt`：精确匹配 S14/S13 启动源码接入新增节点；不改已验收文件。
- [stage15.py:66](../../../optimization_v2/ws/src/motion_prediction/scripts/stage15.py#L66)，`main`：实际矩阵、检查点、前置校验、报告和回执。

C++ 生产实现：

- [risk15.hpp:16](../../../optimization_v2/ws/src/prediction_risk_guard/include/risk15.hpp#L16)：协方差检查、机器人短时运动包络与目标分支相交；模式质量是代理而非已校准碰撞概率。
- [decision15.hpp:8](../../../optimization_v2/ws/src/prediction_risk_guard/include/decision15.hpp#L8)：组合原 S14 优先级，仅为原 EXECUTE 加入预测等待。
- [guard15.cpp:22](../../../optimization_v2/ws/src/prediction_risk_guard/src/guard15.cpp#L22)：仿真时间、odom 和墙钟的新鲜度检查，发布风险及速度上限。
- [generate15.py:9](../../../optimization_v2/ws/src/prediction_risk_guard/scripts/generate15.py#L9)：构建时生成附加速度门；保留原定位、工具、所有权、时钟、制动逻辑。

任务恢复并没有复制一套新执行器。`prediction_risk_guard/scripts/mission15.py` 对原 `mission14.py` 做精确源码适配：新增 `receive_prediction`、`prediction_context`、预测恢复迟滞和固定两点基准，保留真实账本与幂等重试。因为这些方法是构建/运行时适配产生的，不伪造原文件中的函数行号；具体替换锚点都在该适配文件中，原文变化会拒绝启动。

官方核心：`vendor/Trajectron-plus-plus/trajectron/model/mgcvae.py` 的 `obtain_encoded_tensors`、`p_z_x`、`p_y_xz` 负责真实条件编码、潜变量概率和 GRU 解码；`model/dynamics/single_integrator.py` 将速度分布积分为位置。`compat15.py` 保留上游文件字节不变，运行时应用有哈希和等价测试的最小适配。

Python 不是推理算法的假包装：IMM 有独立的混合和四种转移；神经网络是实际预训练模块。项目自写 C++ 负责风险和 BT 接口，没有虚构一个 Trajectron++ C++ 推理后端。

教学示例见 [README](../../../optimization_v2/lessons/S15/README.md)；合成例与真实 Gazebo 证据分开。当前实测数字、版本和退出码以 [RESULTS.md](RESULTS.md) 为准。
