# S16 核心代码导读

生产入口是 stage16 → runtime16 → node16 → worker16 → MemoryEngine。Python 管 SQLite 与实验，C++ admission 通过 ctypes 在生产检索路径执行；原 S15 C++ 风险与 S14 BT/底盘安全链继续处理当前预测。不存在第二个速度出口。

- [common16.py:7](../../../optimization_v2/ws/src/experience_memory/scripts/common16.py#L7) `config`：见函数实际实现；辅助版本绑定、进程隔离或报告/检查点逻辑。大函数内部状态见同文件。
- [common16.py:8](../../../optimization_v2/ws/src/experience_memory/scripts/common16.py#L8) `versions`：见函数实际实现；辅助版本绑定、进程隔离或报告/检查点逻辑。大函数内部状态见同文件。
- [engine16.py:7](../../../optimization_v2/ws/src/experience_memory/scripts/engine16.py#L7) `MemoryEngine`：每次更新真实观测；full/event/cache 只改变预测器选择；命中也重新计算当前 IMM。
- [evaluate16.py:13](../../../optimization_v2/ws/src/experience_memory/scripts/evaluate16.py#L13) `load_episode`：只把观测时刻之前的定位与里程计上下文配入，未来标签留在离线评分器。
- [evaluate16.py:30](../../../optimization_v2/ws/src/experience_memory/scripts/evaluate16.py#L30) `prepare`：只用 train 场景资格评分建立冻结 SQLite。
- [evaluate16.py:53](../../../optimization_v2/ws/src/experience_memory/scripts/evaluate16.py#L53) `replay`：三次循环次序、同输入同库同事件；计入特征/查询/维护和推理，分别计精度。
- [features16.py:6](../../../optimization_v2/ws/src/experience_memory/scripts/features16.py#L6) `Admission`：装载真实 C++ 共享库；返回拒绝理由，不输出车辆命令。
- [features16.py:14](../../../optimization_v2/ws/src/experience_memory/scripts/features16.py#L14) `context`：17 维情境，不存绝对位置/永久目标身份；实时净距与质量单独门控。
- [features16.py:28](../../../optimization_v2/ws/src/experience_memory/scripts/features16.py#L28) `Trigger`：新历史、周期、运动变化和当前风险的统一事件规则。
- [report16.py:4](../../../optimization_v2/ws/src/experience_memory/scripts/report16.py#L4) `render`：见函数实际实现；辅助版本绑定、进程隔离或报告/检查点逻辑。大函数内部状态见同文件。
- [runtime16.py:7](../../../optimization_v2/ws/src/experience_memory/scripts/runtime16.py#L7) `adapt`：见函数实际实现；辅助版本绑定、进程隔离或报告/检查点逻辑。大函数内部状态见同文件。
- [runtime16.py:10](../../../optimization_v2/ws/src/experience_memory/scripts/runtime16.py#L10) `run_case`：严格适配前序运行器，保留所有权、停车、工具关闭和原风险门限。
- [stage16.py:17](../../../optimization_v2/ws/src/experience_memory/scripts/stage16.py#L17) `sources`：见函数实际实现；辅助版本绑定、进程隔离或报告/检查点逻辑。大函数内部状态见同文件。
- [stage16.py:19](../../../optimization_v2/ws/src/experience_memory/scripts/stage16.py#L19) `measure`：读取真实 ROS/Gazebo 日志，区分完成、安全、算力和注入拒绝。
- [stage16.py:25](../../../optimization_v2/ws/src/experience_memory/scripts/stage16.py#L25) `checkpoint`：见函数实际实现；辅助版本绑定、进程隔离或报告/检查点逻辑。大函数内部状态见同文件。
- [stage16.py:29](../../../optimization_v2/ws/src/experience_memory/scripts/stage16.py#L29) `reuse`：见函数实际实现；辅助版本绑定、进程隔离或报告/检查点逻辑。大函数内部状态见同文件。
- [stage16.py:43](../../../optimization_v2/ws/src/experience_memory/scripts/stage16.py#L43) `choose`：净收益、准确性、风险暴露和上游资格联合决定关闭/候选；不能凭少调用启用。
- [stage16.py:62](../../../optimization_v2/ws/src/experience_memory/scripts/stage16.py#L62) `receipt`：真实证据、源码/输入哈希、前置回执和四项检查；人工保持 NOT_CHECKED。
- [stage16.py:74](../../../optimization_v2/ws/src/experience_memory/scripts/stage16.py#L74) `main`：有界串行运行、超时/中断、正式回执和演示记录分开。
- [store16.py:6](../../../optimization_v2/ws/src/experience_memory/scripts/store16.py#L6) `Store`：事务、有界原型/审计、TTL、版本、方向、衰减和查询超时；失败回退。
- [worker16.py:7](../../../optimization_v2/ws/src/experience_memory/scripts/worker16.py#L7) `UnavailableStore`：见函数实际实现；辅助版本绑定、进程隔离或报告/检查点逻辑。大函数内部状态见同文件。

C++：[admission.hpp](../../../optimization_v2/ws/src/experience_memory/include/admission.hpp#L8) 的 admit 是生产准入；[admission.cpp](../../../optimization_v2/ws/src/experience_memory/src/admission.cpp#L3) 导出 C ABI。不能把教学文件当另一个导航控制器。

Python 事务 `Store.lookup` 检索后只更新使用计数/时间，不在线学习未来标签；`Store.put` 仅由离线训练原型构建或有界测试调用。维护开销包含同步提交，真实结果可能否定缓存节省假设。

教学例见 [README](../../../optimization_v2/lessons/S16/README.md)，两种语言均有正常与边界断言。S15 来源文件不复制修改；严格文本适配要求唯一匹配，前序接口改变会立即失败而非静默覆盖。

时间对齐回归：[features16.py:44](../../../optimization_v2/ws/src/experience_memory/scripts/features16.py#L44) `CausalContext.at(t)` 从有界缓冲选择时间戳不晚于目标观测的定位/里程计消息，避免把最新回调误当作同刻数据。节点适配使用该函数，缺少合格样本时拒绝缓存。运输 worker 对一个批次中的每个观测帧实际调用预测引擎，仅把最后当前帧的预测交给风险层；计数与耗时累加所有处理帧。
