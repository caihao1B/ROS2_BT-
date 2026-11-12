# 第 1 步核心代码讲解

以下链接对应当前实际源文件和行号。新逻辑集中在新工作区，原 BT 引擎为有来源记录的本地代码复用，教学示例另列。

- [env.sh](/home/ceihao/robot_project/optimization_v2/env.sh:3)
- [源码与资源审计](../../../optimization_v2/ws/src/project_baseline/scripts/baseline_audit.py#L34)
- [阶段流程](../../../optimization_v2/ws/src/project_baseline/scripts/stage_runner.py#L144)
- [回执生成](../../../optimization_v2/ws/src/project_baseline/scripts/stage_runner.py#L117)
- [停止所有权](../../../optimization_v2/ws/src/project_baseline/scripts/session_owner.py#L16)
- [静态观测](../../../optimization_v2/ws/src/project_baseline/scripts/sensor_smoke.py#L47)
- [C++ BT 引擎](../../../optimization_v2/ws/src/project_baseline/src/bt_engine.cpp#L116)
- [C++ 依赖样例](../../../optimization_v2/ws/src/project_baseline/src/dependency_probe.cpp#L9)
- [C++ ROS 发布器](../../../optimization_v2/ws/src/project_baseline/src/isolation_publisher.cpp#L7)
- [Python ROS 接收器](../../../optimization_v2/ws/src/project_baseline/scripts/isolation_check.py#L9)

## 调用顺序

`stepctl.sh → stepctl.py → stages/S01/run.sh → env.sh → stage_runner.py`。阶段脚本加载新环境后启动 Python 编排器。`Commands.run()` 为每条命令保存 argv、退出码、现实耗时和日志；命令与验收检查分开，因为错误输入返回 2、旧入口返回 64 是预期的拒绝行为。

`accept()` 执行构建、审计、BT 与旧组件检查，再做跨进程通信和互斥，最后分别运行原农田与模型的静态传感器验收。检查器订阅传感器和零速度请求，不发送控制命令。原生子进程单独装载原环境，不能把原 overlay 混进新工作区。

`baseline_audit.audit()` 对比两个源码树，沿 SDF/DAE 中的 URI 递归找到依赖模型和纹理；`file_row()` 先确认路径仍在主项目根目录，再计算 SHA-256。正式 test 才冻结配置，demo 只生成独立观测，避免演示改写基线使已有回执过期。

## C++ 的真实职责

`bt_engine.cpp` 使用 BehaviorTree.CPP，读取一行 JSON tick，验证 schema 与严格递增 sequence，将 snapshot 交给条件和异步动作。树运行后输出模式、原因、命令意图和状态转换；输入结束执行 halt。它只输出动作意图，不直接控制车辆。测试验证故障抢占、等待取消确认、超时、过期序列和修复尚未实现时不能误报成功；完整 ROS 执行器适配在 S14 完成。

`dependency_probe.cpp` 实际链接 F2C 2.1.1 和 METIS：多边形→地头→条带→往复排序；随后将一个简单邻接图转为 CSR 数组并请求三分。节点权重、连通选项和种子显式配置。输出条带数量、长度和分区；工具宽度为零时在调用库前拒绝。这个小例不代表不规则区域覆盖路线已完成。

`isolation_publisher.cpp` 使用 rclcpp 定时发布带 domain 标签的测试消息，10 秒后自行结束；Python 接收器是另一个进程。它们验证新环境的实际 ROS 通信，不涉及车辆命令话题。

## Python 的真实职责

Python 负责环境/文件审计、命令编排、传感器观测、图表、证据回执和进程会话。`session_owner.matches()` 同时验证 UID、PID 创建时间和命令标识，避免只凭 PID 杀进程；`stop_session()` 发送中断后等待会话收尾。

`sensor_smoke.py` 是原检查器的受控迁入版，新增可选仿真时间上限、窗口截图和频率/雷达可视数据；原静止、消息质量、TF 拓扑和唯一发布者判据保留。`write_receipt()` 绑定本轮真实指标、文档与实际源码/模型文件；`stepctl check` 只做一致性核验，不重新运行测试。

## 学习时的入口

先运行 lessons/S01/run.sh，观察“存在文件”和“错误配置”怎样产生不同退出码，再阅读 audit→accept→write_receipt 的调用链。之后运行 BT 核心回归，对照 tree.xml 与测试里的 fault/wait/rejoin snapshot，理解条件为何允许或拒绝动作。

生产实现只有一套，lesson.cpp 与 lesson.py 是明确标记的教学等价例，不承载实际机器人状态，也不替代生产审计与 ROS 控制代码。
