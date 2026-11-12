# S14 可运行教学例

这些例子检查生产代码的决策门限、工具联锁和账本语义；不冒充 Gazebo 整簇验收。

```bash
cd /home/ceihao/robot_project
source optimization_v2/env.sh
mkdir -p /tmp/s14_lesson
optimization_v2/venvs/scene/bin/python -c 'import yaml,json; json.dump(yaml.safe_load(open("optimization_v2/config/decision_policy.yaml")),open("/tmp/s14_lesson/policy.json","w"))'
c++ -std=c++17 -O2 -Ioptimization_v2/ws/src/mission_bt/include optimization_v2/lessons/S14/lesson.cpp -o /tmp/s14_lesson/lesson_cpp
/tmp/s14_lesson/lesson_cpp /tmp/s14_lesson/policy.json
optimization_v2/venvs/scene/bin/python optimization_v2/lessons/S14/lesson.py
```

C++ 输出各决策的证据、候选成本、门限以及 `POLICY_AND_TOOL_INTERLOCK_PASS`。它直接使用 `decision.hpp` 与 `tool_permit.hpp`，涵盖等待、绕行、重规划、延后、补扫、失联、预算和工具错误 key。

Python 输出 A1、A2 的剩余义务和 `ACCOUNTING_AND_DEFERRAL_PASS`。重复成功只算一次；不确定反馈不能完成任务；改派 A2 后 A1 原始分母和未完成点仍保留。

实际 BT 的异步取消握手测试：

```bash
optimization_v2/venvs/scene/bin/python optimization_v2/ws/src/mission_bt/tests/async_protocol.py /tmp/s14_lesson/async_protocol
```

它启动真实 BehaviorTree.CPP 进程，以测试时钟和黑板输入驱动：执行中出现危险先输出 CANCEL；收到执行器 idle 确认后才输出下一指令。这是协议测试，不包含机器人动力学。正式 `test S14` 会另外运行真实 Gazebo。
