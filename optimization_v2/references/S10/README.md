# S10 参考与来源

本轮没有新下载模型、没有新增系统安装。使用既有 Ubuntu 24.04 / ROS 2 Jazzy / Gazebo Harmonic；具体版本由 `config/s10_dependency_lock.json` 锁定。

- ROS 2 Jazzy 官方 C++ Action 示例：https://github.com/ros2/examples/blob/jazzy/rclcpp/actions/minimal_action_server/not_composable.cpp
- ROS 2 官方 Action 教程：https://github.com/ros2/ros2_documentation/blob/jazzy/source/Tutorials/Intermediate/Writing-an-Action-Server-Client/Cpp.rst
- SQLite 官方事务：https://www.sqlite.org/lang_transaction.html
- 生产代码思想参考：项目内 `field95/measured_ledger.py`、`feedback.py`、`tool_ingress.py`、`protocol.py`；本轮执行 `python -m unittest field95.test_tool_handoff -v`，不沿用未验证的旧报告结论。

新增接口、仿真插件、服务、账本、测试和教学由本项目编写；依赖库保留各自许可证，没有复制商业机械臂/VLA 模型或引入新的外部三维模型。S03 虚拟工具尺寸沿用；工具转子增加 0.05 kg 的独立测量关节，仅用于抽象执行/反馈测试，不代表真实工具动力学认证。
