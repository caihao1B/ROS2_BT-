# S18 核心代码导读

Python 负责交付编排、审计、存储与跟随提案；实际控制仍走原 C++/Nav2 唯一出口。本包没有新增第二套底盘指令发布者。

## release18

- [current](/home/ceihao/robot_project/optimization_v2/ws/src/release_tools/scripts/release18.py:5)：实际入口位置。
- [doctor](/home/ceihao/robot_project/optimization_v2/ws/src/release_tools/scripts/release18.py:9)：实际入口位置。
- [spec_for](/home/ceihao/robot_project/optimization_v2/ws/src/release_tools/scripts/release18.py:21)：实际入口位置。
- [verified_restart](/home/ceihao/robot_project/optimization_v2/ws/src/release_tools/scripts/release18.py:32)：实际入口位置。
- [launch](/home/ceihao/robot_project/optimization_v2/ws/src/release_tools/scripts/release18.py:50)：实际入口位置。
- [main](/home/ceihao/robot_project/optimization_v2/ws/src/release_tools/scripts/release18.py:106)：实际入口位置。

## audit18

- [receipt_inventory](/home/ceihao/robot_project/optimization_v2/ws/src/release_tools/scripts/audit18.py:5)：实际入口位置。
- [learning](/home/ceihao/robot_project/optimization_v2/ws/src/release_tools/scripts/audit18.py:27)：实际入口位置。
- [source_paths](/home/ceihao/robot_project/optimization_v2/ws/src/release_tools/scripts/audit18.py:56)：实际入口位置。
- [sizes](/home/ceihao/robot_project/optimization_v2/ws/src/release_tools/scripts/audit18.py:70)：实际入口位置。
- [audit](/home/ceihao/robot_project/optimization_v2/ws/src/release_tools/scripts/audit18.py:82)：实际入口位置。

## bounded_store18

- [BoundedStore](/home/ceihao/robot_project/optimization_v2/ws/src/release_tools/scripts/bounded_store18.py:5)：实际入口位置。

## follow18

- [LeaderFollower](/home/ceihao/robot_project/optimization_v2/ws/src/release_tools/scripts/follow18.py:10)：实际入口位置。

## visual18

- [physical_signature](/home/ceihao/robot_project/optimization_v2/ws/src/release_tools/scripts/visual18.py:13)：实际入口位置。
- [material](/home/ceihao/robot_project/optimization_v2/ws/src/release_tools/scripts/visual18.py:21)：实际入口位置。
- [visual_box](/home/ceihao/robot_project/optimization_v2/ws/src/release_tools/scripts/visual18.py:30)：实际入口位置。
- [assets](/home/ceihao/robot_project/optimization_v2/ws/src/release_tools/scripts/visual18.py:35)：实际入口位置。
- [uv_mesh](/home/ceihao/robot_project/optimization_v2/ws/src/release_tools/scripts/visual18.py:58)：实际入口位置。
- [apply](/home/ceihao/robot_project/optimization_v2/ws/src/release_tools/scripts/visual18.py:71)：实际入口位置。

## preview18

- [capture](/home/ceihao/robot_project/optimization_v2/ws/src/release_tools/scripts/preview18.py:7)：实际入口位置。
- [preview](/home/ceihao/robot_project/optimization_v2/ws/src/release_tools/scripts/preview18.py:29)：实际入口位置。

## stage18

- [main](/home/ceihao/robot_project/optimization_v2/ws/src/release_tools/scripts/stage18.py:4)：实际入口位置。

实际 C++ 行为树的预算决定在 [decision.hpp](/home/ceihao/robot_project/optimization_v2/ws/src/mission_bt/include/decision.hpp:29)：剩余时间不够下一动作与停车预留时，输出 STOP/BUDGET_STOP。新的统一入口复用该逻辑。

跟随核心先筛选同向、有效、低不确定度目标，再保持 ID，以车体净距计算限速提案；停止门优先于跟随。真实 ROS/BT 接入尚未实现。`SpeedLimit=0` 在 Nav2 中表示取消限制，后续必须用原停止通道处理停车请求。

Python 教程实际调用受保护有界存储；C++ 教程用三种证据组合说明“进程执行成功”和“项目验收通过”的区别。教程不代替实际验收。
