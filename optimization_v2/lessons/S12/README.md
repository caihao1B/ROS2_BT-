# S12 两种语言核心例

C++ `lesson.cpp` 实际调用 Fields2Cover 的地头、条带、连续曲率连接，并检查零割幅输入；输出曲线存在不代表碰撞可行。生产版 `planner_backend.cpp` 还处理非凸/孔洞、双类连接和实际 OR-Tools。

Python `lesson.py` 使用生产版可见图、Dijkstra、NN+2-opt、C++ OR-Tools。中间有障碍，两个任务点距离 14 m，但可行连接约 18.114 m；不允许用直线代价。任务点贴边不可达会被拒绝。这是明确的合成教学场景，没有生成机器人作业记账。

```bash
bash /home/ceihao/robot_project/optimization_v2/scripts/build_s12.sh
source /home/ceihao/robot_project/optimization_v2/env.sh
/home/ceihao/robot_project/optimization_v2/ws/build/coverage_planner/planner_lesson_s12
/home/ceihao/robot_project/optimization_v2/venvs/scene/bin/python /home/ceihao/robot_project/optimization_v2/lessons/S12/lesson.py
```

预期 C++：11 条条带、14.6507 m 连续曲率回转、PASS；Python：18.114 m 绕障连接，NN+2-opt 与实际 OR-Tools 返回合法闭合访问次序。二者都不访问旧虚拟机环境，不需要新安装 Python OR-Tools。

实际任务调用链、精确行号见 [CODE_WALKTHROUGH](../../../docs/optimization_v2/S12/CODE_WALKTHROUGH.md)，验证数据见 [RESULTS](../../../docs/optimization_v2/S12/RESULTS.md)。
