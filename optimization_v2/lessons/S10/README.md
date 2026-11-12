# S10 两种语言的可执行例子

C++ 例是去掉 ROS 的教学状态机：`request()` 只受理，`cancel()` 立即撤销许可，实际转速连续低于门限才确认停止。生产版的 ROS Action、互斥和异步事务见 `tool_sim/src/action_server.cpp`。

```bash
mkdir -p /home/ceihao/robot_project/optimization_v2/ws/build/tool_sim
c++ -std=c++17 -O2 /home/ceihao/robot_project/optimization_v2/lessons/S10/lesson.cpp -o /home/ceihao/robot_project/optimization_v2/ws/build/tool_sim/lesson_s10
/home/ceihao/robot_project/optimization_v2/ws/build/tool_sim/lesson_s10
```

Python 例直接调用生产账本引擎，使用临时 SQLite 数据库及明确标注的合成观测；不是 Gazebo 性能测试。

```bash
source /home/ceihao/robot_project/optimization_v2/env.sh
/home/ceihao/robot_project/optimization_v2/venvs/scene/bin/python /home/ceihao/robot_project/optimization_v2/lessons/S10/lesson.py
```

两条程序都应输出 PASS。Python 运行后临时数据库自动清除，不写入已有仿真账本。
