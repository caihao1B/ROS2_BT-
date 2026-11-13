# S17 可运行教学例

在普通终端逐条执行；两份教学例使用合成边界输入，不能替代正式矩阵。C++ 直接调用本步独立审计器的生产头文件，Python 调用本步统计函数。

```bash
bash /home/ceihao/robot_project/optimization_v2/scripts/build_s17.sh
c++ -std=c++20 -O2 -UNDEBUG -I/usr/include/eigen3 -I/home/ceihao/robot_project/optimization_v2/ws/src/acceptance_runner/include /home/ceihao/robot_project/optimization_v2/lessons/S17/lesson.cpp -o /home/ceihao/robot_project/optimization_v2/build_s17/lesson_cpp
/home/ceihao/robot_project/optimization_v2/build_s17/lesson_cpp
/home/ceihao/robot_project/optimization_v2/venvs/scene/bin/python /home/ceihao/robot_project/optimization_v2/lessons/S17/lesson.py
```

C++ 演示时钟倒退、超时、障碍接触、工具停止缺失和空证据如何拒绝通过。Python 演示小样本区间与配对约束：不同任务版本不能拿来比较算力。真实碰撞来自独立域 120 的 Gazebo 接触消息；这里没有把规划碰撞检查当作实测零碰撞。

C++ 还演示本轮发现的坐标原点问题：不能直接用 `map→odom` 的平移变化判断车体发生了全局跳变。应把同一时刻的车体里程计位姿分别代入新旧变换，再比较车体位置；原 0.15 m / 0.035 rad 门限不变。

Python 的后三项检查演示接收顺序与采样时间的区别：新到达的健康消息不能证明旧栅格有效；同时间戳的无效健康消息也不能被另一条有效消息替代；超过 0.45 秒的配对不再使用。这里调用生产 `TerrainJoin`，输入是明确标注的教学合成数据。
