# S11 可执行教学例

C++ 是独立的真实 METIS CSR 小例：6×6 单元、不同节点工作量、固定几何切边权，调用库后逐簇 BFS 验证。它省略生产 JSON 输入校验与几何处理，不是伪代码。生产程序另有错误 CSR、断开输入和空簇拒绝测试。

```bash
mkdir -p /home/ceihao/robot_project/optimization_v2/ws/build/coverage_partition
c++ -std=c++17 -O2 /home/ceihao/robot_project/optimization_v2/lessons/S11/lesson.cpp -lmetis -o /home/ceihao/robot_project/optimization_v2/ws/build/coverage_partition/lesson_s11
/home/ceihao/robot_project/optimization_v2/ws/build/coverage_partition/lesson_s11
```

输出三簇节点数/加权负载、切边以及 PASS；不表示生成了覆盖路线。

Python 直接使用生产几何、聚类、连通修复和检查代码，输入是小型合成凹口多边形，另外验证全零标签被拒绝。

```bash
/home/ceihao/robot_project/optimization_v2/venvs/scene/bin/python /home/ceihao/robot_project/optimization_v2/lessons/S11/lesson.py
```

应输出 PASS，图/几何连通分量均为 1，遗漏与重叠为 0 或浮点误差量级。程序不修改候选计划，也不启动 ROS/Gazebo。
