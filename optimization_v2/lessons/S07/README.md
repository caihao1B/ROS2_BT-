# S07 两种语言教学例

在一个普通终端逐条执行：

```bash
g++ -std=c++17 -O2 -I/usr/include/eigen3 /home/ceihao/robot_project/optimization_v2/lessons/S07/lesson.cpp -o /tmp/robot_s07_lesson
/tmp/robot_s07_lesson
/usr/bin/python3 /home/ceihao/robot_project/optimization_v2/lessons/S07/lesson.py
```

两个例子都计算 `T_map_odom = T_map_base × inverse(T_odom_base)`，再验证合成变换恢复观测。有效序列 true/false/true/true/true 展示失败清零、连续三次才允许；过期后关闭许可。Python 另演示坐标版本变化导致旧绑定失效。

这是可运行的最小教学例，不是完整安全实现。实际 C++ 还检查观测跨度、静止条件、质量、局部里程计与地图心跳、时钟、跳变后的重新确认；实际 Python 用文件内容 SHA-256、固定地图契约和传感器配准证据，而不是仅比较两个字符串。
