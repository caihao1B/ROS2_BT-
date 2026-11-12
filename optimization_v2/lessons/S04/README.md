# S04 可运行教学例

C++ 示例解释本步 PointCloud2 的 18 字节布局：x/y/z/intensity 为 float32，ring 为 uint16，使用偏移读取并拒绝截断输入；本机为 little-endian。它同时展示右相机 P[3]=-fx·B。真实上游 ros_gz_bridge 负责 Gazebo 消息到 ROS 消息转换，本例不是替代桥接器。

```bash
mkdir -p /home/ceihao/robot_project/optimization_v2/runtime/lessons
g++ -std=c++17 -Wall -Wextra /home/ceihao/robot_project/optimization_v2/lessons/S04/lesson.cpp -o /home/ceihao/robot_project/optimization_v2/runtime/lessons/s04_lesson
/home/ceihao/robot_project/optimization_v2/runtime/lessons/s04_lesson
/home/ceihao/robot_project/optimization_v2/venvs/scene/bin/python /home/ceihao/robot_project/optimization_v2/lessons/S04/lesson.py
```

两个程序均应打印 PASS、返回 0。Python 直接复用实际 ENU/WGS84 转换与故障区间函数，展示错误 fixed 坐标偏移超过 3 m、时延不改采集时间戳。它不是定位融合或偏移监督算法；那些由 S05/S08 实现。
