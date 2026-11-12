# S02 C++ / Python 学习例

下面是教学程序，不是完整仿真、控制器或导航算法。真实 Python 场景生成在 `ws/src/scenario_world/scripts/generate_scene.py`，真实 C++ 动态物体执行器在 `ws/src/scenario_world/src/ScriptedMotion.cc`。

在一个普通终端逐条运行：

```bash
mkdir -p /home/ceihao/robot_project/optimization_v2/runtime/lessons
g++ -std=c++17 -Wall -Wextra /home/ceihao/robot_project/optimization_v2/lessons/S02/lesson.cpp -o /home/ceihao/robot_project/optimization_v2/runtime/lessons/s02_lesson
/home/ceihao/robot_project/optimization_v2/runtime/lessons/s02_lesson
/home/ceihao/robot_project/optimization_v2/venvs/scene/bin/python /home/ceihao/robot_project/optimization_v2/lessons/S02/lesson.py
```

C++：将局部盒体旋转 90° 再平移，验证世界坐标包络为中心 (10,21,0.5)、尺寸 (1,2,1)；负尺寸会被拒绝。目的在于理解视觉与碰撞体必须使用同一坐标变换。真实插件使用 Gazebo 的三维旋转 API，本例仅示范绕 z 轴旋转。

Python：构造带孔的连续凹多边形，面积应为 63 m²；约束三角化的面积和保持 63；自交多边形和两个分离小块均拒绝。目的在于理解“含洞仍连通”和“多个分离田块”不同，不能把预设三个矩形当成算法分簇。

两个程序均打印 PASS 并返回 0；验收执行器还会真实编译/运行它们，输出位于本轮 `lesson_cpp*.log` 与 `lesson_python.log`。编译产物放 runtime 或结果目录，不污染教材源码。
