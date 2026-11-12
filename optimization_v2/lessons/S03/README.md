# S03 两种语言学习例

真实 C++ 驱动是 `ws/src/chassis_bridge/src/WheelDrive.cc`，真实 Python 仲裁/里程计是 `scripts/drive_bridge.py`。下面两个小程序聚焦运动学和输入边界，不能替代实体运动/停止实验。

```bash
mkdir -p /home/ceihao/robot_project/optimization_v2/runtime/lessons
g++ -std=c++17 -Wall -Wextra /home/ceihao/robot_project/optimization_v2/lessons/S03/lesson.cpp -o /home/ceihao/robot_project/optimization_v2/runtime/lessons/s03_lesson
/home/ceihao/robot_project/optimization_v2/runtime/lessons/s03_lesson
/home/ceihao/robot_project/optimization_v2/venvs/scene/bin/python /home/ceihao/robot_project/optimization_v2/lessons/S03/lesson.py
```

C++ 正常输入 v=0.6 m/s、ω=0.3 rad/s，曲率为 0.5 m⁻¹；前轮转角约为 0.54486、0.40232 rad，内侧角大于外侧，左右后轮速度不同。零速加角速度被拒绝，NaN 输入抛出异常。注意 0.60 m 驱动有效轮距和 0.70 m 转向几何轮距用途不同。

Python 正向计算差速轮速，再反解应回到 v=0.6、ω=0.2；阿克曼左右转角反算回 κ=0.5，检查零速转向和非有限输入。它直接调用当前核心运动学函数，参数来自本版车辆配置。

两个程序打印 PASS 且退出码 0。正式验收真实编译/执行，日志为 `lesson_cpp_build.log`、`lesson_cpp.log`、`lesson_python.log`。继续学习时比较 `TECHNICAL.md` 中的几何轮距、校准轮距、测量漂移和实际停车曲线，避免把代数正确误认为动力学已经验证。
