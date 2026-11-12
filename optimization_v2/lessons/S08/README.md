# S08 两种语言学习例
Python 直接调用生产坐标转换；C++ 是生产创新检验的教学子集，生产 ROS 节点还包含时间、版本、航向与停车迟滞。

```bash
source /home/ceihao/robot_project/optimization_v2/env.sh
python3 /home/ceihao/robot_project/optimization_v2/lessons/S08/lesson.py
g++ -std=c++17 -O2 -I/usr/include/eigen3 /home/ceihao/robot_project/optimization_v2/lessons/S08/lesson.cpp -o /home/ceihao/robot_project/optimization_v2/ws/build/gnss_supervisor/lesson_s08
/home/ceihao/robot_project/optimization_v2/ws/build/gnss_supervisor/lesson_s08
```

Python 输出 map 天线坐标 `[2,1,1]` 米，并拒绝负协方差；天线不是车辆 base_link，生产代码还要扣除经车身姿态旋转的安装杆臂。C++ 输出正常 3cm/2cm 残差及错误 fixed 的 1.2m/−0.8m 残差；后者必须被拒绝。两个例子是已知样例，不作为 Gazebo 性能测量。
