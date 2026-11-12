# S13 可运行教学例

一个普通终端，命令逐条执行：

```bash
cd /home/ceihao/robot_project
c++ -std=c++17 -O2 optimization_v2/lessons/S13/lesson.cpp -o /tmp/s13_lesson
/tmp/s13_lesson
python3 optimization_v2/lessons/S13/lesson.py
```

C++ 演示差速/阿克曼限幅、停车时不能原地旋转、无准入和非有限值拒绝；Python 演示取消时保留单调路线进度。两者均含断言，失败返回非零。它们是教学简化，不是 DWB/RPP/MPPI 的替代实现，也不证明车辆实测通过。

生产闭环由 Nav2 实际 C++ 插件、C++ 命令门控和 Python ROS Action 客户端完成，查看 `docs/optimization_v2/S13/CODE_WALKTHROUGH.md`。

生产版在接近终点时保留最后两个几何点，供 FollowPath 表达方向，同时保持进度索引单调；教学例只展示剩余数据索引，不执行 ROS 跟踪。

C++ 还直接包含生产版 `stop_latch.hpp`，验证准入恢复时仍在运动、稳定时间不足、反馈丢失和时钟倒退四种边界；全部停稳连续 0.25 仿真秒才可解除锁存。限幅部分是教学简化，停车状态机使用实际生产实现。

C++ 同时使用生产版 `self_mask.hpp`，检查真实车体点被排除、距基座 1.1 m 的 8 cm 障碍点及车体旁可见地面仍被保留。

C++ 直接验证长期记忆确认计数：三次连续新鲜危险才能确认，断帧和自由观测重置计数；即时危险并不等待此确认。
