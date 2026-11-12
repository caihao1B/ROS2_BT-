# S09 C++ / Python 教学

生产算法不是复制两份。C++ 负责点云、地形风险和 Nav2 插件；Python 负责检测部署、投影关联、短时跟踪和证据评估。

```bash
cd /home/ceihao/robot_project
source optimization_v2/env.sh
g++ -std=c++17 -O2 -I optimization_v2/ws/src/terrain_perception/include optimization_v2/lessons/S09/lesson.cpp -o optimization_v2/ws/build/terrain_perception/lesson_s09
optimization_v2/ws/build/terrain_perception/lesson_s09
python3 optimization_v2/lessons/S09/lesson.py
python3 optimization_v2/ws/src/object_fusion/tests/algorithms.py
```

C++ 直接调用生产 risk.hpp：平地、缓坡、陡坡、三种低障碍、两底盘不同台阶/粗糙度上限、连续清除以及 NaN 未知。它没有生成点云，PCL 提取由正式 Gazebo 测试验证。

Python 直接调用生产 Tracker：0.5 m/s 目标、短时遮挡、协方差增长、重现关联、超时重建 ID。断言检验当前估计，不是 S15 的长期多模态预测。正式运行时消息还包含图像语义与深度一致性记录。

两个例子在 acceptance 中真实编译执行，退出码和输出写入本轮 commands.jsonl.gz。实际 Nav2 插件与 C++ 时间门控另由 ROS 节点组件测试覆盖，不能只拿教学断言替代系统验收。
