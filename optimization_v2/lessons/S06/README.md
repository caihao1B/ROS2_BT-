# S06 两种语言教学例

下面的例子均为**固定航向二维线性教学简化**：四条有偏差的里程计边累积出闭合误差，再加入一条经过验证的回环边，优化全部节点。它不模拟视觉识别，也不是线上 SE(3) SLAM。

```bash
cd /home/ceihao/robot_project
source optimization_v2/env.sh
mkdir -p optimization_v2/ws/build/map_manager
g++ -std=c++17 -O2 -I/usr/include/eigen3 optimization_v2/lessons/S06/lesson.cpp -o optimization_v2/ws/build/map_manager/lesson_s06
optimization_v2/ws/build/map_manager/lesson_s06
/usr/bin/python3 optimization_v2/lessons/S06/lesson.py
```

应看到闭合误差约 `0.141421 → 0.008319 m`；负信息权重与未连通图均被拒绝。不能把教学误差当作机器人实测精度。

真实 C++ 配准／数据库重开验证入口是 `ws/install/map_manager/lib/map_manager/graph_probe`，传入实际 `mapping.db` 和输出 JSON 路径。它调用安装的 RTAB-Map 0.23.13 库，检查真实回环扫描对，并对错误候选分别运行纯 ICP 和视觉后接 ICP。输出若显示“纯 ICP 接受、视觉验证拒绝”，说明平地上的几何相似性不能单独作为全局回环依据。

`TECHNICAL.md` 解释完整图优化公式，`CODE_WALKTHROUGH.md` 列出生产入口和函数位置。
