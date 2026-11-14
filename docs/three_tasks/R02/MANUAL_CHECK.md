# R02 手工检查

```bash
bash /home/ceihao/robot_project/optimization_v2/three_tasks/advance.sh R02 check
bash /home/ceihao/robot_project/optimization_v2/three_tasks/advance.sh R02 open
```

报告显示三车同尺度尺寸、实际 Gazebo 行驶轨迹、左右转弯拟合半径与理论值、道路轮廓检查。图中的轨迹来自独立真值评价，不是输入的理想轨迹。

可独立复测一个车辆（约 90–110 仿真秒，运行后再 finalize 重新生成本阶段回执）：
```bash
bash /home/ceihao/robot_project/optimization_v2/three_tasks/advance.sh R02 test spray
bash /home/ceihao/robot_project/optimization_v2/three_tasks/advance.sh R02 test inspection
bash /home/ceihao/robot_project/optimization_v2/three_tasks/advance.sh R02 test mowing
bash /home/ceihao/robot_project/optimization_v2/three_tasks/advance.sh R02 finalize
```

逐个执行；启动器共享互斥锁。成功结果应具有真实前进、后退、左右转向，差速原地转向，阿克曼零速旋转被拒绝，结束停稳且所有进程退出 0。默认物理试验不启动 RViz/相机，传感器套件在 R05 复验；原三任务完整链路仍从 R01 入口运行。

已生成的三个模型与配置位于 `optimization_v2/three_tasks/editions/R02/models` 和 `vehicles`；修改配置需要更新版本并复验，不能直接沿用已通过回执。
