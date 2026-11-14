# R06 手工检查

```bash
cd /home/ceihao/robot_project
bash optimization_v2/three_tasks/advance.sh R06 check
bash optimization_v2/three_tasks/advance.sh R06 open
```

报告检查三任务共用树的正常、等待、跟随、取消、绕路、重规划、延期、人工锁定和完成分支。图中的时间是协议夹具发布的 ROS 仿真时钟，不是车辆真实绕障时间。

```bash
bash optimization_v2/three_tasks/advance.sh R06 build
bash optimization_v2/three_tasks/advance.sh R06 test spray
bash optimization_v2/three_tasks/advance.sh R06 test inspection
bash optimization_v2/three_tasks/advance.sh R06 test mowing
bash optimization_v2/three_tasks/advance.sh R06 finalize
```

每次运行约一分钟；仅运行 BT 与协议夹具，不启动 Gazebo。新结果目录独立，保留失败记录。测试期间观察真实节点：

```bash
source /home/ceihao/robot_project/optimization_v2/env.sh
export ROS_DOMAIN_ID=123
ros2 topic echo /r06/directive --once
ros2 topic echo /r06/tree --once
```

检查 `reason_code`、`evidence`、`alternatives`、`pending_action` 和 `stopped_confirmation`。例如未收到取消确认时应先出现 CANCEL，不应直接将 WAIT 或 DETOUR 当作已经执行。人工锁定确认由测试夹具调用专用服务并保存返回值。
