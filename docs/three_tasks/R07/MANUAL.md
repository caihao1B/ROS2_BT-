# R07 手工检查与复现

最终状态以回执为准；没有 PASS 回执时不宣称验收通过。

```bash
cd /home/ceihao/robot_project
bash optimization_v2/three_tasks/advance.sh R07 check
bash optimization_v2/three_tasks/advance.sh R07 open
```

构建和少量点位复测：

```bash
bash optimization_v2/three_tasks/advance.sh R07 build
bash optimization_v2/three_tasks/advance.sh R07 test A1 nominal 3
bash optimization_v2/three_tasks/advance.sh R07 test A1 tool_jam 3
bash optimization_v2/three_tasks/advance.sh R07 finalize
```

`nominal` 检查真实导航、停止后喷洒、反馈记账与幂等重放。`tool_jam` 在第二个点注入声明的短时工具故障，必须记录失败/部分完成、停止、恢复后的残余重试。每次运行有独立目录和仿真/墙钟上限，不会把未选的其余点记为完成。

不连接 ROS 的只读进度查看：

```bash
cd /home/ceihao/robot_project
optimization_v2/venvs/scene/bin/python optimization_v2/ws/src/task_platform/scripts/status_r07.py
```

运行中另开终端观察：

```bash
source /home/ceihao/robot_project/optimization_v2/env.sh
export ROS_DOMAIN_ID=119
ros2 topic echo /r07/mission --once
ros2 topic echo /r06/directive --once
ros2 topic echo /s10/ledger/state --once
ros2 topic echo /r07/gate --once
ros2 topic echo /s08/status --once
```

重点查看：NAVIGATING 时工具应已停止；WORKING 时底盘应实测停止；账本由反馈增长；重复请求有 `replayed=true` 且物理流量不增加。停止不确定的动作必须标记，不能自动记为成功。查看 `mission_result.json` 中完整分母和 `not_selected` 列表。

完整簇入口为 `R07 test A1 nominal 0`（A2/A3 类似），上限 1800 秒仿真。它是后续完整任务验收入口，不代表目前已验证全簇。正常少量点位测试前不要同时启动另一仿真。
