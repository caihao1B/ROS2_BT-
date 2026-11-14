# R09 手工检查

```bash
cd /home/ceihao/robot_project
bash optimization_v2/three_tasks/advance.sh R09 check
bash optimization_v2/three_tasks/advance.sh R09 open
```
查看真实车身轨迹是否沿原地头曲线倒车，转弯区工具是否关闭，账本确认面积与未完成面积是否分别列出。检查 `runtime.json` 的正常退出及 `mission_result.json` 的停止确认。

复验组件案例：
```bash
cd /home/ceihao/robot_project
bash optimization_v2/three_tasks/advance.sh R09 test nominal 5
bash optimization_v2/three_tasks/advance.sh R09 test blocked 5
```
参数 5 表示原 B1 前五段，不是完整簇；不能把该结果解释为全 B1 覆盖完成。受阻案例应留下残余和明确的人工处理原因，不能增加虚假的工作量。

完整 B1 入口，上限 1800 秒仿真：
```bash
cd /home/ceihao/robot_project
bash optimization_v2/three_tasks/advance.sh R09 test nominal 0
```
每次只能运行一辆车，等待进程完整退出后再执行下一条命令。具体耗时与结果以会话回执为准。

报告图左侧比较原 B1 全部计划与实际选取段的行驶轨迹；右侧显示本次选择的工作区域、绿色反馈确认面积和橙色残余。绿色来自实际反馈账本，不是计划扫过面积。未选择的原段及原工作区分母仍在结果中保留。

受阻案例的障碍在定位准入、实际行驶 0.30 米之后创建。检查该会话的 `barrier_injection.json`，应有 `passed: true`；检查 `mission_events.jsonl` 的 `ACK_PLANNER_RESULT` 和最终原因。如果找到候选，当前割草工作器会明确上报需要覆盖重排，不执行该候选，不报告“绝对无路”。从启动即受大障碍遮挡的旧诊断未通过，仍是该定位方案的限制。
