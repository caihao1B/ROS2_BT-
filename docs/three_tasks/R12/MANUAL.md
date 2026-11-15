# R12 手工录制与查看

先查看已经交付的内容：
```bash
cd /home/ceihao/robot_project
bash optimization_v2/three_tasks/advance.sh R12 check
bash optimization_v2/three_tasks/advance.sh R12 open
xdg-open optimization_v2/three_tasks/results/R12/media/index.html
```

独立录制，逐条运行，不并发 Gazebo：
```bash
bash optimization_v2/three_tasks/advance.sh R12 build
bash optimization_v2/three_tasks/advance.sh R12 record spray
bash optimization_v2/three_tasks/advance.sh R12 record inspection
bash optimization_v2/three_tasks/advance.sh R12 record inspection_leader
bash optimization_v2/three_tasks/advance.sh R12 record mowing
bash optimization_v2/three_tasks/advance.sh R12 record inspection_blocked
bash optimization_v2/three_tasks/advance.sh R12 export spray inspection inspection_leader mowing inspection_blocked
bash optimization_v2/three_tasks/advance.sh R12 finalize
```

spray 是三点真实卡滞/残余重试；inspection 是完整连接路；inspection_leader 展示保持距离、停车与恢复；mowing 是两条工作带及其间地头。它们各自声明选择范围。要录制完整 A1/B1 并同时写入 R10 正式独立验收，用 `R10 test record_spray_full_A1`、`R10 test record_mowing_full_B1`。A1 保留原 56 点；B1 执行用户批准的安全路径版本，原 34 段参考仍单列为 PARTIAL，原作业面积不变。两者均保留 1800 秒仿真时限，见 [路径与残余说明](../R10/SAFE_EXECUTION.md)。

inspection_blocked 是持续实体阻塞后的停车上报案例。通过表示有限查询、实测停车和残余保留正确，不表示走完路径，也不声称穷尽所有路径后证明全局无路。

在录像中检查：画面仿真时钟持续变化，BT 原因与停车/重新规划一致；喷洒只在实测停车后计量；割草地头刀具关闭；前车停止时保持距离；原片终止处与 recording_acceptance.json 的实际结果一致。精简版不能代替完整原片与账本检查。Ctrl+C 会进入任务自身取消与关停流程，应等待终端退出后再切换系统。

录制运行时，在另一个终端观察同一个 Gazebo 场景：
```bash
cd /home/ceihao/robot_project
bash optimization_v2/three_tasks/advance.sh R12 view-live
```
这只启动现有会话的 GUI 客户端，不创建第二辆车。关闭观察窗口不等于取消任务；结束任务仍在录制终端按 Ctrl+C，并等物理停车、刀具停止和有序退出。GUI 会增加显卡负载；回看已录原片不占仿真传感器资源。

R10 修复版本新增启动感知中断的录像案例：
```bash
cd /home/ceihao/robot_project
bash optimization_v2/three_tasks/advance.sh R10 test record_mowing_startup_perception_loss
```
同时验证五个当前执行段、两条工作带和启动取消恢复；该案例独立于完整 B1。本轮有界交付中，完整割草录像请求 0.4 实时因子、完整点喷请求 0.5，持续阻塞巡检保留 0.3；这些值只调整墙钟推进目标，仿真时限、车辆动力学、感知频率、任务分母和验收门限不变，不能直接用墙钟耗时比较控制效率。

本轮现成媒体采用启动中断专项替代旧版正常割草片段；三个任务及巡迹交互仍完整保留。需要重新导出当前指针时：
```bash
cd /home/ceihao/robot_project
bash optimization_v2/three_tasks/advance.sh R12 export spray inspection inspection_leader mowing_startup_perception_loss inspection_blocked
```
导出前检查各案例 `recording_acceptance.json`。0.3 实时因子也不能保证长录像合格；超过原画面年龄或帧间隔门限时，保留原片和失败回执，不能标为录像质量通过。

完整 A1、新安全版 B1 和各专项录像都通过当前版本检查后，导出包含全部案例的版本：

```bash
bash optimization_v2/three_tasks/advance.sh R12 export \
  spray_full_A1 spray inspection inspection_leader mowing_full_B1 \
  mowing_startup_perception_loss inspection_blocked
```

`media/manifest.json` 列出每段原片、实际选择范围、倍速、事件片段和哈希。完整 B1 的路径执行与反馈覆盖率分开报告；安全裁剪留下的未作业面积仍保留在原分母中。


## 实时监视界面

另开终端执行 `bash optimization_v2/three_tasks/gui.sh`，浏览器打开 `http://127.0.0.1:8088`。可与一个正在运行的任务同时使用；GUI 只读，关闭界面不关闭任务。详细数据链路和手工核查见 [GUI 说明](../../../optimization_v2/three_tasks/monitor/README.md)。
