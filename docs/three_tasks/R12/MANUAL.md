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

spray 是三点真实卡滞/残余重试；inspection 是完整连接路；inspection_leader 展示保持距离、停车与恢复；mowing 是两条工作带及其间地头。它们各自声明选择范围。要录制完整 A1/B1 并同时写入 R10 正式独立验收，用 `R10 test record_spray_full_A1`、`R10 test record_mowing_full_B1`；两者原始分母和 1800 秒仿真时限不变。

inspection_blocked 是持续实体阻塞后的停车上报案例。通过表示有限查询、实测停车和残余保留正确，不表示走完路径，也不声称穷尽所有路径后证明全局无路。

在录像中检查：画面仿真时钟持续变化，BT 原因与停车/重新规划一致；喷洒只在实测停车后计量；割草地头刀具关闭；前车停止时保持距离；原片终止处与 recording_acceptance.json 的实际结果一致。精简版不能代替完整原片与账本检查。Ctrl+C 会进入任务自身取消与关停流程，应等待终端退出后再切换系统。

录制运行时，在另一个终端观察同一个 Gazebo 场景：
```bash
cd /home/ceihao/robot_project
bash optimization_v2/three_tasks/advance.sh R12 view-live
```
这只启动现有会话的 GUI 客户端，不创建第二辆车。关闭观察窗口不等于取消任务；结束任务仍在录制终端按 Ctrl+C，并等物理停车、刀具停止和有序退出。GUI 会增加显卡负载；回看已录原片不占仿真传感器资源。
