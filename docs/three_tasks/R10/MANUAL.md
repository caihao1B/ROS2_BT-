# R10 手工检查

先看回执及精简结果：
```bash
cd /home/ceihao/robot_project
bash optimization_v2/three_tasks/advance.sh R10 check
bash optimization_v2/three_tasks/advance.sh R10 open
cat optimization_v2/three_tasks/results/R10/case_summary.json
```

仅在 R09 有效通过后运行，每条命令等待完整退出：
```bash
cd /home/ceihao/robot_project
bash optimization_v2/three_tasks/advance.sh R10 build
bash optimization_v2/three_tasks/advance.sh R10 test spray_full_A1
bash optimization_v2/three_tasks/advance.sh R10 test spray_tool_jam
bash optimization_v2/three_tasks/advance.sh R10 test mowing_full_B1
bash optimization_v2/three_tasks/advance.sh R10 finalize
```
完整簇每次最多 1800 秒仿真，实际墙钟通常更长，并有独立有界看门狗。请勿并行运行两个 Gazebo 任务。最后一条命令重新核对巡迹四个同版本真实案例以及割草受阻案例；任何必要项未满足，结果保持 PARTIAL。

检查报告时分别查看：正常任务是否真正完成全部选择义务；失败点和未选择原任务是否保留；工具故障是否只补残余；受阻停车是否得到物理确认；所有进程是否正常退出；原 S17/S18 是否仍按原标准记录。R10 不自动进入贴图或录像步骤 R11/R12。
