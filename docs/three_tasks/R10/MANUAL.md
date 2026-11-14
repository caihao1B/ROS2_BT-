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

检查报告时分别查看：正常任务是否真正完成全部选择义务；失败点和未选择原任务是否保留；工具故障是否只补残余；受阻停车是否得到物理确认；所有进程是否正常退出；原 S17/S18 是否仍按原标准记录。本轮用户已追加授权推进 R11/R12；R10 未满足完整标准时，后续组件可以准备与独立录制，但总体交付仍保持 PARTIAL。

原基元停车换行的较短检查（约两条工作带与中间倒车连接）：
```bash
cd /home/ceihao/robot_project
bash optimization_v2/three_tasks/advance.sh R10 test mowing_headland
```

打开该会话 `mission_events.jsonl`，检查 `SEGMENT_GOAL.original_members`：当前默认每个原始基元分别为一组；每个原始段有按顺序产生的 `ORIGINAL_SEGMENT_PASSAGE`，工作条带仍分别启动和停止工具。旧连续分组三段合并的失败记录保留；该短测试不能替代完整 B1，最终整体回执仍要求全部 34 段。

R09 的 `test nominal 0` 保留原逐段停车方式作历史对照；本轮完整割草采用上面的 **R10 test mowing_full_B1** 当前逐基元停车入口。

正在运行 R10 任务时，可另开终端接入同一 Gazebo 三维场景（会增加显示负载，不会再创建机器人）：
```bash
cd /home/ceihao/robot_project
source optimization_v2/env.sh
python3 - <<'PY'
from pathlib import Path
import json,os,subprocess
base=Path('optimization_v2/three_tasks/results/R10')
p=max((p for p in base.glob('*_20*') if p.is_dir()),key=lambda p:p.name.split('_',1)[1])
if (p/'runtime.json').exists():raise SystemExit('当前最近一次任务已经退出，请先在另一终端启动测试。')
s=json.loads((p/'spec.json').read_text());env=os.environ.copy()
env['GZ_PARTITION']=('r10_mowing_' if s['task']=='mowing' else 'r10_')+s['session']
subprocess.run(['gz','sim','--force-version','8','-g'],env=env,check=False)
PY
```

查看原场地、实体车轮转向和工具关闭换行；GUI 只是观察窗口，验收依据仍是传感器记录、接触检测、导航动作和工作账本。正式录像属于 R12。

同次运行完整任务并录像，先生成独立材质和录制入口：
```bash
cd /home/ceihao/robot_project
bash optimization_v2/three_tasks/advance.sh R11 build
bash optimization_v2/three_tasks/advance.sh R12 build
bash optimization_v2/three_tasks/advance.sh R10 test record_spray_full_A1
bash optimization_v2/three_tasks/advance.sh R10 test record_mowing_full_B1
```
不要在运行时重新 build 或改源码/配置。查看恢复日志中的 `ENDPOINT_POSE_STALL`、`ENDPOINT_RECOVERY_PLAN`、`CONTINUOUS_RECOVERY_CANDIDATE`、`ENDPOINT_FULL_COSTMAP_VALIDATION` 和 `ENDPOINT_RECOVERY_SUCCESS`；前两项仅表示触发与尝试，只有真实动作到达和最后全部原始段通过才能算完整成功。`ENDPOINT_RECOVERY_EXHAUSTED` 等停止原因必须保留，不用手工改回执。

点喷恢复检查：在 `mission_events.jsonl` 中查找 `NEAR_GOAL_PROGRESS_REPLAN`，核对距离、无进展时长、平移命令时长，随后必须有取消确认和实际新导航；后续 `DOCKING_SETTLED_CHECK`、`TOOL_RESULT` 才能证明恢复后停靠和作业。该优化不允许把重规划事件本身记作到点完成。

查看本轮完整割草的关键恢复与终止事件（命令只读取已结束案例）：
```bash
cd /home/ceihao/robot_project
python3 - <<'PY'
from pathlib import Path
import json
pointer=Path('optimization_v2/three_tasks/results/R12/mowing_full_B1_latest.json')
run=Path(json.loads(pointer.read_text())['path'])
for line in (run/'mission_events.jsonl').read_text().splitlines():
    event=json.loads(line)
    if event['event'] in ['ENDPOINT_POSE_STALL','ENDPOINT_RECOVERY_INTERRUPTED','ENDPOINT_OBSERVATION_RESUME','ENDPOINT_RESUME_REJECTED','ENDPOINT_RECOVERY_SUCCESS','TASK_STOP','SELECTED_SEGMENTS_TRAVERSED']:
        print(event)
print(json.loads((run/'mission_result.json').read_text())['terminal'])
PY
```

`ENDPOINT_OBSERVATION_RESUME` 表示短时观测恢复后重新准入原候选，不能当成最终到达；其后仍必须有实际成功和段到达记录。任何人工锁定、六次续行上限或 90 秒总恢复预算触发，都必须停止并保留残余。

本轮新增专项故障命令（等待上一会话完全退出再执行）：
```bash
cd /home/ceihao/robot_project
bash optimization_v2/three_tasks/advance.sh R10 test mowing_startup_perception_loss
```
检查该会话 `startup_fault.json` 的触发阶段与进程恢复，以及 `mission_events.jsonl` 中 `BT_INTERRUPTS_START_WORK` 后是否在原 2 秒预算内出现 `BOTH_PHYSICAL_STOPS_CONFIRMED`。最终仍须走完选择的五个原始段，反馈面积与物理效果一致；工具故障恢复不是额外完成量。该用例失败时不要手动更改回执或强制清除 BT 锁存。

同次录制上述启动故障与五段复验（不需要另跑一次无录像版本）：
```bash
cd /home/ceihao/robot_project
bash optimization_v2/three_tasks/advance.sh R10 test record_mowing_startup_perception_loss
```
该命令同时写入 R10 专项故障验收和 R12 独立录像回执；案例名明确为启动感知中断，不作为正常无故障案例。完整 B1 仍另外运行。

新全局管理器的原始异常监督检查（隔离 ROS 域 133，无 Gazebo、无底盘控制）：
```bash
cd /home/ceihao/robot_project
bash optimization_v2/three_tasks/advance.sh R10 test global_filter_guards
```
它发送合成的已知冲突观测，分别验证 GNSS 冲突拒绝与地图跳变停止锁存；输出在 `results/R10/global_filter_probe/result.json`。该测试不代替实际定位/作业验收。重新测试后执行 `R10 finalize` 更新回执，整体未满足条件时仍返回 PARTIAL。

账本时间基准回归（隔离 ROS 域 134，无 Gazebo）：
```bash
cd /home/ceihao/robot_project
bash optimization_v2/three_tasks/advance.sh R10 test ledger_clock
cat optimization_v2/three_tasks/results/R10/ledger_clock_probe/result.json
```
核对原版在慢时钟下的发布周期、新版小于原 0.25 墙钟秒超时以及暂停后 `CLOCK_WATCHDOG`。这只是传输专项检查，不能代替完整 B1。不要通过修改工具服务器超时值掩盖账本停顿。
