# 点喷无法精停或无法安全延期时的人工接管

独立候选分派器保留本次任务的原账本键、已计量数量、残余量和原 56 点分母。它记录 `HUMAN_REQUIRED`，停止发送运动请求，在候选会话目录写入 `operator_handoff.json`，同时发布 `/r10/operator_attention`。`automatic_resume` 为 `false`。重新作业需要操作员核查后开启新的任务。

## 任务状态和进程退出

| 现场结果 | 任务报告 | 观察者退出码 |
| --- | --- | --- |
| 已请求人工，车身和工具均有新鲜停稳量测，系统已完成退出协议 | `HUMAN_REQUIRED`，`mission_passed=false`，保留残余 | `0`，可正常回收本次进程 |
| 请求人工后缺少车身或工具停稳、动作仍活动、未收到 `STOP_WORK` 确认、反馈过期，或清理出现异常 | `HUMAN_REQUIRED`，`mission_passed=false`，保留残余及不确定状态 | `2`，需检查停车证据 |

独立分派器会核对底盘里程计线速度和角速度、执行器 `gate`、导航闲置状态、喷头原始流量与原始转速、反馈会话与修订标识、`STOP_WORK` 回执以及清理异常。原精停精度、工作剂量和停车速度门限继续由现场配置读取。程序异常等未识别情况走原故障路径。

## 来自原现场的回放

正式 A1 在 `point_10_20` 耗尽两次局部微调，当时 22/56 完成，本点待作业 3.0 单位。原运行观察者退出码为 `2`，任务仍为 `PARTIAL`。新候选在回放这些真实测量时生成了待人工核查的原键、原剂量与零额外信用。末次里程计速度为 0.00135 m/s，角速度为 −0.00143 rad/s。原始喷头测得流量为零。独立真值评估记录末 0.28 秒车体无平移与转动。

回放程序真正启动子进程，分别核对其终态报告和操作系统返回码。原始完整证据案例得到 `0`，19 个反例得到 `2`。反例覆盖缺少任一停稳证据、仍在移动、工具转动、反馈过期、动作未结束、反馈会话不符、缺失停止回执和错误宣称已完成。原记录只有应用层 `STOPPED` 回执；从本候选开始还记录 ROS 动作终态码。历史回放依据末次导航终态、gate 与喷头反馈推断动作已闲置，在线候选直接检查自己持有的动作句柄。[输入快照](../../validation/closeout_20261119/spray_handoff_inputs.json) · [逐项退出结果](../../validation/closeout_20261119/spray_handoff_probe.json)

## 人工复验

在仓库根目录运行下列命令。纯 Python 回放无需启动 Gazebo。它编译生成独立工作节点并向 20 个子进程逐项传入记录。`/tmp` 用于本次独立输出。

```bash
python3 optimization_v2/ws/src/task_platform/scripts/probe_spray_handoff_r10.py \
  --bundle validation/closeout_20261119/spray_handoff_inputs.json \
  --out /tmp/spray_handoff_probe.json
python3 -m json.tool /tmp/spray_handoff_probe.json | less
```

完整四点自然场景使用原地图、原传感器、原四个连续目标和原录制链路。请先按仓库主页准备 R10/R12 运行环境。在模拟器空闲时执行以下命令。

```bash
source optimization_v2/env.sh
optimization_v2/venvs/scene/bin/python \
  optimization_v2/ws/src/task_platform/scripts/spray_handoff_candidate_r10.py \
  run --seconds 300 --rtf .55 --natural
```

结果位于 `optimization_v2/three_tasks/results/experiments/spray_operator_handoff/`。查看最新会话的 `candidate_handoff_acceptance.json`、`mission_result.json`、`runtime.json` 和 `video/recording.json`。现场触发人工接管时会额外出现 `operator_handoff.json`。`handoff_status=NOT_EXERCISED` 说明本次没有现场触发人工接管，四点物理复验结果单独记录在 `separately_confirmed_four_points`。

以下命令使用独立 ROS 域复验人工通知。测试抽取候选工作节点实际使用的持久化与发布函数，先发布请求，再让操作员节点订阅。初始请求和最终关闭回执均通过真实 DDS 收到。发布节点退出后仍可从 JSON 文件查看原键、残余和终态。

```bash
source optimization_v2/env.sh
python3 optimization_v2/ws/src/task_platform/scripts/spray_handoff_candidate_r10.py prepare
(
  export ROS_DOMAIN_ID=157 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
  optimization_v2/venvs/scene/bin/python \
    optimization_v2/ws/src/task_platform/scripts/probe_spray_handoff_transport_r10.py \
    --worker optimization_v2/three_tasks/build/spray_handoff_candidate/mission_spray_handoff.py \
    --inputs validation/closeout_20261119/spray_handoff_inputs.json \
    --out /tmp/spray_operator_notice_transport.json
)
```

[人工通知的八项验收](../../validation/closeout_20261119/spray_operator_notice_transport.json)

## 最后一处已派任务遭设备箱持续占用

另一个独立版本只选择原顺序中相邻的 `point_11_20` 和 `point_10_20`。地图、两点坐标、零工作信用规则、工具停稳和感知准入与四点实体箱试验一致。本例的设备箱仍占据第二点。已派清单到此结束，剩余 54 个原点未派入本次试验。原生行为树可根据实际传感器和导航结果评估是否需要人工接管。

```bash
source optimization_v2/env.sh
optimization_v2/venvs/scene/bin/python optimization_v2/ws/src/task_platform/scripts/spray_last_blocked_latch_r10.py check
optimization_v2/venvs/scene/bin/python optimization_v2/ws/src/task_platform/scripts/spray_last_blocked_latch_r10.py run --seconds 118 --rtf .55
```

运行中可另开终端订阅操作员告警。`--qos-durability transient_local` 使稍晚加入的订阅者能收到仍在线发布者保留的最新状态。会话结束后直接打开结果目录中的 `operator_handoff.json`，其中 `shutdown.checks` 给出各项实测证据。

```bash
source optimization_v2/env.sh
export ROS_DOMAIN_ID=119
ros2 topic echo /r10/operator_attention std_msgs/msg/String --qos-durability transient_local
```

### 原生动作和工作节点如何配合

原生行为树把“允许重新规划”与“已经重新下发运动动作”分开处理。本例已派清单的最后一点长期被实体设备箱占用。原生 max_replans=2 耗尽且没有下一处已派任务时，行为树会在一个新 tick 锁存人工处理。工作节点每次收到 REPLAN 都要继续订阅后续动作，只有 EXECUTE 才允许发送下一动作。若先收到 HUMAN 或 STOP，工作节点保存该消息的派发号与上下文序号，进入停车处理。BOUNDED_REPLAN_EXHAUSTED 表示现场在有界重规划次数内未能通行。

| 同两点、同一实体箱 | 首版实测 | 修订版实测 |
| --- | --- | --- |
| 末点 NavigateToPose 真实终态 | 44.082、57.290 秒，均错误 208 | 43.374、56.568 秒，均错误 208 |
| 原生 REPLAN | 56.146、69.350 秒 | 55.430、68.630 秒 |
| 原生 HUMAN 锁存 | 69.400 秒，派发号 13 | 68.676 秒，派发号 12 |
| 工作节点、操作员与最终退出 | 只等待 EXECUTE，74.354 秒超时，独立操作员收到 0 条通知，专项失败 | 68.688 秒完成双停，独立操作员收到请求及关闭两条，观察者退出 0，专项通过 |
| 作业结果 | 首点完成，末点未能喷洒 | 首点确认 3.052，独立物理评分 3.190。末点保留 3.000，任务为 HUMAN_REQUIRED，任务成功标志为 false |

两次均只调度原 A1 的 2 个点，另外 54 个原点没有派入。修订版的 11 项退出确认全部通过。机器可自动关闭本轮进程，操作员应核实安全通道或移走设备箱，再决定是否重新启动任务。原正式 A1 仍然是 22/56。其他有下一点的场景请查看[真实停车、延期与重选试验](spray_defer_candidate_20261119.md)。

### 不启动 Gazebo 的发布验收

克隆本仓库后，从根目录运行下列命令。程序直接读取两次现场导出的压缩或未压缩记录及退出回执，仅依赖 Python 3 标准库。输出文件保留事件时刻、原生派发号、任务和进程结果、未压缩源文件的 SHA-256 以及逐项判定。

```bash
python3 optimization_v2/ws/src/task_platform/scripts/audit_spray_last_handoff_r10.py \
  --initial validation/closeout_20261119/spray_last_blocked_initial \
  --revised validation/closeout_20261119/spray_last_blocked \
  --out /tmp/spray_last_latch_comparison.json
python3 -m json.tool /tmp/spray_last_latch_comparison.json | less
```

8 项现场对照检查均通过。此外，回放依次在内存中遮蔽首条操作员通知、最终通知、持久文件一致性、底盘停稳、工具停稳和显式停止回执，6 个反例分别被拒绝。上面的完整历史测量回放还会启动 20 个独立子进程，验证证据缺失时得到错误退出码。

[完整比较与原始数据摘要](../../validation/closeout_20261119/spray_last_latch_comparison.json) · [原生告警片段](../../media/closeout_20261119/spray_last_blocked_handoff.mp4) · [最后点原片](../../media/closeout_20261119/spray_last_blocked_full.mp4)

### 在人工重跑时保留独立订阅回执

用上面的 spray_last_blocked_latch_r10.py run 命令开始仿真。在另一个终端执行以下命令。等新的会话子目录已经出现后再运行，并在首个告警前开始监听。ls -dt 按修改时间倒序列出目录，head -1 取最新一项。该只读节点仅订阅 /r10/operator_attention，拒绝向已有录像或已关闭会话补写回执。

```bash
source optimization_v2/env.sh
export ROS_DOMAIN_ID=119
latest_spray_case=$(ls -dt optimization_v2/three_tasks/results/experiments/spray_last_blocked_latch/spray_defer_* | head -1)
optimization_v2/venvs/scene/bin/python \
  optimization_v2/ws/src/task_platform/scripts/observe_spray_operator_notice.py \
  --run "$latest_spray_case" --max-wall-s 420
python3 -m json.tool "$latest_spray_case/operator_attention_receipt.json"
```
