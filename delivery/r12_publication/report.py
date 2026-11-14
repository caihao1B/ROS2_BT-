"""Generate learning/result documents from actual finalized evidence, without changing credit."""
from pathlib import Path
import json
ROOT=Path(__file__).resolve().parents[2];APP=ROOT/'optimization_v2/three_tasks';DOC=ROOT/'docs/three_tasks'
def read(p):return json.loads(p.read_text())
def write(stage,lines): (DOC/stage/'RESULTS.md').write_text('\n'.join(lines)+'\n')
s=read(APP/'results/R10/case_summary.json');status='PASS' if s['passed'] else 'PARTIAL'
lines=['# R10 三任务分别验收','',f'**{status}**。'+('可以按前置进入 R11。' if s['passed'] else '尚不能按完整验收通过 R10；按本轮授权，R11 材质与 R12 独立演示可以交付，不能替代本阶段缺项。'),'','原 A1 56 点、连接路 88.481 米、B1 34 段的要求保留；其余簇与六十次正式矩阵未由本轮推定通过。原 S17/S18 保持 PARTIAL。','','|案例|结果|仿真秒|终态|确认量及残余|','|---|---|---:|---|---|']
for d in s['cases']:
 amount=[]
 if d.get('selected_points') is not None:amount.append(f"点 {d['confirmed_points']}/{d['selected_points']}，原任务 {d['original_points']}")
 if d.get('selected_segments') is not None:amount.append(f"段 {d['confirmed_segments']}/{d['selected_segments']}，原任务 {d['original_segments']}")
 if d.get('confirmed_area_m2') is not None:amount.append(f"反馈 {d['confirmed_area_m2']:.3f} m²，残余 {d['residual_area_m2']:.3f} m²")
 if d.get('remaining_original_route_m') is not None:amount.append(f"原路径剩余 {d['remaining_original_route_m']:.3f} m")
 lines.append(f"|{d['case']}|{'PASS' if d['passed'] else '未通过'}|{d.get('simulation_seconds')}|{d.get('terminal')}|{'；'.join(amount)}|")
lines+=['','受阻案例通过表示物理停车、有界处理与残余保留正确，不表示工作已经完成。条带抵达和反馈确认面积分别统计；不降低原始覆盖分母，不把模拟电机工作等同于真实药效或切割效果。','','## 未完成项','']+['- '+x for x in (s['pending'] or ['本阶段声明范围内无未完成项。'])]
for d in s['cases']:
 if d['case']!='spray_full_A1':continue
 p=ROOT/d['path'];a=read(p/'analysis_spray_r10.json');jobs=a.get('jobs',[])
 matched=[j for j in jobs if j['state']=='SUCCEEDED' and not j['uncertain'] and j['requested']<=j['confirmed']<=j['physical_dose']+.10 and j['maximum_physical_docking_error_m'] is not None and j['maximum_physical_docking_error_m']<.20]
 lines+=['','## 完整点喷本次停止的解释','',f"已记录 {len(jobs)} 笔工作，其中 {len(matched)} 笔满足原独立物理反馈核对条件。完整案例的 `feedback_and_independent_physics_agree` 同时要求记录数等于选中的 {d['selected_points']} 点，因此未执行点也会导致该总检查失败；不能把它直接解释成已经完成的工作全部存在反馈冲突。未确认点仍保留，完整任务仍为未通过。"]
 events=[json.loads(x) for x in (p/'mission_events.jsonl').read_text().splitlines()]
 settle=next((e for e in reversed(events) if e['event']=='DOCKING_SETTLED_CHECK'),None)
 if settle:
  q=settle['stability'];lines+=[f"最后精停检查：点 `{settle['point']}`，瞬时目标距离 {settle['distance_m']:.4f} m，位置样本 P95 散布 {q['position_scatter_p95_m']:.4f} m。瞬时距离很小并不保证定位稳定；原准入门限与重试上限不降低。终止原因：`{d.get('reason')}`。"]
diag=APP/'results/R10/docking_diagnosis.json'
if diag.exists():
 q=read(diag);lines+=['',f"最后停靠窗口的独立事后诊断：全局估计 P95 散布 {q['global_estimate']['p95_scatter_m']:.6f} m，局部里程计 {q['local_odometry']['p95_scatter_m']:.6f} m，真值位置最大位移 {q['offline_true_position']['max_displacement_m']:.6f} m。该窗口主要是全局修正抖动，并非车辆仍在运动；此结果没有反向参与在线决策。下一轮先处理全局停靠精度、局部静止与定位不确定性的区别，保留独立 GNSS 监督与工具反馈门限。"]
diag=APP/'results/R10/mowing_start_diagnosis.json'
if diag.exists():
 q=read(diag);lines+=['','## 完整割草的下一轮修复点','',f"工具重新启动时，{q['first_perception_invalid_t']} 秒感知失效，{q['first_tool_off_after_start_t']} 秒已关闭工具请求，但任务仍等待工作确认直到 {q['mission_timeout_t']} 秒。BT 的取消确认提前超时。根据日志与代码，应修复 START_WORK 等待阶段的取消协同，并专门复验开刀时的感知中断；该修复尚未实施。证据见 `mowing_start_diagnosis.json`，最终实测停车已确认。"]
lines+=['','## 原始证据','']+[f"- `{d['case']}`：`{d['path']}`；终止原因 `{d.get('reason')}`。" for d in s['cases']]
for d in s['cases']:
 if d.get('termination_context'):lines+=['',f"`{d['case']}` 的编排停止说明：`{json.dumps(d['termination_context'],ensure_ascii=False)}`。这不算完整任务通过。"]
 if 'endpoint_recovery_successes' in d:lines+=['',f"当前完整 B1 运行记录 {len(d['endpoint_recovery_successes'])} 次 `ENDPOINT_RECOVERY_SUCCESS`，{d['observation_resume_count']} 次观测恢复事件。恢复成功仍需与最终段完成数分别阅读；原候选续行被拒绝后，重新生成候选并到达，也记录为有界恢复成功。"]
 if not d['passed'] and d.get('terminal_snapshot'):lines+=['',f"`{d['case']}` 结束快照：`{json.dumps(d['terminal_snapshot'],ensure_ascii=False)}`。取消后的命令过期等状态不自动作为故障根因；需与先前事件和停止原因一起检查。"]
lines+=['','命令见 MANUAL.md；技术选择见 TECHNICAL.md；Python/C++ 调用链、运动学及工具互锁见 CODE.md。'];write('R10',lines)
r=read(APP/'results/R12/latest.json');s=r['summary'];lines=['# R12 真实录像与交付','',f"**{r['automated_status']}**。三任务录像组件：{'PASS' if s.get('recording_components_passed') else '未全部通过'}。总体结论依据 R10/R11 和当前录像绑定，不以视频数量替代完整任务验收。",'','|案例|任务检查|录像时长/秒|仿真终时/秒|当前全部检查|','|---|---|---:|---:|---|']
for name,d in s['cases'].items():lines.append(f"|{name}|{'PASS' if d.get('task_passed') else '未通过'}|{d.get('video_seconds')}|{d.get('sim_seconds')}|{'PASS' if d.get('passed') else '未通过'}|")
lines+=['','完整录像包括启动、真实运动、等待、任务终止与停车，精简版明确标注每段倍速。小规模点喷仅包含原 A1 前三个点及一次实体工具 JAM/残余补作；割草小规模仅含原 B1 前五段。名称带 full_A1/full_B1 的案例才选择完整原任务，仍须看实际通过状态。','','## 查看','', '```bash','cd /home/ceihao/robot_project','xdg-open optimization_v2/three_tasks/results/R12/media/index.html','bash optimization_v2/three_tasks/advance.sh R12 check','```','','## 限制及后续','']+['- '+x for x in (s['pending'] or ['本阶段声明范围内无未完成项。'])]
lines+=['- R11 使用轻量原创面板贴图与材质，仍是简化几何；不是商品机器人精细外观或实物验证。','- 完整数据库、构建环境和逐帧记录保存在本机；GitHub 提供源码、配置、说明、精简证据和展示视频。','- 原 S17/S18 与未运行的簇保持原状态，不自动开始更大规模正式矩阵。','','## 原片及验收出处','']+[f"- `{name}`：`{d['path']}/video/full.mp4`；同目录 `recording_acceptance.json` 和对应 `analysis_*.json`。" for name,d in s['cases'].items()]
write('R12',lines)
statuses={stage:read(APP/'results'/stage/'latest.json')['automated_status'] for stage in ['R08','R09','R10','R11','R12']}
lines=['# 本轮 R08–R12 推进结果','', '本轮按用户追加的 3.5 小时时限推进，最终以各阶段哈希回执及本次 GitHub 同步回执为准。','','|阶段|封存结果|说明|','|---|---|---|']
notes={'R08':'完整连接路、跟随停车恢复、绕行、持续受阻的独立实际案例。','R09':'原 B1 前五段与实体持续障碍的组件验收；不代表完整 B1。','R10':'完整 A1、连接路、B1 及声明故障分别评分。','R11':'三种材质的实际渲染与性能对照；前置未通过时总体保持 PARTIAL。','R12':'真实录像、精简版、逐帧时间线及启动说明；前置和未完成任务单列。'}
lines += [f'|{stage}|{value}|{notes[stage]}|' for stage,value in statuses.items()]
lines += ['','## 本机检查','', '```bash','cd /home/ceihao/robot_project','bash optimization_v2/three_tasks/advance.sh status','bash optimization_v2/three_tasks/advance.sh R08 check','bash optimization_v2/three_tasks/advance.sh R09 check','bash optimization_v2/three_tasks/advance.sh R10 open','xdg-open optimization_v2/three_tasks/results/R12/media/index.html','```','','每次只启动一个仿真。手工录制命令见 R12/MANUAL.md，完整任务及退出说明见 R10/MANUAL.md。Ctrl+C 后等待该任务确认物理停车和自身进程有序退出，再启动下一项。','','## 下一轮提示词','','> 从本轮 R08–R12 最终回执继续。先核对封存哈希、R10 case_summary、R12 录像检查及本轮未通过原因。保留原地图、任务分母、原 S17/S18 状态和已通过视频。优先修复尚未通过的当前任务，仅复验受影响版本；完整 A1/B1 运行时同步录制，不能用局部片段替代全簇。完成后刷新 R10→R11→R12 的真实结果和学习材料，并同步 GitHub；未满足原条件继续标 PARTIAL，不运行六十次正式矩阵。','','## 未完成项','']+['- '+x for x in (list(dict.fromkeys(read(APP/'results/R10/case_summary.json')['pending']+s['pending'])) or ['本阶段声明范围内无未完成项。'])]
lines+=['','下一轮两项优先修复：结合 `R10/docking_diagnosis.json` 区分点喷的实测静止、全局修正抖动和目标位置误差，保留 GNSS 创新监督；结合 `R10/mowing_start_diagnosis.json` 修复割草 START_WORK 等待阶段对 BT CANCEL/WAIT 的及时响应，并专门复验开刀时感知中断。不得因一个恢复事件或五段录像通过而省略剩余原始段。']
(DOC/'本轮推进结果_R08-R12_20261114.md').write_text('\n'.join(lines)+'\n')
print(statuses)
