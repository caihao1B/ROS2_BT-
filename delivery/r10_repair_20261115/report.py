"""Current evidence reports; previous release and videos remain available."""
from pathlib import Path
import json
ROOT=Path(__file__).resolve().parents[2];APP=ROOT/'optimization_v2/three_tasks';DOC=ROOT/'docs/three_tasks'
def read(p):return json.loads(p.read_text())
def write(p,lines):p.write_text('\n'.join(lines)+'\n')

r10=read(APP/'results/R10/case_summary.json');r12=read(APP/'results/R12/latest.json');r11=read(APP/'results/R11/latest.json')
status={s:read(APP/'results'/s/'latest.json')['automated_status'] for s in ['R08','R09','R10','R11','R12']}
table=['|案例|结果|仿真秒|实际完成与残余|终止原因|','|---|---|---:|---|---|']
for c in r10['cases']:
 amount=[]
 if c.get('selected_points') is not None:amount.append(f"{c['confirmed_points']}/{c['selected_points']} 点（原 {c['original_points']}）")
 if c.get('selected_segments') is not None:amount.append(f"{c['confirmed_segments']}/{c['selected_segments']} 段（原 {c['original_segments']}）")
 if c.get('confirmed_area_m2') is not None:amount.append(f"确认 {c['confirmed_area_m2']:.3f} m²；残余 {c['residual_area_m2']:.3f} m²")
 if c.get('remaining_original_route_m') is not None:amount.append(f"原路径剩余 {c['remaining_original_route_m']:.3f} m")
 table.append(f"|{c['case']}|{'PASS' if c['passed'] else '未通过'}|{c.get('simulation_seconds')}|{'；'.join(amount)}|{c.get('reason') or c.get('terminal')}|")
changes=['全局修正采用独立 C++ 低通处理，原始 GNSS 创新、地图跳变、时间有效性和停止锁存先于滤波，精停门限保持原值。', '割草 START_WORK 等待每次处理 BT 中断；取消优先于同次工作确认，确认车体和刀具停止后才依据新的 BT 指令恢复。同一条带的幂等键、残余和重试预算保留。','割草账本状态发布改用 STEADY_TIME，匹配工具服务器原 0.25 墙钟秒监督；仿真时间戳、反馈配对、事务和有效作业几何不变。','原地图、任务点和 34 段条带分母不变；六十次矩阵未执行，原 S17/S18 保持 PARTIAL。']
lines=['# R10 修复与实际复验','',f"**{status['R10']}**。当前完整任务和专项故障的实测结果如下。",'']+table+['','## 实现与原门限','']+['- '+c for c in changes]
probe=r10.get('global_policy_probe',{})
lines+=['',f"独立 C++ 全局管理器探针：{'PASS' if probe.get('passed') else '未通过'}；25 厘米 GNSS 冲突拒绝、20 厘米原始地图跳变触发停止锁存。它使用合成消息，只验证监督逻辑，不算真实传感器任务完成。",'','## 未完成项','']+['- '+p for p in r10['pending']]
if not r10['pending']:lines+=['本阶段声明范围内无未完成项；不能外推其他簇或实物作业效果。']
for c in r10['cases']:
 if c.get('endpoint_recovery_successes'):lines+=['',f"完整 B1 本次记录 {len(c['endpoint_recovery_successes'])} 次实际端点恢复成功；最终是否完成仍以全部原始段记录为准。"]
diag_path=APP/'results/R10/global_hold_diagnosis.json'
if diag_path.exists():
 diag=read(diag_path);lines+=['','## 当前完整 A1 的新增停止条件','',f"本次完成 {diag['completed_points']}/{diag['original_points']} 点，定位 RMSE {diag['map_xy_rmse_m']:.5f} 米；{diag['first_hold_t']} 秒触发 `{diag['raw_map_reason']}`，进入 `{diag['reason']}`。前后有效地图观测相隔 {diag['map_gap_s']:.2f} 秒，GNSS 当时仍通过独立监督。",'近邻记录的二维重建表明修正角度变化接近原 0.035 弧度门限；缺少被拒绝候选的同步三维记录，不能把这次锁存完全归因于滤波滞后，也不能凭该近似证明滤波没有影响。下一步应记录精确候选与配对里程计，再验证观测恢复和生产计划重验；不直接清除锁存。','详细记录：`optimization_v2/three_tasks/results/R10/global_hold_diagnosis.json`。']
endpoint_path=APP/'results/R10/endpoint_tracking_diagnosis.json'
if endpoint_path.exists():
 ep=read(endpoint_path);lines+=['','## 当前完整 B1 的新增停止条件','',f"当前版本完成 {ep['completed_segments']}/{ep['original_segments']} 段，确认 {ep['confirmed_area_m2']:.3f} m²、残余 {ep['residual_area_m2']:.3f} m²；原 90 秒恢复预算耗尽后以 `{ep['terminal']}` 停稳。首条连续恢复候选长度 {ep['first_continuous_path_length_m']:.3f} 米，但传感器定位显示最大偏离候选约 {ep['maximum_observed_distance_to_candidate']['distance_m']:.3f} 米。", '这是已通过几何/代价检查的候选未被控制器正确执行，不是已证明无路可走。后续需要检查 MPPI 换向点处理、按挡位分段的实测停车边界及轨迹偏离监督。未放宽到达、曲率、整车边界或预算门限。']
startup_path=APP/'results/R10/startup_recovery_diagnosis.json'
if startup_path.exists():
 st=read(startup_path);lines+=['','## START_WORK 当前版本实测','',f"BT 中断后 {st['startup_stop_delay_sim_s']:.3f} 仿真秒确认车体和刀具停止（原上限 {st['original_limit_sim_s']} 秒）；恢复后完成 {st['completed_segments']} 段，账本仅有原来的 {st['unique_jobs']} 个键，无不确定性。确认面积 {st['confirmed_area_m2']:.3f} m²、选择范围残余 {st['residual_area_m2']:.3f} m²。它是五段专项通过，不是完整 B1 通过。"]
clock=r10.get('ledger_clock_probe',{})
lines+=['','## 账本时间基准复验','',f"隔离 ROS 时钟探针：{'PASS' if clock.get('passed') else '未通过'}。原版仿真定时发布与工具墙钟超时存在基准不一致；新版只切换心跳时钟。真实失败尝试、已确认量和不确定性保留在 `results/R10/repair_attempts.json`，该尝试不计作成功。"]
lines+=['','## 可视证据','', '本机诊断图：`optimization_v2/three_tasks/results/R10/repair_docking.png`、`repair_start_work.png`；GitHub 对应 `media/r10_repair_docking.png`、`media/r10_repair_start_work.png`。', '', '图表只解释对应实测窗口，不能代替完整任务验收。']
lines+=['','## 证据入口','']+[f"- `{c['case']}`：`{c['path']}`。" for c in r10['cases']]
lines+=['','上一轮定位抖动和启动超时诊断仍保留为历史输入；当前故障原因以本表对应新运行的事件、回执为准。学习说明见 TECHNICAL.md、CODE.md；启动和退出见 MANUAL.md。'];write(DOC/'R10/RESULTS.md',lines)

lines=['# R12 当前真实录像','',f"整体 **{status['R12']}**；三任务录像组件：{'PASS' if r12['summary']['recording_components_passed'] else '未全部通过'}。",'','|案例|任务检查|录像秒|仿真终时|全部检查|','|---|---|---:|---:|---|']
for name,d in r12['summary']['cases'].items():
 saved=read(ROOT/d['path']/'recording_acceptance.json');label='PASS' if d.get('task_passed') else ('原回执 PASS；当前版本复验未通过' if saved.get('passed') else '未通过')
 lines.append(f"|{name}|{label}|{d.get('video_seconds')}|{d.get('sim_seconds')}|{'PASS' if d.get('passed') else '未通过'}|")
lines+=['','三点点喷和五段割草是独立小规模演示，不代替完整 A1/B1。原片保留实际等待、重试、失败和终止；精简视频注明倍速。上轮通过的原片仍在原运行目录，未删除或改写。当前版本是否合格以当前回执为准。','','## 当前未完成项','']+['- '+p for p in r12['summary']['pending']]
lines+=['','## 原片','']+[f"- `{name}`：`{d['path']}/video/full.mp4`。" for name,d in r12['summary']['cases'].items()]
lines+=['','本机播放：`xdg-open optimization_v2/three_tasks/results/R12/media/index.html`。录制和退出命令见 MANUAL.md。完整数据库和构建环境保留本机，GitHub 提供源码、配置、精简证据与媒体；尚不等于可移植安装包。'];write(DOC/'R12/RESULTS.md',lines)
write(DOC/'R11/RESULTS.md',['# R11 材质对照','',f"整体 **{status['R11']}**；材质与成对渲染组件：{'PASS' if r11['summary']['components_passed'] else '未通过'}。",'', '几何、碰撞、传感器和物理参数相同的原始/材质场景对照保留；仅做外观材质，不是精细商品模型。任务源码变化通过 R12 新运行复验，不把停放渲染当成自主任务通过。','']+['- '+p for p in r11['summary']['pending']])
lines=['# R10 修复与完整任务复验（2026-11-15）','', '本轮按用户要求复验受影响版本和完整 A1/B1，不执行六十次矩阵。所有结论来自回执，不凭录像数量推定整体验收。','','|阶段|当前状态|','|---|---|']+[f'|{s}|{v}|' for s,v in status.items()]+['','## 本轮任务结果','']+table+['','## 修复内容','']+['- '+c for c in changes]
lines+=['','## 手工检查','', '```bash','cd /home/ceihao/robot_project','bash optimization_v2/three_tasks/advance.sh status','bash optimization_v2/three_tasks/advance.sh R10 check','bash optimization_v2/three_tasks/advance.sh R12 check','xdg-open optimization_v2/three_tasks/results/R12/media/index.html','```','','逐项重跑与退出方法见 [R10 手工命令](R10/MANUAL.md)、[R12 录制命令](R12/MANUAL.md)；学习材料见 [技术原理](R10/TECHNICAL.md)、[核心代码](R10/CODE.md)。','','## 当前缺项','']+['- '+p for p in dict.fromkeys(r10['pending']+r12['summary']['pending'])]
lines+=['','## 下一步判断','', '当前完整任务仍有未通过项时，暂不能认定 R10–R12 全部完成；可以检查当前通过的独立演示。后续先记录 A1 被拒绝的精确 SE(3) 候选和配对里程计，再完善停车后的定位恢复与计划重验，修复 B1 第七段末端恢复的换向点跟踪及轨迹偏离监督。保留所有原门限、作业分母和未确认量，不自动清除锁存。']
write(DOC/'R10修复与完整任务复验_20261115.md',lines)
print(json.dumps(status))
