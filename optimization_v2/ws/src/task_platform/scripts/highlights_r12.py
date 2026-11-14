"""Choose real recorded event intervals; do not manufacture missing behavior."""
import math

def select(case,events,frames,analysis):
 def first(name,after=-1,**match):return next((e for e in events if e['event']==name and e['t']>after and all(e.get(k)==v for k,v in match.items())),None)
 start=end=None;label=None
 if case=='spray':
  fault=first('TOOL_RESULT',reason='TOOL_HEALTH');retry=first('RESIDUAL_RETRY',fault['t']) if fault else None;done=first('TOOL_RESULT',retry['t'],state='SUCCEEDED') if retry else None
  if done:start,end,label=fault['t']-4,done['t']+4,'工具故障 / 关停 / 按账本补作残余'
 elif case=='inspection':
  admitted=first('OBSERVED_PATCH_ADMITTED');rejoin=first('PATCH_REJOIN_CONFIRMED',admitted['t']) if admitted else None
  if rejoin:start,end,label=admitted['t']-8,rejoin['t']+5,'传感器检测 / 实际重规划 / 绕行后接回'
 elif case=='inspection_leader':
  checks=analysis['checks']
  if all(checks.get(k) for k in ['measured_leader_stop','measured_follower_stop','measured_resume','physical_follow_clearance']):start,end,label=55.,90.,'跟随前车 / 前车停止后停车 / 恢复跟随'
 elif case=='inspection_blocked':
  checks=analysis['checks'];stop=first('TASK_STOP',terminal='HUMAN_REQUIRED');hold=first('BT_INTERRUPTS_PATH')
  if stop and hold and all(checks.get(k) for k in ['expected_controlled_hold','residual_preserved','bounded_attempts','stopped']):start,end,label=hold['t']-4,stop['t']+2,'持续受阻 / 有限规划查询 / 停车请求人工'
 elif case.startswith('mowing'):
  turn=first('SEGMENT_GOAL',kind='TURN_TOOL_OFF')
  work=first('TOOL_WORKING_CONFIRMED',turn['t']) if turn else None
  stop=next((e for e in reversed(events) if e['event']=='BOTH_PHYSICAL_STOPS_CONFIRMED'),None)
  if turn and (work or stop):
   start,end=turn['t']-4,(work or stop)['t']+4
   label='关刀换行 / 停车选挡 / 开始下一条带' if work else '关刀转场 / 实际停止 / 保留未完成任务'
 if start is None:return None
 start=max(start,frames[0]['sim_t']);end=min(end,frames[-1]['sim_t'])
 a=next(f for f in frames if f['sim_t']>=start);b=next((f for f in frames if f['sim_t']>=end),frames[-1]);duration=b['video_s']-a['video_s']
 if duration<=0:return None
 return dict(start_sim=a['sim_t'],end_sim=b['sim_t'],start_video_s=a['video_s'],duration_video_s=duration,speed=max(1,math.ceil(duration/45)),label=label,selection='actual recorded events; no synthesized scene motion or behavior')
