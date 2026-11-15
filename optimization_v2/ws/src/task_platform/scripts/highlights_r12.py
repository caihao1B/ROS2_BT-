"""Choose real recorded event intervals; do not manufacture missing behavior."""
import math

def select(case,events,frames,analysis):
 def first(name,after=-1,**match):return next((e for e in events if e['event']==name and e['t']>after and all(e.get(k)==v for k,v in match.items())),None)
 start=end=None;label=None
 if case=='spray_full_A1':
  recovered=first('LOCALIZATION_REVALIDATED_RESUME')
  request=next((e for e in reversed(events) if recovered and e['event']=='STOPPED_RELOCALIZATION_REQUEST' and e['t']<recovered['t'] and e.get('point')==recovered.get('point')),None)
  if request:
   start,end,label=request['t']-4,recovered['t']+5,'定位异常停车 / 独立候选与路径重验 / 恢复原任务'
  else:
   dock=first('LOCAL_DOCK_STOP_FOR_GLOBAL_OBSERVATION')
   done=first('TOOL_RESULT',dock['t'],point=dock.get('point'),state='SUCCEEDED',replayed=False) if dock else None
   if done:start,end,label=dock['t']-5,done['t']+3,'局部精停 / 全局稳定性复核 / 工具反馈确认'
 elif case=='spray':
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
 elif case=='mowing_startup_perception_loss':
  interrupted=first('BT_INTERRUPTS_START_WORK')
  stopped=first('BOTH_PHYSICAL_STOPS_CONFIRMED',interrupted['t']) if interrupted else None
  work=first('TOOL_WORKING_CONFIRMED',stopped['t']) if stopped else None
  if work:start,end,label=interrupted['t']-3,work['t']+5,'启动中感知中断 / BT取消 / 实测停刀 / 恢复原义务'
 elif case.startswith('mowing'):
  rejoin=first('COVERAGE_REJOIN_CONFIRMED') if case=='mowing_full_B1' else None
  search=first('ORIGINAL_WORK_WINDOW_RECHECK',segment=rejoin['segment']) if rejoin else None
  endpoint_recovery=None
  for done in events:
   if case!='mowing_full_B1' or done['event']!='ENDPOINT_RECOVERY_SUCCESS':continue
   stalls=[e for e in events if e['event']=='ENDPOINT_POSE_STALL' and e.get('segment')==done.get('segment') and e['t']<done['t']]
   if stalls and any(e['event']=='RECOVERY_GEAR_ARMED' and stalls[-1]['t']<e['t']<done['t'] for e in events):
    endpoint_recovery=(stalls[-1],done);break
  turn=first('SEGMENT_GOAL',kind='TURN_TOOL_OFF')
  work=first('TOOL_WORKING_CONFIRMED',turn['t']) if turn else None
  stop=next((e for e in reversed(events) if e['event']=='BOTH_PHYSICAL_STOPS_CONFIRMED'),None)
  if rejoin and search:
   start,end,label=search['t']-5,rejoin['t']+6,'受限地形 / 实际停刀绕行 / 接回原条带 / 保留残余'
  elif endpoint_recovery:
   start,end,label=endpoint_recovery[0]['t']-4,endpoint_recovery[1]['t']+5,'末端姿态未达标 / 关刀有界恢复 / 实测到达'
  elif turn and (work or stop):
   start,end=turn['t']-4,(work or stop)['t']+4
   label='关刀换行 / 停车选挡 / 开始下一条带' if work else '关刀转场 / 实际停止 / 保留未完成任务'
 if start is None:return None
 start=max(start,frames[0]['sim_t']);end=min(end,frames[-1]['sim_t'])
 a=next(f for f in frames if f['sim_t']>=start);b=next((f for f in frames if f['sim_t']>=end),frames[-1]);duration=b['video_s']-a['video_s']
 if duration<=0:return None
 return dict(start_sim=a['sim_t'],end_sim=b['sim_t'],start_video_s=a['video_s'],duration_video_s=duration,speed=max(1,math.ceil(duration/45)),label=label,selection='actual recorded events; no synthesized scene motion or behavior')
