"""ROS protocol fixture. It has no actuator publisher and grants no real work credit."""
import sys,argparse,json,time,signal
from pathlib import Path
import rclpy
from std_msgs.msg import String
from rosgraph_msgs.msg import Clock
from std_srvs.srv import Trigger

def main():
 ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path,required=True);ap.add_argument('--config',type=Path,required=True);a=ap.parse_args();cfg=json.loads(a.config.read_text());rclpy.init();n=rclpy.create_node('r06_protocol_fixture');cp=n.create_publisher(Clock,'/clock',10);ctx=n.create_publisher(String,'/r06/context',10);reset=n.create_client(Trigger,'/r06/acknowledge_manual_hold');last={};counts={};received=[];seq=0;t=0.;log=(a.out/'protocol.jsonl').open('w')
 def response(m):
  d=json.loads(m.data);last.clear();last.update(d);received.append(d);log.write(m.data+'\n')
 n.create_subscription(String,'/r06/directive',response,10)
 base=dict(stamp=0.,sequence=0,task_version=cfg['task_version'],vehicle_hash=cfg['vehicle_hash'],rig_revision=cfg['rig_revision'],session=cfg['session'],localized=True,sensors_healthy=True,remaining_s=600.,stopped=True,tool_stopped=True,nav_idle=True,blocked=False,progress_m=0.,commanded_motion=False,diagnostic=True)
 cases=[]
 def segment(name,duration,patch,expected,send=True):cases.append(dict(name=name,duration=duration,patch=patch,expected=expected,send=send))
 segment('normal',2,{},'EXECUTE');segment('obstacle_short',2,{'blocked':True},'WAIT');segment('cancel_before_wait',1,{'blocked':True,'nav_idle':False,'stopped':False,'tool_stopped':False},'CANCEL');segment('validated_detour',2,{'blocked':True,'detour_valid':True,'detour_s':1.},'DETOUR');segment('wait_for_persistence',8,{'blocked':True},'REPLAN');segment('planner_exhausted_other_job',2,{'blocked':True,'replans':2,'alternative':True},'DEFER');segment('no_verified_alternative',2,{'blocked':True,'replans':2},'HUMAN');segment('manual_latch_survives_clear',2,{},'HUMAN');segment('ack_reset',2,{},'EXECUTE');segment('follow_leader',3,{'blocked':True,'leader_on_path':True,'leader_same_direction':True,'leader_confidence':.9,'leader_speed':.3,'clearance_m':3.,'ttc_s':10.,'path_valid':True},'FOLLOW');segment('crossing_not_follow',2,{'blocked':True,'leader_on_path':True,'leader_same_direction':False,'leader_confidence':.9,'leader_speed':.3,'clearance_m':3.,'ttc_s':10.,'path_valid':True},'WAIT');segment('emergency',2,{'unsafe':True},'STOP');segment('stale_context',2,{},'STOP',False);segment('wrong_version',2,{'task_version':'invalid'},'STOP');segment('replayed_sequence',2,{},'STOP');segment('future_timestamp',2,{},'STOP');segment('cancel_timeout',4,{'localized':False,'nav_idle':False,'stopped':False,'tool_stopped':False},'STOP');segment('ack_after_timeout',2,{},'EXECUTE');segment('motion_stall',12,{'commanded_motion':True,'progress_m':0.},'HUMAN');segment('budget_reserve',2,{'remaining_s':1.},'STOP');segment('final_ack',2,{},'EXECUTE');segment('feedback_complete',2,{'complete':True},'COMPLETE')
 deadline=time.monotonic()+15
 while time.monotonic()<deadline and not ctx.get_subscription_count():rclpy.spin_once(n,timeout_sec=.05)
 results=[]
 try:
  for case in cases:
   start=t;observed=[];ack=None
   while t-start<case['duration']:
    t=round(t+.05,8);clk=Clock();clk.clock.sec=int(t);clk.clock.nanosec=int(round((t-int(t))*1e9));cp.publish(clk);rclpy.spin_once(n,timeout_sec=.01)
    if case['send']:
     seq+=1;e=dict(base,stamp=t,sequence=seq);e.update(case['patch']);
     if case['name']=='replayed_sequence':e['sequence']=1
     if case['name']=='future_timestamp':e['stamp']=t+1.
     m=String();m.data=json.dumps(e);ctx.publish(m)
    end=time.monotonic()+.035
    while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.003)
    if 'ack' in case['name'] and t-start>.3 and ack is None and reset.service_is_ready():ack=reset.call_async(Trigger.Request())
    if t-start>case['duration']-.7 and last:observed.append(dict(action=last['action'],reason=last['reason_code'],selected=last.get('selected_action'),cap=last.get('speed_cap_mps'),confirmed=last.get('stopped_confirmation')))
   ok=bool(observed) and all(x['action']==case['expected'] for x in observed)
   if case['name']=='follow_leader':ok &= all(x['cap']<=.300001 for x in observed if x['action']=='FOLLOW')
   if case['name']=='cancel_before_wait':ok &= all(x['action']!='WAIT' for x in observed)
   results.append(dict(name=case['name'],expected=case['expected'],passed=ok,observed=observed[-5:],start=start,end=t,ack_success=ack.result().success if ack and ack.done() else None))
 finally:
  log.close();(a.out/'protocol_result.json').write_text(json.dumps(dict(passed=all(x['passed'] for x in results),cases=results,scope='Declared synthetic context and cancellation fixture over ROS; no Gazebo/vision interaction claim'),indent=2));n.destroy_node();rclpy.try_shutdown()
 return 0 if all(x['passed'] for x in results) else 2
if __name__=='__main__':raise SystemExit(main())
