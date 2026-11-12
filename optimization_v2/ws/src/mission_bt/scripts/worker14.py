"""Reuse reviewed Nav2 client primitives; replace only the task orchestrator.
All exact-text adaptations fail closed on an upstream source change.
"""
from common14 import *

def adapted_source():
    s=(NAV/'scripts/client13.py').read_text()
    s=replace_once(s,'from common13 import *','from common13 import *\nfrom binding14 import active_plan as plan')
    s=replace_once(s,"def main():", "from mission14 import Mission\n\ndef main():")
    s=replace_once(s," def tick():\n", " mission=None\n def tick():\n")
    s=replace_once(s," def revalidate():\n", " def revalidate():\n  if os.environ.get('S14_REGISTERED_REGION'):\n   current=plan()\n   if current['plan_revision']!=m['plan_revision']:raise ValueError('REGIONAL_PLAN_CHANGED')\n")
    s=replace_once(s,"  rclpy.spin_once(n,timeout_sec=.005)\n", "  rclpy.spin_once(n,timeout_sec=.005)\n  if mission is not None:mission.pulse()\n")
    s=replace_once(s,"if st['t']>=a.seconds-6:raise TimeoutError('CASE_SIM_DEADLINE')", "if st['t']>=a.seconds-6 and not (mission and mission.stopping):raise TimeoutError('CASE_SIM_DEADLINE')")
    s=replace_once(s,"curvature=curvature,action_id=action_id,plan_revision=m['plan_revision']", "curvature=curvature,action_id=action_id,plan_revision=m['plan_revision'],**(mission.lease_fields() if mission else {}))"[:-1])
    s=replace_once(s,"and tool.get('stopped_confirmed') and not tool.get('active') and st['pose']", "and ((mission is not None and mission.work_permission()) or (tool.get('stopped_confirmed') and not tool.get('active'))) and st['pose']")
    start=s.index(" try:\n  if a.case.startswith('calibration_')")
    end=s.index(" except BaseException as e:log('error'",start)
    body=''' def cancel_active():
  nonlocal active
  active=False
  if current_handle is not None and current_result is not None and not current_result.done():
   current_handle.cancel_goal_async();wait(current_result.done,5,15)
  wait(stopped,5,15)
 def record_nav_plan(msg):
  st['nav_plan']=dict(received_t=st['t'],action_id=action_id,frame=msg.header.frame_id,poses=[[p.pose.position.x,p.pose.position.y,yaw(p.pose.orientation)] for p in msg.poses])
  with (a.out/'planned_paths.jsonl').open('a') as output:
   output.write(json.dumps(dict(t=st['t'],frame=msg.header.frame_id,stamp=stamp(msg),poses=[[p.pose.position.x,p.pose.position.y,yaw(p.pose.orientation)] for p in msg.poses]))+'\\n')
 n.create_subscription(ROSPath,'/plan',record_nav_plan,2)
 def record_navigation_decision(msg):
  d=json.loads(msg.data);st['navigation_decision']=d;log('navigation_decision',decision=d)
 n.create_subscription(String,'/s14/navigation_decision',record_navigation_decision,10)
 mission=Mission(a,st,n,dict(tick=tick,wait=wait,delay=delay,stopped=stopped,admitted=admitted,costmaps_ready=costmaps_ready,xy=xy,align=align,execute=execute,follow=follow_resuming,revalidate=revalidate,cancel=cancel_active,log=log))
 try:
  completed=mission.run()
'''
    s=s[:start]+body+s[end:]
    s=replace_once(s," finally:\n  active=False", " finally:\n  if mission is not None:mission.shutdown()\n  active=False")
    return s

if __name__=='__main__':
    scope={'__name__':'__main__','__file__':str(NAV/'scripts/client13.py')}
    exec(compile(adapted_source(),str(Path(__file__)), 'exec'),scope)
