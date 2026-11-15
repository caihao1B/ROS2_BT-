import os,json,time,subprocess,signal,tempfile
from pathlib import Path
import rclpy
from std_msgs.msg import String
from rosgraph_msgs.msg import Clock
from builtin_interfaces.msg import Time
def main():
 root=Path('/home/ceihao/robot_project');app=root/'optimization_v2/three_tasks';cfg=json.loads((app/'editions/R09/tool.json').read_text());cfg['session']='component-timing';out=app/'results/R09/tool_timing';out.mkdir(exist_ok=True);(out/'config.json').write_text(json.dumps(cfg));log=(out/'server.log').open('w');p=subprocess.Popen([str(app/'build/r09/r09_tool_action_server'),'--ros-args','-p','config_file:='+str(out/'config.json')],stdout=log,stderr=log,start_new_session=True)
 rclpy.init();n=rclpy.create_node('r09_timing_test');cp=n.create_publisher(Clock,'/clock',10);fp=n.create_publisher(String,'/s10/tool/feedback',30);states=[];n.create_subscription(String,'/s10/action/state',lambda m:states.append(json.loads(m.data)),10);checks={}
 def spin(sec):
  end=time.monotonic()+sec
  while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.01)
 def clock(t):cp.publish(Clock(clock=Time(sec=int(t),nanosec=round((t-int(t))*1e9))));spin(.04)
 def feedback(t,**kwargs):
  d=dict(t=t,tool_revision=cfg['tool_revision'],sensor_epoch=cfg['session'],rpm_rad_s=0.,healthy=True,request_on=False);d.update(kwargs);fp.publish(String(data=json.dumps(d)));spin(.04)
 def fresh():return bool(states and states[-1]['feedback_fresh'])
 try:
  spin(1.5);clock(1.);feedback(1.05);checks['future_not_accepted_before_clock']=not fresh();clock(1.05);checks['same_feedback_accepted_when_clock_arrives']=fresh();spin(.53);checks['wall_age_not_reset_by_buffer']=not fresh();clock(2.);feedback(1.8);checks['stale_sensor_time_rejected']=not fresh();feedback(2.,sensor_epoch='wrong');checks['foreign_session_rejected']=not fresh();clock(2.05);feedback(2.05);checks['valid_current_feedback_recovers']=fresh()
 finally:
  n.destroy_node();rclpy.try_shutdown();p.send_signal(signal.SIGINT);p.wait(timeout=5);log.close()
 r=dict(passed=all(checks.values()) and p.returncode==0,checks=checks,process_exit=p.returncode,scope='Native receiver component timing test on isolated ROS domain 123; synthetic feedback, no physical task credit');(out/'result.json').write_text(json.dumps(r,indent=2));print(json.dumps(r));raise SystemExit(0 if r['passed'] else 2)
if __name__=='__main__':main()
