"""Actual isolated C++ policy probe; synthetic inputs, no Gazebo or work credit."""
import json,time,subprocess,signal,os
from pathlib import Path
import rclpy
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from common import ROOT,APP,edition,dump,sha

def run():
 out=APP/'results/R10/global_filter_probe';out.mkdir(parents=True,exist_ok=True)
 policy=edition('R10')/'spray/manager.json';cfg=json.loads(policy.read_text());binary=APP/'build/r10_global/r10_global_manager';cases={}
 for name in ['gnss_conflict','raw_map_jump']:
  rclpy.init();n=rclpy.create_node('r10_isolated_policy_probe');pubs={key:n.create_publisher(cls,topic,20) for key,cls,topic in [('clock',Clock,'/clock'),('odom',Odometry,'/s05/odometry'),('map',String,'/s08/map'),('gnss',String,'/s08/gnss'),('contract',String,'/s08/contract')]};status=[];n.create_subscription(String,'/s08/status',lambda m:status.append(json.loads(m.data)),20)
  log=(out/(name+'.log')).open('w');p=subprocess.Popen([str(binary),'--ros-args','-p','use_sim_time:=true','-p','policy_file:='+str(policy)],stdout=log,stderr=subprocess.STDOUT,cwd=ROOT,start_new_session=True);started=time.monotonic();ready=False;fault_at=None;t=0.
  try:
   while time.monotonic()-started<15:
    t=round(t+.05,6);c=Clock();c.clock.sec=int(t);c.clock.nanosec=round((t-int(t))*1e9);pubs['clock'].publish(c)
    m=Odometry();m.header.stamp=c.clock;m.header.frame_id='odom';m.child_frame_id='base_link';m.pose.pose.orientation.w=1.;pubs['odom'].publish(m)
    fault=fault_at is not None
    for key,d in [('contract',dict(t=t,valid=True,map_id=cfg['map_id'])),('map',dict(t=t,valid=True,map_id=cfg['map_id'],pose=[.2 if fault and name=='raw_map_jump' else 0.,0.,0.,0.,0.,0.,1.])),('gnss',dict(t=t,valid=True,mode='fixed',covariance=[.0004,0.,0.,0.,.0004,0.,0.,0.,.0004],antenna=[.25 if fault and name=='gnss_conflict' else 0.,0.,0.],antenna_base_m=[0.,0.,0.]))]:pubs[key].publish(String(data=json.dumps(d)))
    until=time.monotonic()+.05
    while time.monotonic()<until:rclpy.spin_once(n,timeout_sec=.005)
    if not ready and status and status[-1]['navigation_allowed'] and status[-1]['state']=='TRUSTED_MAP':ready=True;fault_at=t
    if fault_at is not None and t-fault_at>1.2:break
   last=status[-1] if status else {}
   fault_status=[s for s in status if fault_at is not None and s['t']>fault_at+.05]
   # Cross-topic DDS delivery may leave STALE_OR_REORDERED as the final reason.
   # Require a real innovation rejection, then continuous GNSS exclusion;
   # inspecting only the final diagnostic string confuses this with a bypass.
   if name=='gnss_conflict':
    index=next((i for i,s in enumerate(fault_status) if s.get('gnss_reason')=='INNOVATION_REJECTED_FIXED' and s.get('innovation_m',0)>=.24 and s.get('gps_count')==0),None)
    witness=fault_status[index] if index is not None else None;after=fault_status[index:] if index is not None else []
    checks=dict(initial_trusted=ready,actual_raw_innovation_rejected=witness is not None,gnss_stays_excluded=bool(after) and all(s.get('gps_count')==0 and s.get('source')!='GNSS_POSITION_LOCAL_HEADING' for s in after))
   else:
    index=next((i for i,s in enumerate(fault_status) if s.get('plan_revalidation_required') and s.get('jump_count',0)>0),None)
    witness=fault_status[index] if index is not None else None;after=fault_status[index:] if index is not None else []
    checks=dict(initial_trusted=ready,actual_raw_jump_latches=witness is not None,hold_never_permits_motion=bool(after) and all(s.get('plan_revalidation_required') and not s.get('navigation_allowed') for s in after))
   cases[name]=dict(passed=all(checks.values()),checks=checks,initial_trusted=ready,fault_at=fault_at,first_fault_witness=witness,verified_fault_status_count=len(after),last=last)
   # Multiple wall-timer publications can share one simulation timestamp.
   # Preserve receive order, including the pre-fault sample at that same stamp.
   dump(out/(name+'_status.json'),status)
  finally:
   p.send_signal(signal.SIGINT)
   try:p.wait(timeout=5)
   except subprocess.TimeoutExpired:p.kill();p.wait();cases.setdefault(name,{})['passed']=False
   log.close();n.destroy_node();rclpy.shutdown()
  cases[name]['clean_exit']=p.returncode==0;cases[name]['passed']=cases[name]['passed'] and p.returncode==0
 result=dict(passed=all(c['passed'] for c in cases.values()),cases=cases,binary_sha256=sha(binary),policy_sha256=sha(policy),probe_sha256=sha(Path(__file__)),domain=os.environ.get('ROS_DOMAIN_ID'),scope='Synthetic isolated C++ policy test; no real sensor, navigation or work completion credit');dump(out/'result.json',result);print(json.dumps(result));return 0 if result['passed'] else 2
if __name__=='__main__':raise SystemExit(run())
