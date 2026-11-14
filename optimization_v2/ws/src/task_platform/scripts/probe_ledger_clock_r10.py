"""Actual ROS ledger heartbeat at 0.1 RTF, then frozen /clock; no actuators."""
from common import *
import time,signal,statistics
import rclpy
from std_msgs.msg import String
from rosgraph_msgs.msg import Clock
from rclpy.qos import qos_profile_sensor_data

def main():
 if os.environ.get('ROS_DOMAIN_ID')!='134':raise ValueError('ISOLATED_DOMAIN_134_REQUIRED')
 out=APP/'results/R10/ledger_clock_probe';out.mkdir(parents=True,exist_ok=True)
 rclpy.init();node=rclpy.create_node('r10_ledger_clock_probe');pub=node.create_publisher(Clock,'/clock',qos_profile_sensor_data);received=[]
 sub=node.create_subscription(String,'/s10/ledger/state',lambda m:received.append(dict(wall=time.monotonic(),**json.loads(m.data))),20);cases=[]
 try:
  for variant,script in [('original','ledger_r09.py'),('steady','ledger_mowing_r10.py')]:
   folder=out/(variant+'_'+stamp());folder.mkdir();received.clear()
   with (folder/'process.log').open('w') as log:
    child=subprocess.Popen([str(BASE/'venvs/scene/bin/python'),str(PKG/'scripts'/script),'--out',str(folder),'--config',str(edition('R10')/'mowing/tool.json')],stdout=log,stderr=log)
    begin=time.monotonic()
    try:
     while time.monotonic()-begin<7.:
      elapsed=time.monotonic()-begin;t=min(elapsed,5.)*.1
      msg=Clock();ns=int(t*1e9);msg.clock.sec=ns//10**9;msg.clock.nanosec=ns%10**9;pub.publish(msg);rclpy.spin_once(node,timeout_sec=.01);time.sleep(.01)
      if child.poll() is not None:raise RuntimeError('LEDGER_PROBE_CHILD_EXITED')
    finally:
     child.send_signal(signal.SIGINT)
     try:child.wait(timeout=5)
     except subprocess.TimeoutExpired:child.kill();child.wait();raise
   active=[r for r in received if 2.<r['wall']-begin<5.];gaps=[b['wall']-a['wall'] for a,b in zip(active,active[1:])]
   cases.append(dict(variant=variant,samples=len(active),median_wall_gap_s=statistics.median(gaps) if gaps else None,max_wall_gap_s=max(gaps,default=999.),clock_watchdog_during_pause=any(r['wall']-begin>6. and r.get('start_guard')=='CLOCK_WATCHDOG' for r in received),exit=child.returncode,source_sha256=sha(PKG/'scripts'/script)))
   dump(folder/'received.json',received)
 finally:node.destroy_subscription(sub);node.destroy_node();rclpy.try_shutdown()
 old,new=cases;checks=dict(original_sim_clock_reproduces_stale_period=old['samples']>=4 and old['median_wall_gap_s']>.25,steady_heartbeat_below_original_watchdog=new['samples']>=30 and new['max_wall_gap_s']<.2,paused_clock_fails_closed=new['clock_watchdog_during_pause'],clean_exit=all(c['exit']==0 for c in cases))
 result=dict(passed=all(checks.values()),checks=checks,cases=cases,original_stale_limit_wall_s=.25,scope='Actual ROS nodes with synthetic clock only; no Gazebo, tool action, motion or work credit',policy_sha256=sha(edition('R10')/'mowing/tool.json'))
 dump(out/'result.json',result);print(json.dumps(result));return 0 if result['passed'] else 2
if __name__=='__main__':raise SystemExit(main())
