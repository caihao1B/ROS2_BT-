"""Production C++ node tests; synthetic controls in ROS domain123, no vehicle."""
import os,sys,time,subprocess,signal
from pathlib import Path
os.environ['ROS_DOMAIN_ID']='123';sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from common08 import *
import rclpy
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from rosgraph_msgs.msg import Clock
from std_srvs.srv import Trigger

def main(out):
 out=Path(out);out.mkdir(parents=True,exist_ok=True);cfg=yaml.safe_load((BASE/'config/localization_supervisor.yaml').read_text());cfg['map_id']='UNIT';dump(out/'policy.json',cfg);f=(out/'node.log').open('w');child=subprocess.Popen([str(BASE/'ws/install/gnss_supervisor/lib/gnss_supervisor/gnss_supervisor'),'--ros-args','-p','use_sim_time:=true','-p','policy_file:='+str(out/'policy.json')],stdout=f,stderr=subprocess.STDOUT)
 rclpy.init();n=rclpy.create_node('s08_unit');clock=n.create_publisher(Clock,'/clock',10);odom=n.create_publisher(Odometry,'/s05/odometry',20);contract=n.create_publisher(String,'/s08/contract',20);mp=n.create_publisher(String,'/s08/map',20);gp=n.create_publisher(String,'/s08/gnss',20);states=[];n.create_subscription(String,'/s08/status',lambda m:states.append(json.loads(m.data)),100);t=10.;checks={}
 def send(pub,d):m=String();m.data=json.dumps(d);pub.publish(m)
 def run(duration,map_mode='good',gps_mode='fixed',bias=0.,bound=True,vel=0.,bad_cov=False,old_stamp=False):
  nonlocal t
  end=t+duration;next_obs=0
  while t<end:
   t+=.05;c=Clock();c.clock.sec=int(t);c.clock.nanosec=int((t-int(t))*1e9);clock.publish(c);o=Odometry();o.header.stamp=c.clock;o.pose.pose.orientation.w=1.;o.twist.twist.linear.x=vel;odom.publish(o);send(contract,dict(t=t,valid=bound,map_id='UNIT'))
   if t>next_obs:
    next_obs=t+.2;send(mp,dict(t=t-.1,map_id='UNIT',valid=map_mode!='lost',pose=[1.+(1 if map_mode=='jump' else 0),0,.2 if map_mode=='zjump' else 0,0,0,math.sin(.05) if map_mode=='yawjump' else 0,math.cos(.05) if map_mode=='yawjump' else 1]))
    send(gp,dict(t=t-(2. if old_stamp else .1),valid=gps_mode!='lost',mode=gps_mode,antenna=[.9+bias,0,.8],antenna_base_m=[-.1,0,.8],covariance=[-.0009 if bad_cov else .0009,0,0,0,.0009,0,0,0,.002]))
   until=time.monotonic()+.025
   while time.monotonic()<until:rclpy.spin_once(n,timeout_sec=.003)
  return states[-1]
 try:
  run(1.,'lost','lost');checks['no_sources_stop']=not states[-1]['navigation_allowed']
  run(3.,bias=1.);checks['startup_conflict_no_admission']=not states[-1]['navigation_allowed']
  run(5.);checks['normal']=states[-1]['state']=='TRUSTED_MAP' and states[-1]['navigation_allowed']
  run(1.,gps_mode='fixed',bias=1.);checks['false_fixed_rejected']=states[-1]['state']=='MAP_ONLY' and states[-1]['gnss_reason']=='INNOVATION_REJECTED_FIXED' and states[-1]['max_speed']==.06
  run(1.,bad_cov=True);checks['negative_covariance_rejected']=states[-1]['gnss_reason']=='COVARIANCE_REJECTED' and states[-1]['state']=='MAP_ONLY'
  run(1.,old_stamp=True);checks['stale_gnss_rejected']=states[-1]['gnss_reason']=='STALE_OR_REORDERED' and states[-1]['state']=='MAP_ONLY'
  run(1.,gps_mode='float');checks['float_degraded']=states[-1]['state']=='MAP_ONLY'
  run(2.);run(2.,map_mode='lost');checks['map_loss_gnss_fallback']=states[-1]['state']=='GNSS_ONLY'
  run(7.,map_mode='lost');checks['heading_expiry_stop']=not states[-1]['navigation_allowed'] and states[-1]['reason']=='ABSOLUTE_HEADING_EXPIRED'
  run(3.);checks['recovery_hysteresis']=states[-1]['navigation_allowed']
  run(2.,map_mode='lost',gps_mode='lost');checks['dual_loss_stop']=not states[-1]['navigation_allowed']
  run(.5);checks['no_instant_recovery']=not states[-1]['navigation_allowed'];run(3.)
  run(.5,map_mode='jump');checks['map_jump_latches']=states[-1]['plan_revalidation_required'] and not states[-1]['navigation_allowed'];run(3.)
  client=n.create_client(Trigger,'/s08/acknowledge_diagnostic_hold');assert client.wait_for_service(timeout_sec=2);future=client.call_async(Trigger.Request());run(.5);checks['stationary_diagnostic_revalidation']=future.done() and future.result().success and states[-1]['navigation_allowed']
  for fault in ['zjump','yawjump']:
   before=states[-1]['correction'];run(.6,map_mode=fault);checks[fault+'_holds_without_tf_jump']=states[-1]['plan_revalidation_required'] and not states[-1]['navigation_allowed'] and np.allclose(states[-1]['correction'],before);run(3.);future=client.call_async(Trigger.Request());run(.5)
  run(.5,bound=False);checks['contract_revoked']=not states[-1]['navigation_allowed'];run(3.)
  # Paused clock: no callbacks published, wall watchdog still runs.
  until=time.monotonic()+.8
  while time.monotonic()<until:rclpy.spin_once(n,timeout_sec=.02)
  checks['clock_paused']=not states[-1]['navigation_allowed'];run(3.);t=1.;run(.5);checks['clock_rewind']=not states[-1]['navigation_allowed'] and states[-1]['reason']=='CLOCK_REWIND_NEW_SESSION_REQUIRED'
  dump(out/'result.json',dict(passed=all(checks.values()),checks=checks,states=states,domain=123,scope='Actual production binary; synthetic observations, no actuator'));print(json.dumps(checks));return 0 if all(checks.values()) else 1
 finally:
  child.send_signal(signal.SIGINT)
  try:child.wait(timeout=4)
  except subprocess.TimeoutExpired:child.kill();child.wait()
  f.close();n.destroy_node();rclpy.try_shutdown()
if __name__=='__main__':raise SystemExit(main(sys.argv[1]))
