"""Actual production C++ recovery transaction, isolated synthetic inputs; no work credit."""
import json,time,subprocess,signal,os,math
import rclpy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from std_srvs.srv import Trigger
from common import ROOT,APP,edition,dump,sha

def run():
 out=APP/'results/R10/revalidation_probe';out.mkdir(parents=True,exist_ok=True);policy=edition('R10')/'spray/manager.json';cfg=json.loads(policy.read_text());binary=APP/'build/r10_global/r10_global_manager'
 rclpy.init();n=rclpy.create_node('r10_isolated_revalidation_probe');status=[];n.create_subscription(String,'/s08/status',lambda m:status.append(json.loads(m.data)),20)
 topics=dict(map='/s08/map',gnss='/s08/gnss',contract='/s08/contract',worker='/r07/mission',gate='/r07/gate',safety='/r07/safety',proof='/r10/localization/plan_proof')
 pubs={k:n.create_publisher(String,v,20) for k,v in topics.items()};clock=n.create_publisher(Clock,'/clock',20);odom=n.create_publisher(Odometry,'/s05/odometry',20);prepare=n.create_client(Trigger,'/r10/localization/prepare_revalidation');commit=n.create_client(Trigger,'/r10/localization/commit_revalidation')
 log=(out/'native.log').open('w');p=subprocess.Popen([str(binary),'--ros-args','-p','use_sim_time:=true','-p','policy_file:='+str(policy)],stdout=log,stderr=subprocess.STDOUT,cwd=ROOT,start_new_session=True)
 t=0.;yaw=0.;worker='READY';moving=False;proof=None;epoch=0;checks={};events=[];began=time.monotonic()
 def tick(duration=.05):
  nonlocal t
  if time.monotonic()-began>55:raise TimeoutError('PROBE_WALL_BOUND')
  t=round(t+duration,6);c=Clock();c.clock.sec=int(t);c.clock.nanosec=round((t-int(t))*1e9);clock.publish(c)
  m=Odometry();m.header.stamp=c.clock;m.header.frame_id='odom';m.child_frame_id='base_link';m.pose.pose.orientation.w=1.;m.twist.twist.linear.x=.1 if moving else 0.;odom.publish(m)
  data=dict(contract=dict(t=t,valid=True,map_id=cfg['map_id']),map=dict(t=t,valid=True,map_id=cfg['map_id'],pose=[0.,0.,0.,0.,0.,math.sin(yaw/2),math.cos(yaw/2)]),gnss=dict(t=t,valid=True,mode='fixed',covariance=[.0004,0.,0.,0.,.0004,0.,0.,0.,.0004],antenna=[0.,0.,0.],antenna_base_m=[0.,0.,0.]),worker=dict(t=t,nav_idle=True,phase=worker,task_version=cfg['plan_revision']),gate=dict(t=t,stopped=not moving,tool_stopped=True),safety=dict(t=t,valid=True,localization_epoch=epoch))
  if proof is not None:data['proof']=dict(proof,t=t)
  for k,v in data.items():pubs[k].publish(String(data=json.dumps(v)))
  until=time.monotonic()+duration
  while time.monotonic()<until:rclpy.spin_once(n,timeout_sec=.005)
 def until(pred,limit=6.):
  start=t
  while not pred():
   tick()
   if t-start>limit:raise TimeoutError('PROBE_STATE_DEADLINE')
 def call(client):
  until(client.service_is_ready);f=client.call_async(Trigger.Request());until(f.done);r=f.result();events.append(dict(t=t,success=r.success,message=r.message));return r
 try:
  until(lambda:status and status[-1]['navigation_allowed'],10.);checks['original_initial_confirmation']=True
  yaw=.05;until(lambda:status[-1]['plan_revalidation_required']);checks['original_jump_guard_stops']=not status[-1]['navigation_allowed']
  checks['reject_without_worker_transaction']=not call(prepare).success
  worker='RELOCALIZING';moving=True
  for _ in range(40):tick()
  checks['reject_while_moving']=not call(prepare).success
  moving=False;until(lambda:status[-1]['recovery_ready'],8.);r=call(prepare);epoch=json.loads(r.message)['epoch'];checks['prepare_retains_hold']=r.success and status[-1]['plan_revalidation_required'] and not status[-1]['navigation_allowed']
  until(lambda:status[-1]['recovery_preview']);checks['no_path_proof_rejected']=not call(commit).success
  proof=dict(epoch=epoch-1,plan_revision=cfg['plan_revision'],map_revision=cfg['map_revision'],path_valid=True,footprint_valid=True,path_sha256='a'*64)
  tick();checks['wrong_epoch_rejected']=not call(commit).success
  proof['epoch']=epoch;proof['plan_revision']='wrong';tick();checks['wrong_plan_rejected']=not call(commit).success
  proof['plan_revision']=cfg['plan_revision'];proof['path_valid']=False;tick();checks['blocked_path_rejected']=not call(commit).success
  proof['path_valid']=True;tick();r=call(commit);checks['valid_transaction_commits']=r.success;checks['commit_not_instant_motion']=not status[-1]['navigation_allowed']
  committed=t;worker='READY';until(lambda:status[-1]['navigation_allowed'],8.);checks['original_confirmation_after_commit']=t-committed>=cfg['confirmation_span_s']-.15
  checks['exact_rejected_se3_recorded']=all(k in status[-1]['last_rejected_map'] for k in ['capture_t','matched_odom_t','candidate','previous','angle_rad','paired_odom'])
 except BaseException as e:checks['exception']=False;events.append(dict(error=repr(e),last=status[-1] if status else {}))
 finally:
  p.send_signal(signal.SIGINT)
  try:p.wait(timeout=5)
  except subprocess.TimeoutExpired:p.kill();p.wait()
  log.close();n.destroy_node();rclpy.shutdown()
 checks['clean_exit']=p.returncode==0
 report=dict(passed=all(checks.values()),checks=checks,events=events,binary_sha256=sha(binary),policy_sha256=sha(policy),source_sha256=sha(__file__),domain=os.environ.get('ROS_DOMAIN_ID'),scope='Synthetic isolated production recovery transaction; no Gazebo or work completion credit');dump(out/'result.json',report);print(json.dumps(report));return 0 if report['passed'] else 2
if __name__=='__main__':raise SystemExit(run())
