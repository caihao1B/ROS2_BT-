"""Run the production C++ node in isolated domain 123; synthetic unit inputs only."""
import os,sys,time,signal,subprocess,json,tempfile
from pathlib import Path
os.environ['ROS_DOMAIN_ID']='123'
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from s07_common import *
import rclpy
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from rosgraph_msgs.msg import Clock
from std_srvs.srv import Trigger

def main(out):
    out=Path(out);out.mkdir(parents=True,exist_ok=True);policy=yaml.safe_load((BASE/'config/map_contract.yaml').read_text())['manager'];policy.update(map_id='UNIT_ONLY',diagnostic_fixture=True);dump(out/'policy.json',policy)
    log=(out/'manager.log').open('w');child=subprocess.Popen([str(BASE/'ws/install/global_pose_manager/lib/global_pose_manager/global_pose_manager'),'--ros-args','-p','use_sim_time:=true','-p','policy_file:='+str(out/'policy.json')],stdout=log,stderr=subprocess.STDOUT)
    rclpy.init();n=rclpy.create_node('s07_unit_driver');clock=n.create_publisher(Clock,'/clock',10);odom=n.create_publisher(Odometry,'/s05/odometry',10);contract=n.create_publisher(String,'/s07/map_contract',10);evidence=n.create_publisher(String,'/s07/localization/evidence',10);states=[];n.create_subscription(String,'/s07/status',lambda m:states.append(json.loads(m.data)),100);t=10.;checks={}
    def send(pub,value):m=String();m.data=json.dumps(value);pub.publish(m)
    def drive(duration,observed=None,valid=True,bound=True):
        nonlocal t
        end=t+duration;deadline=time.monotonic()+duration*4+3;next_evidence=0.
        while t<end:
            if time.monotonic()>deadline:raise TimeoutError('unit input loop watchdog')
            t+=.05;c=Clock();c.clock.sec=int(t);c.clock.nanosec=int((t-int(t))*1e9);clock.publish(c);m=Odometry();m.header.stamp=c.clock;m.header.frame_id='odom';m.child_frame_id='base_link';m.pose.pose.orientation.w=1.;odom.publish(m);send(contract,dict(t=t,valid=bound,map_id='UNIT_ONLY'))
            if observed is not None and t>next_evidence:
                next_evidence=t+.55;cov=[0.]*36
                for i in [0,7,14,21,28,35]:cov[i]=.01
                send(evidence,dict(t=t-.1,map_id='UNIT_ONLY',valid=valid,reason='UNIT_REJECTION',pose=[observed,0,0,0,0,0,1],covariance=cov))
            until=time.monotonic()+.05
            while time.monotonic()<until:rclpy.spin_once(n,timeout_sec=.005)
    try:
        drive(1.);checks['no_observation_no_admission']=bool(states) and not states[-1]['navigation_allowed']
        drive(.25,1.);checks['single_observation_no_admission']=not states[-1]['navigation_allowed']
        drive(2.2,1.);checks['multiple_stationary_matches_admitted']=states[-1]['navigation_allowed']
        drive(.25,1.,valid=False);checks['quality_rejection_closes']=not states[-1]['navigation_allowed']
        drive(2.2,1.);checks['quality_recovery']=states[-1]['navigation_allowed']
        drive(2.2,1.5);checks['large_jump_latches_hold']=states[-1]['plan_revalidation_required'] and not states[-1]['navigation_allowed'] and states[-1]['state']=='LOCALIZED'
        client=n.create_client(Trigger,'/s07/acknowledge_diagnostic_hold');assert client.wait_for_service(timeout_sec=2);future=client.call_async(Trigger.Request());drive(.3,1.5);checks['stopped_diagnostic_ack']=future.done() and future.result().success and states[-1]['navigation_allowed']
        drive(.4,1.5,bound=False);checks['contract_revocation_closes']=not states[-1]['navigation_allowed'] and states[-1]['state']=='LOST'
        drive(2.2,1.5);before=t;t=1.;drive(.3,1.5);checks['clock_rewind_latches_fault']=not states[-1]['navigation_allowed'] and states[-1]['state']=='LOST'
        dump(out/'result.json',dict(passed=all(checks.values()),checks=checks,states=states,scope='Actual production C++ state machine; synthetic inputs, no Gazebo or actuators',domain=123));print(json.dumps(checks));return 0 if all(checks.values()) else 1
    finally:
        child.send_signal(signal.SIGINT)
        try:child.wait(timeout=5)
        except subprocess.TimeoutExpired:child.terminate();child.wait(timeout=3)
        log.close();n.destroy_node();rclpy.try_shutdown()
if __name__=='__main__':raise SystemExit(main(sys.argv[1]))
