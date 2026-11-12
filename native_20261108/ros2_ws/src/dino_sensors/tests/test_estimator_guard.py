#!/usr/bin/env python3
"""Real ROS adapter fault injection; isolated domain, no Gazebo or truth input."""
import json,math,os,pathlib,signal,subprocess,sys,time
os.environ['ROS_DOMAIN_ID']=str(180+os.getpid()%20)
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from rosgraph_msgs.msg import Clock
from builtin_interfaces.msg import Time
P=pathlib.Path(__file__).resolve().parents[1];A=pathlib.Path(sys.argv[1]);A.mkdir(parents=True,exist_ok=False)
cmd=['ros2','run','dino_sensors','estimator_io.py','--ros-args','-p','use_sim_time:=true','-p','contract_file:='+str(P/'generated/contract.json')]
for src,dst in [('/dino/wheel/odometry','wheel'),('/dino/imu','imu'),('/dino/estimator/rear_odometry','rear'),('/dino/odometry/filtered','formal'),('/dino/estimator/status','status'),('/dino/estimator/wheel_twist','twist'),('/dino/estimator/imu','imu_out'),('/tf','tf'),('/tf_static','tf_static')]:cmd+=['-r',src+':=/guard/'+dst]
log=(A/'node.log').open('w');proc=subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
rclpy.init();n=Node('estimator_guard_evaluator');out=[];status=[];tfs=[];subs=[];stamp=10.;checks={}
wheel=n.create_publisher(Odometry,'/guard/wheel',10);imu=n.create_publisher(Imu,'/guard/imu',10);rear=n.create_publisher(Odometry,'/guard/rear',10);clock=n.create_publisher(Clock,'/clock',10)
static=n.create_publisher(TFMessage,'/guard/tf_static',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE))
subs+=[n.create_subscription(Odometry,'/guard/formal',lambda m:out.append(m),10),n.create_subscription(String,'/guard/status',lambda m:status.append(json.loads(m.data)),100),n.create_subscription(TFMessage,'/guard/tf',lambda m:tfs.extend(m.transforms),100)]
def st(t):return Time(sec=int(t),nanosec=int(round((t-int(t))*1e9)))
def spin(seconds):
    end=time.monotonic()+seconds
    while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.01)
def sample(inputs=True,imu_time=None):
    global stamp
    stamp=round(stamp+.033,6);clock.publish(Clock(clock=st(stamp)));spin(.015)
    if inputs:
        w=Odometry();w.header.stamp=st(stamp);w.header.frame_id='odom';w.child_frame_id='base_footprint';w.twist.twist.linear.x=.5;wheel.publish(w)
        i=Imu();i.header.stamp=st(stamp if imu_time is None else imu_time);i.header.frame_id='imu_link';i.angular_velocity.z=.1;imu.publish(i)
    spin(.015)
    r=Odometry();r.header.stamp=st(stamp);r.header.frame_id='odom';r.child_frame_id='rear_axle_link';r.pose.pose.position.x=-.819;r.pose.pose.orientation.w=1.;r.twist.twist.linear.x=.5;r.twist.twist.angular.z=.1;rear.publish(r);spin(.02)
result={'status':'FAIL','command':cmd,'domain':os.environ['ROS_DOMAIN_ID']}
try:
    deadline=time.monotonic()+12
    while n.count_subscribers('/guard/wheel')<1 or n.count_publishers('/guard/status')<1:
        spin(.1)
        if time.monotonic()>deadline:raise TimeoutError('adapter discovery')
    spin(.4)
    for _ in range(8):sample()
    checks['missing_imu_tf_no_formal_output']=not out and any(s['reason']=='IMU_TF_MISSING_DROP' for s in status)
    tr=TransformStamped();tr.header.frame_id='rear_axle_link';tr.child_frame_id='imu_link';tr.transform.rotation.w=1.;static.publish(TFMessage(transforms=[tr]));spin(.2)
    for _ in range(10):sample()
    checks['valid_inputs_publish']=len(out)>=5 and len(tfs)>=5
    checks['correct_base_lever_arm']=bool(out) and abs(out[-1].pose.pose.position.x)<1e-9 and abs(out[-1].twist.twist.linear.y-.0819)<1e-9
    for _ in range(12):sample(inputs=False)
    old=len(out)
    for _ in range(8):sample(inputs=False)
    checks['stale_inputs_stop_formal_output']=len(out)==old and any(s['reason']=='INPUT_STALE_NO_FORMAL_ODOM' for s in status)
    for _ in range(5):sample()
    checks['fresh_inputs_resume']=len(out)>old
    old=len(out);sample(imu_time=stamp-.001)
    for _ in range(6):sample()
    checks['rewind_latches_no_blind_resume']=len(out)==old and any(s['reason']=='CLOCK_OR_SENSOR_REWIND' for s in status)
    checks['tf_only_odom_base_link']=bool(tfs) and all(t.header.frame_id=='odom' and t.child_frame_id=='base_link' for t in tfs)
    result['status']='PASS' if all(checks.values()) else 'FAIL'
except Exception as e:result['error']=str(e)
finally:
    os.killpg(proc.pid,signal.SIGINT)
    try:proc.wait(timeout=8)
    except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait(timeout=3)
    result.update(checks=checks,formal_messages=len(out),tf_messages=len(tfs),statuses=status,adapter_exit_code=proc.returncode)
    (A/'result.json').write_text(json.dumps(result,indent=2));print(json.dumps({k:v for k,v in result.items() if k!='statuses'},indent=2))
    n.destroy_node();rclpy.try_shutdown();log.close()
sys.exit(0 if result['status']=='PASS' else 1)
