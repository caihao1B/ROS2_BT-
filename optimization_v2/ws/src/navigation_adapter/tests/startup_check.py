"""Real Nav2 lifecycle / delayed synthetic-clock regression; no physical motion claim."""
from pathlib import Path
import os,sys,time,json,subprocess,signal
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from middleware13 import local_dds
base=Path('/home/ceihao/robot_project/optimization_v2')
env=local_dds(os.environ);env['ROS_DOMAIN_ID']='117';os.environ.update(env)
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
from lifecycle_msgs.srv import ChangeState,GetState
out=Path(sys.argv[1]);out.mkdir(parents=True,exist_ok=True)
rclpy.init();node=rclpy.create_node('s13_startup_clock_probe')
clock_pub=node.create_publisher(Clock,'/clock',qos_profile_sensor_data)
tf_pub=node.create_publisher(TFMessage,'/tf',100)
enabled=False;origin=time.monotonic();results=[];process=None

def spin():
 if enabled:
  value=180.+time.monotonic()-origin;clock=Clock();clock.clock.sec=int(value);clock.clock.nanosec=int((value-int(value))*1e9);clock_pub.publish(clock)
  transforms=[]
  for parent,child in [('map','odom'),('odom','base_link')]:
   transform=TransformStamped();transform.header.stamp=clock.clock;transform.header.frame_id=parent;transform.child_frame_id=child;transform.transform.rotation.w=1.;transforms.append(transform)
  tf_pub.publish(TFMessage(transforms=transforms))
 rclpy.spin_once(node,timeout_sec=.01)

def wait(predicate,seconds):
 deadline=time.monotonic()+seconds
 while not predicate():
  if time.monotonic()>deadline:raise TimeoutError('bounded startup probe')
  spin()

def state(client):
 future=client.call_async(GetState.Request());wait(future.done,3);return future.result().current_state.id

try:
 for role,provide_clock in [('controller',True),('planner',True),('controller',False)]:
  enabled=False;label=role+('_delayed_clock' if provide_clock else '_missing_clock');name=role+'_server';log=out/(label+'.log')
  args=[f'/opt/ros/jazzy/lib/nav2_{role}/{name}','--ros-args','--params-file',str(base/'config/nav2_diff.yaml'),'-p','s13_startup_sim_time_floor:=180.0','-r','cmd_vel:=/s13/startup_probe/nav_cmd']
  with log.open('w') as stream:
   process=subprocess.Popen(args,env=env,stdout=stream,stderr=subprocess.STDOUT)
   try:
    change=node.create_client(ChangeState,'/'+name+'/change_state');get=node.create_client(GetState,'/'+name+'/get_state');wait(lambda:change.service_is_ready() and get.service_is_ready(),10)
    assert state(get)==1
    request=ChangeState.Request();request.transition.id=1;future=change.call_async(request)
    wait(lambda:'STARTUP_CLOCK_WAIT' in log.read_text(),8)
    until=time.monotonic()+.3
    while time.monotonic()<until:spin()
    assert not future.done(), 'configured before receiving a clock'
    enabled=provide_clock;origin=time.monotonic();wait(future.done,13)
    success=future.result().success
    if provide_clock:
     assert success and state(get)==2
     request=ChangeState.Request();request.transition.id=3;activate=change.call_async(request);wait(activate.done,8)
     assert activate.result().success and state(get)==3
     assert 'STARTUP_CLOCK_READY' in log.read_text()
    else:
     assert not success and state(get)!=3
     assert 'STARTUP_CLOCK_NOT_RECEIVED' in log.read_text()
    results.append(dict(case=label,passed=True,configure_success=success,command=args,fixture='synthetic /clock and TF; real installed Nav2 lifecycle; no vehicle'))
   finally:
    if process.poll() is None:process.send_signal(signal.SIGINT)
    try:process.wait(timeout=5)
    except subprocess.TimeoutExpired:process.terminate();process.wait(timeout=5)
    process=None
    if 'change' in locals():node.destroy_client(change)
    if 'get' in locals():node.destroy_client(get)
 result=dict(passed=len(results)==3,cases=results)
except BaseException as error:result=dict(passed=False,cases=results,error=repr(error))
finally:
 if process is not None and process.poll() is None:process.terminate();process.wait(timeout=5)
 node.destroy_node();rclpy.try_shutdown()
(out/'result.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result));raise SystemExit(0 if result['passed'] else 1)
