"""Three real Nav2 bringups on one surviving client; synthetic clock/TF, no vehicle."""
from pathlib import Path
import sys,os,time,json,subprocess,signal
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from middleware13 import local_dds
from nav_lifecycle13 import bringup_after_restart
base=Path('/home/ceihao/robot_project/optimization_v2');env=local_dds(os.environ);env['ROS_DOMAIN_ID']='117';os.environ.update(env)
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
out=Path(sys.argv[1]);out.mkdir(parents=True,exist_ok=True);rclpy.init();node=rclpy.create_node('s13_restart_probe');clock_pub=node.create_publisher(Clock,'/clock',qos_profile_sensor_data);tf_pub=node.create_publisher(TFMessage,'/tf',100);origin=time.monotonic();children=[];logs=[];cases=[];events=[];suppress_until=0.;injected=False;cycle=0

def sim_time():return 180.+.4*(time.monotonic()-origin)
def tick():
 value=sim_time();c=Clock();c.clock.sec=int(value);c.clock.nanosec=int((value-int(value))*1e9);clock_pub.publish(c);ts=[]
 for parent,child in [('map','odom'),('odom','base_link')]:
  t=TransformStamped();t.header.stamp=c.clock;t.header.frame_id=parent;t.child_frame_id=child;t.transform.rotation.w=1.;ts.append(t)
 tf_pub.publish(TFMessage(transforms=ts))
 if time.monotonic()<suppress_until:time.sleep(.01)
 else:rclpy.spin_once(node,timeout_sec=.01)
def log(event,**kw):
 global injected,suppress_until
 events.append(dict(event=event,cycle=cycle,t=sim_time(),**kw))
 if cycle==0 and not injected and event=='lifecycle_query_sent':
  injected=True;suppress_until=time.monotonic()+3.0;events.append(dict(event='client_reply_dispatch_paused',wall_seconds=3.0))
def launch(role,args):
 f=(out/f'{cycle}_{role}.log').open('w');logs.append(f);p=subprocess.Popen(args,env=env,stdout=f,stderr=subprocess.STDOUT);children.append(p)
def stop_children():
 for p in children:
  if p.poll() is None:p.send_signal(signal.SIGINT)
 for p in children:
  try:p.wait(timeout=5)
  except subprocess.TimeoutExpired:p.terminate();p.wait(timeout=5)
 children.clear()
 for f in logs:f.close()
 logs.clear()
try:
 for cycle in range(3):
  for role,package in [('controller','nav2_controller'),('planner','nav2_planner'),('bt_navigator','nav2_bt_navigator')]:
   binary=role+'_server' if role!='bt_navigator' else role
   launch(role,[f'/opt/ros/jazzy/lib/{package}/{binary}','--ros-args','--params-file',str(base/'config/nav2_diff.yaml'),'-p',f's13_startup_sim_time_floor:={sim_time():.9f}','-r','cmd_vel:=/s13/startup_probe/nav_cmd'])
  launch('manager',['/opt/ros/jazzy/lib/nav2_lifecycle_manager/lifecycle_manager','--ros-args','-p','use_sim_time:=true','-p','autostart:=false','-p','node_names:=[controller_server,planner_server,bt_navigator]','-p','bond_timeout:=5.0'])
  begin=time.monotonic();bringup_after_restart(node,tick,sim_time,lambda:[x.node_name for x in node.get_publishers_info_by_topic('/s13/startup_probe/nav_cmd')]==['controller_server'],log)
  cases.append(dict(cycle=cycle,passed=True,wall_seconds=time.monotonic()-begin));stop_children()
 retries=[e for e in events if e['event']=='lifecycle_query_retry'];assert injected and retries,'missing forced reply-timeout recovery'
 result=dict(passed=True,cases=cases,retries=len(retries),events=events,fixture='synthetic clock/TF and paused client response dispatch; real Nav2 processes and production restart helper; no physical vehicle claim')
except BaseException as error:result=dict(passed=False,cases=cases,events=events,error=repr(error))
finally:
 stop_children();node.destroy_node();rclpy.try_shutdown()
(out/'result.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps({k:v for k,v in result.items() if k!='events'}));raise SystemExit(0 if result['passed'] else 1)
