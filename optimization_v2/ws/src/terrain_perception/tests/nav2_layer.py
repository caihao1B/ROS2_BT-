"""Exercise installed production Nav2 plugin: marking, clearing, unknown, stale, frame."""
import os,sys,time,json,subprocess,signal
from pathlib import Path
os.environ['ROS_DOMAIN_ID']='123'
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from common09 import *
import rclpy
from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
from rosgraph_msgs.msg import Clock
from lifecycle_msgs.srv import ChangeState
out=Path(sys.argv[1]);out.mkdir(parents=True,exist_ok=True)
params=dict(use_sim_time=True,global_frame='odom',robot_base_frame='odom',update_frequency=10.,publish_frequency=10.,rolling_window=False,width=6,height=6,resolution=.2,origin_x=-3.,origin_y=-3.,track_unknown_space=True,always_send_full_costmap=True,plugins=['observed'],observed=dict(plugin='s09::ObservedRiskLayer',topic='/test/risk',max_age=.6))
config=out/'nav2.yaml';config.write_text(yaml.safe_dump({'/**':{'ros__parameters':params}}));log=(out/'nav2.log').open('w');proc=subprocess.Popen(['/opt/ros/jazzy/lib/nav2_costmap_2d/nav2_costmap_2d','--ros-args','-r','__ns:=/s09_test','--params-file',str(config)],stdout=log,stderr=subprocess.STDOUT)
rclpy.init();n=rclpy.create_node('s09_layer_test');clock=n.create_publisher(Clock,'/clock',10);pub=n.create_publisher(OccupancyGrid,'/test/risk',2);received=[];n.create_subscription(OccupancyGrid,'/s09_test/costmap',lambda m:received.append(m),QoSProfile(depth=5,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE));t=1.;checks={}
def drive(seconds,grid=None):
 global t
 end=time.monotonic()+seconds
 while time.monotonic()<end:
  t+=.02;c=Clock();c.clock.sec=int(t);c.clock.nanosec=int((t-int(t))*1e9);clock.publish(c)
  if grid is not None:grid.header.stamp=c.clock;pub.publish(grid)
  rclpy.spin_once(n,timeout_sec=.01);time.sleep(.01)
def transition(i):
 client=n.create_client(ChangeState,'/s09_test/costmap/change_state');assert client.wait_for_service(timeout_sec=15),'lifecycle service unavailable';req=ChangeState.Request();req.transition.id=i;f=client.call_async(req);limit=time.monotonic()+12
 while not f.done() and time.monotonic()<limit:drive(.05)
 assert f.done() and f.result().success,'lifecycle failed';n.destroy_client(client)
def grid(value=0,frame='odom'):
 m=OccupancyGrid();m.header.frame_id=frame;m.info.resolution=.2;m.info.width=m.info.height=30;m.info.origin.position.x=m.info.origin.position.y=-3.;m.info.origin.orientation.w=1.;m.data=[value]*900;return m
def latest():assert received,'costmap not published';return list(received[-1].data)
try:
 drive(.5);transition(1);transition(3);m=grid();m.data[465]=100;drive(1.2,m);checks['lethal']=latest()[465]==100
 m=grid();drive(.8,m);checks['explicit_ground_clears_old_lethal']=latest()[465]==0
 m=grid(-1);drive(.8,m);checks['unknown_not_free']=all(v==-1 for v in latest())
 m=grid();drive(.5,m);drive(1.1);checks['stale_is_unknown']=all(v==-1 for v in latest())
 m=grid(0,'wrong_frame');drive(.8,m);checks['frame_mismatch_is_unknown']=all(v==-1 for v in latest())
 m=grid(0);drive(.8,m);checks['fresh_recovers']=latest()[465]==0
finally:
 proc.send_signal(signal.SIGINT)
 try:proc.wait(timeout=10)
 except subprocess.TimeoutExpired:proc.kill();proc.wait()
 log.close();n.destroy_node();rclpy.try_shutdown();dump(out/'result.json',dict(checks=checks,passed=len(checks)==6 and all(checks.values()),exit_code=proc.returncode));print(json.dumps(checks))
raise SystemExit(0 if len(checks)==6 and all(checks.values()) else 1)
