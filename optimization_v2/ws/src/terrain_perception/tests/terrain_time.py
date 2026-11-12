"""Actual compiled terrain node: deterministic timestamp/frame/rolling-window failures."""
import os,sys,time,subprocess,signal
from pathlib import Path
os.environ['ROS_DOMAIN_ID']='124';sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from common09 import *
import rclpy
from nav_msgs.msg import Odometry,OccupancyGrid
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Header,String
out=Path(sys.argv[1]);out.mkdir(parents=True,exist_ok=True);dump(out/'config.json',yaml.safe_load((BASE/'config/traversability.yaml').read_text()));f=(out/'node.log').open('w');proc=subprocess.Popen([str(BASE/'ws/install/terrain_perception/lib/terrain_perception/terrain_node'),'--ros-args','-p','use_sim_time:=true','-p','config_file:='+str(out/'config.json')],stdout=f,stderr=subprocess.STDOUT);rclpy.init();n=rclpy.create_node('s09_terrain_contract_test');cp=n.create_publisher(Clock,'/clock',10);op=n.create_publisher(Odometry,'/s05/odometry',100);pp=n.create_publisher(PointCloud2,'/s04/perception/points',10);health=[];grids=[];n.create_subscription(String,'/s09/terrain/health',lambda m:health.append(json.loads(m.data)),10);n.create_subscription(OccupancyGrid,'/s09/terrain/diff',lambda m:grids.append(m),2);checks={}
x,y=np.meshgrid(np.arange(-7,7,.12),np.arange(-7,7,.12));pts=np.column_stack([x.reshape(-1),y.reshape(-1),np.full(x.size,-.78)]).astype(np.float32)
def spin(sec):
 until=time.monotonic()+sec
 while time.monotonic()<until:rclpy.spin_once(n,timeout_sec=.02)
def send(t,frame='lidar_link',odom=True,age=0.,x=0.):
 c=Clock();c.clock.sec=int(t);c.clock.nanosec=int((t-int(t))*1e9);cp.publish(c);o=Odometry();o.header.stamp=c.clock;o.header.frame_id='odom';o.child_frame_id='base_link';o.pose.pose.position.x=float(x);o.pose.pose.orientation.w=1.
 if odom:op.publish(o)
 spin(.08);h=Header();h.frame_id=frame;h.stamp.sec=int(t+age);h.stamp.nanosec=int(((t+age)-int(t+age))*1e9);pp.publish(point_cloud2.create_cloud_xyz32(h,pts));spin(.22)
try:
 spin(2);send(1);send(1.3);checks['valid_geometry']=bool(health and health[-1]['valid'] and grids)
 send(2,x=3);checks['rolling_preserves_metric_origin']=bool(grids and grids[-1].info.origin.position.x>-14)
 send(3,frame='wrong');checks['wrong_frame_stops']=not health[-1]['valid'] and health[-1]['max_speed']==0
 send(4,age=1);checks['future_stamp_stops']=not health[-1]['valid']
 send(6,odom=False);checks['old_odometry_stops']=not health[-1]['valid']
 send(7);checks['fresh_recovery']=health[-1]['valid']
 send(6);send(8);checks['rewind_requires_restart']=not health[-1]['valid']
finally:
 proc.send_signal(signal.SIGINT)
 try:proc.wait(timeout=10)
 except subprocess.TimeoutExpired:proc.kill();proc.wait()
 f.close();n.destroy_node();rclpy.try_shutdown();dump(out/'result.json',dict(checks=checks,health=health,passed=len(checks)==7 and all(checks.values()),exit_code=proc.returncode));print(json.dumps(checks))
raise SystemExit(0 if len(checks)==7 and all(checks.values()) else 1)
