"""Only estimated pose, measured odometry and noisy tool feedback enter online accounting."""
from common10 import *
import argparse,signal
from collections import deque
from ledger_core import latest_at
import rclpy
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from shapely.geometry import shape,Polygon
from shapely.ops import unary_union
from shapely.affinity import rotate,translate

def main():
 p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True);p.add_argument('--config',type=Path,required=True);a=p.parse_args();a.out.mkdir(parents=True,exist_ok=True);c=read(a.config);features=[shape(f['geometry']) for p in [BASE/'maps/prior/workspaces.geojson',BASE/'maps/prior/connector.geojson',BASE/'config/perception_test_area.geojson'] for f in read(p)['features'] if f['geometry']['type'] in ['Polygon','MultiPolygon']];allowed=unary_union(features);vehicle=yaml.safe_load((BASE/f"config/vehicles/{c['vehicle']}.yaml").read_text());body=Polygon(vehicle['footprint']).buffer(.1);rclpy.init();n=rclpy.create_node('s10_online_adapter');st=dict(t=0.,stop=False,global_status=None,safety=None,odom=None,intent=None,fault=None);fb=n.create_publisher(String,'/s10/tool/feedback',30);cp=n.create_publisher(String,'/s10/context',30);rng=np.random.default_rng(41);log=(a.out/'observations.jsonl').open('w')
 def record(k,d):log.write(json.dumps(dict(kind=k,**d))+'\n')
 history={k:deque(maxlen=100) for k in ['global_status','safety','odom']};pending=deque(maxlen=15)
 def clock(m):st['t']=m.clock.sec+m.clock.nanosec*1e-9
 def update(m,k):
  st[k]=json.loads(m.data)
  if k in history:history[k].append(st[k])
 def odom(m):
  st['odom']=dict(t=stamp(m),v=m.twist.twist.linear.x,w=m.twist.twist.angular.z);history['odom'].append(st['odom'])
 def loc(m):pending.append(m)
 def process_pose(m):
  xyz,q=pose(m);yaw=float(Rotation.from_quat(q).as_euler('xyz')[2]);t=stamp(m);offset=c['tool']['mount_xyz'][0];s=latest_at(history['global_status'],t) or {};p=latest_at(history['safety'],t) or {};o=latest_at(history['odom'],t) or {};intent=st['intent'] or {};fault=st['fault'] or {};validfault=st['t']<fault.get('until',-1);mode=fault.get('mode') if validfault else None
  d=dict(t=t,tool_xy=[float(xyz[0]+offset*math.cos(yaw)),float(xyz[1]+offset*math.sin(yaw))],yaw=yaw,v=o.get('v',100),w=o.get('w',100),localization=bool(s.get('navigation_allowed')) and -1e-6<=t-s.get('t',-100)<.2 and mode!='LOCALIZATION',perception=bool(p.get('valid',False) and allowed.covers(translate(rotate(body,yaw,use_radians=True),float(xyz[0])+28,float(xyz[1])+14))) and -1e-6<=t-p.get('t',-100)<.45,target_ok=mode!='TARGET',max_speed=min(s.get('max_speed',0),p.get('max_speed',0)),segment=intent.get('segment','TRANSIT') if st['t']-intent.get('t',-100)<.2 else 'TRANSIT',segment_id=intent.get('segment_id',''),target_condition_source='diagnostic operator preset; not near-field camera recognition')
  if t-o.get('t',-100)>.1:d['v']=100
  if mode=='SPEED':d['v']=1.
  cp.publish(String(data=json.dumps(d)));record('context',d)
 def flush_poses():
  while pending and stamp(pending[0])<=st['t']+1e-6:process_pose(pending.popleft())
 n.create_timer(.01,flush_poses)
 def raw(m):
  d=json.loads(m.data);f=st['fault'] or {};mode=f.get('mode') if st['t']<f.get('until',-1) else None
  if mode=='DROP_FEEDBACK':return
  d['rpm_rad_s']=round(d['rpm_rad_s']+float(rng.normal(0,.005)),2);d['flow']=round(d['flow'],3);d['meter']=round(d['meter'],4)
  if mode=='BAD_VERSION':d['tool_revision']='invalid'
  if mode=='BAD_TIMESTAMP':d['t']-=2.
  fb.publish(String(data=json.dumps(d)));record('feedback',d)
 n.create_subscription(Clock,'/clock',clock,qos_profile_sensor_data);n.create_subscription(Odometry,'/s05/odometry',odom,30);n.create_subscription(PoseWithCovarianceStamped,'/s08/pose',loc,30)
 for topic,k in [('/s08/status','global_status'),('/s09/terrain/health','safety'),('/s10/segment','intent'),('/s10/test/fault','fault')]:n.create_subscription(String,topic,lambda m,key=k:update(m,key),20)
 n.create_subscription(String,'/s10/tool/raw_feedback',raw,30)
 def stop(s,f):st['stop']=True
 signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop);(a.out/'ready').write_text('estimated-only context + delayed noisy feedback')
 try:
  while not st['stop']:rclpy.spin_once(n,timeout_sec=.02)
 finally:log.close();n.destroy_node();rclpy.try_shutdown()
if __name__=='__main__':main()
