"""Authorized task boundaries + observed terrain + tracked geometry, no world oracle."""
import argparse,signal,copy
from common09 import *
import rclpy
from rclpy.qos import QoSProfile,ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid,Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String
from shapely.geometry import shape
from shapely.ops import unary_union
from shapely import contains_xy

def main():
 p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True);p.add_argument('--vehicle',choices=['diff','ackermann'],default='diff');a=p.parse_args();a.out.mkdir(parents=True,exist_ok=True);cfg=yaml.safe_load((BASE/'config/traversability.yaml').read_text());features=[]
 for f in ['workspaces.geojson','connector.geojson']:
  for item in read(BASE/'maps/prior'/f)['features']:
   if item['geometry']['type'] in ['Polygon','MultiPolygon']:features.append(shape(item['geometry']))
 features.extend(shape(f['geometry']) for f in read(BASE/'config/perception_test_area.geojson')['features']);allowed=unary_union(features);regions={kind:allowed.buffer(-max(max(abs(p[1]) for p in pr['footprint']),pr['tool_width_m']/2)-pr['margin_m']) for kind,pr in cfg['profiles'].items()};rclpy.init();n=rclpy.create_node('s09_costmap_adapter',parameter_overrides=[rclpy.parameter.Parameter('use_sim_time',value=True)]);state=dict(stop=False,odom=None,pose=None,status=None,objects=None,health=None,last=-1.);pubs={k:n.create_publisher(OccupancyGrid,'/s09/risk/'+k,2) for k in ['diff','ackermann']};health=n.create_publisher(String,'/s09/safety',10);log=(a.out/'events.jsonl').open('w')
 def update(m,key):state[key]=m
 n.create_subscription(Odometry,'/s05/odometry',lambda m:update(m,'odom'),30);n.create_subscription(PoseWithCovarianceStamped,'/s08/pose',lambda m:update(m,'pose'),20)
 for topic,key in [('/s08/status','status'),('/s09/objects','objects'),('/s09/terrain/health','health')]:n.create_subscription(String,topic,lambda m,k=key:state.update({k:json.loads(m.data)}),20)
 def grid(m,kind):
  t=stamp(m);out=copy.deepcopy(m);v=np.asarray(m.data,dtype=np.int16).reshape(m.info.height,m.info.width).copy();x=m.info.origin.position.x+(np.arange(m.info.width)+.5)*m.info.resolution;y=m.info.origin.position.y+(np.arange(m.info.height)+.5)*m.info.resolution;xx,yy=np.meshgrid(x,y);status=state['status'];bound=False;boundary_count=0
  if state['pose'] and state['odom'] and status and status['navigation_allowed'] and abs(t-status['t'])<.5:
   gp,gq=pose(state['pose']);op,oq=pose(state['odom']);R=Rotation.from_quat(gq).as_matrix()@Rotation.from_quat(oq).as_matrix().T;tr=gp-R@op;wx=R[0,0]*xx+R[0,1]*yy+tr[0]+28;wy=R[1,0]*xx+R[1,1]*yy+tr[1]+14;outside=~contains_xy(regions[kind],wx,wy);v[outside]=100;boundary_count=int(outside.sum());bound=bool(contains_xy(regions[kind],gp[0]+28,gp[1]+14))
  tracks=state['objects'];marked=0
  if tracks and abs(t-tracks['t'])<.6:
   for obj in tracks['objects']:
    # Retain uncertainty-expanded coasting occupancy; semantics never clears geometry.
    P=np.array(obj['covariance']).reshape(4,4);pad=.1+min(.5,2*np.sqrt(max(P[0,0],P[1,1])));mask=(abs(xx-obj['position'][0])<obj['size'][0]/2+pad)&(abs(yy-obj['position'][1])<obj['size'][1]/2+pad);marked+=int(mask.sum());v[mask]=100
  out.data=v.astype('int8').reshape(-1).tolist();pubs[kind].publish(out)
  if kind==a.vehicle:
   h=state['health'];ok=h is not None and h['valid'] and 0<=t-h['t']<.45;limit=min(h['max_speed'] if ok else 0.,.12 if bound else 0.);d=dict(t=t,geometry_fresh=ok,boundary_valid=bound,max_speed=limit,reason='GEOMETRY_OR_GLOBAL_REFERENCE_STOP' if not ok or not bound else h['reason'],boundary_cells=boundary_count,track_cells=marked);msg=String();msg.data=json.dumps(d);health.publish(msg);log.write(msg.data+'\n');log.flush()
  if t-state['last']>1 and kind=='diff':state['last']=t;np.savez_compressed(a.out/f'grid_{int(t):03}.npz',cost=v.astype(np.int8),origin=[m.info.origin.position.x,m.info.origin.position.y],resolution=m.info.resolution,t=t)
 for kind in pubs:n.create_subscription(OccupancyGrid,'/s09/terrain/'+kind,lambda m,k=kind:grid(m,k),2)
 def watchdog():
  h=state['health'];t=n.get_clock().now().nanoseconds*1e-9
  if h is None or t-h['t']>.45:
   d=dict(t=t,geometry_fresh=False,boundary_valid=False,max_speed=0.,reason='STALE_GEOMETRY_STOP',boundary_cells=0,track_cells=0);m=String();m.data=json.dumps(d);health.publish(m);log.write(m.data+'\n');log.flush()
 n.create_timer(.1,watchdog)
 def end(s,f):state['stop']=True
 signal.signal(signal.SIGINT,end);signal.signal(signal.SIGTERM,end);(a.out/'ready').write_text('observed local risk and authorized prior boundaries')
 try:
  while not state['stop']:rclpy.spin_once(n,timeout_sec=.02)
 finally:log.close();n.destroy_node();rclpy.try_shutdown()
if __name__=='__main__':main()
