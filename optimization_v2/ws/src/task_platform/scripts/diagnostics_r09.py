"""Read-only near-body evidence recorder. Never publishes control or estimator inputs."""
import argparse,json,math,signal,time
from pathlib import Path
import numpy as np
from collections import deque
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import read_points_numpy
from rclpy.qos import qos_profile_sensor_data
import rclpy
from std_msgs.msg import String
from nav_msgs.msg import Odometry,OccupancyGrid

def main():
 ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path,required=True);a=ap.parse_args();a.out.mkdir(parents=True,exist_ok=True);rclpy.init();n=rclpy.create_node('r09_readonly_diagnostics');st=dict(stop=False,pose=None,last=-100.,capture_last=-100.,captures=0,loc=None,risk_last=-100.,risk_count=0,nav_grid=None);clouds={k:deque(maxlen=3) for k in ['combined','front','rear','left','right']};log=(a.out/'terrain.jsonl').open('w');grids=(a.out/'costmap.jsonl').open('w')
 def od(m):
  p=m.pose.pose;q=p.orientation;st['pose']=[p.position.x,p.position.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))]
 def body(x,y):
  ox,oy,h=st['pose'];x-=ox;y-=oy;return math.cos(h)*x+math.sin(h)*y,-math.sin(h)*x+math.cos(h)*y
 def cells(m):
  d=json.loads(m.data)
  if st['pose'] is None or d['t']-st['last']<.2:return
  st['last']=d['t'];rows=[]
  for row in d['cells']:
   x,y=body(row[0],row[1])
   if -1.<x<3.2 and abs(y)<2.0:rows.append(dict(body=[x,y],values=row))
  if d['t']>18 and any(r['values'][9]==100 or r['values'][9]<0 for r in rows) and d['t']-st['capture_last']>1. and st['captures']<12:
   arrays={};metadata=dict(terrain=d,odom_pose=st['pose'])
   for key,queue in clouds.items():
    if not queue:continue
    cloud=min(queue,key=lambda m:abs(m.header.stamp.sec+m.header.stamp.nanosec*1e-9-d['t']));ct=cloud.header.stamp.sec+cloud.header.stamp.nanosec*1e-9
    arrays[key]=read_points_numpy(cloud,field_names=['x','y','z'],skip_nans=True).reshape(-1,3);metadata[key+'_t']=ct
   file=a.out/('capture_%02d'%st['captures']);np.savez(file.with_suffix('.npz'),**arrays);file.with_suffix('.json').write_text(json.dumps(metadata));st['captures']+=1;st['capture_last']=d['t']
  log.write(json.dumps(dict(t=d['t'],columns=d['columns'],plane=d['ground_plane'],pose=st['pose'],rows=rows))+'\n');log.flush()
 def grid(m):
  st['nav_grid']=m
  if st['pose'] is None:return
  rows=[]
  for j in range(m.info.height):
   for i in range(m.info.width):
    value=m.data[j*m.info.width+i]
    if 0<=value<99:continue
    x,y=body(m.info.origin.position.x+(i+.5)*m.info.resolution,m.info.origin.position.y+(j+.5)*m.info.resolution)
    if -1.<x<3.2 and abs(y)<2.0:rows.append([x,y,value])
  grids.write(json.dumps(dict(t=m.header.stamp.sec+m.header.stamp.nanosec*1e-9,pose=st['pose'],bad=rows))+'\n');grids.flush()
 for key,topic in [('combined','/s04/perception/points')]+[(k,'/r07/near_'+k+'/points') for k in ['front','rear','left','right']]:n.create_subscription(PointCloud2,topic,lambda m,k=key:clouds[k].append(m),qos_profile_sensor_data)
 def risk_grid(m):
  t=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
  if not st['loc'] or not st['loc'].get('valid') or t-st['risk_last']<1. or st['risk_count']>=80:return
  path=a.out/('risk_%02d'%st['risk_count']);arrays={'risk':np.asarray(m.data).reshape(m.info.height,m.info.width)};meta=dict(t=t,origin=[m.info.origin.position.x,m.info.origin.position.y],resolution=m.info.resolution,loc=st['loc'])
  g=st['nav_grid']
  if g:
   arrays['nav']=np.asarray(g.data).reshape(g.info.height,g.info.width);meta.update(nav_t=g.header.stamp.sec+g.header.stamp.nanosec*1e-9,nav_origin=[g.info.origin.position.x,g.info.origin.position.y],nav_resolution=g.info.resolution,nav_frame=g.header.frame_id)
  np.savez(path.with_suffix('.npz'),**arrays);path.with_suffix('.json').write_text(json.dumps(meta));st['risk_last']=t;st['risk_count']+=1
 n.create_subscription(String,'/r07/localization',lambda m:st.update(loc=json.loads(m.data)),10)
 n.create_subscription(OccupancyGrid,'/r07/risk_local',risk_grid,2)
 n.create_subscription(Odometry,'/s05/odometry',od,10);n.create_subscription(String,'/s09/terrain/cells',cells,2);n.create_subscription(OccupancyGrid,'/local_costmap/costmap',grid,2)
 def stop(s,f):st['stop']=True
 for s in [signal.SIGINT,signal.SIGTERM]:signal.signal(s,stop)
 (a.out/'ready').write_text('read-only observed cells; no truth input or command publisher')
 try:
  while not st['stop']:rclpy.spin_once(n,timeout_sec=.02)
 finally:log.close();grids.close();n.destroy_node();rclpy.try_shutdown()
if __name__=='__main__':main()
