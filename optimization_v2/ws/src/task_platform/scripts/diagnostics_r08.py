"""Read-only near-body evidence recorder. Never publishes control or estimator inputs."""
import argparse,json,math,signal,time
from pathlib import Path
import rclpy
from std_msgs.msg import String
from nav_msgs.msg import Odometry,OccupancyGrid

def main():
 ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path,required=True);a=ap.parse_args();a.out.mkdir(parents=True,exist_ok=True);rclpy.init();n=rclpy.create_node('r08_readonly_diagnostics');st=dict(stop=False,pose=None,last=-100.);log=(a.out/'terrain.jsonl').open('w');grids=(a.out/'costmap.jsonl').open('w')
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
   if -.5<x<1.2 and abs(y)<.6:rows.append(dict(body=[x,y],values=row))
  log.write(json.dumps(dict(t=d['t'],columns=d['columns'],plane=d['ground_plane'],pose=st['pose'],rows=rows))+'\n');log.flush()
 def grid(m):
  if st['pose'] is None:return
  rows=[]
  for j in range(m.info.height):
   for i in range(m.info.width):
    value=m.data[j*m.info.width+i]
    if 0<=value<99:continue
    x,y=body(m.info.origin.position.x+(i+.5)*m.info.resolution,m.info.origin.position.y+(j+.5)*m.info.resolution)
    if -.5<x<1.2 and abs(y)<.6:rows.append([x,y,value])
  grids.write(json.dumps(dict(t=m.header.stamp.sec+m.header.stamp.nanosec*1e-9,pose=st['pose'],bad=rows))+'\n');grids.flush()
 n.create_subscription(Odometry,'/s05/odometry',od,10);n.create_subscription(String,'/s09/terrain/cells',cells,2);n.create_subscription(OccupancyGrid,'/local_costmap/costmap',grid,2)
 def stop(s,f):st['stop']=True
 for s in [signal.SIGINT,signal.SIGTERM]:signal.signal(s,stop)
 (a.out/'ready').write_text('read-only observed cells; no truth input or command publisher')
 try:
  while not st['stop']:rclpy.spin_once(n,timeout_sec=.02)
 finally:log.close();grids.close();n.destroy_node();rclpy.try_shutdown()
if __name__=='__main__':main()
