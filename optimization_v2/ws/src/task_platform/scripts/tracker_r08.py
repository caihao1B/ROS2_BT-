"""Observed geometry tracking; fixture trajectories and object identity are not inputs."""
from common import BASE
import argparse,json,math,signal,sys
from pathlib import Path
import numpy as np
sys.path.insert(0,str(BASE/'ws/src/object_fusion/scripts'))
from tracker import Tracker
from path_envelope_r08 import envelope_free
import rclpy
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid

def main():
 ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path,required=True);ap.add_argument('--config',type=Path,required=True);a=ap.parse_args();a.out.mkdir(parents=True,exist_ok=True);cfg=json.loads(a.config.read_text());route=np.asarray(cfg['route_map']);tracker=Tracker();rclpy.init();n=rclpy.create_node('r08_observed_leader',parameter_overrides=[rclpy.parameter.Parameter('use_sim_time',value=True)]);st=dict(stop=False,loc=None,grid=None);pub=n.create_publisher(String,'/r08/leader',10);log=(a.out/'tracks.jsonl').open('w');n.create_subscription(String,'/r07/localization',lambda m:st.update(loc=json.loads(m.data)),10);n.create_subscription(OccupancyGrid,'/r07/risk_local',lambda m:st.update(grid=m),2)
 def geometry(m):
  d=json.loads(m.data);t=d['t']
  if tracker.last is not None and t<=tracker.last:return
  tracks,ambiguous=tracker.update(t,d['objects']);loc=st['loc'];result=dict(t=t,valid=False,on_path=False,tracks=tracks,ambiguous=ambiguous,path_valid=False,source='CURRENT_PCL_GEOMETRY_AND_MEASURED_ODOMETRY',truth_used=False)
  if loc and loc.get('valid') and abs(t-loc['t'])<.3:
   tr=loc['transform'];co,si=math.cos(tr[2]),math.sin(tr[2]);R=np.array([[co,-si],[si,co]]);ego=np.array(loc['pose'][:2]);idx=int(np.argmin(np.linalg.norm(route[:,:2]-ego,axis=1)));heading=route[idx,2];tangent=np.array([math.cos(heading),math.sin(heading)]);normal=np.array([-tangent[1],tangent[0]]);candidates=[]
   for track in tracks:
    pos=R@np.array(track['position'][:2])+tr[:2];delta=pos-ego;along=float(delta@tangent);lateral=float(delta@normal);gap=along-max(track['size'][:2])/2-max(x for x,y in cfg['vehicle']['footprint'])
    if 0<along<7 and abs(lateral)<.7+max(track['size'][:2])/2 and track['confirmed']:
     velocity=R@np.array(track['velocity']);speed=float(velocity@tangent);side=float(velocity@normal);hits=tracker.tracks[track['id']]['hits'];confidence=min(.98,.5+.08*hits) if track['observed_age_s']<.25 and track['state']=='OBSERVED' else 0.;candidates.append(dict(id=track['id'],gap=gap,lateral=lateral,speed=speed,same_direction=speed>.05 and abs(side)<.15,confidence=confidence,ttc=gap/max(.001,loc.get('v',0)-speed),state=track['state']))
   # Check a measured one-metre braking/continuation envelope. This is not a
   # claim that the entire remaining connector has current sensor coverage.
   grid=st['grid'];path_valid=False
   if grid and abs(t-(grid.header.stamp.sec+grid.header.stamp.nanosec*1e-9))<.3 and grid.header.frame_id=='odom':
    fp=np.asarray(cfg['vehicle']['footprint']);x=np.linspace(fp[:,0].min()-.1,fp[:,0].max()+.1,9);y=np.linspace(fp[:,1].min()-.1,fp[:,1].max()+.1,9);xx,yy=np.meshgrid(x,y);samples=np.c_[xx.ravel(),yy.ravel()];parts=[]
    for u in np.linspace(0,1,6):
     angle=loc['pose'][2]+u*math.atan2(math.sin(heading-loc['pose'][2]),math.cos(heading-loc['pose'][2]));C=np.array([[math.cos(angle),-math.sin(angle)],[math.sin(angle),math.cos(angle)]]);world=samples@C.T+ego+u*tangent;parts.append((world-np.array(tr[:2]))@R)
    points=np.concatenate(parts);ix=np.floor((points[:,0]-grid.info.origin.position.x)/grid.info.resolution).astype(int);iy=np.floor((points[:,1]-grid.info.origin.position.y)/grid.info.resolution).astype(int);inside=(ix>=0)&(iy>=0)&(ix<grid.info.width)&(iy<grid.info.height)
    if inside.all():cost=np.asarray(grid.data)[iy*grid.info.width+ix];path_valid=envelope_free(cost,points,loc['odom'],cfg['vehicle']['footprint'],0.)
   result.update(valid=True,path_valid=path_valid,route_index=idx)
   if candidates:result.update(on_path=True,leader=min(candidates,key=lambda x:x['gap']))
  pub.publish(String(data=json.dumps(result)));log.write(json.dumps(result)+'\n');log.flush();(a.out/'live.json').write_text(json.dumps(result))
 n.create_subscription(String,'/s09/geometry',geometry,10)
 def stop(s,f):st['stop']=True
 for s in [signal.SIGINT,signal.SIGTERM]:signal.signal(s,stop)
 (a.out/'ready').write_text('geometry tracking only; no actor fixture subscription')
 try:
  while not st['stop']:rclpy.spin_once(n,timeout_sec=.01)
 finally:log.close();n.destroy_node();rclpy.try_shutdown()
if __name__=='__main__':main()
