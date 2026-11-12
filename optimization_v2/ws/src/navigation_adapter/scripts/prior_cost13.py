"""Frozen sensor-built occupancy + current S09 risk, in estimated odom frame.
Only known static free cells may fill S09 unknown. Live lethal is immediate. Repeatedly confirmed terrain persists until
an explicitly observed free sample; no world/truth geometry is read.
"""
from common13 import *
import argparse,signal,copy
import rclpy
from nav_msgs.msg import OccupancyGrid,Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String
from PIL import Image

def fuse(live,prior,remembered):
 out=live.copy();out[(out<0)&(prior==0)]=0;out[prior==100]=100
 out[(remembered==100)&(live<0)]=100
 return out

def in_map(values, origin, resolution, rotation, translation, center):
 """Nearest sampling, with conservative forward marking of every occupied cell."""
 height,width=values.shape
 target_origin=np.floor((np.asarray(center)-np.array([width,height])*resolution/2)/resolution)*resolution
 x=target_origin[0]+(np.arange(width)+.5)*resolution
 y=target_origin[1]+(np.arange(height)+.5)*resolution
 xx,yy=np.meshgrid(x,y)
 points=np.stack([xx-translation[0],yy-translation[1]],axis=-1)@rotation
 ix=np.floor((points[:,:,0]-origin[0])/resolution).astype(int)
 iy=np.floor((points[:,:,1]-origin[1])/resolution).astype(int)
 inside=(ix>=0)&(iy>=0)&(ix<width)&(iy<height)
 result=np.full_like(values,-1);result[inside]=values[iy[inside],ix[inside]]
 j,i=np.where(values>=100)
 if len(i):
  occupied=(np.stack([origin[0]+(i+.5)*resolution,origin[1]+(j+.5)*resolution],axis=1)@rotation.T+translation-target_origin)/resolution-.5
  base=np.floor(occupied).astype(int)
  for dx,dy in [(0,0),(0,1),(1,0),(1,1)]:
   ix=base[:,0]+dx;iy=base[:,1]+dy;inside=(ix>=0)&(iy>=0)&(ix<width)&(iy<height)
   result[iy[inside],ix[inside]]=100
 return result,target_origin

def main():
 p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True);p.add_argument('--vehicle',required=True);p.add_argument('--registered',action='store_true');a=p.parse_args();a.out.mkdir(parents=True,exist_ok=True)
 reg=None
 if a.registered:
  from registered_map13 import validate_registration
  reg=validate_registration()
 c=yaml.safe_load((BASE/'maps/frozen/navigation.yaml').read_text());pixels=np.asarray(Image.open(BASE/'maps/frozen/navigation.pgm'))[::-1];rclpy.init();n=rclpy.create_node('s13_prior_cost',parameter_overrides=[rclpy.parameter.Parameter('use_sim_time',value=True)]);st=dict(stop=False,pose=None,odom=None,status=None,confirmation=None);memo=dict(cost=None,origin=None,snapshot=-100.);pub=n.create_publisher(OccupancyGrid,'/s13/risk/'+a.vehicle,2);global_pub=n.create_publisher(OccupancyGrid,'/s13/risk_map/'+a.vehicle,2);log=(a.out/'events.jsonl').open('w')
 n.create_subscription(OccupancyGrid,'/s13/terrain/confirmed/'+a.vehicle,lambda m:st.update(confirmation=m),2)
 n.create_subscription(Odometry,'/s05/odometry',lambda m:st.update(odom=m),20);n.create_subscription(PoseWithCovarianceStamped,'/s08/pose',lambda m:st.update(pose=m),20);n.create_subscription(String,'/s08/status',lambda m:st.update(status=json.loads(m.data)),20)
 def grid(m):
  t=stamp(m);v=np.asarray(m.data,dtype=np.int16).reshape(m.info.height,m.info.width);pr=np.full_like(v,-1);remember=np.full_like(v,-1);s=st['status'];valid=bool(s and s['navigation_allowed'] and not s['plan_revalidation_required'] and abs(t-s['t'])<.3 and st['pose'] and st['odom'] and abs(t-stamp(st['pose']))<.3 and abs(t-stamp(st['odom']))<.2)
  if valid:
   g=st['pose'].pose.pose;o=st['odom'].pose.pose;h=yaw(g.orientation)-yaw(o.orientation);co,si=math.cos(h),math.sin(h);tx=g.position.x-co*o.position.x+si*o.position.y;ty=g.position.y-si*o.position.x-co*o.position.y
   x=m.info.origin.position.x+(np.arange(m.info.width)+.5)*m.info.resolution;y=m.info.origin.position.y+(np.arange(m.info.height)+.5)*m.info.resolution;xx,yy=np.meshgrid(x,y);mx=co*xx-si*yy+tx;my=si*xx+co*yy+ty
   if reg:
    rh=reg['rotation_yaw_rad'];rx=mx-reg['translation_xy_m'][0];ry=my-reg['translation_xy_m'][1];mx=math.cos(rh)*rx+math.sin(rh)*ry;my=-math.sin(rh)*rx+math.cos(rh)*ry
   ix=np.floor((mx-c['origin'][0])/c['resolution']).astype(int);iy=np.floor((my-c['origin'][1])/c['resolution']).astype(int);inside=(ix>=0)&(iy>=0)&(ix<pixels.shape[1])&(iy<pixels.shape[0]);pv=np.full_like(v,205);pv[inside]=pixels[iy[inside],ix[inside]];pr[pv==254]=0;pr[pv==0]=100
   # Vectorized rolling occupancy memory; preserve hazards inside the lidar blind spot.
   origin=np.array([m.info.origin.position.x,m.info.origin.position.y]);previous=memo['cost']
   if previous is not None:
    shift=np.rint((origin-memo['origin'])/m.info.resolution).astype(int);dy,dx=int(shift[1]),int(shift[0]);hh,ww=v.shape
    if abs(dx)<ww and abs(dy)<hh:
     ys=slice(max(0,dy),min(hh,hh+dy));xs=slice(max(0,dx),min(ww,ww+dx));yd=slice(max(0,-dy),min(hh,hh-dy));xd=slice(max(0,-dx),min(ww,ww-dx));remember[yd,xd]=previous[ys,xs]
   remember[(v>=0)&(v<100)]=-1
   confirmation=st['confirmation'];confirmed=np.zeros_like(v,dtype=bool)
   if confirmation is not None and confirmation.header.frame_id=='odom' and abs(t-stamp(confirmation))<.2 and abs(confirmation.info.resolution-m.info.resolution)<1e-6 and confirmation.info.width==m.info.width and confirmation.info.height==m.info.height and math.dist([confirmation.info.origin.position.x,confirmation.info.origin.position.y],origin)<.001:
    confirmed=np.asarray(confirmation.data).reshape(v.shape)==100
   # Persist only three fresh terrain observations. Boundaries are recomputed;
   # moving objects retain the upstream tracker's explicit coasting lifecycle.
   remember[confirmed&(v>=100)]=100;memo.update(cost=remember.copy(),origin=origin)
  fused=fuse(v,pr,remember) if valid else np.full_like(v,-1)
  if valid and t-memo['snapshot']>=5:
   memo['snapshot']=t;np.savez_compressed(a.out/f'memory_{int(t):03}.npz',remembered=remember,live=v,confirmed=confirmed,origin=origin,resolution=m.info.resolution,t=t)
  out=copy.deepcopy(m);out.data=fused.astype('int8').reshape(-1).tolist();pub.publish(out)
  global_grid=copy.deepcopy(m);global_grid.header.frame_id='map'
  if valid:
   transformed,origin=in_map(fused,np.array([m.info.origin.position.x,m.info.origin.position.y]),m.info.resolution,np.array([[co,-si],[si,co]]),np.array([tx,ty]),[g.position.x,g.position.y])
   global_grid.info.origin.position.x=float(origin[0]);global_grid.info.origin.position.y=float(origin[1]);global_grid.data=transformed.astype('int8').reshape(-1).tolist()
  else:global_grid.data=np.full_like(v,-1).astype('int8').reshape(-1).tolist()
  global_pub.publish(global_grid)
  log.write(json.dumps(dict(t=t,admitted=valid,filled_known_static=int(((v<0)&(pr==0)).sum()),retained_lethal=int(np.sum(remember==100)),unknown=int(np.sum(fused<0))))+'\n');log.flush()
 n.create_subscription(OccupancyGrid,'/s09/risk/'+a.vehicle,grid,2)
 def end(s,f):st['stop']=True
 for sig in [signal.SIGINT,signal.SIGTERM]:signal.signal(sig,end)
 (a.out/'ready').write_text('version-bound frozen measured map; unknown preserved; observed lethal wins')
 try:
  while not st['stop']:rclpy.spin_once(n,timeout_sec=.02)
 finally:log.close();n.destroy_node();rclpy.try_shutdown()
if __name__=='__main__':main()
