"""Filled footprint sweep of only the CURRENT executable path prefix."""
import numpy as np,math
from path_envelope_r08 import envelope_free

def validate(cost,origin,resolution,odom_pose,map_pose,transform,path,footprint,horizon=1.,margin=.1):
 if len(path)<2:return dict(valid=False,reason='NO_ACTIVE_PATH',scope='current executable prefix only')
 path=np.asarray(path);i=int(np.argmin(np.linalg.norm(path[:,:2]-map_pose[:2],axis=1)))
 if math.dist(path[i,:2],map_pose[:2])>.65:return dict(valid=False,reason='PATH_JOIN_TOO_FAR')
 route=[np.asarray(map_pose)];travel=0.
 for q in path[min(i+1,len(path)-1):]:
  length=math.dist(route[-1][:2],q[:2])
  if length<1e-6:continue
  if travel+length>horizon:
   u=(horizon-travel)/length;angle=math.atan2(q[1]-route[-1][1],q[0]-route[-1][0]);route.append(np.array([*(route[-1][:2]+u*(q[:2]-route[-1][:2])),angle]));travel=horizon;break
  route.append(q);travel+=length
 if len(route)<2:return dict(valid=False,reason='NO_FORWARD_PREFIX')
 fp=np.asarray(footprint);x=np.linspace(fp[:,0].min()-margin,fp[:,0].max()+margin,max(2,math.ceil((np.ptp(fp[:,0])+2*margin)/.05)+1));y=np.linspace(fp[:,1].min()-margin,fp[:,1].max()+margin,max(2,math.ceil((np.ptp(fp[:,1])+2*margin)/.05)+1));xx,yy=np.meshgrid(x,y);body=np.c_[xx.ravel(),yy.ravel()];samples=[]
 for a,b in zip(route,route[1:]):
  dh=math.atan2(math.sin(b[2]-a[2]),math.cos(b[2]-a[2]));n=max(1,math.ceil(math.dist(a[:2],b[:2])/.05),math.ceil(abs(dh)/.06))
  for u in np.linspace(0,1,n+1):
   h=a[2]+u*dh;C=np.array([[math.cos(h),-math.sin(h)],[math.sin(h),math.cos(h)]]);samples.append(body@C.T+a[:2]+u*(b[:2]-a[:2]))
 # Include the start rotation even when RPP chooses rotate-to-heading.
 h=math.atan2(route[1][1]-route[0][1],route[1][0]-route[0][0]);dh=math.atan2(math.sin(h-map_pose[2]),math.cos(h-map_pose[2]))
 for u in np.linspace(0,1,max(2,math.ceil(abs(dh)/.06)+1)):
  a=map_pose[2]+u*dh;C=np.array([[math.cos(a),-math.sin(a)],[math.sin(a),math.cos(a)]]);samples.append(body@C.T+map_pose[:2])
 h=transform[2];R=np.array([[math.cos(h),-math.sin(h)],[math.sin(h),math.cos(h)]]);points=(np.concatenate(samples)-np.asarray(transform[:2]))@R;ij=np.floor((points-np.asarray(origin))/resolution).astype(int)
 inside=(ij[:,0]>=0)&(ij[:,1]>=0)&(ij[:,0]<cost.shape[1])&(ij[:,1]<cost.shape[0]);scope='current executable prefix; remaining prior candidate is not certified'
 if not inside.all():return dict(valid=False,reason='PREFIX_OUTSIDE_OBSERVATIONS',scope=scope)
 values=cost[ij[:,1],ij[:,0]];ok=envelope_free(values,points,odom_pose,footprint,0.)
 evidence_points={}
 if not ok:
  for name,mask in [('unknown',values<0),('hazard',values>=100)]:
   cells=np.unique(ij[mask],axis=0)[:30];evidence_points[name+'_odom_cells']=(np.asarray(origin)+(cells+.5)*resolution).tolist()
 return dict(**evidence_points,valid=ok,reason='OBSERVED_FULL_FOOTPRINT_PREFIX' if ok else 'PREFIX_UNKNOWN_OR_OBSTACLE',scope=scope,length_m=travel,samples=len(points),unknown_samples=int((values<0).sum()),hazard_samples=int((values>=100).sum()))
