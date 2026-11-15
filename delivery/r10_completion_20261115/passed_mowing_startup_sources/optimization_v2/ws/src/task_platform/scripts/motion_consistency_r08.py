"""Independent GNSS displacement adjudication, with covariance and epoch matching.
No global pose enters local odometry: only acceptance/rejection of a lidar increment.
"""
import numpy as np,math
from scipy.spatial.transform import Rotation,Slerp

def assess(history,fixes,wheels,t):
 out=dict(available=False,icp_valid=False,wheel_valid=False,reason='GNSS_MOTION_EVIDENCE_UNAVAILABLE')
 if len(history)<2:return out
 eligible=[g for g in fixes if g.get('valid') and g.get('mode')=='fixed' and history[0][0]<=g['t']<=history[-1][0] and -.03<=t-g['t']<=1.2]
 if len(eligible)<3:return out
 a,b=eligible[0],eligible[-1];dt=b['t']-a['t']
 if dt<.39 or t-b['t']>.6:return out
 ht=np.array([x[0] for x in history]);positions=np.array([x[1] for x in history]);cov=np.array([x[3] for x in history]);quats=np.array([x[2] for x in history]);R=Slerp(ht,Rotation.from_quat(quats))([a['t'],b['t']]);lever=np.asarray(b['antenna_base_m']);ipos=np.array([[np.interp(u,ht,positions[:,j]) for j in range(3)] for u in [a['t'],b['t']]])+R.apply(np.tile(lever,(2,1)));lidar_delta=np.linalg.norm((ipos[1]-ipos[0])[:2]);gnss_delta=np.linalg.norm((np.array(b['antenna'])-a['antenna'])[:2]);C=sum(np.array(g['covariance']).reshape(3,3)[:2,:2] for g in [a,b])
 if not np.isfinite(C).all() or np.linalg.eigvalsh(C).min()<=0:return out
 gv=float(np.linalg.eigvalsh(C).max());lv=sum(max(np.interp(u,ht,cov[:,0]),np.interp(u,ht,cov[:,1]),.00125) for u in [a['t'],b['t']]);nis=(lidar_delta-gnss_delta)**2/(gv+lv);out.update(available=True,icp_valid=bool(nis<=9.21),reason='INDEPENDENT_GNSS_DISPLACEMENT_VALID' if nis<=9.21 else 'ICP_GNSS_MOTION_CONFLICT',start=a['t'],end=b['t'],lidar_distance_m=float(lidar_delta),gnss_distance_m=float(gnss_delta),icp_nis=float(nis))
 w=np.asarray(wheels)
 if len(w)>2 and w[0,0]<=a['t'] and w[-1,0]>=b['t']:
  ts=np.r_[a['t'],w[(w[:,0]>a['t'])&(w[:,0]<b['t']),0],b['t']];vel=np.interp(ts,w[:,0],w[:,1]);distance=abs(float(np.sum((vel[1:]+vel[:-1])*.5*np.diff(ts))));omega=(R[0].inv()*R[1]).magnitude();lever_bound=np.linalg.norm(lever)*omega;residual=max(0.,abs(distance-gnss_delta)-lever_bound);wv=.0025*dt*dt;wnis=residual*residual/(gv+wv);out.update(wheel_valid=bool(wnis<=9.21),wheel_distance_m=distance,wheel_nis=wnis,lever_rotation_bound_m=float(lever_bound))
 return out
