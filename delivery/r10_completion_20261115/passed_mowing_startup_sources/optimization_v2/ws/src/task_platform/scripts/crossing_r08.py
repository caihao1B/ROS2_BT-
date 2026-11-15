"""Short-horizon crossing interlock, not a learned pedestrian intention model."""
import numpy as np,math
from shapely.geometry import LineString,Point

class MotionMemory:
 """Preserve an observed moving state through a turn or brief stop; never invent tracks."""
 def __init__(self,hold_s=10.):self.hold_s=hold_s;self.last={}
 def update(self,t,tracks):
  active={x['id'] for x in tracks}
  self.last={k:v for k,v in self.last.items() if k in active and t-v<=self.hold_s}
  for tr in tracks:
   v=np.asarray(tr['velocity']);P=np.asarray(tr['covariance']).reshape(4,4)
   if tr['confirmed'] and tr['state']=='OBSERVED' and tr['observed_age_s']<.25 and np.linalg.norm(v)>=.1 and float(v@np.linalg.solve(P[2:,2:]+np.eye(2)*1e-8,v))>=9.21:self.last[tr['id']]=t
  return set(self.last)

def crossing(path,ego,speed,position,velocity,covariance,size,footprint,recent_motion=False):
 line=LineString(np.asarray(path)[:,:2]);s=line.project(Point(ego));near=np.asarray(line.interpolate(min(line.length,s+.2)).coords[0])-np.asarray(line.interpolate(max(0,s-.2)).coords[0]);tangent=near/max(1e-6,np.linalg.norm(near));v=np.asarray(velocity);side=abs(v@np.array([-tangent[1],tangent[0]]));forward=v@tangent
 P=np.asarray(covariance).reshape(4,4)
 motion_nis=float(v@np.linalg.solve(P[2:,2:]+np.eye(2)*1e-8,v))
 on_path=line.distance(Point(position))<max(abs(y) for x,y in footprint)+.1+max(size[:2])/2
 ahead=line.project(Point(position))>s
 if not recent_motion and (np.linalg.norm(v)<.1 or motion_nis<9.21):return dict(conflict=False,reason='NO_CONFIRMED_TRANSLATIONAL_MOTION',motion_nis=motion_nis)
 if on_path and ahead and side<.15 and forward>=0 and (not recent_motion or np.linalg.norm(v)>=.1 and motion_nis>=9.21):return dict(conflict=False,reason='SAME_DIRECTION_LEADER_BRANCH',motion_nis=motion_nis)
 if math.dist(position,ego)>7:return dict(conflict=False,reason='OUTSIDE_SHORT_HORIZON')
 radius=max(math.hypot(x,y) for x,y in footprint)+math.hypot(*size[:2])/2+.2;minimum=100.;hit=None
 for dt in np.linspace(.2,2.,10):
  A=np.array([[1.,0.,dt,0.],[0.,1.,0.,dt]])
  # Propagate future white-acceleration uncertainty (q = 0.8 m^2/s^3).
  # This conservative prediction model is separate from tracker update noise.
  variance=A@P@A.T+np.eye(2)*(.8*dt**3/3.)
  sigma=math.sqrt(max(0,float(np.linalg.eigvalsh(variance).max())));margin=radius+2*sigma+.075*dt*dt
  moving=np.asarray(line.interpolate(min(line.length,s+max(.15,min(.4,speed))*dt)).coords[0]);obstacle=np.asarray(position)+dt*v
  # A stopped robot cannot be assumed to move out of a crossing pedestrian's way.
  clearance=min(float(np.linalg.norm(robot-obstacle)-margin) for robot in [np.asarray(ego),moving]);minimum=min(minimum,clearance)
  if clearance<=0 and hit is None:hit=float(dt)
 return dict(conflict=hit is not None,first_conflict_s=hit,minimum_predicted_margin_m=minimum,horizon_s=2.,motion_nis=motion_nis,acceleration_spectral_density_m2_s3=.8,ego_hypotheses=['stationary','bounded_next_motion'],source='observed CV with future process covariance; no fixture or intention claim')
