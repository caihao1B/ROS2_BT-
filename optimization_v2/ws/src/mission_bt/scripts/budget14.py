"""Remaining motion evidence; elapsed wall/sim time never counts as progress."""
import math
import numpy as np

def remaining_arc(poses,xy,prefix_m=None):
    p=np.asarray(poses,float)[:,:2];xy=np.asarray(xy,float)
    if len(p)<2 or not np.isfinite(p).all():return None
    d=np.diff(p,axis=0);length=np.linalg.norm(d,axis=1);total=np.r_[0,np.cumsum(length)]
    u=np.clip(np.divide(np.sum((xy-p[:-1])*d,axis=1),length**2,out=np.zeros(len(d)),where=length>1e-9),0,1)
    distance=np.linalg.norm(p[:-1]+u[:,None]*d-xy,axis=1)
    if prefix_m is not None:distance[total[:-1]>prefix_m]=np.inf
    i=int(np.argmin(distance))
    if distance[i]>.25:return None
    return float(total[-1]-total[i]-u[i]*length[i]+distance[i])

def estimate(job,xy,t,busy,nav_plan=None,live_replan=False):
    initial=max(0.,float(job['planned_s']))
    result=dict(seconds=initial,source='INITIAL_VALIDATED_SEGMENT',remaining_m=None)
    if xy is None:return result
    if job['kind']=='POINT_SERVICE':
        length=math.dist(xy,job['poses'][-1][:2])
        return dict(seconds=length/.12+3.,source='POINT_DISTANCE_AND_SERVICE',remaining_m=length)
    work=job['kind'] in ['COVERAGE','HEADLAND_WORK'];reference=work or (job.get('route_policy')=='QUALIFIED_REFERENCE' and not live_replan);poses=job['poses'];prefix=None
    if reference and not busy:return result
    if not reference:
        n=nav_plan or {};label=n.get('action_id','');age=t-n.get('received_t',-1e9)
        candidate=n.get('poses',[])
        if n.get('frame')!='map' or not (label==job['id'] or label.startswith(job['id']+'_resume_')) or not math.isfinite(age) or age<0 or not candidate:return result
        dh=candidate[-1][2]-poses[-1][2]
        if math.dist(candidate[-1][:2],poses[-1][:2])>.1 or abs(math.atan2(math.sin(dh),math.cos(dh)))>.1:return result
        poses=candidate;prefix=.12*age+.25
    length=remaining_arc(poses,xy,prefix)
    if length is None:return result
    # Same qualified 0.12m/s upper speed bound as the previous budget check.
    # Keep original non-distance overhead; no artificial 150s cap.
    original_length=float(np.linalg.norm(np.diff(np.asarray(job['poses'])[:,:2],axis=0),axis=1).sum())
    overhead=max(0.,initial-original_length/.12)
    # A fixed map-frame geometry does not expire as an ETA reference during
    # cancel/standstill. It NEVER grants motion admission; fresh localization,
    # perception, costmap and lease checks remain in the independent gates.
    source=('REMAINING_WORK_PATH' if work else 'REMAINING_QUALIFIED_REFERENCE') if reference else ('REMAINING_NAV2_PATH' if age<=2 else 'LAST_NAV2_GEOMETRY_FOR_ETA')
    return dict(seconds=length/.12+overhead,source=source,remaining_m=length,overhead_s=overhead,speed_upper_mps=.12,purpose='ETA_ONLY_NOT_MOTION_PERMISSION',geometry_age_s=None if reference else age)
