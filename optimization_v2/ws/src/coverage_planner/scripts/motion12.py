"""Independent swept-envelope, nonholonomic, steering and timed-path checks."""
import shapely
from common12 import *
def normalize_poses(poses):
    clean=[]
    for p in poses:
        if clean and math.dist(p[:2],clean[-1][:2])<1e-8 and abs(wrap(p[2]-clean[-1][2]))<1e-7:continue
        clean.append(list(p))
    return clean

def straight(a,b,step):
    a=np.asarray(a,dtype=float);b=np.asarray(b,dtype=float);length=np.linalg.norm(b-a);theta=math.atan2(b[1]-a[1],b[0]-a[0]);n=max(2,math.ceil(length/step))
    return [[*(a+(b-a)*i/n),theta] for i in range(n+1)]
def pose_metrics(poses):
    p=np.asarray(poses,dtype=float);ds=np.linalg.norm(np.diff(p[:,:2],axis=0),axis=1);da=np.array([wrap(v) for v in np.diff(p[:,2])]);k=np.divide(da,ds,out=np.zeros_like(ds),where=ds>1e-8)
    mid=p[:-1,2]+da/2;tangent=np.arctan2(np.diff(p[:,1]),np.diff(p[:,0]));slip=np.array([abs(wrap(x-y)) for x,y in zip(mid,tangent)])
    return p,ds,da,k,slip

def swept_check(poses,motion,vehicle,c,rotation=False):
    p,ds,da,k,slip=pose_metrics(poses);local,v,t=body_local(vehicle);radius=float(np.max(np.linalg.norm(local,axis=1)))
    if len(p)<1 or not np.isfinite(p).all():return dict(passed=False,reason='INVALID_POSES')
    # Every interpolated point is within half the translation+rotation travel of a sample.
    pad=c['safety_margin_m']+c['sweep_numerical_margin_m']+float(np.max(ds+radius*np.abs(da),initial=0))/2
    ca=np.cos(p[:,2])[:,None];sa=np.sin(p[:,2])[:,None];x=p[:,0,None]+ca*local[:,0]-sa*local[:,1];y=p[:,1,None]+sa*local[:,0]+ca*local[:,1]
    footprints=shapely.polygons(np.stack([x,y],axis=-1));padded=shapely.buffer(footprints,pad/math.cos(math.pi/16),quad_segs=4)  # circumscribe the requested disk despite polygonal arcs
    safe=shapely.covers(motion,padded);clearance=np.array(shapely.distance(footprints,motion.boundary));minimum=float(clearance.min()) if len(clearance) else 0
    maxk=float(np.max(np.abs(k),initial=0));yaw_rotation=bool(np.any((ds<1e-7)&(np.abs(da)>1e-6)))
    kin=(vehicle=='diff' and rotation) or (not yaw_rotation and maxk<=1/2.11+.001 if vehicle=='ackermann' else not yaw_rotation)
    if not rotation:kin=kin and float(np.max(slip,initial=0))<.045
    return dict(passed=bool(np.all(safe)) and kin,collision_free=bool(np.all(safe)),kinematics=bool(kin),min_clearance_m=minimum,
      max_curvature=maxk,max_lateral_angle_error=float(np.max(slip,initial=0)),sweep_padding_m=pad,bad_indices=np.flatnonzero(~safe)[:8].tolist(),stationary_yaw_change=yaw_rotation)

def profile(poses,vehicle,c,rotation=False,stop_curvature=False):
    p,ds,da,k,slip=pose_metrics(poses);_,v,t=body_local(vehicle);speed=effective_speed(vehicle)
    if rotation:
        omega=min(.3,v['limits']['yaw_rate']);angle=float(np.sum(np.abs(da)));acc=v['limits']['yaw_acceleration'];peak=min(omega,math.sqrt(angle*acc));seconds=angle/peak+peak/acc if angle>0 else 0
        tr=peak/acc if peak else 0.;ar=.5*peak*peak/acc;angles=np.r_[0,np.cumsum(np.abs(da))];times=[];rates=[]
        for x in angles:
            if angle==0:t0=0.;rate=0.
            elif x<=ar:t0=math.sqrt(2*x/acc);rate=acc*t0
            elif x>=angle-ar:t0=seconds-math.sqrt(max(0,2*(angle-x)/acc));rate=acc*(seconds-t0)
            else:t0=tr+(x-ar)/peak;rate=peak
            times.append(float(t0));rates.append(float(rate))
        return dict(length_m=0.,motion_seconds=seconds,max_speed=0.,max_yaw_rate=peak,max_yaw_acceleration=acc if angle else 0.,max_acceleration=0.,max_deceleration=0.,steering_rate=0.,velocities=[0.]*len(p),yaw_rates=rates,sample_times=times)
    limits=np.full(len(p),speed);vel=limits.copy();vel[0]=0;vel[-1]=0;steering_events=[]
    if vehicle=='ackermann' and stop_curvature:
        for i in np.flatnonzero(np.abs(np.diff(k))>.02)+1:vel[i]=0
        wheel=lambda curv: math.atan(v['wheelbase']*curv/(1-v['wheel']['track']/2*abs(curv)))
        for i in [0,*list(np.flatnonzero(np.abs(np.diff(k))>.02)+1),len(p)-1]:
            before=wheel(k[i-1]) if i>0 and len(k) else 0.;after=wheel(k[i]) if i<len(k) else 0.
            steering_events.append(dict(index=int(i),seconds=abs(after-before)/v['steering']['rate_rad_s']+.2,from_inner_rad=before,to_inner_rad=after,requires_measured_stopped=True))
    acc=v['limits']['acceleration'];dec=v['limits']['deceleration']
    for i,s in enumerate(ds):vel[i+1]=min(vel[i+1],math.sqrt(vel[i]**2+2*acc*s))
    for i in range(len(ds)-1,-1,-1):vel[i]=min(vel[i],math.sqrt(vel[i+1]**2+2*dec*ds[i]))
    dt=np.divide(2*ds,vel[:-1]+vel[1:],out=np.zeros_like(ds),where=(vel[:-1]+vel[1:])>1e-9)
    # Very short two-point segment: triangular acceleration/deceleration profile.
    for i in np.flatnonzero((ds>1e-9)&((vel[:-1]+vel[1:])<1e-9)):
        peak=min(speed,math.sqrt(2*ds[i]/(1/acc+1/dec)));dt[i]=ds[i]/peak+peak*.5*(1/acc+1/dec)
    times=np.r_[0,np.cumsum(dt)]
    for e in steering_events:
        e['arrival_s']=float(times[e['index']]);times[e['index']+1:]+=e['seconds']
    maxyaw=float(np.max(np.abs(k)*(vel[:-1]+vel[1:])/2,initial=0));steer_rate=0.
    if vehicle=='ackermann' and len(k)>1:
        inner=np.arctan(v['wheelbase']*k/(1-v['wheel']['track']/2*np.abs(k)))
        rates=np.abs(np.diff(inner))/np.maximum((dt[:-1]+dt[1:])/2,1e-6)
        if stop_curvature:rates[np.abs(np.diff(k))>.02]=0
        steer_rate=float(np.max(rates,initial=0))
    forward=np.divide(vel[1:]**2-vel[:-1]**2,2*ds,out=np.zeros_like(ds),where=ds>1e-9)
    return dict(length_m=float(ds.sum()),motion_seconds=float(times[-1])+sum(e['seconds'] for e in steering_events if e['index']==len(p)-1),steering_events=steering_events,max_speed=float(max(vel)),max_yaw_rate=maxyaw,max_acceleration=float(np.max(forward,initial=0)),max_deceleration=float(-np.min(forward,initial=0)),steering_rate=steer_rate,velocities=vel.tolist(),sample_times=times.tolist())

def segment(poses,kind,vehicle,motion,c,tool=False,**extra):
    poses=normalize_poses(poses);rotation=kind=='ROTATE';check=swept_check(poses,motion,vehicle,c,rotation);timing=profile(poses,vehicle,c,rotation,extra.get('stop_curvature',False))
    _,v,t=body_local(vehicle);check['speed_acceleration']=timing['max_speed']<=effective_speed(vehicle)+1e-9 and timing['max_acceleration']<=v['limits']['acceleration']+1e-6 and timing['max_deceleration']<=v['limits']['deceleration']+1e-6
    check['steering_rate']=timing['steering_rate']<=v['steering']['rate_rad_s']+1e-3
    check['yaw_rate']=timing['max_yaw_rate']<=(.097 if tool else v['limits']['yaw_rate'])+1e-6
    check['passed']=check['passed'] and check['speed_acceleration'] and check['steering_rate'] and check['yaw_rate']
    return dict(kind=kind,vehicle=vehicle,tool_on=tool,poses=poses,timing=timing,validation=check,**extra)

def rotate_at(pose,heading,motion,c):
    da=wrap(heading-pose[2]);n=max(1,math.ceil(abs(da)/.08));return segment([[pose[0],pose[1],pose[2]+da*i/n] for i in range(n+1)],'ROTATE','diff',motion,c)

def summarize(segments,c,service_count=0):
    motion=sum(s['timing']['motion_seconds'] for s in segments);working=sum(s['timing']['motion_seconds'] for s in segments if s['tool_on']);work_segments=sum(s['kind'] in ['COVERAGE','HEADLAND_WORK'] for s in segments)
    tools=(work_segments+service_count)*(c['startup_seconds']+c['shutdown_seconds']);dwell=service_count*(c['spray_dwell_seconds']+c['spray_docking_seconds']);subtotal=motion+tools+dwell+c['cluster_setup_seconds']
    return dict(motion_s=motion,work_motion_s=working,tool_switch_s=tools,dwell_and_dock_s=dwell,setup_s=c['cluster_setup_seconds'],reserve_s=subtotal*c['time_reserve_fraction'],total_s=subtotal*(1+c['time_reserve_fraction']),length_m=sum(s['timing']['length_m'] for s in segments),all_segments_valid=all(s['validation']['passed'] for s in segments),actual_sim_seconds=None)

def effect_polygon(poses,c):
    # S10 observed effective rectangle, eroded by the same position/yaw budget.
    tc=yaml.safe_load((BASE/'config/tool_contract.yaml').read_text());er=tc['pose_erosion_m']+math.hypot(.35,.45)*tc['yaw_erosion_rad']+tc['ledger_precision_m'];local=np.asarray(box(.5-.35+er,-.45+er,.5+.35-er,.45-er).exterior.coords)[:-1]
    p=np.asarray(poses)
    if len(p)==0:return Polygon()
    ca=np.cos(p[:,2])[:,None];sa=np.sin(p[:,2])[:,None];coords=np.stack([p[:,0,None]+ca*local[:,0]-sa*local[:,1],p[:,1,None]+sa*local[:,0]+ca*local[:,1]],axis=-1)
    return union_all(shapely.polygons(coords))
