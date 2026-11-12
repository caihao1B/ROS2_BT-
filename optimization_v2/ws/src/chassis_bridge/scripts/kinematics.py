"""Encoder-only kinematics. No world state or evaluation data dependencies."""
import math
def limit(v,w,c):
    if not all(math.isfinite(x) for x in (v,w)):return 0.,0.,'NONFINITE'
    lim=c['limits'];v=max(-lim['reverse'],min(lim['max_speed'],v));w=max(-lim['yaw_rate'],min(lim['yaw_rate'],w))
    if c['kind']=='ackermann':
        if abs(v)<.001:return 0.,0.,'ZERO_SPEED_YAW_REJECTED' if w else 'STOP'
        w=v*max(-c['steering']['max_curvature'],min(c['steering']['max_curvature'],w/v))
    return v,w,'OK'
def steering(k,c):
    L,T=c['wheelbase'],c['wheel']['track'];return math.atan(L*k/(1-T*k/2)),math.atan(L*k/(1+T*k/2))
def curvature(left,right,c):
    L,T=c['wheelbase'],c['wheel']['track'];a,b=math.tan(left),math.tan(right)
    return .5*(a/(L+T*a/2)+b/(L-T*b/2))
def twist(qvel,q,c):
    r,T=c['wheel']['radius'],c.get('calibration',{}).get('effective_track_m',c['wheel']['track']);v=r*(qvel[0]+qvel[1])/2
    return v,(r*(qvel[1]-qvel[0])/T if c['kind']=='differential' else v*curvature(q[2],q[3],c))
def integrate(p,ds,dyaw):
    x,y,yaw=p
    dx,dy=(ds,0) if abs(dyaw)<1e-9 else (ds*math.sin(dyaw)/dyaw,ds*(1-math.cos(dyaw))/dyaw)
    return [x+math.cos(yaw)*dx-math.sin(yaw)*dy,y+math.sin(yaw)*dx+math.cos(yaw)*dy,yaw+dyaw]
def encoder_delta(previous,current,c):
    r,T=c['wheel']['radius'],c.get('calibration',{}).get('effective_track_m',c['wheel']['track']);dl=(current[0]-previous[0])*r;dr=(current[1]-previous[1])*r;ds=(dl+dr)/2
    return ds,(dr-dl)/T if c['kind']=='differential' else ds*curvature((previous[2]+current[2])/2,(previous[3]+current[3])/2,c)
