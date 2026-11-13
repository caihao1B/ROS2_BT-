"""Select online navigation for an inspection part near observed moving objects.

Only current timestamped tracks and sensor-based map/odom poses are used. This
is action selection, not a prediction model and never a motion permission.
"""
import math,numpy as np
from scipy.spatial.transform import Rotation

def pose(message):
    p=message.pose.pose;q=p.orientation
    return np.array([p.position.x,p.position.y,p.position.z]),Rotation.from_quat([q.x,q.y,q.z,q.w])

def conflicts(state,poses):
    packet=state.get('inspection_objects') or {};t=state['t']
    if packet.get('frame')!='odom' or not 0<=t-packet.get('t',-100)<.6 or not state.get('pose') or not state.get('odom'):return []
    gp,gq=pose(state['pose']);op,oq=pose(state['odom']);rotation=gq*oq.inv();translation=gp-rotation.apply(op)
    path=np.asarray(poses,float)[:,:2];result=[]
    if len(path)<2:return result
    a=path[:-1];d=np.diff(path,axis=0);denom=np.sum(d*d,axis=1)
    for item in packet.get('objects',[]):
        if not item.get('confirmed') or item.get('observed_age_s',1)>.6:continue
        position=np.asarray(item['position'],float);speed=np.linalg.norm(item['velocity'])
        if not np.isfinite(position).all() or not math.isfinite(speed) or not .1<=speed<=2.:continue
        p=rotation.apply(position)+translation
        fraction=np.clip(np.divide(np.sum((p[:2]-a)*d,axis=1),denom,out=np.zeros(len(d)),where=denom>1e-12),0,1)
        distance=float(np.min(np.linalg.norm(a+fraction[:,None]*d-p[:2],axis=1)))
        if distance<=2.5 and np.linalg.norm(p[:2]-gp[:2])<=8:
            result.append(dict(id=item['id'],distance_to_reference_m=distance,speed_mps=float(speed),position_map=p.tolist(),observation_t=item.get('observation_t',packet['t'])))
    return result
