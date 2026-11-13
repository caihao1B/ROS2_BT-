"""Observed dynamic occupancy memory; no simulator identity or future pose input.

Repeatedly moving tracks leave a bounded, expiring swept occupancy envelope.
The layer can only add obstacles. It cannot grant motion or clear raw geometry.
"""
import math,copy
from collections import deque
import numpy as np

def to_map(packet,rotation,translation):
    """Use only the supervised estimated transform; simulator truth is absent."""
    if not packet or packet.get('frame')!='odom':return None
    converted=copy.deepcopy(packet);converted['frame']='map'
    for item in converted.get('objects',[]):
        xy=rotation@np.asarray(item['position'][:2])+translation
        item['position'][:2]=xy.tolist();item['velocity']=(rotation@np.asarray(item['velocity'][:2])).tolist()
        cov=np.asarray(item['covariance']).reshape(4,4);A=np.zeros((4,4));A[:2,:2]=rotation;A[2:,2:]=rotation
        item['covariance']=(A@cov@A.T).ravel().tolist()
    return converted

class Envelope:
    def __init__(self, ttl=45., resolution=.15):
        self.ttl,self.resolution=ttl,resolution
        self.last=-1.;self.tracks={};self.cells={}
    def update(self,packet,now):
        if now<self.last:self.tracks.clear();self.cells.clear()
        self.last=now
        self.cells={k:v for k,v in self.cells.items() if now-v[0]<=self.ttl}
        if not packet or packet.get('frame')!='map' or not 0<=now-packet.get('t',-100)<=.6:return
        for item in packet.get('objects',[]):
            if not item.get('confirmed') or not 0<=item.get('observed_age_s',100)<=.6:continue
            p=np.asarray(item['position'][:2],float);v=np.asarray(item['velocity'],float);size=np.asarray(item['size'][:2],float);C=np.asarray(item['covariance'],float).reshape(4,4)
            if not np.isfinite(np.r_[p,v,size,C.ravel()]).all() or np.min(size)<=0 or np.max(size)>4:continue
            t=float(item.get('observation_t',packet['t']));key=item['id'];history,moving=self.tracks.get(key,(deque(),False))
            if history and t<=history[-1][0]:continue
            history.append((t,p));
            while history and t-history[0][0]>3.:history.popleft()
            moving=moving or (len(history)>=3 and t-history[0][0]>=.6 and np.linalg.norm(p-history[0][1])>.35 and .12<=np.linalg.norm(p-history[0][1])/(t-history[0][0])<=2.)
            self.tracks[key]=(history,moving)
            if not moving:continue
            # Physical object half diagonal, localization uncertainty, and a
            # the original 0.1m tracking margin; robot footprint is added by
            # Nav2. Adding a second large robot margin here falsely seals the
            # remaining lane. Quantization is separately bounded in apply().
            radius=.5*float(np.linalg.norm(size))+.1+min(.5,2*math.sqrt(max(0,C[0,0],C[1,1])))
            for ts,q in history:
                cell=tuple(np.floor(q/self.resolution).astype(int));self.cells[cell]=(max(ts,self.cells.get(cell,(-100,0))[0]),max(radius,self.cells.get(cell,(-100,0))[1]))
        self.tracks={k:v for k,v in self.tracks.items() if v[0] and now-v[0][-1][0]<=5.}
        if len(self.cells)>2400:
            self.cells=dict(sorted(self.cells.items(),key=lambda kv:kv[1][0],reverse=True)[:2400])
    def apply(self,cost,x,y,rotation=None,translation=None):
        count=0
        for (ix,iy),(t,radius) in self.cells.items():
            center=np.array([(ix+.5)*self.resolution,(iy+.5)*self.resolution])
            if rotation is not None:center=(center-translation)@rotation
            cx,cy=center
            radius+=self.resolution/math.sqrt(2)
            xs=np.flatnonzero(abs(x-cx)<=radius);ys=np.flatnonzero(abs(y-cy)<=radius)
            if not len(xs) or not len(ys):continue
            block=cost[np.ix_(ys,xs)];mask=(x[xs][None,:]-cx)**2+(y[ys][:,None]-cy)**2<=radius**2
            count+=int(np.sum(mask&(block!=100)));block[mask]=100;cost[np.ix_(ys,xs)]=block
        return count

def inspection_target(state,poses,terminal=False):
    """Move artificial segment boundaries laterally; actual route end is fixed.

    This does not change any point-service target or coverage obligation.
    Only fresh current costmap clearance selects a temporary inspection goal.
    """
    from scipy.ndimage import distance_transform_edt
    grid=state.get('global_map');target=list(poses[-1])
    if terminal or grid is None:return target,0.
    t=grid.header.stamp.sec+grid.header.stamp.nanosec*1e-9
    if grid.header.frame_id!='map' or not 0<=state['t']-t<.6:return target,0.
    values=np.asarray(grid.data).reshape(grid.info.height,grid.info.width);res=grid.info.resolution
    clearance=distance_transform_edt((values>=0)&(values<99))*res
    def distance(p):
        ix=int(math.floor((p[0]-grid.info.origin.position.x)/res));iy=int(math.floor((p[1]-grid.info.origin.position.y)/res))
        return float(clearance[iy,ix]) if 0<=iy<grid.info.height and 0<=ix<grid.info.width else 0.
    # Nav2's inscribed inflated cost already includes the chassis footprint.
    if distance(target)>=.35:return target,0.
    normal=np.array([-math.sin(target[2]),math.cos(target[2])]);candidates=[]
    for offset in [-2.,-1.5,-1.,-.5,.5,1.,1.5,2.]:
        q=[*(np.array(target[:2])+normal*offset),target[2]];free=distance(q)
        if free>=.35:candidates.append((free-.08*abs(offset),q,offset))
    if not candidates:return target,0.
    _,target,offset=max(candidates,key=lambda x:x[0]);return target,offset
