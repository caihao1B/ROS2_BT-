"""Experimental leader-distance speed proposal, not a robot command publisher.

Initial qualification scope: a short, straight, known-free task corridor.
Uses only observed tracks in the same odom frame as ego. ROS/BT integration and
curved-corridor qualification remain NOT_RUN. All emergency gates keep priority.
"""
import math
import numpy as np

class LeaderFollower:
    def __init__(self):
        self.leader=None;self.last=None;self.clear_since=None
        self.max_speed=.12;self.base_gap=1.2;self.headway=2.0
        self.max_age=.3;self.max_sigma=.3;self.kp=.35
    def update(self,now,ego,tracks,*,observations_valid,path_clear=False):
        def result(state,cap,**extra):return dict(state=state,speed_cap_mps=cap,
            selected_id=self.leader,tool_must_be_off=True,
            scope='OFFLINE_CORE_NOT_ROS_QUALIFIED',**extra)
        if not math.isfinite(now) or (self.last is not None and now<=self.last):
            self.last=now;return result('CLOCK_INVALID_STOP',0.)
        self.last=now
        if not observations_valid:return result('OBSERVATION_INVALID_STOP',0.)
        try:
            pose=np.asarray(ego['pose'],float);v=float(ego['speed']);fp=np.asarray(ego['footprint'],float)
            if pose.shape!=(3,) or not np.isfinite(pose).all() or not math.isfinite(v) or v<0 or ego['frame']!='odom':raise ValueError()
            if fp.ndim!=2 or fp.shape[0]<3 or fp.shape[1]!=2 or not np.isfinite(fp).all():raise ValueError()
            tangent=np.array([math.cos(pose[2]),math.sin(pose[2])]);normal=np.array([-tangent[1],tangent[0]])
            front=float(fp[:,0].max());candidates=[];known=None
            for tr in tracks:
                age=now-float(tr['observation_t']);pos=np.asarray(tr['position'][:2],float);vel=np.asarray(tr['velocity'],float)
                size=np.asarray(tr['size'][:2],float);cov=np.asarray(tr['covariance'],float).reshape(4,4)
                if not np.isfinite(cov).all() or not np.allclose(cov,cov.T,atol=1e-9) or np.linalg.eigvalsh(cov).min() < -1e-9:raise ValueError()
                sigma=math.sqrt(max(0.,float(np.linalg.eigvalsh(cov[:2,:2]).max())))
                if not np.isfinite(np.r_[pos,vel,size,cov.ravel()]).all() or np.any(size<=0):raise ValueError()
                fresh=tr['frame']=='odom' and -.05<=age<=self.max_age and tr['state']=='OBSERVED' and tr['confirmed'] and sigma<=self.max_sigma
                delta=pos-pose[:2];along=float(delta@tangent);lateral=float(delta@normal)
                half_length=float(np.abs(tangent)@size/2);half_width=float(np.abs(normal)@size/2)
                corridor=along>0 and abs(lateral)<=.7+half_width+2*sigma
                longitudinal=float(vel@tangent);side=abs(float(vel@normal))
                direction=longitudinal>=.02 and side<=max(.02,longitudinal*math.tan(math.radians(25)))
                item=dict(id=tr['id'],fresh=fresh,corridor=corridor,gap_m=along-front-half_length-2*sigma,
                          leader_speed_mps=longitudinal,sigma_m=sigma,parallel_or_stopped=direction or np.linalg.norm(vel)<.02)
                if tr['id']==self.leader:known=item
                if fresh and corridor and direction:candidates.append(item)
        except (KeyError,TypeError,ValueError,IndexError,np.linalg.LinAlgError):
            return result('TRACK_FORMAT_INVALID_STOP',0.)
        if self.leader is not None:
            if known is None or not known['fresh']:
                self.clear_since=None;return result('TARGET_UNCERTAIN_STOP',0.)
            if not known['corridor']:
                if not path_clear:self.clear_since=None;return result('CLEARANCE_CHECK_REQUIRED',0.)
                if self.clear_since is None:self.clear_since=now
                if now-self.clear_since<1.:return result('CLEAR_HYSTERESIS_WAIT',0.)
                self.leader=None;self.clear_since=None;return result('RESUME_TASK',None)
            if not known['parallel_or_stopped']:
                self.clear_since=None;return result('LEADER_DIRECTION_CHANGED_STOP',0.)
            self.clear_since=None
            chosen=known  # Retain a stopped leader; never silently jump to another ID.
        else:
            if not candidates:return result('NO_LEADER_TASK_GATES_APPLY',None)
            chosen=min(candidates,key=lambda x:x['gap_m']);self.leader=chosen['id']
        gap=chosen['gap_m'];vl=chosen['leader_speed_mps']
        # Conservative stopping reserve assumes the leader may stop immediately.
        safe=.7+v*.4+v*v/(2*.5)
        target=max(self.base_gap+self.headway*v,safe)
        extra=dict(gap_m=gap,target_gap_m=target,minimum_gap_m=safe,leader_speed_mps=vl,ego_speed_mps=v)
        if gap<=safe or vl<-.02:return result('GAP_OR_APPROACH_STOP',0.,**extra)
        cap=max(0.,min(self.max_speed,vl+self.kp*(gap-target)))
        if cap<.02:return result('WAIT_LEADER',0.,**extra)
        return result('FOLLOW_LEADER',cap,**extra)
