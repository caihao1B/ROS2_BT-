"""Propose stopping a stalled near-goal approach; never grant arrival/work credit."""
from collections import deque
import math

class ApproachHandoff:
    def __init__(self,policy):self.p=policy;self.reset()
    def reset(self):self.rows=deque();self.best=None;self.improved_t=None
    def update(self,t,position,target,qualified,command_v):
        d=math.dist(position[:2],target[:2])
        if not qualified or not math.isfinite(t+d+command_v) or d>self.p['current_radius_m'] or d<=self.p['navigation_goal_m']:
            self.reset();return None
        if self.rows and t<=self.rows[-1][0]:return None
        if self.rows and t-self.rows[-1][0]>self.p['max_gap_s']:self.reset()
        if self.best is None or d<self.best-self.p['minimum_improvement_m']:self.best=d;self.improved_t=t
        self.rows.append((t,d,command_v))
        while self.rows and t-self.rows[0][0]>self.p['window_s']+.15:self.rows.popleft()
        if len(self.rows)<self.p['minimum_samples'] or t-self.rows[0][0]<self.p['window_s']:return None
        commanded=sum(b[0]-a[0] for a,b in zip(self.rows,list(self.rows)[1:]) if min(abs(a[2]),abs(b[2]))>.003)
        if self.best<=self.p['visited_radius_m'] and t-self.improved_t>=self.p['stall_s'] and commanded>=self.p['translation_command_s']:
            return dict(reason='NEAR_APPROACH_STALLED_STOP_FOR_PRECISION',distance_m=d,best_distance_m=self.best,seconds_without_improvement=t-self.improved_t,translation_command_seconds=commanded,samples=len(self.rows),navigation_success=False,work_admitted=False)
        return None
