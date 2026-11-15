"""Bounded near-goal progress evidence, using qualified poses and gated commands only."""
from collections import deque
import math

class NearGoalProgress:
 def __init__(self,policy):self.p=policy;self.clear()
 def clear(self):self.samples=deque();self.best=None;self.best_t=None
 def update(self,t,position,target,qualified,command_v):
  d=math.dist(position[:2],target[:2])
  if not qualified or not math.isfinite(t+d+command_v) or d>=self.p['near_radius_m']:
   self.clear();return None
  if self.samples and t<=self.samples[-1][0]:return None
  if self.samples and t-self.samples[-1][0]>self.p['maximum_sample_gap_s']:self.clear()
  if self.best is None or d<self.best-self.p['minimum_improvement_m']:self.best=d;self.best_t=t
  self.samples.append((t,command_v))
  while self.samples and t-self.samples[0][0]>self.p['stall_s']+.5:self.samples.popleft()
  if len(self.samples)<20 or t-self.samples[0][0]<self.p['stall_s']-.1:return None
  translation=sum(b[0]-a[0] for a,b in zip(self.samples,list(self.samples)[1:]) if min(abs(a[1]),abs(b[1]))>.003)
  if t-self.best_t>=self.p['stall_s'] and translation>=self.p['minimum_translation_command_s']:
   return dict(reason='QUALIFIED_NEAR_GOAL_WITHOUT_DISTANCE_PROGRESS',distance_m=d,best_distance_m=self.best,seconds_without_improvement=t-self.best_t,translation_command_seconds=translation,qualified_samples=len(self.samples))
  return None
