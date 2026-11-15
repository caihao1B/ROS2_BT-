"""Preserve original path geometry; group only contiguous same-gear tool-off motion."""
import copy,math

def close_pose(a,b):
 return math.dist(a[:2],b[:2])<1e-6 and abs(math.atan2(math.sin(a[2]-b[2]),math.cos(a[2]-b[2])))<1e-6

def navigation_groups(segments):
 groups=[]
 for original in segments:
  s=copy.deepcopy(original)
  merge=bool(groups and s['kind']=='TURN_TOOL_OFF' and not s['tool_on'] and groups[-1]['kind']=='TURN_TOOL_OFF' and not groups[-1]['tool_on'] and groups[-1]['direction']==s['direction'] and close_pose(groups[-1]['poses'][-1],s['poses'][0]))
  member=dict(id=s['id'],endpoint=copy.deepcopy(s['poses'][-1]))
  if merge:
   groups[-1]['poses']+=s['poses'][1:];groups[-1]['poses_world']+=s['poses_world'][1:];groups[-1]['members'].append(member)
  else:s['members']=[member];groups.append(s)
 assert [m['id'] for g in groups for m in g['members']]==[s['id'] for s in segments]
 return groups

def execution_groups(segments,mode):
 if mode=='original_primitive_boundaries':return [navigation_groups([s])[0] for s in segments]
 if mode=='contiguous_same_gear_tool_off_groups':return navigation_groups(segments)
 raise ValueError('UNKNOWN_MOWING_EXECUTION_MODE')

def passage(p,q):
 d=math.dist(p[:2],q[:2]);h=abs(math.atan2(math.sin(p[2]-q[2]),math.cos(p[2]-q[2])))
 return d<=.10 and h<=.12,d,h

def retry_exhausted(attempts,member_count,done):
 # Preserve six attempts per original obligation; success is terminal before failure limits.
 return not done and attempts>6*member_count
