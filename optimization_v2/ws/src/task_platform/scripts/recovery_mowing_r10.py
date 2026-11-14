"""Admission for tool-off endpoint recovery; no path or arrival substitution."""
import math

def yaw_error(a,b):
    return abs(math.atan2(math.sin(a-b),math.cos(a-b)))

def gear_permitted(previous,requested,stopped,tool_stopped):
    return not(previous and requested and previous!=requested) or bool(stopped and tool_stopped)

def arm_direction(previous,requested,phase,stopped,tool_stopped):
    """Select a normal segment's gear before starting its working implement."""
    return requested if phase=='READY' and requested in (-1,1) and stopped and tool_stopped else previous

def observed_cursor(points,pose,cursor,window_m=.20):
    """Advance only through a local prefix, never jump to a nearby return branch."""
    end=cursor;distance=0.
    while end+1<len(points) and distance+math.dist(points[end][:2],points[end+1][:2])<=window_m:
        distance+=math.dist(points[end][:2],points[end+1][:2]);end+=1
    admitted=[i for i in range(cursor,end+1) if math.dist(points[i][:2],pose[:2])<=.15 and yaw_error(points[i][2],pose[2])<=.10]
    return min(admitted,key=lambda i:math.dist(points[i][:2],pose[:2])+.25*yaw_error(points[i][2],pose[2])) if admitted else cursor

def validate_candidate(points,start,goal,cfg):
    from shapely.geometry import shape
    from footprint_r09 import body_polygon
    if len(points)<2 or any(not all(math.isfinite(v) for v in p) for p in points):return False,'MALFORMED_PATH'
    if math.dist(points[0][:2],start[:2])>.15 or yaw_error(points[0][2],start[2])>.10:return False,'START_DISCONTINUITY'
    if math.dist(points[-1][:2],goal[:2])>.05 or yaw_error(points[-1][2],goal[2])>.08:return False,'GOAL_MISMATCH'
    # The authoritative prior is in world coordinates; navigation is ENU.
    allowed=shape(cfg['allowed_prior']).buffer(-cfg['footprint_margin_m'])
    for p in points:
        if not allowed.covers(body_polygon(cfg['vehicle']['footprint'],p[0]+28,p[1]+14,p[2])):return False,'FOOTPRINT_OUTSIDE_PRIOR'
    length=0.
    for a,b in zip(points,points[1:]):
        ds=math.dist(a[:2],b[:2]);angle=yaw_error(a[2],b[2]);length+=ds
        if ds>.4:return False,'PATH_DISCONTINUITY'
        # Arc-length curvature from chord; identical cusp points may keep yaw.
        if ds<1e-5:
            if angle>1e-4:return False,'IN_PLACE_ROTATION'
        elif 2*math.sin(angle/2)/ds>cfg['vehicle']['steering']['max_curvature']+.005:return False,'CURVATURE_EXCEEDED'
        signed=math.atan2(math.sin(b[2]-a[2]),math.cos(b[2]-a[2]));steps=max(1,math.ceil(ds/.04),math.ceil(angle/.02))
        for j in range(1,steps):
            u=j/steps;p=[a[0]+u*(b[0]-a[0]),a[1]+u*(b[1]-a[1]),a[2]+u*signed]
            if not allowed.covers(body_polygon(cfg['vehicle']['footprint'],p[0]+28,p[1]+14,p[2])):return False,'SWEPT_FOOTPRINT_OUTSIDE_PRIOR'
    if length>12:return False,'RECOVERY_TOO_LONG'
    return True,'PRIOR_AND_KINEMATICS_VALID_OBSERVED_RISK_STILL_REQUIRED'
