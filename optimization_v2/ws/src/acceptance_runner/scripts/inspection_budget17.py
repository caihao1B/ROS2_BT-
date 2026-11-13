"""Remaining inspection ETA from completed parts and current Nav2 geometry.

No elapsed time earns progress. This is scheduling evidence only; it cannot
admit motion, bypass observations, or extend the case's hard deadline.
"""
import math

def length(poses):return sum(math.dist(a[:2],b[:2]) for a,b in zip(poses,poses[1:]))

def estimate(job,xy,t,busy,nav_plan,completed=(),executed=None):
    from budget14 import estimate as segment_estimate
    done=set(completed);executed=executed or {};parts=[p for p in job['reference_parts'] if p['id'] not in done]
    if not parts:return dict(seconds=0.,source='ALL_INSPECTION_PARTS_CONFIRMED',remaining_m=0.,purpose='ETA_ONLY_NOT_MOTION_PERMISSION')
    # Retain the original non-distance overhead until the entire job completes.
    overhead=max(0.,job['planned_s']-length(job['poses'])/.12)
    current=parts[0];poses=executed.get(current['id'],current['poses'])
    part=dict(id=current['id'],kind='INSPECTION_PART',route_policy='ONLINE_NAVIGATION',poses=poses,planned_s=length(poses)/.12)
    measured=segment_estimate(part,xy,t,busy,nav_plan)
    future=sum(length(executed.get(p['id'],p['poses']))/.12 for p in parts[1:])
    return dict(seconds=measured['seconds']+future+overhead,source='REMAINING_INSPECTION_PARTS_AND_NAV2',current=measured,current_part=current['id'],completed_parts=[p['id'] for p in job['reference_parts'] if p['id'] in done],pending_parts=len(parts),future_parts_s=future,overhead_s=overhead,speed_upper_mps=.12,purpose='ETA_ONLY_NOT_MOTION_PERMISSION')
