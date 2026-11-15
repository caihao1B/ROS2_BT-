"""Confirm the existing segment admission after measured physical stop.

Nav2's internal goal tolerance stays unchanged. The task's original 0.10 m /
0.12 rad arrival gate is evaluated over a fresh, stationary window instead of
turning an already admitted endpoint into another Ackermann parking manoeuvre.
"""
import math


def stable_arrival(samples, goal, xy_limit=.10, yaw_limit=.12):
    if len(samples) < 15:
        return False
    times=[s[0] for s in samples]
    if times[-1]-times[0] < .6 or any(not 0 < b-a <= .15 for a,b in zip(times,times[1:])):
        return False
    for _,pose in samples:
        if not all(math.isfinite(v) for v in pose):return False
        if math.dist(pose[:2],goal[:2]) > xy_limit:return False
        if abs(math.atan2(math.sin(pose[2]-goal[2]),math.cos(pose[2]-goal[2]))) > yaw_limit:return False
    return True
