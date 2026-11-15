"""Geometric half of stopped plan revalidation; live costmap result is also required."""
import math,hashlib,json
def validate_path(points,start,goal,cfg):
    from shapely.geometry import Polygon,shape
    from shapely.affinity import rotate,translate
    if len(points)<2 or any(not all(math.isfinite(v) for v in p) for p in points):return False,'MALFORMED_PATH'
    if math.dist(points[0][:2],start[:2])>.15:return False,'START_DISCONTINUITY'
    if math.dist(points[-1][:2],goal[:2])>.05:return False,'WRONG_GOAL'
    allowed=shape(cfg['allowed_prior']).buffer(-cfg['footprint_margin_m']);foot=Polygon(cfg['vehicle']['footprint'])
    for a,b in zip(points,points[1:]):
        distance=math.dist(a[:2],b[:2]);dh=math.atan2(math.sin(b[2]-a[2]),math.cos(b[2]-a[2]))
        if distance>.4:return False,'PATH_DISCONTINUITY'
        steps=max(1,math.ceil(distance/.04),math.ceil(abs(dh)/.04))
        for j in range(steps+1):
            u=j/steps;x=a[0]+u*(b[0]-a[0])+28;y=a[1]+u*(b[1]-a[1])+14;h=a[2]+u*dh
            if not allowed.covers(translate(rotate(foot,h,origin=(0,0),use_radians=True),x,y)):return False,'SWEPT_FOOTPRINT_OUTSIDE_PRIOR'
    return True,'ORIGINAL_GOAL_AND_FOOTPRINT_VALID'
def path_hash(points):return hashlib.sha256(json.dumps(points,separators=(',',':'),allow_nan=False).encode()).hexdigest()
