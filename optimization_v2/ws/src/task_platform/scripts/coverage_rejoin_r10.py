"""Forward rejoin proposals on an unchanged original coverage strip.

 Proposals carry no motion or work authority. The worker must validate the
 observed costmap, full kinematics/body and physically execute a tool-off path.
 Skipped work remains in the original feedback ledger, with the same job key.
"""
import math

def rejoin_candidates(points,pose,spacing_m=2.,maximum_advance_m=12.,maximum_candidates=6):
    if len(points)<2 or spacing_m<=0 or maximum_advance_m<spacing_m:raise ValueError('INVALID_REJOIN_REQUEST')
    current=min(range(len(points)),key=lambda i:math.dist(points[i][:2],pose[:2]))
    distance=0.;next_distance=spacing_m;result=[]
    for i in range(current+1,len(points)):
        distance+=math.dist(points[i-1][:2],points[i][:2])
        if distance>maximum_advance_m+.15:break
        if distance+1e-6>=next_distance:
            result.append(dict(index=i,pose=list(points[i]),advance_m=distance,original_cursor=current))
            next_distance+=spacing_m
            if len(result)>=maximum_candidates:break
    return result

def forward_window(points,index,length_m=1.5):
    end=index;length=0.
    while end+1<len(points) and length<length_m:
        length+=math.dist(points[end][:2],points[end+1][:2]);end+=1
    return points[index:end+1]
