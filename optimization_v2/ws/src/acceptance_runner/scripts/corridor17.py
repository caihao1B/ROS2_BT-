"""Prepare independent corridor stations without claiming route execution."""
from common17 import *
import numpy as np
from shapely.geometry import LineString

def prepare():
    p=read(BASE/'plans/validated/mission_plan.json');route=next(r for r in p['connector_routes'] if r['vehicle']=='diff' and r['direction']=='A_to_B');line=LineString([x[:2] for s in route['segments'] for x in s['poses']]);stations={}
    for i,s in enumerate(np.linspace(12,line.length-10,11)):
        xy=line.interpolate(s);before=line.interpolate(max(0,s-.1));after=line.interpolate(min(line.length,s+.1));h=float(np.arctan2(after.y-before.y,after.x-before.x));stations[f'C{i:02}']=dict(region='C',role='train',vehicle='diff',seed=17101+i,prior_pose=[xy.x,xy.y,h],distance_along_prior_m=float(s),source='simulator deployment on approved prior connector; noisy observations drive registration')
    for i,s in enumerate([22.,42.,62.,82.]):
        xy=line.interpolate(s);before=line.interpolate(s-.1);after=line.interpolate(s+.1);h=float(np.arctan2(after.y-before.y,after.x-before.x));stations[f'H{i:02}']=dict(region='C',role='heldout',vehicle='diff',seed=17201+i,prior_pose=[xy.x,xy.y,h],distance_along_prior_m=s,source='independent heldout station; not used to build map')
    out=RESULTS/'corridor_qualification';out.mkdir(exist_ok=True);dump(out/'stations.json',dict(schema='S17-CORRIDOR-STATIONS/1',stations=stations,mode='VACANT_FIELD_MAPPING_ONLY',formal_obstacle_scene_unchanged=True));return out
if __name__=='__main__':print(prepare())
