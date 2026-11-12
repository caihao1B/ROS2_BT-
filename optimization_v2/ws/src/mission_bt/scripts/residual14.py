"""Bounded residual repair using existing validated work lanes and F2C connectors.
Never changes the original target polygon or awards coverage for a planned lane.
"""
from common14 import *
from shapely.geometry import shape
from shapely.affinity import translate
from mowing12 import connection
from motion12 import effect_polygon
from common12 import config

def candidate(cluster, residual_map, pose_map, out, attempt):
    if residual_map.is_empty:return None
    c=config();residual=translate(residual_map,xoff=28,yoff=14)
    scope=read(BASE/'plans/validated/task_scope.geojson')
    motion=shape(next(f['geometry'] for f in scope['features'] if f['properties']['zone']=='B' and f['properties']['layer']=='motion_domain'))
    start=[pose_map[0]+28,pose_map[1]+14,pose_map[2]];options=[]
    for s in cluster['segments']:
        if s['kind'] not in ['COVERAGE','HEADLAND_WORK']:continue
        gain=effect_polygon(s['poses'],c).intersection(residual).area
        if gain>.005:options.append(dict(start=s['poses'][0],segment=s,gain=gain))
    choices=connection(start,options,motion,c,out,f'patch_{attempt}')
    if not choices:return None
    seconds,i,turn=min(choices,key=lambda x:(x[0]+options[x[1]]['segment']['timing']['motion_seconds'])/options[x[1]]['gain'])
    work=options[i]['segment'];return dict(segments=[turn,work],cost_s=seconds+work['timing']['motion_seconds']+8.,expected_gain_m2=options[i]['gain'],basis='remaining ledger geometry intersected with conservative planned footprint; live collision checking still required')

if __name__=='__main__':
    request=read(Path(sys.argv[1]));current=plan()
    if current['plan_revision']!=request['plan_revision']:raise ValueError('PATCH_PLAN_VERSION_CHANGED')
    result=candidate(request['cluster'],shape(request['residual']),request['pose_map'],Path(request['out']),request['attempt'])
    dump(Path(sys.argv[2]),dict(plan_revision=current['plan_revision'],candidate=result))
