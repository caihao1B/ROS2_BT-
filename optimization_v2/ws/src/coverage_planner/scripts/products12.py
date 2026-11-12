"""Publish an offline-valid plan; keep original task residue explicit."""
import csv,shutil
from common12 import *
from contracts12 import check_binding

def export_plan(out,mission):
    out.mkdir(parents=True,exist_ok=True);dump(out/'mission_plan.json',mission);routes=[];clusters=[];scopes=[]
    for zone,d in mission['zones'].items():
        for p in d['plans']:
            cluster=p['cluster'];unit_geoms=[shape(f['geometry']) for f in d['units']['features'] if f['properties']['cluster']==cluster];clusters.append(feature(union_all(unit_geoms),zone=zone,cluster=cluster,status='VALIDATED',nominal_seconds=p['timing']['total_s']))
            for i,s in enumerate(p['segments']):
                xy=[x[:2] for x in s['poses']];geom=Point(xy[0]) if len(xy)==1 or all(math.dist(xy[0],p)<1e-9 for p in xy) else LineString(xy)
                routes.append(feature(geom,zone=zone,cluster=cluster,segment=i,kind=s['kind'],tool_on=s['tool_on'],duration_s=s['timing']['motion_seconds']+s.get('action_seconds',0),frame='prior_local_enu'))
        scopes+= [dict(type='Feature',geometry=d[k],properties={'zone':zone,'layer':k,'completed':False}) for k in ['original_domain','task_domain','out_of_scope','motion_domain']]
    for i,p in enumerate(mission['connector_routes']):
        for j,s in enumerate(p['segments']):
            xy=[x[:2] for x in s['poses']];geom=LineString(xy) if len(xy)>1 else Point(xy[0]);routes.append(feature(geom,zone='connector',vehicle=p['vehicle'],direction=p['direction'],segment=j,kind=s['kind'],tool_on=False))
    dump(out/'routes.geojson',collection(routes));dump(out/'clusters.geojson',collection(clusters));dump(out/'task_scope.geojson',collection(scopes))
    with (out/'time_budget.csv').open('w',newline='') as f:
        w=csv.writer(f);w.writerow(['zone','cluster','motion_s','tool_switch_s','dwell_and_dock_s','setup_s','reserve_s','nominal_s','normal_limit_s','actual_sim_s'])
        for zone,d in mission['zones'].items():
            for p in d['plans']:
                t=p['timing'];w.writerow([zone,p['cluster'],t['motion_s'],t['tool_switch_s'],t['dwell_and_dock_s'],t['setup_s'],t['reserve_s'],t['total_s'],1440,'NOT_RUN'])
    check_binding(read(out/'mission_plan.json'))
    dump(out/'manifest.json',dict(status='VALIDATED',online_execution_allowed=False,plan_revision=mission['plan_revision'],files={p.name:sha(p) for p in files(out) if p.name!='manifest.json'}))

def validate_products(out):
    m=read(out/'manifest.json');check_binding(read(out/'mission_plan.json'))
    for name,h in m['files'].items():
        if sha(out/name)!=h:raise ValueError('PRODUCT_HASH_MISMATCH')
    return True
