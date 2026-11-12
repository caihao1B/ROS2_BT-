"""Candidate products, immutable bindings and S12 revalidation contract."""
import shutil
from geometry11 import *
from common11 import *

def domain_record(meta):
    from shapely.geometry import mapping
    d={k:mapping(meta[k]) for k in ['workspace','permanent','domain','centre','unresolved_boundary']}
    d.update({k:meta[k] for k in ['frame','frame_note','map_id','radius_m','minimum_turn_radius_m','speed_mps','swath_spacing_m']})
    d['work_area_m2']=meta['domain'].area;d['boundary_screen_area_m2']=meta['unresolved_boundary'].area
    d['centre_components']=len(polygons(meta['centre']));return d

def export_bundle(out,records,graphs,domains,seed,binding):
    """Full work obligations, not just centre erosion. Manifest written last."""
    out.mkdir(parents=True,exist_ok=True);paths=[]
    for zone in ['A','B']:
        r=next(x for x in records if x['zone']==zone and x['variant']=='base' and x['algorithm']=='metis' and x['seed']==seed)
        g=graphs[zone+'_base'];stem='zone_'+zone.lower();d=domains[zone+'_base']
        dump(out/(stem+'_clusters.geojson'),r['clusters']);paths.append(out/(stem+'_clusters.geojson'))
        points_by_unit={}
        for p in r['points']:points_by_unit.setdefault(p['unit'],[]).append(p['id'])
        units=[feature(p,unit=i,cluster=r['labels'][i],weight_seconds=float(g['weights'][i]),grid=g['grid'][i],point_ids=points_by_unit.get(i,[])) for i,p in enumerate(g['cells'])]
        dump(out/(stem+'_units.geojson'),collection(units));paths.append(out/(stem+'_units.geojson'))
        dump(out/(stem+'_points.json'),r['points']);paths.append(out/(stem+'_points.json'))
        dump(out/(stem+'_domains.geojson'),collection([dict(type='Feature',geometry=d[key],properties={'layer':key}) for key in ['workspace','permanent','domain','centre','unresolved_boundary']]));paths.append(out/(stem+'_domains.geojson'))
    manifest={'schema':'S11-CANDIDATE/1','status':'PROVISIONAL','execution_allowed':False,'route_status':'NOT_RUN_S12','actual_task_sim_seconds':None,
      'frame':'prior_local_enu','frame_registration':'NOT_REGISTERED_TO_SLAM_MAP; S12/S13 must bind a verified transform, never relabel coordinates',
      'source_map_id':read(BASE/'maps/prior/map_metadata.json')['map_id'],'k':3,'seed':seed,'algorithm':'metis+connectivity_repair',
      'runtime_budget_status':'NOT_ACCEPTED','binding':binding,'files':{p.name:sha(p) for p in paths},
      'task_semantics':'All task cells retained exactly once. Boundary-screen unresolved obligations remain assigned; clusters are not no-go boundaries.',
      'next_required':['orientation-aware vehicle/tool clearance','Ackermann connectors and shared whole-zone headland','actual TSP/swath routes and workload rebalance','resolve current speed/scale budget conflict','frame registration','online obstacle handling']}
    manifest['revision']=identity(manifest);dump(out/'partition_manifest.json',manifest);return manifest

def validate_bundle(path):
    m=read(path/'partition_manifest.json')
    if m['status']!='PROVISIONAL' or m['execution_allowed'] or m['k']!=3:raise ValueError('CANDIDATE_CONTRACT')
    original=dict(m);revision=original.pop('revision')
    if identity(original)!=revision:raise ValueError('MANIFEST_REVISION')
    for name,digest in m['files'].items():
        if '/' in name or name.startswith('.'):raise ValueError('INVALID_BUNDLE_PATH')
        if sha(path/name)!=digest:raise ValueError('BUNDLE_HASH_MISMATCH')
    for p,digest in m['binding']['input_hashes'].items():
        if sha(ROOT/p)!=digest:raise ValueError('INPUT_VERSION_CHANGED')
    for p,digest in m['binding']['source_hashes'].items():
        if sha(ROOT/p)!=digest:raise ValueError('SOURCE_VERSION_CHANGED')
    for step,digest in m['binding']['dependency_receipts'].items():
        if sha(BASE/f'results/{step}/latest.json')!=digest:raise ValueError('DEPENDENCY_VERSION_CHANGED')
    return m
