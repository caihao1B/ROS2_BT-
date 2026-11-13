"""Version admission for a separate demonstration; formal obligations untouched."""
from common_showcase import *
import copy
from binding_formal14 import atlas,region,signatures,ATLAS,active_plan as formal_plan
_CACHE={}

def active_plan():
    from shapely.geometry import shape
    from shapely import union_all
    path=Path(os.environ['SHOWCASE_PLAN']).resolve();name=os.environ.get('S14_REGISTERED_REGION')
    if not path.is_relative_to(OUT):raise ValueError('SHOWCASE_PLAN_SCOPE')
    key=(str(path),name)
    if key in _CACHE:
        data,known=_CACHE[key]
        if signatures([Path(p) for p in known])!=known:raise ValueError('SHOWCASE_INPUT_CHANGED_RESTART_REQUIRED')
        return copy.deepcopy(data)
    parent=formal_plan();data=read(path);body=copy.deepcopy(data);revision=body.pop('plan_revision')
    if canonical(body)!=revision or data['scope']!='INDEPENDENT_DEMO_NOT_FORMAL_S17' or data['formal_credit']:raise ValueError('SHOWCASE_VERSION_OR_SCOPE')
    if data['base_plan_revision']!=parent['plan_revision'] or data['frame']!=parent['frame'] or data['atlas_revision']!=parent['atlas_revision']:raise ValueError('SHOWCASE_PARENT_CHANGED')
    paths={path,path.parent/'manifest.json'}
    manifest=read(path.parent/'manifest.json')
    if manifest['sha256']!=sha(path) or manifest['plan_revision']!=revision:raise ValueError('SHOWCASE_MANIFEST_CHANGED')
    for rel,digest in data['versions'].items():
        p=(ROOT/rel).resolve()
        if not p.is_relative_to(ROOT) or sha(p)!=digest:raise ValueError('SHOWCASE_BOUND_INPUT_CHANGED '+rel)
        paths.add(p)
    for zone in ['A','B']:
        z=data['zones'][zone];domain=shape(z['task_domain']);parts=[shape(f['geometry']) for f in z['partition']['clusters']['features']]
        if len(parts)!=3 or any(p.geom_type!='Polygon' or not p.is_valid for p in parts) or domain.symmetric_difference(union_all(parts)).area>1e-6 or abs(sum(p.area for p in parts)-domain.area)>1e-6:raise ValueError('SHOWCASE_PARTITION_NOT_COMPLETE_CONNECTED')
        if not z['route_verification']['passed'] or not z['partition']['verification']['passed']:raise ValueError('SHOWCASE_OFFLINE_NOT_ELIGIBLE')
        if zone=='B':
            for p,target in zip(z['plans'],parts):
                if shape(p['coverage_check']['covered']).symmetric_difference(target).area>1e-6 or abs(p['coverage_check']['domain_area_m2']-target.area)>1e-6:raise ValueError('SHOWCASE_DENOMINATOR_CHANGED')
    _CACHE[key]=(data,signatures(paths));return copy.deepcopy(data)

def execution_cluster(kind):
    d=active_plan();zone='A' if kind=='diff' else 'B';key=os.environ['S17_CLUSTER']
    if key[0]!=zone or key not in d['runtime_eligible_clusters']:raise ValueError('SHOWCASE_CLUSTER_VEHICLE_MISMATCH')
    return copy.deepcopy(d['zones'][zone]['plans'][int(key[1])-1])
