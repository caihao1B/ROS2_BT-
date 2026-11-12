"""Explicit S14 regional atlas and derived-plan bindings; upstream stays immutable."""
from common14 import ROOT,BASE,PKG,read,dump,sha
import json,hashlib,os,copy
from pathlib import Path
ATLAS=BASE/'maps/s14_regions/manifest.json'
PLAN=BASE/'plans/s14_registered/mission_plan.json'
_PLAN_CACHE={}

def signatures(paths):
    result={}
    for p in paths:
        s=p.stat();result[str(p)]=(s.st_dev,s.st_ino,s.st_size,s.st_mtime_ns,s.st_ctime_ns)
    return result

def canonical(d):return hashlib.sha256(json.dumps(d,sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest()

def atlas(validate=True):
    d=read(ATLAS)
    if canonical({k:v for k,v in d.items() if k!='revision'})!=d['revision']:raise ValueError('ATLAS_REVISION_MISMATCH')
    if not d['passed']:raise ValueError('REGIONAL_VALIDATION_FAILED')
    frozen=read(BASE/'maps/frozen/manifest.json')
    if d['source_map_revision']!=frozen['revision'] or d['calibration_revision']!=frozen['calibration_revision']:raise ValueError('REGIONAL_SOURCE_LINEAGE_CHANGED')
    if d['map_id']==frozen['map_id'] or d['frame_epoch']==frozen['frame_epoch']:raise ValueError('REGIONAL_DERIVATION_REQUIRES_NEW_IDENTITY')
    if validate:
        for path,h in d['bindings'].items():
            p=(ROOT/path).resolve()
            if not p.is_relative_to(ROOT) or not p.is_file() or sha(p)!=h:raise ValueError('ATLAS_INPUT_CHANGED '+path)
    return d

def verify_derived(d,original,a,name):
    body=copy.deepcopy(d);version=body.pop('plan_revision')
    if canonical(body)!=version:raise ValueError('REGISTERED_PLAN_HASH')
    if d['base_plan_revision']!=original['plan_revision'] or d['atlas_revision']!=a['revision']:raise ValueError('REGISTERED_PLAN_STALE')
    for key in ['zones','connector_routes','task_revision','normal_budget_s','hard_budget_s']:
        if d[key]!=original[key]:raise ValueError('ORIGINAL_OBLIGATION_CHANGED '+key)
    if d['frame']['map_id']!=a['map_id'] or d['frame']['frame_epoch']!=a['frame_epoch']:raise ValueError('REGISTERED_FRAME_EPOCH')
    if d['frame']['map_revision']!=a['revision'] or d['frame']['calibration_revision']!=a['calibration_revision']:raise ValueError('REGISTERED_FRAME_REVISION')
    if d['runtime_eligible_clusters']!=['A1','B1']:raise ValueError('UNVALIDATED_CLUSTER_SCOPE')
    for key in ['rotation','translation','prior_frame','target_frame']:
        if d['frame'][key]!=original['frame'][key]:raise ValueError('PLAN_COORDINATE_VALUES_CHANGED '+key)
    if name not in a['regions']:raise ValueError('UNREGISTERED_REGION')
    for key,ref in d.get('execution_routes',{}).items():
        if key!='B1':raise ValueError('UNVALIDATED_EXECUTION_SCOPE')
        path=(ROOT/ref['path']).resolve()
        if not path.is_relative_to(BASE/'plans/s14_registered') or sha(path)!=ref['sha256']:raise ValueError('EXECUTION_ROUTE_FILE_CHANGED')
        route=read(path)
        if route['revision']!=ref['revision']:raise ValueError('EXECUTION_ROUTE_REVISION')
        from routing14 import verify_route
        verify_route(route,original,a)
    return d

def active_plan():
    from common13 import plan as original_plan
    name=os.environ.get('S14_REGISTERED_REGION')
    if name in _PLAN_CACHE:
        d,known=_PLAN_CACHE[name]
        try:current=signatures([Path(p) for p in known])
        except OSError as e:raise ValueError('VERIFIED_PLAN_INPUT_MISSING') from e
        if current!=known:raise ValueError('VERIFIED_PLAN_INPUT_CHANGED_RESTART_REQUIRED')
        return copy.deepcopy(d)
    # Full content verification happens once before ROS callbacks start. During
    # execution check all bound file identities, never hash a DB in the BT loop.
    original=original_plan();directory=BASE/'plans/validated'
    paths={ROOT/p for p in original['versions']}|{directory/'manifest.json'}|{directory/p for p in read(directory/'manifest.json')['files']}
    if not name:d=original
    else:
        a=atlas();d=verify_derived(read(PLAN),original,a,name)
        paths|={ATLAS,PLAN,PLAN.parent/'manifest.json'}|{ROOT/p for p in a['bindings']}
        for ref in d.get('execution_routes',{}).values():
            paths.add(ROOT/ref['path']);paths|={ROOT/p for p in read(ROOT/ref['path'])['bindings']}
    _PLAN_CACHE[name]=(d,signatures(paths))
    return copy.deepcopy(d)

def region():
    a=atlas();name=os.environ.get('S14_REGISTERED_REGION','')
    if name not in a['regions']:raise ValueError('REGIONAL_SCOPE_REQUIRED')
    return a,a['regions'][name]

def execution_cluster(kind):
    """Task geometry stays in zones; only qualified execution routes can differ."""
    d=active_plan();zone='A' if kind=='diff' else 'B';c=copy.deepcopy(d['zones'][zone]['plans'][0]);ref=d.get('execution_routes',{}).get(zone+'1')
    if ref:
        route=read(ROOT/ref['path']);c.update(segments=route['segments'],timing=route['timing'],execution_route_revision=route['revision'])
    return c
