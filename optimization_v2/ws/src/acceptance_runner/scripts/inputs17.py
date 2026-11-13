"""Explicit immutable map/plan edition selected before a simulator is started."""
from common17 import *

CONFIG=EVAL/'active_inputs17.json'

def active():
    config=read(CONFIG) if CONFIG.exists() else dict(atlas='optimization_v2/maps/s17_regions/manifest.json',plan='optimization_v2/plans/s17_registered/mission_plan.json')
    paths={k:(ROOT/config[k]).resolve() for k in ['atlas','plan']}
    if not paths['atlas'].is_relative_to(BASE/'maps') or not paths['plan'].is_relative_to(BASE/'plans'):raise ValueError('INPUT_EDITION_OUTSIDE_PROJECT')
    for p in paths.values():
        if not p.is_file():raise ValueError('INPUT_EDITION_MISSING '+str(p))
    return paths

def snapshot(out):
    paths=active();dump(Path(out)/'input_edition17.json',{k:dict(path=str(p.relative_to(ROOT)),sha256=sha(p)) for k,p in paths.items()});return paths

def receipt_assets():
    """Bind actual atlas/route payloads, not just manifests listing their hashes."""
    from binding_check17 import validate
    atlas,plan=validate();result={str(p.relative_to(ROOT)):sha(p) for p in active().values()}
    result.update(atlas['bindings'])
    for ref in plan['execution_routes'].values():
        p=(ROOT/ref['path']).resolve()
        if not p.is_relative_to(ROOT) or sha(p)!=ref['sha256']:raise ValueError('RECEIPT_ROUTE_CHANGED')
        result[ref['path']]=ref['sha256'];route=read(p)
        for path,expected in route['bindings'].items():
            q=(ROOT/path).resolve()
            if not q.is_relative_to(ROOT) or sha(q)!=expected:raise ValueError('RECEIPT_ROUTE_INPUT_CHANGED '+path)
            result[path]=expected
    for p in [active()['plan'].parent/'manifest.json',CONFIG]:
        if p.exists():result[str(p.relative_to(ROOT))]=sha(p)
    return result
