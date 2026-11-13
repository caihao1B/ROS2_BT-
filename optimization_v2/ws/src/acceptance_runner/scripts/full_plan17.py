"""Rebind independently verified routes after extending the atlas to the road."""
from common17 import *
from overlay17 import replace
import copy,types

def build():
    add14()
    import common14,routing14
    source=BASE/'plans/s17_registered';out=BASE/'plans/s17_full'
    original=read(BASE/'plans/validated/mission_plan.json');atlas=read(BASE/'maps/s17_full/manifest.json')
    if not atlas['regions']['C']['passed']:raise ValueError('CONNECTOR_NOT_QUALIFIED')
    previous=read(BASE/'maps/s17_regions/manifest.json')
    if any(atlas['regions'][z]!=previous['regions'][z] for z in ['A','B']):raise ValueError('WORK_MAP_CHANGED_REQUIRES_REPLANNING')
    src=Path(routing14.__file__).read_text();src=replace(src,"route['cluster']!='B1'","route['cluster'] not in ['B1','B2','B3']")
    src=replace(src,"base=original['zones']['B'];target=","base=copy.deepcopy(original['zones']['B']);base['plans']=[base['plans'][int(route['cluster'][1])-1]];target=")
    module=types.ModuleType('route_check17');exec(compile(src,str(Path(routing14.__file__)),'exec'),module.__dict__)
    plan=copy.deepcopy(read(source/'mission_plan.json'));refs={};checks={}
    for key,ref in plan['execution_routes'].items():
        old=ROOT/ref['path']
        if sha(old)!=ref['sha256']:raise ValueError('PREVIOUS_ROUTE_CHANGED')
        route=read(old);route.pop('revision');route['atlas_revision']=atlas['revision'];route['bindings'][str(old.relative_to(ROOT))]=sha(old)
        route['bindings'][str(Path(__file__).resolve().relative_to(ROOT))]=sha(__file__);route['revision']=canonical(route)
        checks[key]=module.verify_route(route,original,atlas);dest=out/('execution_'+key+'.json');dump(dest,route);refs[key]=dict(path=str(dest.relative_to(ROOT)),sha256=sha(dest),revision=route['revision'])
    plan.pop('plan_revision');plan['atlas_revision']=atlas['revision'];plan['execution_routes']=refs
    plan['frame'].update(map_id=atlas['map_id'],map_revision=atlas['revision'],frame_epoch=atlas['frame_epoch'],source='S17 sensor-qualified continuous atlas')
    plan['plan_revision']=canonical(plan);dump(out/'mission_plan.json',plan);dump(out/'manifest.json',dict(plan_revision=plan['plan_revision'],atlas_revision=atlas['revision'],files={p.name:sha(p) for p in out.iterdir() if p.is_file() and p.name!='manifest.json'}))
    dump(RESULTS/'corridor_qualification/route_revalidation17.json',checks)
    # Activation is the final atomic write after all three routes verify.
    dump(EVAL/'active_inputs17.json',dict(atlas='optimization_v2/maps/s17_full/manifest.json',plan='optimization_v2/plans/s17_full/mission_plan.json'))
if __name__=='__main__':build()
