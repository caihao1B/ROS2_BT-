"""Replan all three mower obligations against measured, conservative occupancy."""
from common17 import *
import copy,math,numpy as np,yaml,types
from PIL import Image
from shapely.geometry import shape,mapping

def build():
    add14()
    import common14  # Establish upstream planner paths before loading common12.
    from common12 import config,backend
    from routing14 import candidate,CollisionMap
    from overlay17 import replace
    atlas=read(BASE/'maps/s17_regions/manifest.json');original=read(BASE/'plans/validated/mission_plan.json');out=BASE/'plans/s17_registered';evidence=RESULTS/'qualification/routes';evidence.mkdir(parents=True,exist_ok=True)
    r=atlas['regions']['B'];grid=ROOT/r['navigation_pgm'];params=yaml.safe_load((ROOT/r['navigation_yaml']).read_text());pixels=np.asarray(Image.open(grid))[::-1];values=np.full(pixels.shape,-1,np.int8);values[pixels>=250]=0;values[pixels<128]=100
    observation=out/'routing_observation.npz';np.savez_compressed(observation,data=values,resolution=params['resolution'],origin=np.array(params['origin'][:2]),frame='map')
    collision=CollisionMap(observation,r['site_extent_xy']);c=config();zone=original['zones']['B'];work=shape(zone['task_domain']).buffer(.4,join_style=2);motion=shape(zone['motion_domain'])
    raw=[backend(dict(mode='swaths',work=mapping(work),motion=mapping(motion),spacing=c['work_spacing_m'],headland=c['whole_zone_headland_m'],angle=angle),evidence,'whole_zone_'+str(i)) for i,angle in enumerate(np.arange(12)*math.pi/12)]
    import routing14
    src=Path(routing14.__file__).read_text();src=replace(src,"route['cluster']!='B1'","route['cluster'] not in ['B1','B2','B3']");src=replace(src,"base=original['zones']['B'];target=", "base=copy.deepcopy(original['zones']['B']);base['plans']=[base['plans'][int(route['cluster'][1])-1]];target=")
    module=types.ModuleType('routing17');exec(compile(src,str(PKG/'scripts/routes17.py'),'exec'),module.__dict__)
    refs={}
    for i in range(3):
        key='B'+str(i+1);z=copy.deepcopy(zone);z['plans']=[zone['plans'][i]];checks=[];solutions=[]
        for swaths in raw:
            for phase in [-.30,-.25,-.20,-.15,0.,.15,.30]:
                route,check=candidate(z,swaths,phase,collision,c);checks.append(check);print(key,phase,check['reason'],check.get('timing',{}).get('total_s'),flush=True)
                if route:solutions.append(route)
        dump(evidence/(key+'_candidates.json'),checks)
        if not solutions:raise ValueError('NO_MEASURED_COLLISION_FREE_ROUTE '+key)
        route=min(solutions,key=lambda x:x['timing']['total_s']);inputs=[Path(__file__).resolve(),Path(routing14.__file__),grid,ROOT/r['navigation_yaml'],observation,BASE/'config/planning.yaml',BASE/'config/tool_contract.yaml',BASE/'config/tool_profiles.yaml',BASE/'config/vehicles/ackermann.yaml',BASE/'ws/install/coverage_planner/lib/coverage_planner/planner_backend']
        route.update(schema='S17-EXECUTION-ROUTE/1',cluster=key,base_plan_revision=original['plan_revision'],atlas_revision=atlas['revision'],observation=str(observation.relative_to(ROOT)),bindings={str(p.relative_to(ROOT)):sha(p) for p in inputs},method='Fields2Cover swaths, fixed target, padded footprint on frozen measured occupancy, directed Dubins graph, Held-Karp closed route')
        route['revision']=canonical(route);check=module.verify_route(route,original,atlas);path=out/('execution_'+key+'.json');dump(path,route);dump(evidence/(key+'_validation.json'),check);refs[key]=dict(path=str(path.relative_to(ROOT)),sha256=sha(path),revision=route['revision'])
    p=read(out/'mission_plan.json');p['execution_routes']=refs;p.pop('plan_revision');p['plan_revision']=canonical(p);dump(out/'mission_plan.json',p);dump(out/'manifest.json',dict(plan_revision=p['plan_revision'],atlas_revision=atlas['revision'],files={x.name:sha(x) for x in out.iterdir() if x.is_file() and x.name!='manifest.json'}))
if __name__=='__main__':build()
