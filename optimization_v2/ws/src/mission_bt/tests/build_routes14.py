"""Build a new execution edition; do not mutate any S12 task obligations."""
import sys,copy,shutil
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from common14 import *
from routing14 import candidate,CollisionMap,verify_route
from binding14 import atlas,canonical
from common12 import config,backend
from shapely.geometry import shape,mapping

SOURCE=BASE/'results/S14/demo_20261112_084400_1582964/ackermann_navigation_probe'
OUT=BASE/'plans/s14_registered'

def main():
    original=base_plan();a=atlas();source=SOURCE/'diagnostics/global_costmap.npz';runtime=read(SOURCE/'runtime.json')
    run_inputs=read(SOURCE.parent/'input_bindings.json');tool=read(SOURCE/'scene/tool.json')
    binding=dict(atlas_revision=run_inputs['regional_atlas']['revision'],map_id=tool['map_id'],frame_epoch=tool['frame_epoch'],plan_revision=tool['plan_revision'])
    if binding['atlas_revision']!=a['revision'] or binding['map_id']!=a['map_id'] or binding['frame_epoch']!=a['frame_epoch'] or binding['plan_revision']!=run_inputs['registered_plan_revision'] or not runtime['database_unchanged']:raise ValueError('ROUTING_OBSERVATION_VERSION')
    grid=np.load(source);t=float(grid['t']);statuses=[x for x in rows(SOURCE/'online/observations.jsonl') if x.get('record_type')=='status' and x.get('t',1e9)<=t]
    status=max(statuses,key=lambda x:x['t'])
    if t-status['t']>.3 or not status['navigation_allowed'] or status['plan_revalidation_required']:raise ValueError('ROUTING_OBSERVATION_NOT_ADMITTED')
    observation=OUT/'routing_observation.npz';shutil.copyfile(source,observation)
    context=OUT/'routing_observation_context.json';dump(context,dict(source=str(source.relative_to(ROOT)),source_sha256=sha(source),stamp=t,status=status,binding=binding,scope='Real sensor-built costmap snapshot, not a future obstacle oracle; live safety always overrides offline routes'))
    collision=CollisionMap(observation,a['regions']['B']['site_extent_xy']);c=config();zone=original['zones']['B'];work=shape(zone['task_domain']).buffer(.4,join_style=2);motion=shape(zone['motion_domain']);audit=BASE/'results/S14/routing_revalidation_20261112';audit.mkdir(exist_ok=True);evidence=[];candidates=[]
    for angle in [0.,math.pi/12]:
        raw=backend(dict(mode='swaths',work=mapping(work),motion=mapping(motion),spacing=c['work_spacing_m'],headland=c['whole_zone_headland_m'],angle=angle),audit,'whole_zone_'+str(round(math.degrees(angle))))
        for phase in [-.30,-.25,-.20,-.15,0.,.15,.30]:
            result,check=candidate(zone,raw,phase,collision,c);evidence.append(check);print(round(math.degrees(angle)),phase,check['reason'],check.get('timing',{}).get('total_s'),flush=True)
            if result:candidates.append(result)
    dump(audit/'candidate_evidence.json',evidence)
    if not candidates:raise ValueError('NO_QUALIFIED_EXECUTION_ROUTE; original obligations remain pending')
    route=min(candidates,key=lambda x:x['timing']['total_s'])
    inputs=[observation,context,source,SOURCE/'runtime.json',SOURCE/'online/observations.jsonl',SOURCE.parent/'input_bindings.json',SOURCE/'scene/tool.json',Path(__file__).resolve(),PKG/'scripts/routing14.py',BASE/'config/planning.yaml',BASE/'config/tool_contract.yaml',BASE/'config/tool_profiles.yaml',BASE/'config/vehicles/ackermann.yaml',BASE/'ws/install/coverage_planner/lib/coverage_planner/planner_backend']
    route.update(schema='S14-MEASURED-EXECUTION/1',cluster='B1',base_plan_revision=original['plan_revision'],atlas_revision=a['revision'],observation=str(observation.relative_to(ROOT)),bindings={str(p.relative_to(ROOT)):sha(p) for p in inputs},method='Fields2Cover whole-zone line phase; endpoint overrun removal; fixed-area tool-center invariance; closed directed Dubins graph; bounded Held-Karp',scope='B1 only; unchanged original task domain; real feedback still required')
    route['revision']=canonical(route);validation=verify_route(route,original,a);dump(OUT/'execution_B1.json',route);dump(audit/'validation.json',validation)
    derived=read(OUT/'mission_plan.json');derived['execution_routes']={'B1':dict(path=str((OUT/'execution_B1.json').relative_to(ROOT)),sha256=sha(OUT/'execution_B1.json'),revision=route['revision'])};derived.pop('plan_revision');derived['plan_revision']=canonical(derived);dump(OUT/'mission_plan.json',derived)
    manifest=read(OUT/'manifest.json');manifest['plan_revision']=derived['plan_revision'];manifest['files']={name:sha(OUT/name) for name in ['mission_plan.json','execution_B1.json','routing_observation.npz','routing_observation_context.json']};dump(OUT/'manifest.json',manifest)
    dump(BASE/'maps/s14_regions/geometry_invariance.json',dict(unchanged={k:derived[k]==original[k] for k in ['zones','connector_routes','task_revision']},base_plan_revision=original['plan_revision'],registered_plan_revision=derived['plan_revision'],original_fixed_points=21,original_fixed_area_m2=16.35945787959566,execution_geometry='B1 re-routed on measured collision map; full target preserved',execution_route_revision=route['revision']))
    print(json.dumps(dict(plan_revision=derived['plan_revision'],route_revision=route['revision'],validation=validation)),flush=True)
if __name__=='__main__':main()
