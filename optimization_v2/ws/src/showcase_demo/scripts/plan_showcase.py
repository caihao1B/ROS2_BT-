"""Independent small work obligations on the unchanged measured S17 atlas.

Three connected workload clusters are computed from each continuous polygon;
no physical fences or preassigned islands are introduced. Offline eligibility
is separate from measured task completion.
"""
from common_showcase import *
import copy,math,argparse
import numpy as np
from common17 import add14
add14()
import common14,routing14
from common12 import config,backend
from geometry11 import parameters,load_domain,build_graph
from partition11 import solve
from spray12 import plan_spray
from motion12 import effect_polygon
from planning12 import validate_routes
from shapely.geometry import shape,mapping
from shapely.affinity import scale
from shapely import set_precision,union_all

def build(out):
    from inputs17 import active
    inputs=active();original=read(inputs['plan']);atlas=read(inputs['atlas']);c=config();pc=parameters()
    declaration=dict(schema='SHOWCASE-TASK/1',scope='INDEPENDENT_DEMO_NOT_FORMAL_S17',parent_plan_revision=original['plan_revision'],parent_atlas_revision=atlas['revision'],domain_scale={'A':.32,'B':.30},scale_origin='centroid of original continuous task polygon',k=3,seed=17041,original_map_unchanged=True,original_task_unchanged=True,recording_wall_budget_s=10800,hard_cluster_sim_budget_s=1800)
    dump(out/'declaration.json',declaration)
    zones={};qualified={};bounds={}
    for zone in ['A','B']:
        base=original['zones'][zone];factor=declaration['domain_scale'][zone];domain=set_precision(scale(shape(base['task_domain']),xfact=factor,yfact=factor,origin='centroid'),pc['geometry_precision_m']);motion=shape(base['motion_domain'])
        if not shape(base['task_domain']).covers(domain):raise ValueError('DEMO_DOMAIN_MUST_BE_INSIDE_PARENT')
        meta=load_domain(zone,'base',pc);meta.update(domain=domain,motion=motion,original_domain=domain,centre=motion.buffer(-meta['radius_m']))
        graph=build_graph(domain,pc,meta);partition=solve(meta,graph,pc,'metis',declaration['seed'],out/zone/'partition');labels=np.asarray(partition['labels']);regions=[shape(f['geometry']) for f in partition['clusters']['features']]
        print(zone,'partition',domain.area,'points',len(graph['points']),flush=True)
        if zone=='A':plans=plan_spray(meta,graph,labels,c,out/zone/'routes')
        else:
            # Keep the already qualified depot. New task obligations do not
            # grant unobserved turning space, erase perimeter, or shrink credit.
            observation=ROOT/read(ROOT/original['execution_routes']['B1']['path'])['observation'];collision=routing14.CollisionMap(observation,atlas['regions']['B']['site_extent_xy'])
            plans=[];headland=mapping(domain)
            raw=[backend(dict(mode='swaths',work=mapping(domain.buffer(.4,join_style=2)),motion=mapping(motion),spacing=c['work_spacing_m'],headland=c['whole_zone_headland_m'],angle=i*math.pi/12),out/zone/'swaths','angle_'+str(i)) for i in range(12)]
            for i,target in enumerate(regions):
                template=dict(cluster=i,zone='B',vehicle='ackermann',method='F2C + padded measured-map collision checks + directed Dubins + Held-Karp',start=base['plans'][0]['start'],end=base['plans'][0]['end'],whole_zone_headland=headland,coverage_check=dict(covered=mapping(target),domain_area_m2=target.area))
                z=dict(task_domain=mapping(domain),motion_domain=mapping(motion),plans=[template]);checks=[];solutions=[]
                for swaths in raw:
                    for phase in [0.,-.15,.15,-.3,.3]:
                        route,evidence=routing14.candidate(z,swaths,phase,collision,c);checks.append(evidence)
                        if route:solutions.append(route)
                    if len(solutions)>=3:break
                dump(out/zone/f'cluster_{i}_candidates.json',checks)
                if not solutions:raise ValueError('NO_VERIFIED_DEMO_ROUTE_B'+str(i+1))
                route=min(solutions,key=lambda r:r['timing']['total_s']);credit=union_all([effect_polygon([p for p,v in zip(s['poses'],s['timing']['velocities']) if v>=.02],c) for s in route['segments'] if s['tool_on']]).intersection(target)
                template.update(segments=route['segments'],timing=route['timing'],end=route['segments'][-1]['poses'][-1],execution_route_revision=canonical(route),planned_coverage_certificate=dict(covered=mapping(credit),fraction=credit.area/target.area),original_obligation=mapping(target))
                template['coverage_check'].update(fraction=credit.area/target.area,covered_m2=credit.area,residual_m2=target.difference(credit).area,residual=mapping(target.difference(credit)),unconnected=[],all_work_segments=True)
                plans.append(template);print('B',i+1,'seconds',route['timing']['total_s'],'coverage',credit.area/target.area,flush=True)
        check=validate_routes(plans,c)
        if not check['passed']:raise ValueError('DEMO_ROUTE_VALIDATION '+str(check))
        for i,p in enumerate(plans):
            p['showcase_cluster']=zone+str(i+1)
            for s in p['segments']:s['showcase_cluster']=p['showcase_cluster']
        zones[zone]=dict(task_domain=mapping(domain),motion_domain=mapping(motion),plans=plans,partition=partition,partition_revision=canonical(partition),route_verification=check)
        bounds[zone]=[p['timing']['total_s'] for p in plans]
    files=[Path(__file__).resolve(),out/'declaration.json',inputs['plan'],inputs['atlas'],BASE/'config/planning.yaml',BASE/'config/partition.yaml',BASE/'config/tool_profiles.yaml',BASE/'config/tool_contract.yaml',BASE/'ws/install/coverage_planner/lib/coverage_planner/planner_backend',BASE/'ws/install/coverage_partition/lib/coverage_partition/partition_backend']
    files=[p for p in files if p.exists()]+[Path(routing14.__file__),BASE/'ws/install/coverage_partition/lib/coverage_partition/partition_metis']
    for module in ['geometry11','partition11','spray12','motion12','planning12','common12']:
        files.append(Path(sys.modules[module].__file__))
    d=dict(schema='SHOWCASE-PLAN/1',scope='INDEPENDENT_DEMO_NOT_FORMAL_S17',task_revision=canonical(declaration),base_plan_revision=original['plan_revision'],atlas_revision=atlas['revision'],frame=original['frame'],normal_budget_s=1440.,hard_budget_s=1800.,zones=zones,connector_routes=original['connector_routes'],runtime_eligible_clusters=['A1','A2','A3','B1','B2','B3'],versions={str(p.relative_to(ROOT)):sha(p) for p in files},nominal_cluster_seconds=bounds,formal_credit=False,closed_loop_status='NOT_RUN')
    d['plan_revision']=canonical(d);dump(out/'mission_plan.json',d);dump(out/'manifest.json',dict(plan_revision=d['plan_revision'],path=str((out/'mission_plan.json').relative_to(ROOT)),sha256=sha(out/'mission_plan.json'),scope=d['scope'],closed_loop_status='NOT_RUN'))
    print(json.dumps(dict(plan_revision=d['plan_revision'],nominal_cluster_seconds=bounds,nominal_total_work_s=sum(sum(v) for v in bounds.values())),indent=2))
    return d
if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path,default=OUT/'v1');a=ap.parse_args()
    if not a.out.resolve().is_relative_to(OUT):raise ValueError('SHOWCASE_OUTPUT_SCOPE_REQUIRED')
    build(a.out)
