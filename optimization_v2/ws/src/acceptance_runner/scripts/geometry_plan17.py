"""Independent variant qualification, without modifying the six formal domains.

Variants change known authorization geometry and are separate tasks. Obstacles
are excluded from motion as well as work; a work-only polygon hole cannot earn
physical executability credit. This producer never claims closed-loop PASS.
"""
from common17 import *
import argparse,copy,math,types
import numpy as np
from shapely.geometry import shape,mapping,box,Point
from shapely.affinity import translate
from shapely import set_precision,union_all

def definitions(domain,precision,zone=None,base=None):
    x0,y0,x1,y1=domain.bounds
    cuts={'base':domain.difference(domain),'notch':domain.intersection(box((x0+x1)/2-.4,y1-1,(x0+x1)/2+.4,y1+1))}
    center=domain.representative_point();placement='central representative point'
    if zone=='A' and base:
        # Scene design uses only declared service geometry. An obstacle cannot
        # be planted over a required docking pose and called an executable map.
        # Maximise empty-circle clearance on a fixed 0.1m design lattice near
        # the work centroid; keep all original service/docking coordinates.
        docking=[s['poses'][0][:2] for p in base['plans'] for s in p['segments'] if s['kind']=='POINT_SERVICE']
        docking += [p['start'][:2] for p in base['plans']]
        nodes=np.asarray(docking);near=domain.buffer(-1.5).intersection(domain.centroid.buffer(3.))
        x0,y0,x1,y1=near.bounds;choices=[]
        for x in np.arange(math.ceil(x0*10)/10,x1,.1):
            for y in np.arange(math.ceil(y0*10)/10,y1,.1):
                q=Point(x,y)
                if near.covers(q):choices.append((float(np.min(np.linalg.norm(nodes-[x,y],axis=1)))-.001*q.distance(domain.centroid),x,y))
        if not choices:raise ValueError('NO_PRIOR_ONLY_HOLE_PLACEMENT')
        _,x,y=max(choices);center=Point(x,y);placement='0.1m prior-only max-clearance lattice against every original service docking pose and depot'
    notch_placement='declared boundary perturbation'
    if zone=='A' and base:
        add14()
        from common12 import body_local
        radius=float(np.max(np.linalg.norm(body_local('diff')[0],axis=1)))+.20
        services=[s for p in base['plans'] for s in p['segments'] if s['kind']=='POINT_SERVICE']
        options=[]
        for x in np.arange(math.ceil(domain.bounds[0]*10)/10,domain.bounds[2],.1):
            cut=domain.intersection(box(x-.4,domain.bounds[3]-1,x+.4,domain.bounds[3]+1));work=domain.difference(cut)
            if cut.area<.2 or work.geom_type!='Polygon' or not work.is_valid:continue
            required=[s for s in services if work.covers(Point(s['target_xy']))]
            docking=[s['poses'][0][:2] for s in required]+[p['start'][:2] for p in base['plans']]
            clearance=min(cut.distance(Point(p)) for p in docking)
            if clearance>radius:options.append((len(required),-abs(x-domain.centroid.x),cut))
        if not options:raise ValueError('NO_ACCESSIBLE_PRIOR_NOTCH_PLACEMENT')
        _,_,cuts['notch']=max(options,key=lambda r:r[:2]);notch_placement='fixed 0.8m by 1m boundary notch on 0.1m lattice; every remaining in-domain service docking pose must retain original circumscribed clearance'
    cuts['hole']=center.buffer(min(.3,center.distance(domain.boundary)/3))
    return {k:dict(work=set_precision(domain.difference(cut),precision),exclusion=set_precision(cut,precision),placement=placement if k=='hole' else notch_placement if k=='notch' else 'base geometry') for k,cut in cuts.items()}

def qualify(zone,variant,out,seed=17020):
    add14()
    import common14,routing14
    from common12 import config,backend
    from geometry11 import parameters,load_domain,build_graph
    from partition11 import metis,repair,verify
    from spray12 import plan_spray
    from mowing12 import plan_mowing
    from planning12 import validate_routes
    from overlay17 import replace
    from inputs17 import active
    c=config();pc=parameters();original=read(active()['plan']);atlas=read(active()['atlas']);base=original['zones'][zone];domain=shape(base['task_domain'])
    definition=definitions(domain,pc['geometry_precision_m'],zone,base)[variant];work=definition['work'];exclusion=definition['exclusion'];motion=set_precision(shape(base['motion_domain']).difference(exclusion),pc['geometry_precision_m'])
    meta=load_domain(zone,'base',pc);meta.update(domain=work,motion=motion,original_domain=domain,variant=variant,centre=motion.buffer(-meta['radius_m']))
    declaration=dict(zone=zone,variant=variant,seed=seed,work=mapping(work),motion=mapping(motion),known_exclusion=mapping(exclusion),parent_task_revision=original['task_revision'],parent_plan_revision=original['plan_revision'],parent_atlas_revision=atlas['revision'],definition='physical variant v2: exclusion applies to motion and work; central-hole v1 failures are retained',placement=definition['placement'],no_formal_cluster_credit=True)
    dump(out/'declaration.json',declaration)
    result=dict(status='NOT_RUN',declaration_sha256=sha(out/'declaration.json'),closed_loop_status='NOT_RUN',formal_credit=False)
    try:
        if work.geom_type!='Polygon' or not work.is_valid:raise ValueError('DISCONNECTED_VARIANT_WORK')
        graph=build_graph(work,pc,meta);labels=metis(graph,3,seed,pc,out/'partition');labels,changes=repair(graph,labels,3,pc);checked=verify(work,graph,labels,3,pc)
        if not checked['passed']:raise ValueError('VARIANT_PARTITION_REJECTED')
        dump(out/'partition.json',dict(labels=labels.tolist(),verification=checked,repair=changes,units=[dict(geometry=mapping(g),cluster=int(labels[i])) for i,g in enumerate(graph['cells'])]))
        if zone=='A':
            # Variant task identity permits a new depot, but retain the original
            # qualified one when possible instead of using a polygon centroid
            # that can jump onto the new hole's boundary.
            import spray12
            meta['depot']=base['plans'][0]['start'][:2]
            source=Path(spray12.__file__).read_text();source=replace(source,"centre=meta['domain'].representative_point();depot=[centre.x-.42,centre.y]","depot=meta['depot']")
            module=types.ModuleType('variant_spray17');exec(compile(source,str(Path(__file__)),'exec'),module.__dict__)
            plans=module.plan_spray(meta,graph,labels,c,out/'routes');validation=validate_routes(plans,c)
        else:
            # This preliminary planner supplies depots and whole-zone headland,
            # never the denominator: every graph cell remains in its obligation.
            plans,_=plan_mowing(meta,graph,labels,c,out/'initial_routes')
            z=copy.deepcopy(base);z.update(task_domain=mapping(work),motion_domain=mapping(motion),plans=plans)
            observation=ROOT/read(ROOT/original['execution_routes']['B1']['path'])['observation']
            collision=routing14.CollisionMap(observation,atlas['regions']['B']['site_extent_xy'])
            collision.allowed=collision.allowed.intersection(translate(motion,xoff=-28,yoff=-14))
            # The project requires >=95%; retaining the exact full target makes
            # unreachable perimeter explicit rather than erasing it from area.
            src=Path(routing14.__file__).read_text();src=replace(src,'coverage>=1-1e-8','coverage>=c[\'coverage_fraction\']')
            module=types.ModuleType('variant_route_candidate17');exec(compile(src,str(Path(__file__)),'exec'),module.__dict__)
            raw=[backend(dict(mode='swaths',work=mapping(work.buffer(.4,join_style=2)),motion=mapping(motion),spacing=c['work_spacing_m'],headland=c['whole_zone_headland_m'],angle=i*math.pi/12),out/'swaths','angle_'+str(i)) for i in range(12)]
            for cluster in range(3):
                target=union_all([cell for i,cell in enumerate(graph['cells']) if labels[i]==cluster]);template=copy.deepcopy(plans[cluster]);template['coverage_check'].update(covered=mapping(target),domain_area_m2=target.area)
                z['plans']=[template];candidates=[];solutions=[]
                for swaths in raw:
                    for phase in [-.3,-.25,-.2,-.15,0.,.15,.3]:
                        route,evidence=module.candidate(z,swaths,phase,collision,c);candidates.append(evidence)
                        if route:solutions.append(route)
                dump(out/f'B{cluster+1}_candidates.json',candidates)
                if not solutions:raise ValueError('NO_EXECUTABLE_VARIANT_ROUTE B'+str(cluster+1))
                route=min(solutions,key=lambda r:r['timing']['total_s']);template.update(segments=route['segments'],timing=route['timing'],end=route['segments'][-1]['poses'][-1])
                from motion12 import effect_polygon
                credited=union_all([effect_polygon([p for p,v in zip(s['poses'],s['timing']['velocities']) if v>=.02],c) for s in route['segments'] if s['tool_on']]).intersection(target)
                template['coverage_check']=dict(fraction=credited.area/target.area,domain_area_m2=target.area,covered=mapping(credited),covered_m2=credited.area,residual=mapping(target.difference(credited)),residual_m2=target.difference(credited).area,unconnected=[])
                template['original_obligation']=mapping(target);plans[cluster]=template
            validation=validate_routes(plans,c)
        dump(out/'planned_tasks.json',dict(zone=zone,variant=variant,plans=plans,work=mapping(work),motion=mapping(motion),partition_verification=checked,route_verification=validation))
        result.update(status='GEOMETRY_VALID' if validation['passed'] else 'INFEASIBLE',partition=checked,route=validation,nominal_s=[p['timing']['total_s'] for p in plans])
    except (ValueError,TimeoutError) as exc:result.update(status='INFEASIBLE',reason=str(exc))
    dump(out/'geometry_qualification.json',result);return result

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--zone',choices=['A','B'],required=True);parser.add_argument('--variant',choices=['base','notch','hole'],required=True);parser.add_argument('--out',type=Path,required=True);args=parser.parse_args()
    if not args.out.resolve().is_relative_to(RESULTS):raise ValueError('OUTPUT_SCOPE')
    print(json.dumps(qualify(args.zone,args.variant,args.out),ensure_ascii=False))
