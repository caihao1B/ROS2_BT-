"""Offline scale correction, isolated from the recorded v1 and formal S17.

No runtime manifest is emitted: measured-map route geometry is necessary but
does not qualify a new task for closed-loop execution or fresh obstacle safety.
"""
from common_showcase import *
import argparse, copy, math
import numpy as np
from common17 import add14
add14()
import scale_routing as routing14
from common12 import config, backend
from geometry11 import parameters, load_domain, build_graph
from partition11 import solve
from motion12 import effect_polygon
from planning12 import validate_routes
from shapely.geometry import Polygon, shape, mapping, box
from shapely import set_precision, union_all
from shapely.affinity import translate


def balanced_strip_partition(domain, k=3):
    """Area-weighted sweep cuts for this flat, convex parcel; reject other shapes.

    A monotone cumulative area function is inverted by bisection. At constant
    cutting width and qualified speed area is proportional to work time. Turns
    are accounted for later by the timed path, never by this lower-level proxy.
    """
    if domain.symmetric_difference(domain.convex_hull).area>1e-8:
        raise ValueError('SWEEP_PARTITION_REQUIRES_CONVEX_PARCEL')
    x0,y0,x1,y1=domain.bounds;cuts=[y0]
    for i in range(1,k):
        lo,hi=y0,y1
        for _ in range(60):
            mid=(lo+hi)/2
            if domain.intersection(box(x0-1,y0-1,x1+1,mid)).area<domain.area*i/k:lo=mid
            else:hi=mid
        cuts.append((lo+hi)/2)
    cuts.append(y1)
    regions=[domain.intersection(box(x0-1,a,x1+1,b)) for a,b in zip(cuts[:-1],cuts[1:])]
    checks=dict(exactly_three=len(regions)==3,
        connected=all(g.geom_type=='Polygon' and g.is_valid and g.area>0 for g in regions),
        complete=union_all(regions).symmetric_difference(domain).area<1e-8,
        disjoint=sum(a.intersection(b).area for i,a in enumerate(regions) for b in regions[i+1:])<1e-8,
        adjacent=all(a.boundary.intersection(b.boundary).length>1e-6 for a,b in zip(regions[:-1],regions[1:])))
    if not all(checks.values()):raise ValueError('STRIP_PARTITION_REJECTED')
    return dict(algorithm='equal area cumulative sweep bisection, convex flat parcel only',
        workload_proxy='area / (cutting width * qualified speed); final timed routes include turns',
        verification=dict(passed=True,checks=checks),cuts_y=cuts,
        clusters=dict(type='FeatureCollection',features=[dict(type='Feature',geometry=mapping(g),properties=dict(cluster=i,area_m2=g.area)) for i,g in enumerate(regions)]))


def build(out):
    if out.exists():
        raise ValueError('NEW_OUTPUT_DIRECTORY_REQUIRED')
    old = read(OUT/'v1/mission_plan.json')
    formal_path = BASE/'plans/s17_full/mission_plan.json'
    formal = read(formal_path)
    atlas_path = BASE/'maps/s17_full/manifest.json'
    atlas = read(atlas_path)
    c, pc = config(), parameters()
    domain = set_precision(Polygon([
        [123.5,55.5], [130.8,55.5], [131.5,57.0], [131.0,59.8],
        [130.0,60.3], [123.8,60.3], [123.2,58.3]]), pc['geometry_precision_m'])
    parent_motion = shape(formal['zones']['B']['motion_domain'])
    if not parent_motion.covers(domain):
        raise ValueError('NEW_TASK_OUTSIDE_EXISTING_WORKSPACE')
    # Explicit tool-off maneuvering apron. It is not a credit denominator and
    # is not advertised as completely mowed lawn. Never use the whole map as
    # unbounded turning permission. Existing measured obstacles still apply.
    motion = domain.buffer(c['whole_zone_headland_m'], quad_segs=12).intersection(parent_motion)
    observation = ROOT/read(ROOT/formal['execution_routes']['B1']['path'])['observation']
    collision = routing14.CollisionMap(observation, atlas['regions']['B']['site_extent_xy'])
    collision.allowed = collision.allowed.intersection(translate(motion,xoff=-28,yoff=-14))
    import shapely
    shapely.prepare(collision.allowed)
    meta = load_domain('B', 'base', pc)
    meta.update(domain=domain, motion=motion, original_domain=domain,
                centre=motion.buffer(-meta['radius_m']))
    partition = balanced_strip_partition(domain)
    dump(out/'partition/partition.json',partition)
    regions = [shape(f['geometry']) for f in partition['clusters']['features']]
    # Search the long-axis family first, keeping a finite, declared budget.
    angles = [0., math.pi/12, -math.pi/12]
    raws = [backend(dict(mode='swaths', work=mapping(domain.buffer(.4,join_style=2)),
                         motion=mapping(motion), spacing=c['work_spacing_m'],
                         headland=c['whole_zone_headland_m'], angle=a),
                    out/'swaths', 'angle_'+str(i)) for i,a in enumerate(angles)]
    plans = []
    for i, target in enumerate(regions):
        template = dict(cluster=i, zone='B', vehicle='ackermann',
            method='F2C swaths + connected area sweep partition + collision-checked directed Dubins Held-Karp',
            start=formal['zones']['B']['plans'][0]['start'],
            end=formal['zones']['B']['plans'][0]['end'],
            whole_zone_headland=mapping(Polygon()),
            coverage_check=dict(covered=mapping(target),domain_area_m2=target.area))
        zone = dict(task_domain=mapping(domain),motion_domain=mapping(motion),plans=[template],
                    turn_waypoints=[[122.0,54.5,0.],[122.0,60.5,math.pi],
                                    [132.0,54.0,0.],[133.0,60.0,math.pi]])
        checks, solutions = [], []
        for raw in raws:
            for phase in [0., -.15, .15]:
                try:
                    route, evidence = routing14.candidate(zone,raw,phase,collision,c)
                except TimeoutError as exc:
                    route, evidence = None, dict(reason=str(exc))
                checks.append(evidence)
                if route:
                    solutions.append(route)
            if solutions:
                break
        dump(out/f'B{i+1}_search.json',checks)
        if not solutions:
            raise ValueError('NO_OFFLINE_CANDIDATE_B'+str(i+1))
        route = min(solutions,key=lambda r:r['timing']['total_s'])
        credit = union_all([effect_polygon([p for p,v in zip(s['poses'],s['timing']['velocities']) if v>=.02],c)
                            for s in route['segments'] if s['tool_on']]).intersection(target)
        template.update(segments=route['segments'],timing=route['timing'],
            original_obligation=mapping(target), planned_coverage_certificate=dict(
                covered=mapping(credit), fraction=credit.area/target.area))
        template['coverage_check'].update(fraction=credit.area/target.area,
            covered_m2=credit.area,residual_m2=target.difference(credit).area,
            residual=mapping(target.difference(credit)),unconnected=[],all_work_segments=True)
        plans.append(template)
        print('B'+str(i+1), 'area',target.area,'nominal_s',route['timing']['total_s'],flush=True)
    verification = validate_routes(plans,c)
    if not verification['passed']:
        raise ValueError('OFFLINE_SWEEP_VALIDATION_FAILED')
    sources = {ROOT/rel for rel in old['versions'] if (ROOT/rel).is_file()}
    sources.update([Path(__file__).resolve(), Path(routing14.__file__), formal_path, atlas_path, observation,
                    BASE/'config/vehicles/ackermann.yaml',BASE/'config/vehicles/qualified_profiles.json',
                    ROOT/'offline_coverage/motion.py'])
    d = dict(schema='MOWER-SCALE-CANDIDATE/1',scope='OFFLINE_DESIGN_ONLY',
        parent_demo_revision=old['plan_revision'],base_plan_revision=formal['plan_revision'],
        frame=formal['frame'],vehicle_profile_unchanged=True,
        task_domain=mapping(domain),motion_domain=mapping(motion),
        apron=mapping(motion.difference(domain)),
        apron_policy='Authorized tool-off maneuvering only; not claimed mowed. Full-lawn headland mowing remains a separate obligation.',
        partition=partition,partition_parameters=pc,plans=plans,route_verification=verification,
        observation=str(observation.relative_to(ROOT)),
        provenance={str(p.relative_to(ROOT)):sha(p) for p in sorted(sources)},
        online_execution_allowed=False,closed_loop_status='NOT_RUN',formal_credit=False,
        changed_obligation=True,performance_comparison='Task-scale correction, not same-task algorithm speedup',
        original_demo_untouched=True,formal_s17_status='PARTIAL')
    d['revision']=canonical(d)
    dump(out/'candidate.json',d)
    print(json.dumps(dict(revision=d['revision'],area_m2=domain.area,
        nominal_seconds=[p['timing']['total_s'] for p in plans],online_execution_allowed=False)))


if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path,default=OUT/'mower_scale_v2c')
    a=ap.parse_args()
    if not a.out.resolve().is_relative_to(OUT):raise ValueError('SHOWCASE_OUTPUT_SCOPE_REQUIRED')
    build(a.out)
