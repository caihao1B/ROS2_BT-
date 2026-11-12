"""Whole-field rejection, surveyed frame binding, independently rechecked plans."""
import copy
from common12 import *
from motion12 import swept_check,profile,effect_polygon,summarize

def original_infeasible(c):
    a=read(BASE/'plans/candidate/zone_a_points.json');b=read(BASE/'plans/candidate/zone_b_domains.geojson')['features'];domain=shape(next(f['geometry'] for f in b if f['properties']['layer']=='domain'))
    n=len(a);spray=(n-3)*2/.12+n*c['spray_dwell_seconds'];mow=domain.area*.95/(.9*.12)
    return dict(status='INFEASIBLE',original_map_unchanged=True,k=3,speed_mps=.12,normal_total_budget_s=3*c['normal_budget_s'],hard_total_budget_s=3*c['hard_budget_s'],
       spray=dict(points=n,optimistic_total_s=spray,assumption='fixed heading docking, 2 m centre grid; no return legs/rotation/start-stop overhead'),
       mowing=dict(area_m2=domain.area,coverage_required=.95,optimistic_total_s=mow,assumption='straight swath model, full 0.9 m width, zero turns/transit/overlap'),
       reason='Original mission under current speed/tool contract cannot meet three 24-minute clusters; original obligations retained separately')

def frame_binding():
    # Reuse S08's real surveyed-site solver; no true robot trajectory or spawn pose.
    sys.path.insert(0,str(BASE/'ws/src/gnss_supervisor/scripts'))
    from alignment import solve
    a=yaml.safe_load((BASE/'config/gnss_alignment.yaml').read_text());R,t=solve(a)
    return dict(source='S08 independently declared site controls',prior_frame='prior_local_enu',target_frame='map',rotation=R.tolist(),translation=t.tolist(),
       map_id=a['map_id'],map_revision=a['map_revision'],frame_epoch=a['frame_epoch'],calibration_revision=a['calibration_revision'],
       sigma_translation_m=a['translation_sigma_m'],sigma_yaw_rad=a['yaw_sigma_rad'],nonrigid_map_error_corrected=False,
       live_execution_requires='S13/S14 localization integrity, start-pose admission, and runtime perception; S08 connector/B deformation remains an online limitation')

def version_bindings():
    paths=inputs()+[BASE/'maps/frozen/manifest.json',BASE/'results/S11/latest.json',BASE/'ws/src/gnss_supervisor/scripts/alignment.py']+files(BASE/'ws/src/coverage_planner')+[BIN,BASE/'config/s12_dependency_lock.json',BASE/'scripts/build_s12.sh',BASE/'stages/S12/run.sh']
    return {str(p.relative_to(ROOT)):sha(p) for p in sorted(set(paths)) if p.exists()}

def verify_selected(selected,c):
    result={};passed=True
    for zone,d in selected.items():
        from planning12 import resized_domain
        meta,pc=resized_domain(zone,d['scale'],c);g=build_graph(meta['domain'],pc,meta);geometry=partition_verify(meta['domain'],g,d['labels'],3,pc)
        domain=shape(d['task_domain']);motion=shape(d['motion_domain']);areas=[];labels=d['labels'];units=[shape(f['geometry']) for f in d['units']['features']]
        clusters=[union_all([u for i,u in enumerate(units) if labels[i]==k]) for k in range(3)];seen=[];details=[]
        for plan in d['plans']:
            checks=[];effects=[];recomputed=[]
            for s in plan['segments']:
                v=swept_check(s['poses'],motion,plan['vehicle'],c,s['kind']=='ROTATE');t=profile(s['poses'],plan['vehicle'],c,s['kind']=='ROTATE',s.get('stop_curvature',False))
                checks.append(v['passed'] and abs(t['motion_seconds']-s['timing']['motion_seconds'])<1e-6 and np.allclose(t['velocities'],s['timing']['velocities'],atol=1e-8) and t['steering_rate']<=.601 and t['max_speed']<=.120001 and t['max_acceleration']<=.500001 and t['max_deceleration']<=.800001)
                recomputed.append(s|dict(timing=t))
                if zone=='A' and s['kind']=='POINT_SERVICE':
                    p=s['poses'][0];target=[p[0]+.42*math.cos(p[2]),p[1]+.42*math.sin(p[2])];checks.append(math.dist(target,s['target_xy'])<=.08 and s['timing']['max_speed']==0);seen.append(s['task_id'])
                if zone=='B' and s['tool_on']:
                    good=[p for p,v in zip(s['poses'],t['velocities']) if v>=.02];effects.append(effect_polygon(good,c))
            checks.append(all(math.dist(a['poses'][-1][:2],b['poses'][0][:2])<.002 and abs(wrap(a['poses'][-1][2]-b['poses'][0][2]))<.002 for a,b in zip(plan['segments'],plan['segments'][1:])))
            expected=summarize(recomputed,c,sum(s['kind']=='POINT_SERVICE' for s in recomputed));checks.append(abs(expected['total_s']-plan['timing']['total_s'])<1e-6)
            coverage=True;fraction=None
            if zone=='B':fraction=union_all(effects).intersection(clusters[plan['cluster']]).area/clusters[plan['cluster']].area;coverage=fraction>=c['coverage_fraction']
            details.append(dict(cluster=plan['cluster'],segments_valid=all(checks),coverage=fraction,time_s=plan['timing']['total_s'],budget=plan['timing']['total_s']<=c['normal_budget_s'],passed=all(checks) and coverage and plan['timing']['total_s']<=c['normal_budget_s']))
        count_ok=sorted(seen)==sorted(p['id'] for p in d['points']) if zone=='A' else True
        union_error=union_all(clusters).symmetric_difference(domain).area;overlap=sum(clusters[i].intersection(clusters[j]).area for i in range(3) for j in range(i+1,3))
        ok=geometry['passed'] and len(details)==3 and {p['cluster'] for p in d['plans']}=={0,1,2} and all(x['passed'] for x in details) and count_ok and union_error<1e-6*domain.area and overlap<1e-6*domain.area
        result[zone]=dict(passed=ok,partition_geometry=geometry,clusters=details,unique_points=count_ok,union_difference_m2=union_error,overlap_m2=overlap);passed &= ok
    return dict(passed=passed,zones=result)

def bind_mission(selected,connectors,c):
    payload=dict(schema='S12-MISSION/1',status='VALIDATED',validation_scope='OFFLINE_GEOMETRY_KINEMATICS_TIME_ON_KNOWN_PRIORS',online_execution_allowed=False,
       task_revision=identity({k:v['task_domain'] for k,v in selected.items()}),frame=frame_binding(),k=3,seed=c['seed'],
       normal_budget_s=c['normal_budget_s'],hard_budget_s=c['hard_budget_s'],zones=selected,connector_routes=connectors,
       versions=version_bindings(),requires_runtime=['start pose within tolerance','unique command ownership','localization integrity','S09 dynamic/low obstacle costmap','S10 feedback-gated work','honor STOP_AND_STEER events','global 1800 s deadline including shutdown and residual save'],
       online_baseline='offline complete obligations + online local repair; purely online coverage comparison deferred to S17',
       original_mission=original_infeasible(c))
    payload['plan_revision']=identity(payload);return payload

def check_binding(plan,current=None):
    body=copy.deepcopy(plan);revision=body.pop('plan_revision',None)
    if identity(body)!=revision:raise ValueError('PLAN_HASH_MISMATCH')
    if plan['status']!='VALIDATED' or plan['online_execution_allowed']:raise ValueError('OFFLINE_VALIDATION_CONTRACT')
    current=version_bindings() if current is None else current
    if current!=plan['versions']:raise ValueError('STALE_INPUT_VERSION')
    return True
