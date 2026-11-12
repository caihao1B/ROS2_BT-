"""Versioned task-scale search and actual route-time feedback into S11 labels."""
from shapely.affinity import scale as scale_geometry
from common12 import *
from spray12 import plan_spray
from mowing12 import plan_mowing

def resized_domain(zone,factor,c):
    pc=partition_parameters();meta=load_domain(zone,'base',pc);original=meta['domain'];workspace=meta['workspace'];small=scale_geometry(workspace,xfact=factor,yfact=factor,origin='centroid').difference(meta['permanent'])
    task=set_precision(small,pc['geometry_precision_m'])
    if task.geom_type!='Polygon' or not task.is_valid or not original.covers(task.buffer(-1e-7)):raise ValueError('INVALID_RESIZED_TASK')
    meta.update(domain=task,motion=original,centre=original.buffer(-meta['radius_m']),unresolved_boundary=task.difference(original.buffer(-meta['radius_m'])))
    meta['original_domain']=original;meta['scope_factor']=factor;return meta,pc

def rebalanced(g,labels,plans,pc):
    actual=np.zeros(len(g['cells']))
    # Aggregate actual route-model cost is redistributed using original cell
    # workload proportions. This is an approximation, not per-segment attribution.
    for p in plans:
        ids=np.flatnonzero(np.asarray(labels)==p['cluster']);cluster_area=sum(g['area'][i] for i in ids)
        total=p['timing']['total_s'];weights=g['weights'][ids];weighted=weights/weights.sum()
        actual[ids]=total*weighted
    adjusted=g|dict(weights=actual,iw=np.maximum(1,np.rint(actual*100)).astype(int))
    proposal,history=repair(adjusted,labels,3,pc);return proposal,history,actual

def validate_routes(plans,c):
    checks=dict(six_or_three_nonempty=len(plans)==3 and {p['cluster'] for p in plans}=={0,1,2},segment_geometry=all(p['timing']['all_segments_valid'] for p in plans),time_budget=all(p['timing']['total_s']<=c['normal_budget_s'] for p in plans))
    continuity=True
    for p in plans:
        ss=p['segments']
        for a,b in zip(ss,ss[1:]):
            continuity &= math.dist(a['poses'][-1][:2],b['poses'][0][:2])<.002 and abs(wrap(a['poses'][-1][2]-b['poses'][0][2]))<.002
        continuity &= math.dist(p['start'][:2],p['end'][:2])<.002 and abs(wrap(p['start'][2]-p['end'][2]))<.002
    checks['continuity_and_return']=bool(continuity)
    checks['coverage']=all(p['coverage_check'].get('all_serviceable_once',False) if p['zone']=='A' else p['coverage_check']['fraction']>=c['coverage_fraction'] and not p['coverage_check']['unconnected'] for p in plans)
    return dict(passed=all(checks.values()),checks=checks)

def plan_zone(zone,c,out,factors=None):
    attempts=[];best=None
    for factor in (factors or c['scale_candidates'])[:c['max_scope_iterations']]:
        meta,pc=resized_domain(zone,factor,c);g=build_graph(meta['domain'],pc,meta);folder=out/f'{zone}_scale_{factor:g}';initial=partition_solve(meta,g,pc,'metis',c['seed'],folder/'initial_partition');labels=np.array(initial['labels']);seen=set()
        for iteration in range(c['max_rebalance_iterations']):
            label_hash=identity(labels.tolist())
            if label_hash in seen:break
            seen.add(label_hash);where=folder/f'iteration_{iteration}';where.mkdir(parents=True,exist_ok=True);started=time.monotonic()
            if zone=='A':plans=plan_spray(meta,g,labels,c,where);extra={}
            else:plans,extra=plan_mowing(meta,g,labels,c,where)
            verified=validate_routes(plans,c);geometry=partition_verify(meta['domain'],g,labels,3,pc)
            data=dict(zone=zone,scale=factor,iteration=iteration,plans=plans,verification=verified,partition_verification=geometry,
                task_domain=mapping(meta['domain']),original_domain=mapping(meta['original_domain']),out_of_scope=mapping(meta['original_domain'].difference(meta['domain'])),motion_domain=mapping(meta['motion']),
                original_area_m2=meta['original_domain'].area,task_area_m2=meta['domain'].area,labels=labels.tolist(),units=collection([feature(poly,unit=i,cluster=int(labels[i])) for i,poly in enumerate(g['cells'])]),points=g['points'],extra=extra,planning_wall_s=time.monotonic()-started,
                status='VALIDATED' if verified['passed'] and geometry['passed'] else 'INFEASIBLE',execution_scope='OFFLINE_KNOWN_PRIORS_ONLY')
            next_labels,history,weights=rebalanced(g,labels,plans,pc);data['feedback']=dict(method='actual_route_time_scaled_cell_weights_with_connected_boundary_exchange',changes=int(np.count_nonzero(next_labels!=labels)),history=history,cell_weights_s=weights.tolist(),next_assignment_sha256=identity(next_labels.tolist()))
            dump(where/'plan.json',data);attempts.append(dict(scale=factor,iteration=iteration,status=data['status'],checks=verified['checks'],times_s=[p['timing']['total_s'] for p in plans],changed_cells=data['feedback']['changes'],path=str((where/'plan.json').relative_to(ROOT))))
            print(zone,factor,iteration,data['status'],[round(p['timing']['total_s'],1) for p in plans],verified['checks'],flush=True)
            score=max(p['timing']['total_s'] for p in plans)
            if data['status']=='VALIDATED' and (best is None or factor>best['scale'] or (factor==best['scale'] and score<max(p['timing']['total_s'] for p in best['plans']))):best=data
            # Always feed actual costs back; a fixed point is an explicit stopping condition.
            labels=next_labels
            if best is not None and iteration>=1:break
        if best is not None:break
    return dict(selected=best,attempts=attempts,status='VALIDATED' if best else 'INFEASIBLE')
