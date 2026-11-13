"""Paired current-domain offline partition and route-time experiment."""
from common17 import *
import copy,numpy as np,math
from shapely.geometry import shape,mapping,box,Point
from shapely import set_precision

def run(out=None):
    sys.path.insert(0,str(BASE/'ws/src/coverage_planner/scripts'))
    from common12 import config
    from geometry11 import parameters,load_domain,build_graph
    from partition11 import metis,kmeans,repair,verify
    from planning12 import rebalanced,validate_routes
    from spray12 import plan_spray
    from mowing12 import plan_mowing
    from darp17 import divide
    c=config();pc=parameters();original=read(BASE/'plans/validated/mission_plan.json');out=Path(out or RESULTS/'partition_comparison').resolve()
    if not out.is_relative_to(RESULTS):raise ValueError('OUTPUT_SCOPE')
    out.mkdir(parents=True,exist_ok=True);results=[]
    for z in ['A','B']:
        base_meta=load_domain(z,'base',pc);domain=shape(original['zones'][z]['task_domain']);x0,y0,x1,y1=domain.bounds
        variants={'base':domain,'notch':domain.difference(box((x0+x1)/2-.4,y1-1,(x0+x1)/2+.4,y1+1))}
        center=domain.representative_point();radius=min(.3,center.distance(domain.boundary)/3);variants['hole']=domain.difference(center.buffer(radius))
        for name,geometry in variants.items():
            meta=copy.deepcopy(base_meta);meta['variant']=name;meta['domain']=set_precision(geometry,pc['geometry_precision_m']);meta['motion']=shape(original['zones'][z]['motion_domain']);meta['original_domain']=domain
            if meta['domain'].geom_type!='Polygon' or not meta['domain'].is_valid:raise ValueError('VARIANT_NOT_CONTINUOUS '+z+name)
            g=build_graph(meta['domain'],pc,meta);input_hash=canonical(dict(domain=mapping(meta['domain']),weights=g['weights'].tolist(),edges=g['edges'],config=c,partition=pc))
            for seed in read(EVAL/'seeds.json')['paired']:
                for method in ['kmeans_repaired','darp','metis_time']:
                    where=out/f'{z}_{name}_{seed}_{method}';where.mkdir(exist_ok=True);started=time.monotonic();detail={};route_history=[]
                    if method=='kmeans_repaired':labels=kmeans(g,3,seed)
                    elif method=='darp':labels,detail=divide(g,3,seed)
                    else:labels=metis(g,3,seed,pc,where)
                    labels,history=repair(g,labels,3,pc);geometry_check=verify(meta['domain'],g,labels,3,pc)
                    # Identical planners/speeds/geometry for all partitions. Only
                    # METIS+time gets its explicitly named bounded feedback step.
                    for iteration in range(2 if method=='metis_time' else 1):
                        folder=where/f'route_{iteration}';folder.mkdir(exist_ok=True)
                        plans=plan_spray(meta,g,labels,c,folder) if z=='A' else plan_mowing(meta,g,labels,c,folder)[0]
                        check=validate_routes(plans,c);route_history.append(dict(iteration=iteration,labels=labels.tolist(),times=[p['timing']['total_s'] for p in plans],validation=check))
                        if method=='metis_time' and iteration==0:
                            labels,new_history,_=rebalanced(g,labels,plans,pc);geometry_check=verify(meta['domain'],g,labels,3,pc)
                    row=dict(zone=z,variant=name,seed=seed,method=method,input_hash=input_hash,wall_s=time.monotonic()-started,geometry=geometry_check,route_history=route_history,detail=detail,repair=history,status='FEASIBLE' if geometry_check['passed'] and check['passed'] else 'REJECTED_BY_GEOMETRY_OR_TIME',scope='offline model cost; no measured driving time')
                    dump(where/'comparison.json',row);results.append(row);dump(out/'result.json',dict(executed=len(results),required=54,results=results,formal_cluster_credit=False));print(z,name,seed,method,row['status'],route_history[-1]['times'],flush=True)
    return results
if __name__=='__main__':
    import argparse
    parser=argparse.ArgumentParser();parser.add_argument('--out',type=Path);args=parser.parse_args();run(args.out)
