"""Summarize every predeclared offline pair, including infeasible partitions."""
from common17 import *
from statistics17 import quantiles
import statistics

def summarize(out=None):
    out=Path(out or RESULTS/'partition_comparison');source=out/'result.json';data=read(source);values=data['results'];groups={};pairs=[]
    for zone in ['A','B']:
        for variant in ['base','notch','hole']:
            for method in ['kmeans_repaired','darp','metis_time']:
                selected=[r for r in values if (r['zone'],r['variant'],r['method'])==(zone,variant,method)]
                groups[zone+'_'+variant+'_'+method]=dict(executed=len(selected),feasible=sum(r['status']=='FEASIBLE' for r in selected),worst_cluster_nominal_s=quantiles([max(r['route_history'][-1]['times']) for r in selected]),wall_s=quantiles([r['wall_s'] for r in selected]))
            for seed in read(EVAL/'seeds.json')['paired']:
                r=[r for r in values if (r['zone'],r['variant'],r['seed'])==(zone,variant,seed)]
                if len(r)!=3 or len({x['input_hash'] for x in r})!=1:raise ValueError('UNPAIRED_PARTITION_COMPARISON')
                baseline=next(x for x in r if x['method']=='kmeans_repaired')
                pairs.append(dict(zone=zone,variant=variant,seed=seed,input_hash=r[0]['input_hash'],max_time_s={x['method']:max(x['route_history'][-1]['times']) for x in r},scope='same domains, weights, planners, speeds and safety margins'))
    result=dict(status='PASS' if len(values)==54 and len(pairs)==18 else 'PARTIAL',groups=groups,pairs=pairs,source_sha256=sha(source),closed_loop_credit=False,formal_credit=False,limitations=['Offline timing model; not Gazebo task duration.','DARP-inspired weighted graph adaptation, not the original uniform-grid optimality theorem.','Geometry variants are task polygons; physical scene changes and driving remain unqualified.'])
    dump(out/'summary17.json',result);return result
if __name__=='__main__':print(json.dumps(summarize(),ensure_ascii=False,indent=2))
