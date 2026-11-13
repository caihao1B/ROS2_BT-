"""Current task polygons: adapted DARP must not lose disconnected edge cells."""
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from common17 import *
sys.path.insert(0,str(BASE/'ws/src/coverage_partition/scripts'))
from geometry11 import load_domain,build_graph,parameters
from partition11 import repair,verify
from darp17 import divide
from shapely.geometry import shape
import numpy as np

out=RESULTS/'component_contracts';out.mkdir(parents=True,exist_ok=True);results=[];pc=parameters();p=read(BASE/'plans/validated/mission_plan.json')
for zone in ['A','B']:
    meta=load_domain(zone,'base',pc);meta['domain']=shape(p['zones'][zone]['task_domain']);g=build_graph(meta['domain'],pc,meta)
    a,e=divide(g,3,42001);b,_=divide(g,3,42001)
    assert np.array_equal(a,b),'same seed must reproduce labels'
    a,r=repair(g,a,3,pc);v=verify(meta['domain'],g,a,3,pc);assert v['passed'],v
    results.append(dict(zone=zone,verification=v,iterations=e['iterations'],scope='geometry and determinism only; path feasibility not implied'))
dump(out/'darp_contract.json',dict(passed=True,results=results));print('DARP graph adaptation: two actual task polygons preserve all obligations and connectivity; same-seed repeat matches.')
