"""Real production visibility graph and TSP on an explicitly synthetic obstacle."""
import sys,tempfile
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'ws/src/coverage_planner/scripts'))
from spray12 import *
c=config();motion=box(0,0,20,20).difference(box(8,6,12,14));points=[[3,10],[17,10],[17,3],[3,3]];cost,paths,info=visibility(points,motion,c)
assert cost[0,1]>14
order,length=nearest_2opt(cost)
with tempfile.TemporaryDirectory() as t:
    actual=backend(dict(mode='tsp',cost=np.rint(cost*1000).astype(int).tolist()),Path(t),'lesson')
assert sorted(actual['order'])==[0,1,2,3]
try:visibility([[.01,.01],[5,5]],box(0,0,10,10),c)
except ValueError:pass
else:raise AssertionError('unreachable point accepted')
print('PASS: obstacle detour',round(cost[0,1],3),'m; NN+2-opt',order,round(length,3),'m; real OR-Tools',actual['order'])
