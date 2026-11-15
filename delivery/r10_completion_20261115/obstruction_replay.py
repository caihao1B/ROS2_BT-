"""Offline geometry diagnostic only; never grants online task credit."""
import sys,json,math
from pathlib import Path
ROOT=Path('/home/ceihao/robot_project')
sys.path.insert(0,str(ROOT/'optimization_v2/ws/src/task_platform/scripts'))
from recovery_mowing_r10 import retreat_prefix,validate_candidate,gear_runs
from ompl_endpoint_r10 import launch,sample
from footprint_r09 import body_polygon
from shapely.geometry import shape
p=ROOT/'optimization_v2/three_tasks/results/R12/mowing_20261114_235742_951592'
cfg=json.loads((p/'mission.json').read_text());live=json.loads((p/'live.json').read_text())
start=live['loc']['pose'];goal=cfg['clusters'][0]['segments'][11]['poses'][-1]
allowed=shape(cfg['allowed_prior']).buffer(-cfg['footprint_margin_m'])
folder=Path(__file__).parent/'obstruction_replay';rows=[]
for retreat in [0.,.3,.6,.8]:
    prefix=retreat_prefix(start,retreat) if retreat else [start]
    child,response=launch(prefix[-1],goal,1/cfg['vehicle']['steering']['max_curvature']+.10,folder/str(retreat))
    child.communicate(timeout=5);candidate,details=sample(prefix[-1],response);candidate=prefix[:-1]+candidate
    ok,why=validate_candidate(candidate,start,goal,cfg)
    runs=gear_runs(candidate);envelope_blocked=[]
    for r in runs:
        x1=-.571-(.6 if r['direction']<0 else .1);x2=2.209+(.6 if r['direction']>0 else .1)
        footprint=[[x1,-1.485],[x2,-1.485],[x2,1.485],[x1,1.485]]
        invalid=[q for q in r['poses'] if not allowed.covers(body_polygon(footprint,q[0]+28,q[1]+14,q[2]))]
        envelope_blocked.append(dict(direction=r['direction'],count=len(invalid),of=len(r['poses'])))
    rows.append(dict(retreat_m=retreat,start=start,goal=goal,accepted=ok,reason=why,details=details,envelope_blocked=envelope_blocked))
print(json.dumps(rows,indent=2));(folder/'summary.json').write_text(json.dumps(rows,indent=2))
