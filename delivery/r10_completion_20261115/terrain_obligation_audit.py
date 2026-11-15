"""Offline original-world/plan conflict audit; never imported by online tasks."""
import json,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2];OUT=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT/'optimization_v2/ws/src/task_platform/scripts'))
from footprint_r09 import body_polygon
from shapely.geometry import box,Point
import matplotlib;matplotlib.use('Agg')
import matplotlib.pyplot as plt
run=ROOT/'optimization_v2/three_tasks/results/R12/mowing_20261115_003535_670288'
cfg=json.loads((run/'mission.json').read_text());segments=cfg['clusters'][0]['segments'];ramp=box(120.,44.,126.,47.)
rows=[]
for i,s in enumerate(segments):
    goal=s['poses_world'][-1];body=body_polygon(cfg['vehicle']['footprint'],*goal)
    overlap=body.intersection(ramp).area
    if overlap>1e-5:rows.append(dict(original_segment_index=i,id=s['id'],kind=s['kind'],endpoint_world=goal,base_point_inside_ramp=ramp.contains(Point(goal[:2])),body_overlap_m2=overlap))
report=dict(source_run=str(run.relative_to(ROOT)),original_segments=len(segments),world_grade_degrees=10.,current_terrain_qualification_degrees=5.,conflicts=rows,scope='Offline original mesh and original endpoint geometry only. Potential physical incompatibility requiring actual observed checks; not online knowledge or global no-path proof. Original route and acceptance standards unchanged.')
(OUT/'terrain_obligation_audit.json').write_text(json.dumps(report,indent=2)+'\n')
fig,ax=plt.subplots(figsize=(10,6));ax.fill(*ramp.exterior.xy,color='#cc7355',alpha=.45,label='Original 10 degree ramp footprint')
for i,s in enumerate(segments):
    ps=s['poses_world'];ax.plot([p[0] for p in ps],[p[1] for p in ps],color='#297b8d' if s['kind']=='COVERAGE' else '#89959a',lw=1.6 if s['kind']=='COVERAGE' else .7)
for row in rows:
    goal=row['endpoint_world'];body=body_polygon(cfg['vehicle']['footprint'],*goal);ax.plot(*body.exterior.xy,color='#b23f37',alpha=.7,lw=1);ax.scatter(*goal[:2],s=16,color='#b23f37');ax.annotate(str(row['original_segment_index']),goal[:2],xytext=(4,6),textcoords='offset points',fontsize=9)
ax.set(title='Original B1 endpoint / terrain conflict (offline audit only)',xlabel='World X [m]',ylabel='World Y [m]',aspect='equal');ax.legend();ax.grid(alpha=.2);fig.tight_layout();fig.savefig(OUT/'terrain_obligation_audit.png',dpi=140);plt.close(fig)
print(json.dumps(report))
