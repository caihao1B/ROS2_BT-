from pathlib import Path
import json,sys
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT/'optimization_v2/ws/src/task_platform/scripts'))
from common import read,edition,APP,dump
from safe_plan_r10 import load_checked,validate
from shapely.geometry import shape,Polygon
from shapely import union_all
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
original=read(edition('R09')/'mission.json');plan=load_checked();new=plan['cluster'];old=original['clusters'][0]
fig,axes=plt.subplots(1,2,figsize=(13,6),constrained_layout=True)
for ax,route,title in zip(axes,[old,new],['Original reference: 34 segments (PARTIAL)',f"Approved safe edition: {len(new['segments'])} segments (plan geometry only)"]):
 area=shape(old['area_map']);area=__import__('shapely.affinity',fromlist=['translate']).translate(area,28,14)
 for poly in getattr(area,'geoms',[area]):ax.fill(*poly.exterior.xy,color='#e3efd9',label='Unchanged work area')
 hazard=shape(plan['known_static_exclusion']['geometry']);ax.fill(*hazard.exterior.xy,color='#e79380',alpha=.8,label='Known 10-degree ramp exclusion')
 for kind,color,label in [('COVERAGE','#168b4a','Work fragment'),('TURN_TOOL_OFF','#5078c4','Tool-off connection')]:
  first=True
  for s in route['segments']:
   if s['kind']!=kind:continue
   p=s['poses_world'];ax.plot([x[0] for x in p],[x[1] for x in p],color=color,lw=1.2,label=label if first else None);first=False
 ax.set(title=title,xlabel='World X (m)',ylabel='World Y (m)');ax.set_aspect('equal');ax.grid(alpha=.2);ax.legend(fontsize=8,loc='upper left');allposes=[p for r in [old,new] for s in r['segments'] for p in s['poses_world']];ax.set_xlim(min(p[0] for p in allposes)-2,max(p[0] for p in allposes)+2);ax.set_ylim(min(p[1] for p in allposes)-2,max(p[1] for p in allposes)+2)
fig.suptitle('B1: unchanged map, task area and 5-degree terrain gate\nNew offline prior is explicit; only measured tool feedback earns work credit',fontsize=12)
out=APP/'results/R10';fig.savefig(out/'safe_plan_comparison.png',dpi=160);dump(out/'safe_plan_validation.json',validate(plan,original))
print(out/'safe_plan_comparison.png')
