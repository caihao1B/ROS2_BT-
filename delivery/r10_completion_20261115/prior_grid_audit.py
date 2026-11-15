"""Classify the closed V2 collision neighborhood; check V3 against saved cells.

These are recorded risk cells, including a declared prior. They are not all
sensor detections, and this offline check grants no online motion permission.
"""
from pathlib import Path
import json,math,sys
import numpy as np
from shapely.geometry import shape,Point,Polygon
from shapely import contains_xy
ROOT=Path(__file__).resolve().parents[2];OUT=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT/'optimization_v2/ws/src/task_platform/scripts'))
from footprint_r09 import body_polygon
run=ROOT/'optimization_v2/three_tasks/results/R12/mowing_20261115_043537_296551'
cfg=json.loads((run/'mission.json').read_text());allowed=shape(cfg['allowed_prior']).buffer(-cfg['footprint_margin_m'])
frame=sorted((run/'late_grid_diagnostic').glob('grid_*.json'))[-1]
meta=json.loads(frame.read_text());arrays=np.load(frame.with_suffix('.npz'));loc=meta['localization'];mx,my,yaw=loc['transform'];pose=[loc['pose'][0]+28,loc['pose'][1]+14,loc['pose'][2]]
body=body_polygon(cfg['vehicle']['footprint'],*pose)
grid=meta['risk'];a=arrays['risk'];iy,ix=np.where(a>=100)
x=grid['origin'][0]+(ix+.5)*grid['resolution'];y=grid['origin'][1]+(iy+.5)*grid['resolution']
wx=mx+28+np.cos(yaw)*x-np.sin(yaw)*y;wy=my+14+np.sin(yaw)*x+np.cos(yaw)*y
dist=np.array([body.distance(Point(x,y)) for x,y in zip(wx,wy)]);near=dist<.7
outside=~contains_xy(allowed,wx[near],wy[near]);ps=np.asarray(cfg['clusters'][0]['segments'][7]['poses_world']);closest=ps[np.argmin(np.linalg.norm(ps[:,:2]-pose[:2],axis=1))]
plan=json.loads((ROOT/'optimization_v2/three_tasks/editions/R10/mowing_safe_plan.json').read_text())
if shape(plan['navigation_prior']).symmetric_difference(shape(cfg['allowed_prior'])).area>1e-8:raise SystemExit('Hard prior changed; recorded-prior comparison is inapplicable')
# A local replay only: nominal Nav2 padded body plus full grid half diagonal.
# Unknown terrain and future observations still require the live runtime checks.
fp=cfg['vehicle']['footprint'];padded=[[-.671,-1.485],[2.309,-1.485],[2.309,1.485],[-.671,1.485]]
checked=0;hits=[]
for s in plan['cluster']['segments']:
    for i,p in enumerate(s['poses_world']):
        if not (117<=p[0]<=130 and 43<=p[1]<=53):continue
        poly=body_polygon(padded,*p).buffer(grid['resolution']*math.sqrt(2)/2)
        x0,y0,x1,y1=poly.bounds;subset=(wx>=x0)&(wx<=x1)&(wy>=y0)&(wy<=y1)
        checked+=1
        if np.any(contains_xy(poly,wx[subset],wy[subset])):hits.append(dict(segment=s['id'],pose_index=i,pose=p))
report=dict(run=str(run.relative_to(ROOT)),frame=frame.name,t=meta['t'],nearby_risk_lethal_cells=int(near.sum()),nearby_cells_outside_declared_prior=int(outside.sum()),actual_world_pose=pose,nearest_planned_pose=closest.tolist(),center_tracking_error_m=float(np.linalg.norm(closest[:2]-pose[:2])),heading_tracking_error_rad=abs(math.atan2(math.sin(closest[2]-pose[2]),math.cos(closest[2]-pose[2]))),v2_extra_side_m=.2,nav2_padding_m=.1,grid_half_diagonal_m=grid['resolution']*math.sqrt(2)/2,v3_revision=plan['revision'],v3_hard_prior_unchanged=True,v3_checked_local_poses=checked,v3_saved_hard_cell_collisions=hits,scope='Offline saved-grid replay around the failed connector. Prior-generated cells are distinguished from observations. Unknown space, moving objects and future tracking still need actual runtime checks; no global no-path or full execution claim.')
(OUT/'prior_grid_audit.json').write_text(json.dumps(report,indent=2)+'\n')
import matplotlib;matplotlib.use('Agg')
import matplotlib.pyplot as plt
fig,ax=plt.subplots(figsize=(9,6),constrained_layout=True)
hazard=shape(plan['known_static_exclusion']['geometry']);ax.fill(*hazard.exterior.xy,color='#edb3a0',alpha=.55,label='Unchanged declared hard exclusion (V2/V3)')
ax.plot(*hazard.buffer(.1).exterior.xy,ls='--',color='#a94436',label='Original 0.10 m prior margin')
ax.scatter(wx[near],wy[near],s=15,marker='s',c=np.where(outside,'#bd3434','#684aad'),label='Recorded nearby lethal cells (all prior-excluded)')
ax.plot(*body.exterior.xy,color='#246b89',lw=2,label='Actual estimated physical footprint')
navbody=body_polygon(padded,*pose);ax.plot(*navbody.exterior.xy,color='#246b89',ls='--',label='Nav2 padded footprint')
ax.plot(ps[:,0],ps[:,1],color='#828b93',lw=1.5,label='V2 nominal connection')
for s in plan['cluster']['segments']:
    p=np.asarray(s['poses_world']);ax.plot(p[:,0],p[:,1],color='#288d56',alpha=.6,lw=.9)
ax.plot([],[],color='#288d56',label='V3 route with derived tracking reserve')
ax.set(xlim=(117.5,125),ylim=(46.5,52.5),aspect='equal',xlabel='World X [m]',ylabel='World Y [m]',title=f"V2: {int(outside.sum())}/{int(near.sum())} nearby lethal cells belong to the declared prior")
ax.legend(fontsize=8,loc='upper left');ax.grid(alpha=.2);fig.savefig(OUT/'prior_grid_audit.png',dpi=150);plt.close(fig)
print(json.dumps({k:v for k,v in report.items() if k!='v3_saved_hard_cell_collisions'}));print('saved_grid_collision_count',len(hits))
