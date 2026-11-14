"""Independent exported-route checks; no planner success flag grants acceptance."""
from common import *
import math
import numpy as np
from shapely.geometry import shape,Point,Polygon,LineString
from shapely import union_all
import shapely

def validate(m):
 checks={};facts={};e=edition('R03');src=read(BASE/'maps/prior/workspaces.geojson')['features'];permanent=union_all([shape(f['geometry']) for f in read(BASE/'maps/prior/permanent.geojson')['features']]);full={f['properties']['id']:shape(f['geometry']).difference(permanent) for f in src};road=shape(read(BASE/'maps/prior/connector.geojson')['features'][0]['geometry'])
 checks['source_world_unchanged']=sha(BASE/'assets/worlds/irregular_farm.sdf')==m['map_source']['sha256'];checks['manifest_version']=m['version']==canonical({k:v for k,v in m.items() if k!='version'})
 checks['three_independent_tasks']=set(m['task_plans'])==set(TASKS);checks['not_execution_or_formal_credit']=not m['execution_allowed'] and not m['hidden_truth_used'] and not m['unknown_low_obstacles_used']
 for task,plans in m['task_plans'].items():
  c=read(edition('R02')/'vehicles'/f'{task}.json');allowed=full['A' if task=='spray' else 'B'] if task!='inspection' else union_all([*full.values(),road]);body=np.asarray(c['footprint']);point_ids=[]
  checks[task+'_plan_count']=len(plans)==(1 if task=='inspection' else 3)
  for p in plans:
   name=p['cluster'];ok=dict(version=p['version']==canonical({k:v for k,v in p.items() if k!='version'}),vehicle=p['vehicle_hash']==c['config_hash'],budget=0<p['estimated_seconds']<=1800,connected=bool(p['segments']),finite=True,footprint=True,curvature=True,gear=True,tool=True)
   previous=None;maximum=0.;reverse=0.;count=0
   for s in p['segments']:
    a=np.asarray(s['poses']);count+=len(a);ok['finite'] &= bool(a.ndim==2 and a.shape[1]==3 and len(a)>0 and np.isfinite(a).all())
    if not ok['finite']:continue
    if previous is not None:ok['connected'] &= math.dist(previous[:2],a[0,:2])<1e-5 and abs(math.atan2(math.sin(previous[2]-a[0,2]),math.cos(previous[2]-a[0,2])))<1e-5
    previous=a[-1];co=np.cos(a[:,2]);si=np.sin(a[:,2]);xy=np.stack([a[:,0,None]+co[:,None]*body[:,0]-si[:,None]*body[:,1],a[:,1,None]+si[:,None]*body[:,0]+co[:,None]*body[:,1]],axis=2)
    # Independent 9 cm envelope (planner uses 10 cm); checks all samples.
    ok['footprint'] &= bool(np.all(shapely.covers(allowed.buffer(-.09),shapely.polygons(xy))))
    ds=np.linalg.norm(np.diff(a[:,:2],axis=0),axis=1);yaw=np.arctan2(np.sin(np.diff(a[:,2])),np.cos(np.diff(a[:,2])))
    if c['kind']=='ackermann':
     ok['curvature'] &= bool(np.all(np.abs(yaw[ds<1e-8])<1e-8));k=np.abs(yaw[ds>1e-8])/ds[ds>1e-8];maximum=max(maximum,float(max(k,default=0)))
     ok['curvature'] &= maximum<=c['steering']['max_curvature']+1e-4
     direction=s.get('direction',1);projections=(np.diff(a[:,:2],axis=0)*np.c_[np.cos(a[:-1,2]),np.sin(a[:-1,2])]).sum(axis=1)
     ok['gear'] &= direction in [-1,1] and bool(np.all(direction*projections>=-1e-8))
     if direction<0:reverse+=s['length_m'];ok['gear'] &= s.get('stop_before',False) and s.get('stop_after',False)
     if s['kind']=='TURN_TOOL_OFF':ok['tool'] &= not s['tool_on']
    if s['kind']=='POINT_SERVICE':point_ids.append(s['point_id']);ok['tool'] &= s['tool_on'] and math.dist(a[0,:2],s['target'])<1e-8
   ok['endpoint']=previous is not None and math.dist(previous[:2],p['end'][:2])<1e-6
   if task=='mowing':ok['all_swaths']=p['all_swaths_connected'] and not p['unconnected_swath_ids'];ok['planned_cut_accounting']=0<=p['planned_fraction']<=1 and abs(p['planned_cut_m2']+shape(p['planned_residual']).area-p['work_area_m2'])<1e-5
   checks[name]=bool(all(ok.values()));facts[name]=dict(checks={k:bool(v) for k,v in ok.items()},samples=count,max_curvature=maximum,reverse_m=reverse,estimate_s=p['estimated_seconds'],motion_m=p['motion_m'],planned_fraction=p.get('planned_fraction'),point_count=p.get('point_count'))
  if task=='spray':checks['point_services_exactly_once']=len(point_ids)==len(set(point_ids))==sum(len(p['service_points']) for p in plans) and set(point_ids)=={x['id'] for p in plans for x in p['service_points']}
 for zone in ['A','B']:
  partition=read(e/zone/'partition/partition.json');regions=[shape(f['geometry']) for f in partition['clusters']['features']];domain=union_all(regions);excluded=shape(m['scope_changes'][zone]['geometry']);good=len(regions)==3 and all(p.geom_type=='Polygon' and p.is_valid and p.area>0 for p in regions)
  good &= sum(regions[i].intersection(regions[j]).area for i in range(3) for j in range(i))<1e-6
  good &= domain.union(excluded).symmetric_difference(full[zone]).area<1e-5 and domain.intersection(excluded).area<1e-5
  edges=[(i,j) for i in range(3) for j in range(i) if regions[i].boundary.intersection(regions[j].boundary).length>.01];seen={0}
  for _ in range(3):seen|={j if i in seen else i for i,j in edges if i in seen or j in seen}
  checks[zone+'_three_connected_partition_and_scope']=bool(good and len(seen)==3 and partition['verification']['passed'])
 return dict(passed=all(checks.values()),checks=checks,plans=facts,total_estimated_sim_seconds=sum(p['estimated_seconds'] for ps in m['task_plans'].values() for p in ps),scope='Offline feasibility of independent tasks; dynamic travel time and closed-loop execution pending R04–R10')

def figures(m,result):
 import matplotlib;matplotlib.use('Agg');import matplotlib.pyplot as plt
 out=APP/'results/R03';out.mkdir(parents=True,exist_ok=True)
 fig,ax=plt.subplots(figsize=(14,7));colors=['#2488ab','#f0a844','#65a557'];src=read(BASE/'maps/prior/workspaces.geojson')['features']
 def paint(poly,**kw):
  for p in getattr(poly,'geoms',[poly]):
   if p.geom_type=='Polygon':ax.fill(*p.exterior.xy,**kw)
 for f in src:paint(shape(f['geometry']),color='#d9dce1',label='Original zone / headland' if f==src[0] else None)
 paint(shape(read(BASE/'maps/prior/connector.geojson')['features'][0]['geometry']),color='#d9dce1')
 for zone in ['A','B']:
  fs=read(edition('R03')/zone/'partition/clusters.geojson')['features']
  for k,f in enumerate(fs):
   p=shape(f['geometry']);paint(p,color=colors[k],alpha=.5);pos=p.representative_point();ax.text(pos.x,pos.y,zone+str(k+1),fontweight='bold',fontsize=15)
 for task,plans in m['task_plans'].items():
  for p in plans:
   for s in p['segments']:
    a=np.array(s['poses']);ax.plot(a[:,0],a[:,1],color='#173a41' if s['tool_on'] or task=='inspection' else '#9b5297',lw=.8,alpha=.8 if s['tool_on'] else .5)
   ax.scatter(*p['start'][:2],s=35,c='green',marker='o',zorder=10);ax.scatter(*p['end'][:2],s=35,c='red',marker='x',zorder=10)
 ax.set(aspect='equal',xlabel='Prior ENU x [m]',ylabel='Prior ENU y [m]',title='R03 independent tasks | dark: work / purple: tool OFF / green: start / red: end');ax.grid(alpha=.2);fig.tight_layout();p1=out/'task_map.png';fig.savefig(p1,dpi=170);plt.close(fig)
 fig,axs=plt.subplots(1,2,figsize=(12,4));items=[p for ps in m['task_plans'].values() for p in ps];labels=[p['cluster'] for p in items];axs[0].bar(labels,[p['estimated_seconds']/60 for p in items]);axs[0].axhline(30,color='red',ls='--');axs[0].set(title='Conservative planned duration (not actual)',ylabel='Simulation minutes');work=np.array([p.get('work_m',0) for p in items]);total=np.array([p['motion_m'] for p in items]);axs[1].bar(labels,work,label='Work distance');axs[1].bar(labels,total-work,bottom=work,label='Tool OFF / inspection');axs[1].legend();axs[1].set(ylabel='Metres',title='Travel distance accounting');fig.tight_layout();p2=out/'workload.png';fig.savefig(p2,dpi=170);plt.close(fig);return [p1,p2]
