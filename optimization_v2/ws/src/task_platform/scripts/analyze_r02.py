from common import *
import numpy as np,math
from shapely.geometry import shape,Polygon,Point
from shapely.ops import unary_union
from shapely.affinity import rotate,translate
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

def rows(path):return [json.loads(l) for l in path.read_text().splitlines() if l.strip()]
def fit_circle(xy):
 A=np.c_[2*xy,np.ones(len(xy))];b=(xy**2).sum(axis=1);cx,cy,q=np.linalg.lstsq(A,b,rcond=None)[0];r=math.sqrt(max(0,q+cx*cx+cy*cy));res=np.sqrt(((xy-[cx,cy])**2).sum(axis=1))-r
 return dict(center=[cx,cy],radius_m=r,radial_rmse_m=float(np.sqrt(np.mean(res**2))))
def analyze(out):
 runtime=read(out/'runtime.json');spec=read(out/'spec.json');c=read(out/'scene/vehicle.json');truth=rows(out/'evaluation/truth.jsonl');online=rows(out/'online.jsonl');t=np.array([r['t'] for r in truth]);xyz=np.array([r['xyz'] for r in truth]);yaw=np.unwrap([r['rpy'][2] for r in truth]);result={};cursor=0;errors=[]
 for phase in spec['phases']:
  start=cursor;cursor+=phase['duration'];name=phase['name']
  if phase['mode']!='DRIVE':continue
  ix=(t>start+2)&(t<cursor-.25);xy=xyz[ix,:2];ys=yaw[ix];ts=t[ix]
  if len(xy)<20:errors.append('INSUFFICIENT_SAMPLES '+name);continue
  v=np.linalg.norm(np.diff(xy,axis=0),axis=1)/np.diff(ts);w=np.diff(ys)/np.diff(ts);d=dict(samples=len(xy),speed_mps=float(np.median(v)),yaw_rate_rad_s=float(np.median(w)),travel_m=float(np.linalg.norm(np.diff(xy,axis=0),axis=1).sum()),yaw_change_rad=float(ys[-1]-ys[0]))
  if name in ['left','right']:
   d.update(fit_circle(xy));d['expected_radius_m']=abs(phase['v']/phase['w']);d['relative_radius_error']=abs(d['radius_m']/d['expected_radius_m']-1)
   if d['relative_radius_error']>.25 or d['radial_rmse_m']>.10 or abs(d['yaw_change_rad'])<2:errors.append('TURN_GEOMETRY '+name)
   if np.sign(d['yaw_rate_rad_s'])!=np.sign(phase['w']):errors.append('TURN_DIRECTION '+name)
  elif name in ['straight','reverse']:
   heading=float(ys[0]);delta=xy[-1]-xy[0];d['signed_forward_m']=float(delta@[math.cos(heading),math.sin(heading)])
   if np.sign(d['signed_forward_m'])!=np.sign(phase['v']) or abs(d['speed_mps']/abs(phase['v'])-1)>.15:errors.append('FORWARD_REVERSE '+name)
  elif name=='spin':
   if c['kind']=='ackermann':
    if d['travel_m']>.03 or abs(d['yaw_change_rad'])>.03:errors.append('ACKERMANN_SPIN_NOT_REJECTED')
   elif abs(d['yaw_change_rad'])<.6:errors.append('SKID_SPIN_NOT_WORKING')
  result[name]=d
 last=online[-1]['encoders'];stopped=last['limited_v']==0 and max(abs(v*c['wheel']['radius']) for v in last['velocity'][:2])<.01
 if not stopped:errors.append('NOT_STOPPED')
 if not runtime['passed']:errors.append('RUNTIME_FAILED')
 result=dict(task=spec['task'],passed=not errors,errors=errors,phases=result,stopped=stopped,runtime=runtime,vehicle_hash=c['config_hash'],sim_seconds=float(t[-1]),truth_evaluation_only=True)
 dump(out/'analysis.json',result);return result

def static_contracts():
 worlds=read(BASE/'maps/prior/workspaces.geojson')['features'];road=read(BASE/'maps/prior/connector.geojson')['features'];work=unary_union([shape(x['geometry']) for x in worlds]+[shape(road[0]['geometry'])]);obstacles=unary_union([shape(x['geometry']) for x in read(BASE/'maps/prior/permanent.geojson')['features']]);drive=work.difference(obstacles);path=np.array(road[1]['geometry']['coordinates']);path=path[np.r_[True,np.linalg.norm(np.diff(path,axis=0),axis=1)>1e-8]];delta=np.gradient(path,axis=0);yaw=np.unwrap(np.arctan2(delta[:,1],delta[:,0]));arc=np.r_[0,np.cumsum(np.linalg.norm(np.diff(path,axis=0),axis=1))];k=np.gradient(yaw,arc);result={}
 for task in TASKS:
  c=read(edition('R02')/'vehicles'/f'{task}.json');poly=Polygon(c['footprint']);bad=0;clearance=[]
  for xy,h in zip(path,yaw):
   p=translate(rotate(poly,h,origin=(0,0),use_radians=True),*xy)
   bad+=not drive.covers(p);clearance.append(p.distance(drive.boundary))
  ack=c['kind']=='ackermann';R=c['physical_contract']['nominal_bicycle_radius_m'];outer=max(math.hypot(x,y+R) for x,y in c['footprint']) if ack else max(math.hypot(x,y) for x,y in c['footprint'])
  B=shape(worlds[1]['geometry']).difference(obstacles);room=not B.buffer(-(outer+.25)).is_empty if ack else True
  result[task]=dict(passed=bad==0 and (not ack or max(abs(k))<=c['steering']['max_curvature']) and room,road_samples=len(path),outside_or_obstacle_samples=int(bad),road_min_footprint_clearance_m=min(clearance),road_max_centerline_curvature=float(max(abs(k))),turn_outer_radius_m=outer,minimum_full_turn_diameter_m=2*outer,turn_room_exists_in_zone_B=room,full_strip_plan_accepted=False)
 return result

def figures(results):
 out=APP/'results/R02';out.mkdir(parents=True,exist_ok=True);fig,axs=plt.subplots(1,3,figsize=(14,4.6))
 for ax,task in zip(axs,TASKS):
  c=read(edition('R02')/'vehicles'/f'{task}.json');pts=np.array(c['footprint']+[c['footprint'][0]]);ax.fill(pts[:,0],pts[:,1],color='#5991b2',alpha=.35)
  L=c['wheelbase'];T=c['wheel']['track'];ack=c['kind']=='ackermann'
  for x in ([0,L] if ack else [-L/2,L/2]):
   for y in [-T/2,T/2]:ax.add_patch(plt.Rectangle((x-c['wheel']['radius'],y-c['wheel']['width']/2),2*c['wheel']['radius'],c['wheel']['width'],color='#172935'))
  ax.plot(0,0,'ro');ax.set_aspect('equal');ax.set_xlim(-.65,2.4);ax.set_ylim(-1.6,1.6);ax.grid();ax.set_title(task+' / '+('front Ackermann' if ack else '4-wheel skid'));ax.set_xlabel('x / m');ax.set_ylabel('y / m');ax.text(.01,.02,f"L={L:.3f} m, T={T:.3f} m\nbody envelope {c['physical_contract']['declared_outer_length_m']:.3f} x {c['physical_contract']['declared_outer_width_m']:.3f} m",transform=ax.transAxes,fontsize=9)
 fig.tight_layout();fig.savefig(out/'dimensions.png',dpi=150);plt.close(fig)
 fig,axs=plt.subplots(1,3,figsize=(14,4.6))
 for ax,task in zip(axs,TASKS):
  p=ROOT/read(out/(task+'_latest.json'))['path'];tr=rows(p/'evaluation/truth.jsonl');xy=np.array([x['xyz'][:2] for x in tr]);ax.plot(xy[:,0],xy[:,1],lw=1);ax.plot(*xy[0],'go');ax.plot(*xy[-1],'rx');ax.set_aspect('equal');ax.grid();ax.set_title(task+' / Gazebo measured trajectory');ax.set_xlabel('world x / m')
 fig.tight_layout();fig.savefig(out/'trajectories.png',dpi=150);plt.close(fig)
 return [out/'dimensions.png',out/'trajectories.png']
