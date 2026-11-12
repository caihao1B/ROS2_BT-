"""Offline scoring only: simulator fixtures/truth never enter online perception."""
from common09 import *
from mapping_analysis import rows
from runtime import capture09,s08_inputs
from s07_common import capture_inputs

def analyze(out):
 out=Path(out);rt=read(out/'runtime.json');obs=rows(out/'online/observations.jsonl');end=read(out/'online/result.json');fixtures=read(out/'scene/fixtures.json');truth=rows(out/'evaluation/truth.jsonl');odoms=[r for r in obs if r['key']=='odom'];terrain=[r for r in obs if r['key']=='terrain' and 10<r['t']<34];health=[r for r in obs if r['key']=='terrain_health'];tracks=rows(out/'fusion/tracks.jsonl');safety=[r for r in obs if r['key']=='perception_safety'];spawn=fixtures['spawn'];R0=Rotation.from_euler('z',spawn[2]).as_matrix();origin=np.array([*spawn[:2],0.]);tt=np.array([r['t'] for r in truth]);ot=np.array([r['t'] for r in odoms])
 def frames(t):
  tr=truth[int(np.argmin(abs(tt-t)))];od=odoms[int(np.argmin(abs(ot-t)))]['pose'];return Rotation.from_euler('xyz',tr['rpy']).as_matrix(),np.array(tr['xyz']),Rotation.from_quat(od[3:]).as_matrix(),np.array(od[:3])
 def local(points,t):
  R,p,Ro,po=frames(t);return (((np.asarray(points)-po)@Ro)@R.T+p-origin)@R0
 def truth_to_odom(p,t):
  R,rob,Ro,po=frames(t);return Ro@R.T@(np.array(p)-rob)+po
 allrows=[]
 for r in terrain:
  a=np.array(r['cells']);xy=local(np.column_stack([a[:,:2],np.zeros(len(a))]),r['t']);allrows.append((r['t'],a,xy))
 low={}
 for h,y in [(.08,-1.5),(.15,0),(.30,1.5)]:
  samples=[];seen=0;detected=0
  for t,a,xy in allrows:
   b=a[(abs(xy[:,0]-3.6)<.6)&(abs(xy[:,1]-y)<.4)]
   if len(b)>=4:
    seen+=1;positive=b[b[:,3]>h*.5];detected+=len(positive)>=3
    samples.extend(positive.tolist())
  b=np.array(samples);median=float(np.median(b[:,3])) if len(b) else None;lethal=float(np.mean(b[:,8:10]==100)) if len(b) else 0
  low[str(h)]=dict(visible_frames=seen,detected_frames=detected,points_cells=len(samples),height_median_m=median,both_profile_lethal_fraction=lethal,passed=bool(seen>=5 and detected/seen>=.8 and median is not None and abs(median-h)<.045 and lethal>=.9),invisible_not_counted=True)
 slopes={}
 for name,y,target in [('gentle',3.3,3.),('steep',-3.3,9.)]:
  cells=[]
  for t,a,xy in allrows:
   b=a[(xy[:,0]>2.1)&(xy[:,0]<3.8)&(abs(xy[:,1]-y)<.5)];cells.extend(b.tolist())
  b=np.array(cells);fit=b[(b[:,5]>.0)&(b[:,4]>1)] if len(b) else np.zeros((0,10));slopes[name]=dict(expected_deg=target,cells=len(b),slope_median_deg=float(np.median(fit[:,4])) if len(fit) else None,lethal_diff_fraction=float(np.mean(b[:,8]==100)) if len(b) else 0,lethal_ack_fraction=float(np.mean(b[:,9]==100)) if len(b) else 0)
  slopes[name]['passed']=bool(len(fit)>=20 and abs(slopes[name]['slope_median_deg']-target)<2.5 and (slopes[name]['lethal_diff_fraction']<.5 if name=='gentle' else slopes[name]['lethal_diff_fraction']>.7))
 def grids(prefix):return [dict(np.load(p)) for p in sorted((out/'online').glob(prefix+'_*.npz'))]
 nav={k:grids('nav2_'+k) for k in ['diff','ackermann']};raw=grids('terrain_diff');clear=[]
 removal=next((r['t'] for r in rt['events'] if r['event']=='remove_low_8' and r['code']==0 and 'true' in r['response']),None)
 for d in raw:
  t=float(d['t']);v=d['cost'];yy,xx=np.indices(v.shape);xyz=np.column_stack([d['origin'][0]+(xx.reshape(-1)+.5)*d['resolution'],d['origin'][1]+(yy.reshape(-1)+.5)*d['resolution'],np.zeros(v.size)]);xy=local(xyz,t);mask=(abs(xy[:,0]-3.6)<.6)&(abs(xy[:,1]+1.5)<.4);z=v.reshape(-1)[mask];clear.append(dict(t=t,lethal=int((z==100).sum()),free=int((z==0).sum()),unknown=int((z==-1).sum()),total=len(z)))
 before=[r['lethal'] for r in clear if 20<r['t']<34];after=[r['lethal'] for r in clear if removal is not None and removal+6<r['t']];baseline=float(np.median(before)) if before else 0;residual=float(np.median(after)) if after else None;cleared=next((r['t']-removal for r in clear if removal and r['t']>removal and r['lethal']<max(3,baseline*.2) and r['free']>5),None)
 navstats={k:dict(messages=end['grid_counts'].get('nav2_'+k,0),snapshots=len(g),lethal_max=max([int((d['cost']==100).sum()) for d in g] or [0]),inflated_max=max([int(((d['cost']>0)&(d['cost']<100)).sum()) for d in g] or [0]),free_max=max([int((d['cost']==0).sum()) for d in g] or [0])) for k,g in nav.items()}
 objects=[o for r in tracks for o in r['objects']];schema=bool(objects) and all(all(k in o for k in ['t','position','velocity','size','classification','covariance','id','observation_t']) and len(o['covariance'])==16 and np.all(np.isfinite(o['covariance'])) and np.linalg.eigvalsh(np.array(o['covariance']).reshape(4,4)).min()>-1e-8 for o in objects)
 detections=[r['detector_ms'] for r in tracks if r['camera_valid'] and r['detector_ms']>0];dynamic=[];person_truth=rows(out/'evaluation/dynamic.jsonl');pt=[r for r in person_truth if r['name']=='s09_person'];pts=np.array([r['t'] for r in pt]);visible_hits=0;visible_count=0;person_classes=0;occluded_count=0;unmatched_person=0;sudden={}
 for r in tracks:
  if not len(pt):break
  p=pt[int(np.argmin(abs(pts-r['t'])))];target=truth_to_odom(p['xyz'],r['t']);loc=(np.array(p['xyz'])-origin)@R0;occluded=abs(loc[1]+.16)<.65
  candidates=[o for o in r['objects'] if o['state'] in ['OBSERVED','TENTATIVE'] and np.linalg.norm(np.array(o['position'][:2])-target[:2])<.75];best=min(candidates,key=lambda o:np.linalg.norm(np.array(o['position'][:2])-target[:2]),default=None)
  if 10<r['t']:
   if not occluded:visible_count+=1;visible_hits+=best is not None
   else:occluded_count+=1
  if best and best['classification']=='person':person_classes+=1
  unmatched_person+=sum(o['classification']=='person' and np.linalg.norm(np.array(o['position'][:2])-target[:2])>=.75 for o in r['objects'])
  dynamic.append(dict(t=r['t'],truth=target.tolist(),occlusion_geometric_label=bool(occluded),id=best['id'] if best else None,error_m=float(np.linalg.norm(np.array(best['position'][:2])-target[:2])) if best else None,classification=best['classification'] if best else None,velocity=best['velocity'] if best else None))
 matched=[d for d in dynamic if d['id'] is not None];switches=sum(a['id']!=b['id'] for a,b in zip(matched,matched[1:]) if b['t']-a['t']<.6);connections=sum(b['t']-a['t']<.6 for a,b in zip(matched,matched[1:]));stability=1-switches/connections if connections else 0
 tall=[]
 for t,a,xy in allrows:
  tall.extend(a[(abs(xy[:,0]-7)<.8)&(abs(xy[:,1]+3)<.7)&(a[:,3]>.3)].tolist())
 tall=np.array(tall);dynamic_risk=[]
 if len(pt):
  for d in raw:
   t=float(d['t']);p=pt[int(np.argmin(abs(pts-t)))];target=truth_to_odom(p['xyz'],t);v=d['cost'];yy,xx=np.indices(v.shape);xx=d['origin'][0]+(xx+.5)*d['resolution'];yy=d['origin'][1]+(yy+.5)*d['resolution'];z=v[(abs(xx-target[0])<.5)&(abs(yy-target[1])<.5)];loc=(np.array(p['xyz'])-origin)@R0
   if t>10 and abs(loc[1]+.16)>=.65:dynamic_risk.append(bool(np.any(z==100)))
 checks=dict(runtime_completed=rt['success'] and end['completed'],all_low_detected=all(v['passed'] for v in low.values()),slopes=all(v['passed'] for v in slopes.values()),actual_nav2_layers=all(s['messages']>20 and s['lethal_max']>20 and s['inflated_max']>20 and s['free_max']>100 for s in navstats.values()),authorized_boundary=any(r['boundary_valid'] and r['boundary_cells']>0 for r in safety),structured_objects=schema,temporary_clearing=bool(cleared is not None and cleared<8 and residual is not None and residual<max(3,baseline*.2)),database_unchanged=rt['database_unchanged'],source_versions=read(out/'s09_inputs.json')==capture09() and read(out/'s08_inputs.json')==s08_inputs() and read(out/'capture_inputs.json')==capture_inputs(),truth_isolation=not any('/evaluation/' in topic or '/s04/raw/' in topic or topic.startswith('/world/') for topic in end['subscriptions']),unique_owners=bool(end['audits']) and all(sorted(r['tf'])==['s05_local_odometry','s08_global_pose_manager'] and r['actuator']==['s03_drive_bridge'] for r in end['audits'] if r['t']>10))
 checks['tall_obstacle_lethal']=bool(len(tall)>20 and np.mean(tall[:,8:10]==100)>.9)
 if rt['case'] in ['occluded_pedestrian','camera_loss']:
  checks['dynamic_geometry_lethal']=len(dynamic_risk)>10 and np.mean(dynamic_risk)>.8;checks['visible_person_geometry']=visible_count>20 and visible_hits/visible_count>=.6;checks['person_semantic_measured']=person_classes>=5;checks['occlusion_test']=occluded_count>=5 and stability>=.85;checks['stereo_measured']=sum(r['stereo_available'] for r in tracks)>10
  # Full low-obstacle/slope qualification is in dedicated unoccluded fixtures.
  checks.pop('all_low_detected');checks.pop('slopes')
 if len(pt)>5:
  ptxyz=np.array([p['xyz'][:2] for p in pt]);dt=np.diff(pts);speed=np.linalg.norm(np.diff(ptxyz,axis=0),axis=1)/np.maximum(dt,1e-6);times=(pts[:-1]+pts[1:])/2;stopped=speed[(times>21)&(times<23)];resumed=speed[(times>24.2)&(times<27)]
  sudden=dict(stopped_speed_p95_m_s=float(np.percentile(stopped,95)) if len(stopped) else None,resumed_peak_speed_m_s=float(np.max(resumed)) if len(resumed) else None)
  checks['sudden_motion_truth_measured']=bool(len(stopped)>5 and len(resumed)>5 and np.percentile(stopped,95)<.15 and np.max(resumed)>.5)
 if rt['case']=='camera_loss':
  fault=[r for r in tracks if 26<r['t']<44];geom=[r for r in health if 26<r['t']<44];checks['camera_failure_geometry_continues']=len(fault)>15 and all(not r['camera_valid'] for r in fault) and sum(r['valid'] for r in geom)>20;checks['camera_recovers']=sum(r['camera_valid'] for r in tracks if r['t']>46)>10
 checks={k:bool(v) for k,v in checks.items()}
 good=[r for r in health if r['valid']];resources=read(out/'resources.json');m=dict(case=rt['case'],vehicle=rt['vehicle'],checks=checks,low_obstacles=low,slopes=slopes,nav2=navstats,clearing=dict(removal_t=removal,baseline_lethal_cells=baseline,residual_lethal_median=residual,clear_delay_sim_s=cleared),detector_latency_ms=dict(p50=float(np.median(detections)) if detections else None,p95=float(np.percentile(detections,95)) if detections else None,samples=len(detections)),terrain_latency_ms_p95=float(np.percentile([r['compute_ms'] for r in good],95)) if good else None,dynamic=dict(visible_frames=visible_count,geometry_hits=visible_hits,person_classified_frames=person_classes,occluded_frames=occluded_count,short_gap_id_stability=stability,id_switches=switches,observed_ids=sorted(set(d['id'] for d in matched)),visible_geometry_lethal_fraction=float(np.mean(dynamic_risk)) if dynamic_risk else None,unmatched_fused_person_labels=int(unmatched_person),sudden_motion=sudden),camera_valid_fraction=float(np.mean([r['camera_valid'] for r in tracks])) if tracks else 0,stereo_frames=sum(r['stereo_available'] for r in tracks),wall_seconds=rt['wall_seconds'],peak_rss_bytes=max(r['owned_rss_bytes'] for r in resources),scope='Parked perception qualification; no closed-loop path avoidance claim');m['passed']=all(checks.values());dump(out/'metrics.json',m)
 # Small real replay, no external web assets. Terrain rows remain actual observations.
 dump(out/'plot_data.json',dict(terrain=[r for r in obs if r['key']=='terrain'][::3],objects=tracks[::3],clearing=clear,dynamic=dynamic[::3],health=health[::5]));return m
if __name__=='__main__':
 m=analyze(Path(sys.argv[1]));print(json.dumps(m,ensure_ascii=False,indent=2));raise SystemExit(0 if m['passed'] else 1)
