#!/usr/bin/env python3
import json,math,pathlib,sys
import numpy as np
P=pathlib.Path(__file__).resolve().parents[1];C=json.loads((P/'generated/contract.json').read_text());A=pathlib.Path(sys.argv[1])
rows=[json.loads(s) for s in (A/'samples.jsonl').read_text().splitlines()];run=json.loads((A/'execution.json').read_text())
active=[r for r in rows if r['phase'] not in ('startup','cleanup','settle')];truth=[r for r in rows if r['kind']=='truth' and r['phase']!='cleanup']
truth.sort(key=lambda r:r['t']);tt=np.array([r['t'] for r in truth]);th=np.unwrap([r['yaw'] for r in truth])
def pose(t):
    return float(np.interp(t,tt,[r['x'] for r in truth])),float(np.interp(t,tt,[r['y'] for r in truth])),float(np.interp(t,tt,th))
def expected(scan,key):
    x,y,h=pose(scan['t']);sx,sy,_=C['sensor_poses']['lidar_'+key+'_link']['xyz'];ox=x+math.cos(h)*sx-math.sin(h)*sy;oy=y+math.sin(h)*sx+math.cos(h)*sy
    a=scan['angle_min']+np.arange(len(scan['ranges']))*scan['angle_increment']+h;dx=np.cos(a);dy=np.sin(a)
    with np.errstate(divide='ignore',invalid='ignore'):
        tx=np.where(dx>=0,(6-ox)/dx,(-6-ox)/dx);ty=np.where(dy>=0,(6-oy)/dy,(-6-oy)/dy)
    return np.minimum(tx,ty),a,ox,oy
def array(scan):return np.array([x if x is not None else np.nan for x in scan['ranges']])
rates={}
for name,selection in [(k,[r for r in active if r['kind']==k]) for k in ['imu','wheel','fused','gnss']]+[(k,[r for r in active if r['kind']=='scan' and r['topic']==k]) for k in ['front_raw','front_filtered','rear_raw','rear_filtered']]:
    if len(selection)<2:continue
    ts=np.array([r['t'] for r in selection]);ws=np.array([r['wall'] for r in selection]);positive=np.diff(ts);positive=positive[positive>1e-9]
    rates[name]={'messages':len(selection),'hz_sim':float((len(np.unique(ts))-1)/(ts[-1]-ts[0])),'hz_wall':float((len(selection)-1)/(ws[-1]-ws[0])),'median_period_sim_s':float(np.median(positive)),'max_period_sim_s':float(np.max(positive))}
phases={}
for phase in ['static','straight','left','right']:
    ts=[r for r in truth if r['phase']==phase]
    if len(ts)>2:phases[phase]={'rtf':(ts[-1]['t']-ts[0]['t'])/(ts[-1]['wall']-ts[0]['wall']),'sim_duration_s':ts[-1]['t']-ts[0]['t']}
scan_results={};sample_views={}
for key in ['front','rear'] if run['dual'] else ['front']:
    raw=[r for r in active if r['kind']=='scan' and r['topic']==key+'_raw'];filtered=[r for r in active if r['kind']=='scan' and r['topic']==key+'_filtered'];fmap={round(r['t'],6):r for r in filtered}
    errors={'static':[],'moving':[]};self_total=0;self_removed=0;wall_total=0;wall_removed=0;residual=0;matched=0
    for r in raw:
        f=fmap.get(round(r['t'],6))
        if not f:continue
        matched+=1
        if run['trajectory']:continue
        exp,a,ox,oy=expected(r,key);rv=array(r);fv=array(f);valid=np.isfinite(rv)&(rv>=r['range_min'])&(rv<=r['range_max'])
        selfhit=valid&(rv<exp-.10);wall=valid&(np.abs(rv-exp)<=.10)
        self_total+=int(selfhit.sum());self_removed+=int((selfhit&~np.isfinite(fv)).sum());wall_total+=int(wall.sum());wall_removed+=int((wall&~np.isfinite(fv)).sum())
        residual+=int((selfhit&np.isfinite(fv)).sum())
        group='static' if r['phase']=='static' else 'moving'
        # Check every retained finite return; do not preselect accurate wall hits.
        if r['phase'] in ('static','straight','left','right'):
            retained=np.isfinite(fv);errors[group].extend(np.abs(fv[retained]-exp[retained]).tolist())
        if r['phase']=='static' and key not in sample_views:
            sample_views[key]={'scan':r,'filtered':f,'expected':exp.tolist(),'self_mask':selfhit.tolist(),'world_origin':[ox,oy],'world_angles':a.tolist()}
    data={'matched_scans':matched,'raw_scans':len(raw),'filtered_scans':len(filtered),'self_returns':self_total,'self_removed':self_removed,'residual_self_returns':residual,'known_wall_returns':wall_total,'wall_returns_removed':wall_removed,
       'self_removal_fraction':self_removed/self_total if self_total else None,'wall_retention_fraction':1-wall_removed/wall_total if wall_total else None,
       'raw_scan_time_values':sorted({r['scan_time'] for r in raw}),'raw_time_increment_values':sorted({r['time_increment'] for r in raw}),'filtered_scan_time_median':float(np.median([r['scan_time'] for r in filtered])) if filtered else None,
       'frame_ids':sorted({r['frame'] for r in raw+filtered})}
    for name,values in errors.items():
        data[name+'_range_error']={'samples':len(values),'median_m':float(np.median(values)),'p95_m':float(np.percentile(values,95)),'max_m':float(np.max(values))} if values else None
    checks={'filtered_delivery':matched>=.9*len(raw) and matched>=20,'preserved_header_frame':data['frame_ids']==['lidar_'+key+'_link'],
      'instantaneous_scan_contract':data['raw_time_increment_values']==[0.0],'scan_period':data['filtered_scan_time_median'] is not None and abs(data['filtered_scan_time_median']-.2)<.01,
      'frequency':key+'_raw' in rates and 4.7<=rates[key+'_raw']['hz_sim']<=5.3}
    if not run['trajectory']:
        checks.update(self_filter=self_total>100 and self_removed/self_total>=.99,wall_preserved=wall_total>100 and wall_removed/wall_total<=.01,
           static_accuracy=bool(errors['static']) and np.percentile(errors['static'],95)<=.05,
           moving_accuracy=bool(errors['moving']) and np.percentile(errors['moving'],95)<=.05,
           no_large_retained_outlier=all(not v or max(v)<=.15 for v in errors.values()))
    data['checks']=checks;data['status']='PASS' if all(checks.values()) else 'FAIL';scan_results[key]=data

trajectory={}
for phase in ['static','straight','left','right','cumulative']:
    out={}
    for kind in ('wheel','fused'):
        data=[r for r in active if r['kind']==kind and (r['phase']==phase or phase=='cumulative' and r['phase'] in ('static','straight','stop','left','stop_left','right','stop_right')) and tt[0]<=r['t']<=tt[-1]]
        if len(data)<20:continue
        data=[r for r in data if r['t']>=data[0]['t']+.5]
        t=np.array([r['t'] for r in data]);xy=np.array([[r['x'],r['y']] for r in data]);h=np.unwrap([r['yaw'] for r in data]);gt=np.array([pose(v) for v in t])
        # Align only initial pose of this segment; report drift, not arbitrary odom origin offsets.
        def local(xy,h):
            d=xy-xy[0];cs,ss=math.cos(h[0]),math.sin(h[0]);return np.c_[cs*d[:,0]+ss*d[:,1],-ss*d[:,0]+cs*d[:,1]]
        est=local(xy,h);tr=local(gt[:,:2],gt[:,2]);err=np.linalg.norm(est-tr,axis=1);yawerr=np.abs((h-h[0])-(gt[:,2]-gt[0,2]));dist=float(np.sum(np.linalg.norm(np.diff(gt[:,:2],axis=0),axis=1)))
        out[kind]={'samples':len(data),'distance_m':dist,'end_position_error_m':float(err[-1]),'rmse_position_m':float(np.sqrt(np.mean(err**2))),'end_yaw_error_rad':float(yawerr[-1]),'max_yaw_error_rad':float(yawerr.max())}
    if out:
        checks={'both_estimates':len(out)==2}
        if 'fused' in out:
            f=out['fused'];checks.update(local_position=f['end_position_error_m']<=.15+.05*f['distance_m'],local_yaw=f['end_yaw_error_rad']<=.04)
            if phase=='static':checks['stationary_drift']=f['end_position_error_m']<.02
        out['checks']=checks;out['status']='PASS' if all(checks.values()) else 'FAIL';trajectory[phase]=out
owners=run.get('tf_owners',{})
graph=run.get('publisher_graph',{})
tfcheck={'sole_odom_base_link':len(owners.get('odom->base_link',[]))==1,
         'no_odom_base_footprint':'odom->base_footprint' not in owners,'correct_fixed_footprint':'base_link->base_footprint' in owners,
         'no_map_tf':not any(k.startswith('map->') for k in owners),
         'single_parent_per_child':len({k.split('->')[1] for k in owners})==len(owners),
         'sole_formal_odom_publisher':[p['node'] for p in graph.get('/dino/odometry/filtered',[])]==['dino_local_estimator_io'],
         'expected_tf_publishers':set(p['node'] for p in graph.get('/tf',[]))=={'robot_state_publisher','dino_local_estimator_io'},
         'stamped_lookup':run.get('tf_checks',{}).get('stamped_lookup_pass',0)>=20 and run.get('tf_checks',{}).get('stamped_lookup_fail',0)==0}
checks={'execution':run['status']=='PASS','scan_acceptance':all(r['status']=='PASS' for r in scan_results.values()),'trajectory':all(r['status']=='PASS' for r in trajectory.values()) and len(trajectory)>=3,
        'tf':all(tfcheck.values()),'imu_frequency':28<=rates.get('imu',{}).get('hz_sim',0)<=32,'fused_frequency':28<=rates.get('fused',{}).get('hz_sim',0)<=32}
result={'status':'PASS' if all(checks.values()) else 'FAIL','checks':checks,'tf_checks':tfcheck,'rtf_by_phase':phases,'rates':rates,'scans':scan_results,'trajectories':trajectory,
 'negative_checks':run.get('negative_checks',{}),'rendering':'Ogre2 + LIBGL_ALWAYS_SOFTWARE=1, desktop Xwayland; no rendering switch',
 'timing_interpretation':'time_increment=0 means instantaneous simulated scan, not a rolling scanner. Output scan_time is observed acquisition period; header unchanged.',
 'thresholds':{'range_p95_m':.05,'self_removal_min':.99,'wall_retention_min':.99,'fused_end_position':'.15 m + 5% travelled distance','fused_end_yaw_rad':.04,'static_drift_m':.02}}
encoded=json.dumps(result,indent=2,default=lambda v:v.item() if isinstance(v,np.generic) else str(v))
(A/'acceptance.json').write_text(encoded);(A/'scan_views.json').write_text(json.dumps(sample_views,indent=2));print(encoded);sys.exit(0 if result['status']=='PASS' else 1)
