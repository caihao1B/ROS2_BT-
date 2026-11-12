"""Offline fixed-origin scoring. Truth is never imported by any online estimator."""
import argparse,sys,gzip
from support import *
def rows(p):
    p=Path(p)
    stream=p.open() if p.exists() else gzip.open(str(p)+'.gz','rt')
    with stream:return [json.loads(s) for s in stream if s.strip()]
def trajectory_score(samples,truth,start=5,end=None):
    rr=[r for r in samples if start<=r['t'] and (end is None or r['t']<=end)];tt=np.array([r['t'] for r in truth]);xyz=np.array([r['xyz'] for r in truth]);rp=np.array([r['rpy'] for r in truth]);rp[:,2]=np.unwrap(rp[:,2])
    # One fixed physical origin per run, determined before motion. No fitted scale/yaw or per-segment alignment.
    origin=np.array([np.interp(5.,tt,xyz[:,i]) for i in range(3)]);r0=Rotation.from_euler('xyz',[0,0,float(np.interp(5.,tt,rp[:,2]))]);t=np.array([r['t'] for r in rr]);p=np.array([r['xyz'] for r in rr]);ref=np.array([np.interp(t,tt,xyz[:,i]) for i in range(3)]).T;ref=r0.inv().apply(ref-origin)
    truth_rpy=np.array([np.interp(t,tt,rp[:,i]) for i in range(3)]).T;truth_rpy[:,2]-=float(np.interp(5.,tt,rp[:,2]));angles=np.array([r['rpy'] for r in rr]);error=np.linalg.norm(p-ref,axis=1);rpe=[]
    for i in range(0,len(t),max(1,len(t)//500)):
        j=int(np.searchsorted(t,t[i]+1.))
        if j<len(t) and abs(t[j]-t[i]-1)<.1:rpe.append(float(np.linalg.norm((p[j]-p[i])-(ref[j]-ref[i]))))
    steps=np.linalg.norm(np.diff(p,axis=0),axis=1);dts=np.diff(t);steps=steps[dts<.1]
    result=dict(n=len(t),ate_rmse_m=float(np.sqrt(np.mean(error**2))),ate_max_m=float(error.max()),endpoint_error_m=float(error[-1]),rpe_1s_rmse_m=float(np.sqrt(np.mean(np.array(rpe)**2))) if rpe else None,step_max_m=float(steps.max()) if len(steps) else 0,pitch_rmse_rad=float(np.sqrt(np.mean((angles[:,1]-truth_rpy[:,1])**2))),estimated_pitch_max_rad=float(np.max(np.abs(angles[:,1]))),truth_pitch_max_rad=float(np.max(np.abs(truth_rpy[:,1]))))
    plot=dict(t=t[::3].tolist(),estimate=p[::3].tolist(),truth=ref[::3].tolist(),error=error[::3].tolist(),rpy=angles[::3].tolist(),truth_rpy=truth_rpy[::3].tolist());return result,plot
def analyze(out):
    out=Path(out);rt=read(out/'runtime.json');end=read(out/'online/result.json');obs=rows(out/'online/observations.jsonl');truth=rows(out/'evaluation/truth.jsonl');quality=rows(out/'icp_gate/quality.jsonl');inputs=rows(out/'inputs/inputs.jsonl');motion=rows(out/'motion/commands.jsonl');checks={};m=dict(vehicle=rt['vehicle'],case=rt['case'],checks=checks,trajectories={},faults={});plots={};cfg=policy()
    checks['completed']=rt['success'] and end['completed'];checks['monotonic_stamps']=not end['errors'];checks['sole_tf_owner']=all(r['publishers']==['s05_local_odometry'] for r in end['tf_audits'] if r['t']>5);checks['correct_tf_tree']=set(end['edges'])=={'odom->base_link','base_link->lidar_link','base_link->imu_link','base_link->gnss_link','base_link->left_optical','base_link->right_optical'}
    checks['truth_isolation']=not any('/evaluation/' in t or ('/raw/' in t and t not in ['/s05/raw/fused','/s05/raw/wheel_imu']) or t.startswith('/world/') for t in end['graph']);checks['pure_icp']=not any('s05_icp' in nodes for topic,nodes in end['subscriptions'].items() if any(x in topic for x in ['imu','wheel','odometry']))
    checks['sole_actuator_owner']=end['publishers'].get('/s03/actuator/request')==['s03_drive_bridge'];checks['no_global_feedback']=not any(k.startswith(('map','gnss')) for k in yaml.safe_load((out/'scene/fused.yaml').read_text())['/**']['ros__parameters'] if k.endswith('0'))
    if rt['case']=='reset':
        epochs=[r for r in obs if r['key']=='epoch'];rewinds=[r for r in obs if r['key']=='clock_rewind']
        checks['new_epoch_on_clock_rewind']=bool(epochs and rewinds) and end['epoch']>=1 and read(out/'inputs/result.json')['epoch']>=1
        acks=[r for r in inputs if r.get('key')=='reset_ack'];checks['all_estimators_reset']=len({r['service'] for r in acks if r['success']})==3
        checks['reset_service_succeeded']=rt.get('clock_reset',{}).get('returncode')==0 and 'true' in rt.get('clock_reset',{}).get('response','')
        checks['new_epoch_initial_stop']=any(r['key']=='health' and r['epoch']>=1 and r['state']=='STOP' and r['reason']=='INITIALIZING_NEW_EPOCH' for r in obs)
        checks['new_epoch_recovers']=any(r['key']=='health' and r['epoch']>=1 and r['state']=='OK' and r['t']>9 for r in obs)
        last_epoch=max(r.get('epoch',0) for r in obs);obs=[r for r in obs if r.get('epoch',last_epoch)==last_epoch]
        truth=[r for r in truth if r['epoch']==max(x['epoch'] for x in truth)]
    projection=[r for r in obs if r['key']=='planning_projection'];checks['planning_projection']=bool(projection) and all(abs(r['xyz'][2])<1e-12 and max(abs(v) for v in r['rpy'][:2])<1e-12 for r in projection)
    odoms=[r for r in obs if r['key'] in ['wheel','wheel_imu','fused','icp','raw_fused']];checks['covariance_finite_psd']=bool(odoms) and all(np.isfinite(r['pose_cov']+r['twist_cov']).all() and r['cov_min']>=-1e-8 and min(r['pose_cov'][i] for i in [0,7,14,21,28,35])>0 for r in odoms)
    checks['innovation_observable']=sum(r.get('key')=='wheel' and r.get('innovation_v') is not None for r in inputs)>30
    for key in ['wheel','wheel_imu','fused','icp']:
        rs=[r for r in obs if r['key']==key]
        if len(rs)<20:checks['three_trajectories']=False;continue
        m['trajectories'][key],plots[key]=trajectory_score(rs,truth,5,rt['seconds']-3)
    checks.setdefault('three_trajectories',len(m['trajectories'])==4);checks['odom_continuous']=m['trajectories'].get('fused',{}).get('step_max_m',999)<cfg['qualification']['continuity_step_m']
    xyz=np.array([r['xyz'] for r in truth]);m['physical_distance_m']=float(np.linalg.norm(np.diff(xyz,axis=0),axis=1).sum());checks['physical_motion']=m['physical_distance_m']>(1 if rt['case']=='reset' else 5)
    contacts=read(out/'evaluation/contacts.json');m['obstacle_contacts']={k:v for k,v in contacts.items() if 'vehicle::' in k and not ('ground::' in k or 'grade_' in k)};checks['no_obstacle_contact']=not m['obstacle_contacts']
    imu=[r for r in inputs if r.get('key')=='imu' and 5<r['t']<8];m['stationary_gravity_residual_mps2']=float(np.median([np.linalg.norm(r['gravity_free']) for r in imu]));checks['gravity_compensation']=m['stationary_gravity_residual_mps2']<.15
    valid=[r for r in quality if r['valid'] and 9<r['t']<60];m['icp_processing_p95_s']=float(np.quantile([r['processing_s'] for r in valid],.95)) if valid else None;m['icp_inlier_median']=float(np.median([r['inlier_ratio'] for r in valid])) if valid else 0
    fused=[r for r in obs if r['key']=='fused' and r['t']>5];m['rtf']=(fused[-1]['t']-fused[0]['t'])/(fused[-1]['wall']-fused[0]['wall']);res=read(out/'resources.json');m['peak_owned_rss_bytes']=max(x['owned_rss'] for x in res);m['owned_cpu_p95_percent']=float(np.quantile([x['owned_cpu_one_core_percent'] for x in res],.95));m['case_disk_bytes']=sum(p.stat().st_size for p in out.rglob('*') if p.is_file());checks['resource_bounds']=m['peak_owned_rss_bytes']<cfg['limits']['owned_rss_bytes'] and m['case_disk_bytes']<cfg['limits']['case_disk_bytes']
    if rt['case']=='reset':checks['post_reset_accuracy']=m['trajectories']['fused']['ate_rmse_m']<cfg['qualification']['nominal_ate_rmse_m']
    if rt['case']=='loop':
        checks['nominal_accuracy']=m['trajectories']['fused']['ate_rmse_m']<cfg['qualification']['nominal_ate_rmse_m'];checks['completed_loop']=read(out/'motion/result.json')['goal']
    if rt['case']=='faults':
        for key in ['wheel','wheel_imu','fused']:
            m['faults'][key+'_slip_score'],_=trajectory_score([r for r in obs if r['key']==key],truth,20,50)
        ratio=m['faults']['fused_slip_score']['ate_rmse_m']/max(.001,m['faults']['wheel_slip_score']['ate_rmse_m']);m['faults']['fused_to_wheel_ate_ratio']=ratio;checks['fusion_improves_slip']=ratio<=cfg['qualification']['slip_fused_to_wheel_max_ratio']
        health=[r for r in obs if r['key']=='health'];degraded=[r for r in health if 65<=r['t']<70 and r['state']=='DEGRADED'];icpstop=[r for r in health if 65<=r['t']<79 and r['state']=='STOP' and r['reason']=='ICP_DEGENERATE_OR_STALE'];imustop=[r for r in health if 86<=r['t']<94 and r['state']=='STOP' and r['reason']=='IMU_STALE'];checks['icp_degradation_interface']=bool(degraded and icpstop);checks['imu_loss_interface']=bool(imustop);m['faults']['icp_stop_t']=icpstop[0]['t'] if icpstop else None;m['faults']['imu_stop_t']=imustop[0]['t'] if imustop else None
        checks['fault_changes_observations']=any(not r['valid'] for r in quality if 66<r['t']<78) and any(r['slip_suspected'] for r in inputs if r.get('key')=='wheel' and 22<r['t']<49)
        # Reuse the already-qualified physical stop thresholds, not looser new thresholds.
        import importlib.util
        spec=importlib.util.spec_from_file_location('s03_score',BASE/'ws/src/chassis_bridge/scripts/analyze.py');mod=importlib.util.module_from_spec(spec);sys.path.insert(0,str(BASE/'ws/src/chassis_bridge/scripts'));spec.loader.exec_module(mod);physical=mod.physical(truth)
        for name,rs in [('icp',icpstop),('imu',imustop)]:
            score=mod.stop_score(physical,rs[0]['t'],rs[0]['t']+3) if rs else dict(passed=False);m['faults'][name+'_physical_stop']=score;checks[name+'_physical_stop']=score['passed']
    if rt['case']=='imu_loss':
        health=[r for r in obs if r['key']=='health'];stops=[r for r in health if 20<=r['t']<28 and r['reason']=='IMU_STALE' and r['state']=='STOP']
        checks['independent_imu_stop_interface']=bool(stops)
        import importlib.util
        spec=importlib.util.spec_from_file_location('s03_score',BASE/'ws/src/chassis_bridge/scripts/analyze.py');mod=importlib.util.module_from_spec(spec);sys.path.insert(0,str(BASE/'ws/src/chassis_bridge/scripts'));spec.loader.exec_module(mod);physical=mod.physical(truth)
        score=mod.stop_score(physical,stops[0]['t'],stops[0]['t']+3) if stops else dict(passed=False);m['independent_imu_stop']=score;checks['independent_imu_physical_stop']=score['passed']
        checks['moving_before_imu_loss']=any(r['mode']=='DRIVE' and r['v']>.15 for r in motion if 19<r['t']<20)
        # Actual body displacement before the injected outage, independent of commands.
        before=[r['xyz'] for r in truth if 19<=r['t']<20];checks['physical_moving_before_imu_loss']=bool(len(before)>2 and np.linalg.norm(np.array(before[-1])-before[0])>.15)
    if rt['case']=='slope':
        f=m['trajectories']['fused'];checks['measured_3d_slope']=f['truth_pitch_max_rad']>math.radians(3) and f['estimated_pitch_max_rad']>math.radians(2) and f['pitch_rmse_rad']<cfg['qualification']['slope_pitch_rmse_rad'];checks['slope_progress']=m['physical_distance_m']>7
    m['passed']=all(checks.values());dump(out/'metrics.json',m);dump(out/'plot_data.json',plots);return m
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('out',type=Path);a=p.parse_args();m=analyze(a.out);print(json.dumps(m,indent=2));raise SystemExit(0 if m['passed'] else 1)
