"""Offline sensor acceptance. Reads private generation audit only here, never online."""
import json,math,sqlite3
from pathlib import Path
import numpy as np
from common import *
def rows(p):return [json.loads(s) for s in Path(p).read_text().splitlines() if s.strip()]
def rate(rs):
    if len(rs)<3:return 0.
    ts=[r['t'] for r in rs];return (len(ts)-1)/(ts[-1]-ts[0]) if ts[-1]>ts[0] else 0.
def analyze(out):
    out=Path(out);rt=read(out/'runtime.json');end=read(out/'online/result.json');obs=rows(out/'online/observations.jsonl');gen=rows(out/'generation/generation.jsonl')+rows(out/'vision/generation.jsonl');cfg=settings();checks={};metrics=dict(vehicle=rt['vehicle'],profile=rt['profile'],checks=checks,rates={},faults={})
    def observed(k,start=5,end_t=None):return [r for r in obs if r['key']==k and start<=r['t']<(end_t if end_t is not None else rt['seconds']-2)]
    def generated(k,start,end_t):return [r for r in gen if r['sensor']==k and r['event']=='generated' and start<=r['t']<end_t]
    checks['completed']=rt['success'] and end['completed'];checks['five_sensor_classes']=all(end['counts'].get(k,0)>10 for k in ['points','left','right','imu','wheel','gnss'])
    expected={'points':'lidar_link','left':'left_optical','right':'right_optical','imu':'imu_link','wheel':'base_link','gnss':'gnss_link'};checks['frames']=all(end['frames'].get(k)==[v] for k,v in expected.items())
    checks['timestamps_monotonic']=not end['errors'];checks['extrinsic_tree']=set(expected.values())-{'base_link'}<=set(end['tf_static']) and all(v['parent']=='base_link' for v in end['tf_static'].values())
    a=end['calibration']['left_info'];b=end['calibration']['right_info'];fx=cfg['stereo']['width']/(2*math.tan(cfg['stereo']['hfov_rad']/2))
    checks['stereo_calibration']=abs(a['k'][0]-fx)<1e-8 and abs(-b['p'][3]/b['p'][0]-cfg['stereo']['baseline_m'])<1e-9 and a['width']==640 and a['height']==360
    pairs=observed('pair');metrics['stereo_sync_max_s']=max((r['error_s'] for r in pairs),default=1e9);checks['stereo_sync']=bool(pairs) and metrics['stereo_sync_max_s']<=cfg['stereo']['max_pair_error_s']
    clouds=observed('points',0);native=[r for r in gen if r['sensor']=='points' and r['event']=='generated'];checks['native_3d_raster']=bool(native) and all(r['native_shape']==[32,1024] for r in native) and all(r['height']==32 and r['width']==1024 for r in clouds)
    detail=[r for r in clouds if 'z_span' in r];metrics['vertical_bins_max']=max((r['vertical_bins'] for r in detail),default=0);metrics['height_span_m']=max((r['z_span'] for r in detail),default=0);checks['real_height_returns']=metrics['vertical_bins_max']>=16 and metrics['height_span_m']>1
    checks['camera_content']=all(any(r.get('pixel_std',0)>5 for r in observed(k,0)) for k in ['left','right'])
    imu=observed('imu');wheel=observed('wheel');gnss=observed('gnss');checks['covariances']=all(r['orientation_cov_0']==-1 and min(r['gyro_cov'][i] for i in [0,4,8])>0 and min(r['accel_cov'][i] for i in [0,4,8])>0 for r in imu) and all(r['cov'][0]>0 and r['cov'][35]>0 for r in wheel) and all(min(r['cov'][i] for i in [0,4,8])>0 for r in gnss)
    targets={'points':10,'left':15,'right':15,'imu':200,'wheel':100,'gnss':5,'localization_points':10,'perception_points':5}
    for k,target in targets.items():
        rr=observed(k,5,11 if rt['profile']=='sensor_faults' else None);hz=rate(rr);metrics['rates'][k]=dict(sim_hz=hz,target_hz=target,missing_fraction=max(0,1-hz/target),n=len(rr),age_p95_s=float(np.quantile([r['arrival_sim']-r['t'] for r in rr],.95)) if rr else None)
    checks['frequency_targets']=all(abs(v['sim_hz']/v['target_hz']-1)<=.10 for v in metrics['rates'].values())
    gr=generated('imu',5,rt['seconds']-2);bias=np.array([r['bias'] for r in gr]);metrics['gyro_bias_span']=np.ptp(bias,axis=0).tolist();checks['imu_bias_drift']=max(metrics['gyro_bias_span'])>1e-5
    noise=[r['range_noise_std'] for r in native if r['t']>5];metrics['lidar_noise_std_m']=float(np.median(noise));checks['range_noise']=.8*cfg['lidar']['sigma_m']<metrics['lidar_noise_std_m']<1.2*cfg['lidar']['sigma_m']
    truth=rows(out/'evaluation/truth.jsonl');metrics['physical_travel_extent_m']=float(np.ptp(np.array([r['xyz'] for r in truth])[:,0]));checks['moving_rig']=metrics['physical_travel_extent_m']>.8
    commands=rt['commands'];graph=end['graph'];checks['truth_isolation']=not any('/raw/' in t or '/evaluation/' in t or t.startswith('/world/') for t in graph) and all(r['domain']==121 for r in commands if r['role'] in ['raw_bridge','generator','vision_generator']) and all(r['domain']==120 for r in commands if r['role'] in ['evaluation_bridge','evaluator'])
    checks['single_owners']=end['publishers'].get('/tf')==['s03_drive_bridge'] and end['publishers'].get('/tf_static')==['s04_sensor_fanout'] and end['publishers'].get('/s03/actuator/request')==['s03_drive_bridge']
    rss=read(out/'resources.json');metrics['peak_owned_rss_bytes']=max(r['owned_rss'] for r in rss);metrics['host_cpu_percent_mean']=float(np.mean([r['host_cpu_percent'] for r in rss]));metrics['owned_cpu_one_core_percent_p95']=float(np.quantile([r['owned_cpu_one_core_percent'] for r in rss],.95));gpus=[list(map(float,r['gpu'].split(',')[:2])) for r in rss if r['gpu'] and '\n' not in r['gpu']];metrics['gpu_util_percent_p95']=float(np.quantile([x[0] for x in gpus],.95)) if gpus else None;metrics['gpu_memory_mib_peak']=max(x[1] for x in gpus) if gpus else None
    measured=observed('imu',5);metrics['rtf']=(measured[-1]['t']-measured[0]['t'])/(measured[-1]['wall']-measured[0]['wall']);checks['realtime_target']=metrics['rtf']>=.9
    metrics['case_disk_bytes']=sum(p.stat().st_size for p in out.rglob('*') if p.is_file());checks['resource_bounds']=metrics['peak_owned_rss_bytes']<cfg['limits']['owned_rss_bytes'] and metrics['case_disk_bytes']<cfg['limits']['case_disk_bytes']
    checks['bounded_recording']=end['bag_bytes']<=cfg['limits']['record_bytes'] and all(end['bag_counts'].get(k,0)>0 for k in ['points','left','right','imu','wheel','gnss']) and (out/'online/sample_bag/metadata.yaml').exists()
    # Read back the real SQLite/CDR artifact rather than trusting writer counters.
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    bag=out/'online/sample_bag';db=next(bag.glob('*.db3'));recorded={};decodable=True
    with sqlite3.connect('file:'+str(db)+'?mode=ro',uri=True) as conn:
        for topic_id,name,typ in conn.execute('SELECT id,name,type FROM topics'):
            n,lo,hi=conn.execute('SELECT count(*),min(timestamp),max(timestamp) FROM messages WHERE topic_id=?',(topic_id,)).fetchone()
            recorded[name]=dict(count=n,type=typ,first_ns=lo,last_ns=hi)
            if n:
                blob=conn.execute('SELECT data FROM messages WHERE topic_id=? ORDER BY timestamp LIMIT 1',(topic_id,)).fetchone()[0]
                msg=deserialize_message(blob,get_message(typ))
                if name.endswith('/points'):decodable &= msg.height==32 and msg.width==1024 and len(msg.data)==msg.height*msg.row_step
                elif name.endswith('/image_rect'):decodable &= msg.height==360 and msg.width==640 and msg.encoding=='rgb8'
                elif name=='/tf_static':decodable &= len(msg.transforms)==5
    metrics['bag_readback']=recorded;metrics['bag_disk_bytes']=sum(p.stat().st_size for p in bag.iterdir() if p.is_file())
    lo=min(x['first_ns'] for x in recorded.values() if x['count']);hi=max(x['last_ns'] for x in recorded.values() if x['count'])
    checks['bag_readback']=bool(decodable) and all(x['count']>0 for x in recorded.values()) and (hi-lo)*1e-9<=cfg['limits']['record_duration_s']+.01 and metrics['bag_disk_bytes']<=cfg['limits']['record_bytes']
    if rt['profile']=='sensor_faults':
        f=metrics['faults'];f['delay_s']=float(np.median([r['arrival_sim']-r['t'] for r in observed('imu',13,19)]));f['image_hz']=rate(observed('left',23,29))
        slip=generated('wheel',33,39);slip=[r for r in slip if abs(r['shaft_v'])>.1];f['wheel_scale_error']=float(np.median([(r['observed_v']/r['shaft_v']-1) for r in slip]))
        fixed=generated('gnss',5,11);floating=generated('gnss',43,49);falsefix=generated('gnss',63,71);f['fixed_error_rms_m']=float(np.sqrt(np.mean([np.dot(r['error_enu'],r['error_enu']) for r in fixed])));f['float_error_rms_m']=float(np.sqrt(np.mean([np.dot(r['error_enu'],r['error_enu']) for r in floating])));f['false_fixed_error_m']=float(np.median([np.linalg.norm(r['error_enu']) for r in falsefix]));f['dropout_fix_count']=len(observed('gnss',53,59));f['gyro_step_error']=float(np.median([r['observed_z']-r['raw_z'] for r in generated('imu',75,81)]))
        checks['fault_delay']=f['delay_s']>=.15;checks['fault_drop']=4<f['image_hz']<11;checks['fault_slip']=.25<f['wheel_scale_error']<.35;checks['fault_float']=f['float_error_rms_m']>3*f['fixed_error_rms_m'];checks['fault_dropout']=f['dropout_fix_count']==0
        delivered={round(r['t'],6):r for r in observed('gnss',63,71)};checks['fault_false_fixed']=f['false_fixed_error_m']>3 and all(r['reported_mode']=='fixed' for r in falsefix) and len(delivered)>20 and all(np.linalg.norm(np.array(delivered[round(r['t'],6)]['enu'])-r['observed_enu'])<1e-5 for r in falsefix if round(r['t'],6) in delivered) and all(r['cov'][0]<.01 and r['status']==2 for r in delivered.values())
        checks['fault_imu_step']=.03<f['gyro_step_error']<.06
        checks['observable_diagnostics']=any('gnss_STALE' in r['reasons'] for r in end['diagnostics'] if 53<r['t']<59) and any('GNSS_POSITION_JUMP' in r['reasons'] for r in end['diagnostics'] if 62<r['t']<66)
    metrics['passed']=all(checks.values());dump(out/'metrics.json',metrics);return metrics
if __name__=='__main__':
    import argparse
    p=argparse.ArgumentParser();p.add_argument('out',type=Path);a=p.parse_args();m=analyze(a.out);print(json.dumps(m,indent=2));raise SystemExit(0 if m['passed'] else 1)
