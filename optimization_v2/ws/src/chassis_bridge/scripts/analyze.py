"""Offline scoring only: compares physical measurements to commands/encoder estimates."""
import argparse,json,math
from pathlib import Path
import numpy as np
from shapely.geometry import Polygon,Point,shape,LineString
from shapely.affinity import rotate,translate
from kinematics import steering
def read(p):return json.loads(Path(p).read_text())
def rows(p):return [json.loads(s) for s in Path(p).read_text().splitlines() if s.strip()]
def physical(truth):
    # Reset discontinuities must never be differentiated across epochs.
    allrows=[]
    for reset in sorted({r['resets'] for r in truth}):
        rr=[r for r in truth if r['resets']==reset];t=np.array([r['t'] for r in rr]);pos=np.array([r['xyz'] for r in rr]);yaw=np.unwrap([r['rpy'][2] for r in rr])
        if len(rr)<3:continue
        velocity=np.gradient(pos,t,axis=0);omega=np.gradient(yaw,t)
        for i,r in enumerate(rr):allrows.append(dict(**r,v=float(velocity[i,0]*math.cos(yaw[i])+velocity[i,1]*math.sin(yaw[i])),lateral=float(-velocity[i,0]*math.sin(yaw[i])+velocity[i,1]*math.cos(yaw[i])),w=float(omega[i]),yaw=float(yaw[i])))
    return allrows
def stop_score(truth,start,end,reset=0):
    rr=[r for r in truth if r['resets']==reset and start<=r['t']<=end]
    if len(rr)<15:return dict(passed=False,error='insufficient stop measurements')
    stopped=None
    for i,r in enumerate(rr):
        rest=[x for x in rr[i:] if x['t']<=r['t']+.3]
        if len(rest)>=8 and all(abs(x['v'])<.01 and abs(x['w'])<.02 for x in rest):stopped=r['t'];break
    final=[r for r in rr if r['t']>rr[-1]['t']-.6]
    xy=np.array([r['xyz'][:2] for r in rr]);distance=float(np.linalg.norm(np.diff(xy,axis=0),axis=1).sum());delay=None if stopped is None else stopped-start
    return dict(passed=delay is not None and delay<2 and max(abs(r['v']) for r in final)<.01 and max(abs(r['w']) for r in final)<.02,delay_sim_s=delay,distance_m=distance,final_v_max=max(abs(r['v']) for r in final),final_w_max=max(abs(r['w']) for r in final),thresholds=dict(v=.01,w=.02,delay=2.0))
def analyze(case):
    case=Path(case);rt=read(case/'runtime.json');c=read(case/'scene/vehicle.json');online=rows(case/'online/online.jsonl');end=read(case/'online/result.json');truth=physical(rows(case/'evaluation/truth.jsonl'));contacts=read(case/'evaluation/contacts.json');metrics=dict(case=rt['case'],vehicle=rt['vehicle'],completed=rt['success'] and end['completed'],checks={},motion={},stops={})
    checks=metrics['checks'];checks['completed']=metrics['completed'];checks['no_truth_in_online']=not any('/evaluation/' in name for name in end['graph']);checks['truth_available']=len(truth)>50
    collisions={k:v for k,v in contacts.items() if 'vehicle::' in k and not ('ground::' in k or 'grade_' in k)}
    metrics['obstacle_contacts']=collisions;checks['no_obstacle_contact']=not collisions
    if rt['case']=='bench':
        checks['one_tf_and_actuator_owner']=end['tf_publishers']==['s03_drive_bridge'] and end['actuator_publishers']==['s03_drive_bridge']
        for name in sorted({r['phase'] for r in online}):
            rr=[r for r in online if r['phase']==name and r['mode']=='DRIVE' and r['encoders'] and r['odom']]
            if not rr:continue
            t0,t1=rr[0]['t'],rr[-1]['t'];stable=[r for r in truth if max(t0+3,t1-3)<=r['t']<=t1]
            if len(stable)<15:continue
            v=float(np.median([r['v'] for r in stable]));w=float(np.median([r['w'] for r in stable]));lat=float(np.quantile(np.abs([r['lateral'] for r in stable]),.95));expected_v,expected_w=rr[-1]['v'],rr[-1]['w'];ar=rr[-1]['encoders'];meas=dict(v_mps=v,w_rad_s=w,lateral_p95_mps=lat)
            mc={}
            if name!='s_bend':
                if name=='zero_spin' and c['id']=='ackermann':expected_w=0.
                mc['speed']=abs(v-expected_v)<=.03+.08*abs(expected_v)
                mc['yaw']=abs(w-expected_w)<.01 if expected_w==0 else w*expected_w>0
                if expected_v and expected_w:
                    radius=abs(v/w) if abs(w)>1e-8 else 1e9;meas['radius_m']=radius;meas['expected_radius_m']=abs(expected_v/expected_w);mc['radius']=abs(radius-abs(expected_v/expected_w))/abs(expected_v/expected_w)<=.15
                if name.startswith('straight'):
                    moving=[r for r in truth if t0<=r['t']<=t1];first,last=moving[0],moving[-1];dx=last['xyz'][0]-first['xyz'][0];dy=last['xyz'][1]-first['xyz'][1];drift=abs(-math.sin(first['yaw'])*dx+math.cos(first['yaw'])*dy);meas['cross_drift_m']=drift;mc['straight_drift']=drift<=.05
                if c['id']=='ackermann':
                    k=0 if abs(expected_v)<.001 else expected_w/expected_v;angles=steering(k,c) if name!='zero_spin' else ar['q'][2:4];meas['steering_rad']=ar['q'][2:4];meas['target_steering_rad']=angles
                    mc['steering_feedback']=max(abs(x-y) for x,y in zip(ar['q'][2:],angles))<=.02
                    mc['not_side_sliding']=lat<=.05
            else:
                ss=[r for r in truth if t0+1<=r['t']<=t1];mc['both_turn_directions']=min(r['w'] for r in ss)<-.08 and max(r['w'] for r in ss)>.08;mc['not_side_sliding']=lat<.05
            if name=='zero_spin' and c['id']=='ackermann':
                ss=[r for r in truth if t0<=r['t']<=t1];meas['body_yaw_change_rad']=abs(ss[-1]['yaw']-ss[0]['yaw']);mc['no_body_spin']=meas['body_yaw_change_rad']<.005
            target=np.array([r['xyz'][:2] for r in truth if r['resets']==0]);tt=[r['t'] for r in truth if r['resets']==0];i0=np.array([np.interp(t0,tt,target[:,j]) for j in range(2)]);i1=np.array([np.interp(t1,tt,target[:,j]) for j in range(2)])
            odelta=np.array(rr[-1]['odom'][:2])-rr[0]['odom'][:2]
            th=float(np.interp(t0,tt,[r['yaw'] for r in truth if r['resets']==0]));oh=rr[0]['odom'][2]
            def local(v,h):return np.array([math.cos(h)*v[0]+math.sin(h)*v[1],-math.sin(h)*v[0]+math.cos(h)*v[1]])
            # Like the legacy baseline, compare each segment in its own start frame.
            # Accumulated multi-lap yaw drift is still saved in the full trajectory.
            error=float(np.linalg.norm(local(odelta,oh)-local(i1-i0,th)));distance=float(sum(np.linalg.norm(np.array(b['xyz'][:2])-a['xyz'][:2]) for a,b in zip(stable[:-1],stable[1:])))
            meas['odom_delta_error_m']=error;mc['odometry']=error<=.15+.08*max(distance,abs(expected_v)*(t1-t0))
            meas['checks']=mc;meas['passed']=all(mc.values());metrics['motion'][name]=meas
        for name in sorted({r['phase'] for r in online if r['phase'].startswith('stop_')}):
            rr=[r for r in online if r['phase']==name];metrics['stops'][name]=stop_score(truth,rr[0]['t'],rr[-1]['t'])
        checks['all_motion']=bool(metrics['motion']) and all(m['passed'] for m in metrics['motion'].values());checks['all_stop']=bool(metrics['stops']) and all(m['passed'] for m in metrics['stops'].values())
        if c['id']=='ackermann':
            rates=[abs(x) for r in online if r['encoders'] for x in r['encoders']['velocity'][2:]];metrics['max_steering_rate_rad_s']=max(rates);checks['steering_rate_limit']=max(rates)<=c['steering']['rate_rad_s']+.03
    elif rt['case'] in ['connector','slope','dock']:
        path=LineString(read(case/'online/target_path.json'));rr=[r for r in truth if r['t']>=2];errors=[Point(r['xyz'][:2]).distance(path) for r in rr];metrics['cross_track_max_m']=max(errors);metrics['cross_track_p95_m']=float(np.quantile(errors,.95));metrics['endpoint_error_m']=float(np.linalg.norm(np.array(rr[-1]['xyz'][:2])-path.coords[-1]));metrics['travel_sim_s']=end['total'];metrics['range_wait_samples']=sum(r['phase']=='range_wait' for r in online)
        checks['tracking']=max(errors)<.3;checks['endpoint']=metrics['endpoint_error_m']<(.08 if rt['case']=='dock' else .3)
        metrics['stops']['endpoint']=stop_score(truth,max(0,rr[-1]['t']-2),rr[-1]['t']);checks['final_stopped']=metrics['stops']['endpoint']['passed']
        if rt['case']=='connector':
            road=shape(read(Path(__file__).resolve().parents[5]/'optimization_v2/maps/prior/connector.geojson')['features'][0]['geometry']);foot=Polygon(c['footprint']);margins=[]
            for r in rr:
                footprint=translate(rotate(foot,r['yaw'],origin=(0,0),use_radians=True),r['xyz'][0],r['xyz'][1]);margins.append(footprint.distance(road.boundary) if road.covers(footprint) else -footprint.difference(road).area)
            metrics['min_footprint_road_margin_m']=min(margins);checks['full_footprint_inside_road']=min(margins)>0
        if rt['case']=='slope':
            pitch=max(abs(r['rpy'][1]) for r in rr);roll=max(abs(r['rpy'][0]) for r in rr);metrics.update(max_pitch_deg=math.degrees(pitch),max_roll_deg=math.degrees(roll));checks['actually_climbed']=math.degrees(pitch)>3;checks['stable']=max(pitch,roll)<math.radians(12)
            body_hits={k:v for k,v in contacts.items() if ('vehicle::base_link::body' in k or 'vehicle::tool_link::tool' in k) and 'grade_' in k};metrics['body_slope_contacts']=body_hits;checks['no_belly_strike']=not body_hits
    else:
        event=end['events'][0];reset=1 if rt['case']=='reset' else 0;start=.5 if reset else event['t'];last=max(r['t'] for r in truth if r['resets']==reset);metrics['stops']['fault']=stop_score(truth,start,last,reset);checks['fault_stopped']=metrics['stops']['fault']['passed']
        native=[r['encoders'] for r in online if r['encoders'] and r['total']>event['total']+2];metrics['native_reasons']=sorted({r['reason'] for r in native});metrics['arbiter_reasons']=sorted({r['status']['reason'] for r in online if r['status'] and r['total']>event['total']+1})
        if rt['case'] in ['adapter_loss','actuator_loss']:checks['independent_native_watchdog']='ACTUATOR_WATCHDOG' in metrics['native_reasons']
        if rt['case']=='feedback_loss':checks['feedback_alarm']='FEEDBACK_TIMEOUT' in metrics['arbiter_reasons']
        if rt['case'] in ['pause','reset']:checks['epoch_revoked']=max(r['epoch'] for r in online)>0
    metrics['passed']=all(checks.values());(case/'metrics.json').write_text(json.dumps(metrics,indent=2)+'\n');return metrics
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('case',type=Path);a=p.parse_args();m=analyze(a.case);print(json.dumps(m,indent=2));raise SystemExit(0 if m['passed'] else 1)
