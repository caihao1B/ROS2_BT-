"""Live S07/S08/S09 compatibility under the new bound regional reference."""
import sys,fcntl
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from runtime14 import *

def assess(out):
    runtime=read(out/'runtime.json');truth=rows(out/'evaluation/truth.jsonl');obs=rows(out/'online/observations.jsonl');poses=[x for x in obs if x.get('record_type')=='pose'];status=[x for x in obs if x.get('record_type')=='global_status']
    # Only this independent evaluator reads truth, after all online nodes stop.
    errors=[];headings=[]
    if truth:
        tt=np.array([d['t'] for d in truth]);xy=np.array([d['xyz'][:2] for d in truth])-[28,14];yawtrue=np.unwrap([d['rpy'][2] for d in truth])
        for p in poses:
            if p['t']<10:continue
            expected=[np.interp(p['t'],tt,xy[:,i]) for i in range(2)];errors.append(math.dist(p['pose'][:2],expected));q=p['pose'][3:];h=2*math.atan2(q[2],q[3]);dh=h-np.interp(p['t'],tt,yawtrue);headings.append(abs(math.atan2(math.sin(dh),math.cos(dh))))
    # Observer uses the actual topic suffix as its record_type; preserve it.
    if not status:status=[x for x in obs if 'navigation_allowed' in x and 'gnss_reason' in x]
    result=dict(runtime_success=runtime['success'],database_unchanged=runtime['database_unchanged'],pose_samples=len(errors),xy_p95_m=float(np.quantile(errors,.95)) if errors else None,yaw_p95_rad=float(np.quantile(headings,.95)) if headings else None,status_records=len(status),trusted_samples=sum(s.get('state')=='TRUSTED_MAP' for s in status),gnss_reasons=sorted({s.get('gnss_reason','') for s in status}),stopped=runtime.get('tool_shutdown',{}).get('state')=='STOPPED',binding=runtime.get('regional_binding'))
    if 'fault' in runtime['case']:
        result['false_fixed_rejected']=any(26<s['t']<38 and s.get('gnss_reason')=='INNOVATION_REJECTED_FIXED' for s in status)
        result['both_lost_stop']=any(52<s['t']<64 and not s.get('navigation_allowed',True) for s in status)
        result['recovered']=any(s['t']>70 and s.get('state')=='TRUSTED_MAP' for s in status)
    if 'low_obstacle' in runtime['case']:
        result['low_obstacle_protective_stop']=any(x.get('record_type')=='perception' and x.get('t',0)>10 and x.get('reason')=='GEOMETRIC_PROTECTIVE_STOP' and x.get('max_speed')==0 for x in obs)
        result['low_obstacle_cell_occupied']=False
        for f in (out/'cost_adapter').glob('grid_*.npz'):
            grid=np.load(f)
            if float(grid['t'])<10:continue
            cost=grid['cost'];origin=grid['origin'];res=float(grid['resolution']);iy,ix=np.where(cost==100);xy=np.c_[ix+.5,iy+.5]*res+origin
            if np.any((xy[:,0]>.8)&(xy[:,0]<1.4)&(np.abs(xy[:,1])<.3)):result['low_obstacle_cell_occupied']=True
    result['passed']=bool(result['runtime_success'] and result['stopped'] and len(errors)>20 and result['xy_p95_m']<.08 and result['yaw_p95_rad']<.04 and result['trusted_samples']>5 and all(result.get(k,True) for k in ['false_fixed_rejected','both_lost_stop','recovered']))
    result['passed'] &= all(result.get(k,True) for k in ['low_obstacle_protective_stop','low_obstacle_cell_occupied'])
    dump(out/'registration_metrics.json',result);print(json.dumps(result),flush=True);return result

if __name__=='__main__':
    lock=(ROOT/'native_20261108/runs/session.lock').open('a');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    if os.getpgrp()!=os.getpid():os.setsid()
    for spec in sys.argv[1:]:
        kind='ackermann' if spec=='B' else 'diff';case='regional_probe_'+spec;out=BASE/'results/S14'/('registered_'+spec+'_'+str(os.getpid()));print(out,flush=True)
        run_case(out,'zone_a',case,90 if 'fault' in spec else 35,300 if 'fault' in spec else 150,kind=kind,probe=True,survey=True)
        if not assess(out)['passed']:raise SystemExit(1)
