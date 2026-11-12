"""Actual navigation, safety and computing measurements; no invented outcomes."""
from common15 import *
import math,gzip
from collections import Counter,defaultdict
import numpy as np

def rows(path):
    path=Path(path)
    if not path.exists():return []
    op=gzip.open if path.suffix=='.gz' else open
    with op(path,'rt') as f:return [json.loads(x) for x in f if x.strip()]

def duration(records,predicate):
    return sum(max(0.,min(b['t']-a['t'],.5)) for a,b in zip(records,records[1:]) if predicate(a))

def evaluate_case(out):
    out=Path(out);runtime=read(out/'runtime.json');mission=read(out/'client/mission.json') if (out/'client/mission.json').exists() else {}
    decisions=rows(out/'decisions.jsonl');gate=rows(out/'gate.jsonl');risks=rows(out/'risk.jsonl');forecasts=rows(out/'prediction/forecasts.jsonl');events=rows(out/'client/events.jsonl');resources=read(out/'resources.json') if (out/'resources.json').exists() else []
    contacts=read(out/'evaluation/contacts.json') if (out/'evaluation/contacts.json').exists() else None
    obstacles={k:v for k,v in (contacts or {}).items() if 'ground' not in k and 'terrain' not in k}
    good=[x for x in forecasts if x.get('status')=='OK'];latency=[x['transport_wall_ms'] for x in good if 'transport_wall_ms' in x];cpu=[x['cpu_ms'] for x in good if 'cpu_ms' in x]
    prediction_stops=[x for x in gate if x['reason']=='PREDICTED_CONFLICT_WAIT'];gate_valid=all(x['mode']=='STOP' and x['v']==0 and x['w']==0 for x in prediction_stops)
    protected=[x for x in gate if x['reason'] in ['PERCEPTION_STOP','LOCALIZATION_STOP','LOCAL_ODOM_STOP','FEEDBACK_STOP','TOOL_WORK_INTERLOCK','CLOCK_STOP','OWNER_CONFLICT','LEASE_OR_PLAN_STOP']]
    protected_valid=bool(protected) and all(x['mode']=='STOP' and x['v']==0 and x['w']==0 for x in protected)
    audit=[x for x in events if x['event']=='mission_ownership'];owners=bool(audit) and all(x['valid'] for x in audit)
    stopped=runtime.get('tool_shutdown',{}).get('state')=='STOPPED' and not runtime.get('tool_shutdown',{}).get('uncertainty_pending',False)
    safe=contacts is not None and not obstacles and stopped and runtime.get('database_unchanged',False) and gate_valid and protected_valid and owners and not mission.get('uncertainty_pending',True)
    complete=runtime.get('success',False) and mission.get('state')=='SUCCEEDED' and mission.get('required_points')==2 and mission.get('confirmed_points')==2
    fault=runtime.get('fault','none');fallback=Counter(x['reason'] for x in risks)
    fault_observed=(fault=='timeout' and any(x.get('status')=='INFERENCE_TIMEOUT' and x.get('injected_fault') for x in forecasts)) or (fault=='uncertainty' and fallback['PREDICTION_UNCERTAINTY_FALLBACK']>0) or (fault=='channel_loss' and any(x['event']=='prediction_process_paused' for x in runtime.get('events',[])) and (fallback['PREDICTION_TRANSPORT_FALLBACK']+fallback['PREDICTION_STALE_FALLBACK']>0))
    faults_window=[x for x in risks if 19<=x['t']<=23]
    sim_end=max([mission.get('t',0.),runtime.get('sim_seconds_observed',0.)]+[x['t'] for x in gate[-5:]])
    # Evaluate proxy misses on online object continuations; no truth is read by predictor.
    misses=risk_misses(out,risks)
    result=dict(scope='TWO_POINT_DYNAMIC_BENCHMARK_NOT_FULL_CLUSTER',scenario=runtime.get('scenario'),method=runtime.get('method'),fault=fault,benchmark_task_sha=sha(out/'scene/benchmark_task.json'),fixture_sha=sha(out/'scene/fixture_evaluation_only.json'),
        safe=safe,completed=complete,mission_state=mission.get('state'),reason=mission.get('reason'),required_points=mission.get('required_points'),confirmed_points=mission.get('confirmed_points'),
        sim_seconds_including_stop=sim_end,wall_seconds=runtime.get('wall_seconds'),prediction_wait_s=duration(decisions,lambda x:x['reason_code']=='PREDICTED_CONFLICT_WAIT'),all_wait_s=duration(decisions,lambda x:x['action']=='WAIT'),replans=sum(x['event']=='replan_attempt' for x in events),prediction_resumes=sum(x['event']=='prediction_admission_restored' for x in events),observation_resumes=sum(x['event']=='observation_admission_restored' for x in events),
        stop_confirmed=stopped,independent_gate_preserved=protected_valid,prediction_gate_zero=gate_valid,owner_audit_valid=owners,obstacle_contacts=obstacles,database_unchanged=runtime.get('database_unchanged'),
        predictions=len(forecasts),learned_person_forecasts=sum(f.get('candidate_status')=='LEARNED_PERSON' for x in good for f in x['forecasts']),imm_forecasts=sum(f.get('algorithm')=='IMM_CV_CA_CT_LEFT_CT_RIGHT' for x in good for f in x['forecasts']),latency_p95_ms=float(np.percentile(latency,95)) if latency else None,cpu_mean_ms=float(np.mean(cpu)) if cpu else None,
        timeout_fraction=sum(x.get('status')=='INFERENCE_TIMEOUT' for x in forecasts)/len(forecasts) if forecasts else None,risk_reasons=dict(fallback),fault_observed=fault_observed,fault_window_fallback=any('FALLBACK' in x['reason'] for x in faults_window),risk_misses=misses,
        max_owned_rss_bytes=max((x['owned_rss_bytes'] for x in resources),default=0),owned_cpu_seconds=max((x['owned_cpu_seconds'] for x in resources),default=0),nonzero_cleanup_exits={x['role']:x['exit_code'] for x in runtime.get('commands',[]) if x.get('exit_code') not in [0,None]},passed=safe and (fault_observed if fault!='none' else True))
    dump(out/'metrics15.json',result);return result

def risk_misses(out,risks):
    """Measured current footprint conflicts with observed future target positions.

    This diagnoses prediction notice of future conflicts at the *current* robot
    position; it is not a counterfactual full trajectory collision rate.
    """
    frames=rows(out/'fusion/tracks.jsonl');hist=defaultdict(list)
    for frame in frames:
        for o in frame['objects']:
            if o['state']=='OBSERVED' and o['confirmed']:hist[o['id']].append((frame['t'],o['position'][:2],o['size'][:2],o['classification']=='person' or math.hypot(*o['velocity'])>.05))
    online=[x for x in rows(out/'online/observations.jsonl') if x.get('record_type')=='odom']
    # Observer log names are version checked explicitly by stage preflight.
    if not online:return dict(eligible=0,missed=0,fraction=None,scope='NO_MATCHING_OBSERVER_TRACE')
    eligible=missed=0;last=-10.
    for r in risks:
        t=r['t']
        if t-last<.4:continue
        last=t
        live=min(online,key=lambda x:abs(x['t']-t))
        if abs(live['t']-t)>.2:continue
        xy=live['pose'][:2];conflict=False;observable=False
        for track in hist.values():
            past=[x for x in track if t-.6<=x[0]<=t and x[3]]
            future=[x for x in track if t+.1<x[0]<=t+2.]
            if not past or not future:continue
            observable=True
            if any(math.dist(xy,p)<.66+.5*math.hypot(*size)+.15 for _,p,size,_ in future):conflict=True
        if observable and conflict:
            eligible+=1;missed+=r.get('mode_envelope_mass',0)<.15
    return dict(eligible=eligible,missed=missed,fraction=missed/eligible if eligible else None,scope='observed_future_vs_current_robot_footprint; no counterfactual guarantee')

def choose_default(cases,offline,cfg):
    selection=cfg['selection'];choices={}
    for method in ['imm','learned']:
        by={v['scenario']:v for v in cases.values() if v['method']==method and v['fault']=='none'}
        base={v['scenario']:v for v in cases.values() if v['method']=='no_prediction' and v['fault']=='none'}
        metrics=offline['summary']['test'][method];reason=[]
        if len(by)!=4 or len(base)!=4:reason.append('INCOMPLETE_CLOSED_LOOP_MATRIX')
        if metrics['windows']<selection['minimum_test_windows']:reason.append('INSUFFICIENT_TEST_WINDOWS')
        if any(not x['safe'] for x in by.values()):reason.append('SAFETY_FAILURE')
        if any(base[s]['completed'] and not x['completed'] for s,x in by.items() if s in base):reason.append('NEW_FAILED_TASK')
        ratios=[x['sim_seconds_including_stop']/base[s]['sim_seconds_including_stop'] for s,x in by.items() if s in base and base[s]['completed'] and x['completed']]
        if len(ratios)<4:reason.append('INCOMPLETE_PAIRED_SUCCESS_EVIDENCE')
        elif np.mean(ratios)>selection['max_task_time_ratio_vs_none']:reason.append('NO_EFFICIENCY_QUALIFICATION')
        if any(x['latency_p95_ms'] is None or x['latency_p95_ms']>selection['max_latency_p95_ms'] for x in by.values()):reason.append('LATENCY_OR_MISSING_OUTPUT')
        if any(x['timeout_fraction'] is None or x['timeout_fraction']>selection['max_timeout_fraction'] for x in by.values()):reason.append('TIMEOUT_RATE')
        coverage=metrics['moment_ellipse_coverage95']
        if coverage is None or not selection['nominal_coverage95_min']<=coverage<=selection['nominal_coverage95_max']:reason.append('CALIBRATION_NOT_QUALIFIED')
        eligible=sum(x['risk_misses']['eligible'] for x in by.values());missed=sum(x['risk_misses']['missed'] for x in by.values())
        if eligible==0:reason.append('INSUFFICIENT_MEASURED_CONFLICT_EXPOSURE')
        elif missed/eligible>selection['risk_miss_fraction_max']:reason.append('RISK_MISSES')
        if method=='learned' and (metrics['weighted_ade_m'] is None or metrics['weighted_ade_m']>offline['summary']['test']['imm']['weighted_ade_m']*selection['learned_weighted_ade_ratio_vs_imm']):reason.append('NO_LEARNED_ACCURACY_GAIN')
        choices[method]=dict(enabled=not reason,reasons=reason,paired_time_ratio=float(np.mean(ratios)) if ratios else None,measured_conflict_exposures=eligible,misses=missed)
    default='learned' if choices['learned']['enabled'] else 'imm' if choices['imm']['enabled'] else 'no_prediction'
    return dict(default=default,candidates=choices,selection_rule=selection,scope='qualification decision for current small benchmark only; not six-cluster dynamic qualification')
