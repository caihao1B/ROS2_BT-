from common14 import *

def evaluate(out):
    runtime=read(out/'runtime.json');mission=read(out/'client/mission.json') if (out/'client/mission.json').exists() else {}
    observations=rows(out/'online/observations.jsonl');decisions=rows(out/'decisions.jsonl');events=rows(out/'client/events.jsonl')
    audit=read(out/'online/result.json') if (out/'online/result.json').exists() else {}
    active=[a for a in audit.get('audits',[]) if a.get('navigation_active')]
    owners=all(a['request']==['s03_test_driver'] and a['actuator']==['s03_drive_bridge'] and a['nav']==['controller_server'] and a['lease']==['s13_action_client'] and sorted(a['tf'])==['s05_local_odometry','s08_global_pose_manager'] for a in active)
    explain=bool(decisions) and all(all(k in d for k in ['t','reason_code','evidence','thresholds','candidates','prediction_source']) for d in decisions)
    stopped=runtime.get('tool_shutdown',{}).get('state')=='STOPPED'
    tool_audit=[e for e in events if e['event']=='mission_ownership']
    owners=owners and all(e['valid'] for e in tool_audit)
    last=read(out/'online/live.json') if (out/'online/live.json').exists() else {}
    fault=[x for x in events if x['event']=='fault_injected'];reasons=sorted({d['reason_code'] for d in decisions})
    physical_times=[d['t'] for d in rows(out/'tool_evaluation/physical.jsonl')]
    physical=read(out/'tool_evaluation/result.json') if (out/'tool_evaluation/result.json').exists() else {}
    physical_summary={k:{f:j.get(f) for f in ['dose','area_m2','active_samples']} for k,j in physical.get('jobs',{}).items() if k}
    cleaned=read(out/'cleanup14.json') if (out/'cleanup14.json').exists() else dict(survivors=['UNVERIFIED'])
    owners=owners and not cleaned.get('survivors')
    elapsed=max([runtime.get('sim_seconds_observed',mission.get('t',0)),*physical_times])
    passed=bool(runtime.get('success') and stopped and owners and explain and runtime['database_unchanged'] and elapsed<=runtime['seconds'])
    return dict(regional_binding=runtime.get('regional_binding'),physical_effects=physical_summary,cleanup=cleaned,case=runtime['case'],completed=mission.get('state')=='SUCCEEDED',passed=passed,scope=mission.get('scope'),mission=mission,sim_seconds=elapsed,sim_limit_s=runtime['seconds'],shutdown_clock_paused=runtime.get('shutdown_clock_paused'),wall_seconds=runtime.get('wall_seconds'),explanations_valid=explain,ownership_ok=owners,active_ownership_samples=len(active),tool_ownership_samples=len(tool_audit),stopped_confirmed=stopped,database_unchanged=runtime.get('database_unchanged'),reasons=reasons,faults=fault,localization=last.get('status'),runtime_error=runtime.get('error'),decisions=decisions,trace=[d for d in observations if d.get('record_type')=='pose'],tree=rows(out/'client/tree.jsonl')[::10],events=[{k:v for k,v in e.items() if k not in ['residual','candidate']} for e in events],process_exit_codes={c['role']:c.get('exit_code') for c in runtime.get('commands',[])})

def assess(data):
    cases=data['cases'];full=[cases.get(n,{}) for n in ['nominal_spray_cluster','nominal_mow_cluster']]
    c01=bool(data.get('policy_tests_passed')) and all(max([e.get('attempt',0) for e in c.get('events',[]) if e['event']=='replan_attempt'],default=0)<=2 and max([e.get('resumes',0) for e in c.get('events',[]) if e['event']=='observation_admission_restored'],default=0)<=12 for c in cases.values())
    c02=bool(cases) and all(c.get('explanations_valid') and c.get('ownership_ok') and c.get('stopped_confirmed') and c.get('database_unchanged') and c.get('sim_seconds',1e9)<=c.get('sim_limit_s',0) for c in cases.values()) and any(c.get('active_ownership_samples',0)>0 and c.get('tool_ownership_samples',0)>0 for c in cases.values())
    a,b=[c.get('mission',{}) for c in full]
    c03=all(c.get('passed') and c.get('completed') and c.get('sim_seconds',1e9)<=1800 for c in full) and a.get('required_points')==21 and a.get('confirmed_points')==21 and abs(b.get('required_area_m2',0)-16.35945787959566)<1e-5 and b.get('coverage_fraction',0)>=.95
    budget=cases.get('budget_exhaustion',{});block=cases.get('persistent_block',{});loss=cases.get('tool_loss',{})
    c04=all(c.get('stopped_confirmed') and not c.get('completed') and c.get('mission',{}).get('required_points')==2 for c in [budget,block,loss]) and 'BUDGET_STOP' in budget.get('reasons',[]) and bool(block.get('faults')) and 'BLOCK_RETRY_EXHAUSTED' in block.get('reasons',[]) and bool(loss.get('faults')) and any(t.get('uncertain') for t in loss.get('mission',{}).get('tool_results',[]))
    data['checks']={f'S14-C{i:02}':'PASS' if v else 'FAIL' for i,v in enumerate([c01,c02,c03,c04],1)}
    data['status']='PASS' if all([c01,c02,c03,c04]) else 'PARTIAL'
