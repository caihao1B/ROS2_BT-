"""Run the candidate's own exit decision in subprocesses. Rebuild its real A1 inputs or replay the published compact snapshot."""
import argparse
from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path
import subprocess
import sys
from spray_handoff_candidate_r10 import build,operator_request

SCRIPTS=Path(__file__).resolve().parent
ROOT=SCRIPTS.parents[4]

def read(path):return json.loads(path.read_text())
def save(path,doc):path.parent.mkdir(parents=True,exist_ok=True);path.write_text(json.dumps(doc,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
def lines(path):
    with path.open() as stream:
        for line in stream:yield json.loads(line)
def digest(path):
    with path.open('rb') as stream:return hashlib.file_digest(stream,'sha256').hexdigest()

def archive(folder):
    mission=read(folder/'mission_result.json');plan=read(folder/'mission.json');spec=read(folder/'spec.json');tool=read(folder/'scene/tool.json');runtime=read(folder/'runtime.json');events=list(lines(folder/'mission_events.jsonl'));stop=next(e for e in reversed(events) if e['event']=='TASK_STOP');when=mission['t'];goal=next(e for e in reversed(events) if e['event']=='NAV_GOAL');assert mission['error']=="RuntimeError('DOCKING_REALIGNMENT_EXHAUSTED')"
    point=next(p for c in plan['clusters'] if c['id']=='A1' for p in c['points'] if p['id']==goal['point']);
    def last(file,predicate=lambda e:True):
        result=None
        for row in lines(folder/file):
            if row['t']>when:break
            if predicate(row): result=row
        if result is None:raise ValueError(f'Missing input {file}')
        return result
    odom=last('reference/reference.jsonl');fb=last('tool_observer/observations.jsonl',lambda e:e['kind']=='feedback');physical=last('tool_evaluation/physical.jsonl');truth=[r for r in lines(folder/'evaluation/truth.jsonl') if when-.3<=r['t']<=when];first=truth[0];end=truth[-1];dt=end['t']-first['t'];gate=mission['final_gate']
    outstanding_nav=max((e['t'] for e in events if e['event'] in ('NAV_GOAL','LOCAL_DOCK_GOAL')),default=0)
    terminal_nav=max((e['t'] for e in events if e['event'] in ('LOCAL_DOCK_NAV_FAILED','LOCAL_DOCK_NAV_SUCCESS','NAV_SUCCESS')),default=-1)
    request=operator_request(point,events,False,mission['completed'],mission['original_points'],plan['version'],'DOCKING_REALIGNMENT_EXHAUSTED',stop['t'],spec['session']);cfg=plan['vehicle']['safety']
    snap=dict(t=when,request=request,gate=gate,odometry=dict(t=odom['odom_t'],v=odom['v'],w=odom['w']),feedback=fb,events=[e for e in events if e['event'] in ('TASK_STOP','FINAL_TOOL_STOP','SHUTDOWN_UNCONFIRMED')],limits=dict(max_v=cfg['stopped_v_mps'],max_w=cfg['stopped_w_rad_s'],max_feedback_age_s=tool['feedback_age_sim_s'],max_tool_rpm_rad_s=tool['stopped_max_rad_s'],max_ack_age_s=tool['stop_timeout_s'],tool_revision=tool['tool_revision']),mission_passed=mission['passed'],actions_idle=bool(gate['nav_idle'] and terminal_nav>outstanding_nav and end['t']>terminal_nav and not fb['request_on']))
    paths=['mission_events.jsonl','mission.json','mission_result.json','spec.json','runtime.json','scene/tool.json','reference/reference.jsonl','tool_observer/observations.jsonl','tool_evaluation/physical.jsonl','evaluation/truth.jsonl']
    return dict(schema='SPRAY-HANDOFF-RECORDED-SNAPSHOT/1',source_case=folder.name,actual_formal_result=dict(confirmed=len(mission['completed']),original=mission['original_points'],mission_passed=mission['passed'],runtime_clean=runtime['passed'],observer_process_exit=next(p['exit'] for p in runtime['processes'] if p['role']=='observer'),forced_processes=runtime['forced'],exception=mission['error']),
      measurements=snap,independent_offline_physics=dict(window_s=dt,body_translation_speed_mps=math.dist(first['xyz'][:2],end['xyz'][:2])/dt,body_yaw_rate_rps=abs(math.remainder(end['rpy'][2]-first['rpy'][2],math.tau))/dt,tool_t=physical['t'],tool_flow=physical['flow'],tool_angular_speed_rad_s=physical['rpm_rad_s']),
      actions_idle_evidence=dict(latest_sent_nav_s=outstanding_nav,latest_failed_or_completed_nav_s=terminal_nav,original_gate_nav_idle=gate['nav_idle'],original_feedback_request_on=fb['request_on'],scope='Original action handles are inferred from terminal events, gate and tool feedback. The live candidate checks its own handles directly.'),
      source_sha256={str((folder/p).relative_to(ROOT)):digest(folder/p) for p in paths})

def run_tests(bundle):
    driver=SCRIPTS/'spray_handoff_candidate_r10.py';generated=build();s=bundle['measurements'];testcases={};summary=[]
    def add(label,snapshot,expected):testcases[label]=(snapshot,expected)
    add('archive_all_required_checks',deepcopy(s),0)
    changes={'body_stop_missing':('gate','stopped',False),'tool_stop_missing':('gate','tool_stopped',False),'gate_over_age':('gate','t',s['t']-.19),'nav_still_active':('gate','nav_idle',False),'odometry_stale':('odometry','t',s['t']-.19),'body_measured_moving':('odometry','v',.02),'body_measured_rotating':('odometry','w',.03),'tool_flow_present':('feedback','flow',1.),'tool_still_turning':('feedback','rpm_rad_s',.4),'tool_feedback_old':('feedback','t',s['t']-.15),'wrong_tool_session':('feedback','sensor_epoch','other'),'tool_feedback_unhealthy':('feedback','healthy',False),'unexpected_exception':('request','reason','ARBITRARY_INTERNAL_ERROR'),'credit_in_manual_request':('request','credit_added_by_this_handoff',1.)}
    for label,(group,key,val) in changes.items():
        bad=deepcopy(s);bad[group][key]=val;add(label,bad,2)
    no_ack=deepcopy(s);no_ack['events']=[e for e in no_ack['events'] if e['event']!='FINAL_TOOL_STOP'];add('missing_stop_work_ack',no_ack,2)
    bad=deepcopy(s);next(e for e in bad['events'] if e['event']=='FINAL_TOOL_STOP')['action_status']=6;add('ack_returned_abort',bad,2)
    bad=deepcopy(s);bad['events'].append({'event':'SHUTDOWN_UNCONFIRMED','t':s['t']});add('shutdown_failed_after_ack',bad,2)
    bad=deepcopy(s);bad['actions_idle']=False;add('pending_action_handle',bad,2)
    bad=deepcopy(s);bad['mission_passed']=True;add('false_task_success',bad,2)
    for name,(payload,expected) in testcases.items():
        proc=subprocess.run([sys.executable,str(driver),'decision'],input=json.dumps(payload),capture_output=True,text=True,timeout=4);result=json.loads(proc.stdout);observed=proc.returncode
        summary.append(dict(case=name,expected_process_exit=expected,observed_process_exit=observed,task_terminal=result['task_terminal'],mission_passed=result['mission_passed'],failed_checks=[k for k,v in result['checks'].items() if not v],passed=observed==expected==result['process_exit'] and result['mission_passed'] is False))
    phys=bundle['independent_offline_physics'];tol=s['limits'];physics_confirmed=phys['body_translation_speed_mps']<tol['max_v'] and phys['body_yaw_rate_rps']<tol['max_w'] and phys['tool_flow']==0. and abs(phys['tool_angular_speed_rad_s'])<tol['max_tool_rpm_rad_s']
    return dict(schema='SPRAY-MANUAL-HANDOFF-PROBE/1',source_run=bundle['source_case'],scope='Offline real recorded full A1 measurements used by a separate handoff candidate. Exit codes below are actual candidate decision subprocess exits; whole Gazebo run with this new candidate remains unverified.',formal_credit=False,
       passed=physics_confirmed and all(row['passed'] for row in summary),independent_physical_stop_confirmed=physics_confirmed,actual_original=bundle['actual_formal_result'],candidate_operator_request=s['request'],candidate_false_success_allowed=False,full_gazebo_handoff_validated=False,tests=summary,native_source=digest(driver),derived_worker_sha256=digest(generated),probe_sha256=digest(Path(__file__).resolve()))

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);group=p.add_mutually_exclusive_group(required=True);group.add_argument('--recording',type=Path);group.add_argument('--bundle',type=Path);p.add_argument('--save-bundle',type=Path);p.add_argument('--out',type=Path,required=True);a=p.parse_args()
    data=archive(a.recording.resolve()) if a.recording else read(a.bundle)
    if data.get('schema')!='SPRAY-HANDOFF-RECORDED-SNAPSHOT/1':raise ValueError('Unexpected handoff input schema')
    if a.save_bundle:save(a.save_bundle,data)
    r=run_tests(data);r['input_sha256']=digest(a.bundle or a.save_bundle) if a.bundle or a.save_bundle else None;save(a.out,r);print(f"{sum(x['passed'] for x in r['tests'])}/{len(r['tests'])} cases, independent physical stop {r['independent_physical_stop_confirmed']}");raise SystemExit(0 if r['passed'] else 2)
