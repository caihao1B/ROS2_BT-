"""Honor a real native manual latch that arrives between the final replan and the next dispatch."""
import argparse
import json
from pathlib import Path
import sys
from spray_defer_candidate_r10 import replace_one
import spray_last_blocked_r10 as previous

HERE=Path(__file__).resolve()


def worker():
    parent=previous.build_worker();text=parent.read_text()
    old="if action=='REPLAN':attempt+=1;st['replans']+=1;st['nav_failed']=False;wait(lambda:bool(st['bt'] and st['bt']['action']=='EXECUTE'),5)"
    new="""if action=='REPLAN':
                    attempt+=1;st['replans']+=1;st['nav_failed']=False
                    wait(lambda:bool(st['bt'] and st['bt']['action'] in ('EXECUTE','HUMAN','STOP')),5)
                    refreshed=st['bt']
                    if refreshed['action']!='EXECUTE':
                        log('REPLAN_SUPERSEDED_BY_NATIVE_STOP',point=p['id'],action=refreshed['action'],reason=refreshed['reason_code'],
                            dispatch_id=refreshed['dispatch_id'],context_sequence=refreshed['context_sequence'],motion_dispatched=False)
                        raise RuntimeError('MANUAL_OR_SAFETY_HOLD '+str(refreshed['reason_code']))"""
    text=replace_one(text,old,new)
    text=replace_one(text,"proof.get('observed_nav_aborts',0)>=3 and proof.get('actual_replan_decisions',0)>=2", "proof.get('observed_nav_aborts',0)>=2 and proof.get('actual_replan_decisions',0)>=2 and proof.get('nav_idle_before_notice') is True and proof.get('native_replan_limit')==proof.get('native_replan_count')==2")
    text=replace_one(text,"native_session=native.get('session'),native_version=native.get('task_version')", "native_replan_limit=(native.get('thresholds') or {}).get('max_replans'),native_replan_count=(native.get('evidence') or {}).get('replans'),nav_idle_before_notice=(native.get('evidence') or {}).get('nav_idle'),native_session=native.get('session'),native_version=native.get('task_version')")
    out=parent.parent.parent/'spray_last_blocked_latch'/'mission_spray_last_latch.py';out.parent.mkdir(parents=True,exist_ok=True);compile(text,str(out),'exec');out.write_text(text);return out


def variant():
    p=Path(previous.__file__).resolve();text=p.read_text();text=replace_one(text,"ns['EXPERIMENTS']=APP/'results/experiments/spray_last_blocked'","ns['EXPERIMENTS']=APP/'results/experiments/spray_last_blocked_latch'")
    text=replace_one(text,"SCRIPTS/'spray_handoff_candidate_r10.py',SCRIPT]", "SCRIPTS/'spray_handoff_candidate_r10.py',SCRIPT,NEW_SOURCE]")
    anchor=text.index("def analyze(");text=text[:anchor]+replace_one(text[anchor:],"proof.get('observed_nav_aborts',0)>=3 and proof.get('actual_replan_decisions',0)>=2", "proof.get('observed_nav_aborts',0)>=2 and proof.get('actual_replan_decisions',0)>=2 and d.get('manual_latched') and d['evidence'].get('nav_idle') and d['evidence'].get('replans')==d['thresholds'].get('max_replans')==2")
    ns=dict(__file__=str(p),__name__='isolated_last_latch',NEW_SOURCE=HERE);exec(compile(text,str(p),'exec'),ns);ns['build_worker']=worker;return ns


def audit_initial(path):
    events=[json.loads(l) for l in (path/'mission_events.jsonl').open()];ds=[json.loads(l) for l in (path/'decisions.jsonl').open()];m=json.loads((path/'mission_result.json').read_text())
    replans=[e for e in events if e['event']=='RETRY_DECISION' and e.get('action')=='REPLAN'];aborted=[e for e in events if e['event']=='NAV_TERMINAL_FAILURE'];latch=next(d for d in ds if d['t']>=replans[-1]['t'] and d['action']=='HUMAN' and d['manual_latched'] and d['reason_code']=='BOUNDED_REPLAN_EXHAUSTED');s=latch['evidence'];stop=next(e for e in events if e['event']=='TASK_STOP');assert s['alternative'] is False and s['nav_idle'] and s['replans']==latch['thresholds']['max_replans']==2 and len(aborted)==len(replans)==2 and stop['t']>=latch['t']+4. and 'WAIT_DEADLINE' in m['error'];assert not any(e['event']=='NAV_GOAL' and e['t']>latch['t'] for e in events)
    print('Historical native HUMAN',latch['t'],'2 failed action results, 2 native replan decisions; no third dispatch; original timeout',stop['t'])

if __name__=='__main__':
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('mode',choices=('check','run'));ap.add_argument('--initial',type=Path);ap.add_argument('--seconds',type=int,default=138);ap.add_argument('--rtf',type=float,default=.55);a=ap.parse_args();ns=variant();runtime=ns['isolated_runtime']()
    if a.initial:audit_initial(a.initial)
    if a.mode=='check':print('derived latch worker built and original selection bound');raise SystemExit(0)
    case=runtime(seconds=a.seconds,rtf=a.rtf);r=ns['analyze'](case);print(json.dumps({k:r[k] for k in ('session','passed','terminal','observed_process_exit','sim_s','completed')}));raise SystemExit(0 if r['passed'] else 2)
