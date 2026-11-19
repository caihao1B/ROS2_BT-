"""Start the unchanged navigation-progress clock once, on the first accepted transfer movement after deliberate warm-up."""
import argparse
import ast
import hashlib
import json
from pathlib import Path
import sys
from types import FunctionType

import mowing_verified_transfer_r10 as transfer
from common import dump, read, sha
from mowing_yaw_candidate_r10 import replace_once
from recovery_stop_candidate_r10 import RecoveryStopEvidence


PROFILE='verified-entry-first-motion'


def first_motion_origin(already_armed, segment, first_segment, accepted, timestamp, localization):
    if already_armed or segment!=first_segment or not accepted or not localization or not localization.get('valid'):
        return None
    return timestamp,list(localization['pose'][:2])


def worker_source():
    src=transfer.worker_source()
    src=replace_once(src,"spec.get('candidate_profile')=='verified-entry-transfer'",
                     "spec.get('candidate_profile')=='verified-entry-first-motion'")
    src=replace_once(src,'transfer_passages=[]','transfer_passages=[];first_transfer_armed=False')
    marker="    log('SEGMENT_GOAL',segment=s['id']"
    inject="""    start_observation=first_motion_origin(first_transfer_armed,s['id'],transfer[0]['id'],active.accepted,st['t'],st['loc'])
    if start_observation is not None:
     last_progress_t,last_p=start_observation;first_transfer_armed=True
     log('FIRST_TRANSFER_MOTION_CLOCK_ARMED',segment=s['id'],actual_action_accepted=bool(active.accepted),origin_t=last_progress_t,origin_pose=last_p)
"""
    return replace_once(src, marker, inject+marker)


def compose(*args,**kwargs):
    result=transfer.compose(*args,**kwargs)
    out=args[0] if args else kwargs['out']
    fixture=read(out/'fixture.json')
    fixture.update(session_profile=PROFILE,parent_test_profile=transfer.PROFILE)
    dump(out/'fixture.json',fixture)
    return result


def runner():
    run=transfer.runner()
    scope=run.__globals__
    scope.update(__file__=__file__,compose=compose,candidate_profile=PROFILE,
                 candidate_sources=scope['candidate_sources']+('mowing_transfer_clock_r10.py',),
                 worker_hash=hashlib.sha256(worker_source().encode()).hexdigest())
    return run


def probe():
    old=transfer.worker_source();new=worker_source()
    def progress_guard(source):
        conditions=[n.test for n in ast.walk(ast.parse(source)) if isinstance(n,ast.If) and
                    any(isinstance(c,ast.Constant) and c.value=='NO_OBSERVED_PROGRESS' for c in ast.walk(n))
                    and 'last_progress_t is not None' in ast.unparse(n.test)]
        if len(conditions)!=1:raise RuntimeError('Expected one frozen no-progress condition')
        return conditions[0]
    original=progress_guard(old);candidate=progress_guard(new)
    code=compile(ast.Expression(candidate),'<frozen-progress-predicate>','eval')
    def expires(now,start):return bool(eval(code,{'active':True,'last_progress_t':start,'st':{'t':now}}))
    loc={'valid':True,'pose':[82.25,36.38,.26]};sid='QUALIFICATION_TRANSFER:0'
    origin=first_motion_origin(False,sid,sid,True,90.036,loc)
    checks=dict(guard_unchanged=ast.dump(original)==ast.dump(candidate),
                historical_before_motion_expiry=expires(90.036,36.24),
                time_starts_at_real_admission=origin==(90.036,loc['pose'][:2]) and not expires(90.036,origin[0]),
                unchanged_timeout_fires=not expires(135.035,origin[0]) and expires(135.037,origin[0]),
                retry_cannot_extend=first_motion_origin(True,sid,sid,True,125.,loc) is None,
                later_gear_cannot_extend=first_motion_origin(False,'QUALIFICATION_TRANSFER:1',sid,True,125.,loc) is None,
                rejected_or_unobserved_cannot_arm=first_motion_origin(False,sid,sid,False,90.036,loc) is None
                    and first_motion_origin(False,sid,sid,True,90.036,{'valid':False}) is None)
    return dict(passed=all(checks.values()),checks=checks,source=sha(Path(__file__)),
                scope='Recorded 36.24 s first location and 90.036 s accepted action. Actual unmodified worker progress predicate, component-only qualification.')


def verify(run):
    # Use the previous integration verifier with this explicit worker binding.
    checks_scope={**transfer.verify.__globals__,'worker_source':worker_source,'PROFILE':PROFILE}
    verify_base=FunctionType(transfer.verify.__code__,checks_scope)
    result=verify_base(run)
    from analyze_r02 import rows
    events=rows(run/'mission_events.jsonl')
    arms=[r for r in events if r['event']=='FIRST_TRANSFER_MOTION_CLOCK_ARMED']
    first=next((r for r in events if r['event']=='SEGMENT_GOAL' and r['segment']=='QUALIFICATION_TRANSFER:0'),None)
    result['checks']['one_clock_at_first_real_action']=bool(first is not None and len(arms)==1 and
        arms[0]['actual_action_accepted'] and abs(arms[0]['origin_t']-first['t'])<.05)
    result['actual_motion_clock_arming']=arms
    result['passed']=all(result['checks'].values())
    dump(run/'candidate_analysis.json',result)
    return result


if __name__=='__main__':
    if sys.argv[1:2]==['worker']:
        del sys.argv[1];path=transfer.SCRIPTS/'mission_mowing_r10.py'
        exec(compile(worker_source(),str(path),'exec'),
             dict(__name__='__main__',__file__=str(path),hashlib=hashlib,RecoveryStopEvidence=RecoveryStopEvidence,
                  first_motion_origin=first_motion_origin))
    else:
        ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--probe',action='store_true')
        ap.add_argument('--verify',type=Path);ap.add_argument('--rtf',type=float,default=.4);ap.add_argument('--out',type=Path)
        a=ap.parse_args()
        if a.probe:
            runner();compile(worker_source(),'<candidate-worker>','exec');report=probe()
            if a.out:dump(a.out,report)
        else:
            path=a.verify or runner()(task='mowing',seconds=780,rtf=a.rtf,cluster='B1',segment_limit=transfer.COUNT,
                                      case='nominal',segment_start=transfer.START)
            report=verify(path)
        print(json.dumps(report,indent=2));raise SystemExit(0 if report['passed'] else 2)
