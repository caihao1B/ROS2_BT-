"""Generate and exercise the isolated short original A1 point-defer candidate."""
import argparse
from pathlib import Path
import sys

SCRIPTS=Path(__file__).resolve().parent
BUILD=SCRIPTS.parents[3]/'three_tasks/build/spray_defer_candidate'
SOURCE=SCRIPTS/'mission_spray_r10.py'

DEFER_METHODS='''    def check_defer_guard(remaining,dispatch):
        checks=defer_admission(st,cfg,spec,toolcfg,active_nav is not None or active_tool is not None,remaining,dispatch)
        if not all(checks.values()):
            log('CANDIDATE_DEFER_DENIED',checks=checks)
            raise RuntimeError('UNSAFE_TO_DEFER '+','.join(k for k,v in checks.items() if not v))
        return checks
    def defer_point(p,index,stage,proof):
        remaining=points[index+1:]
        if not proof.get('eligible') or not remaining:
            raise RuntimeError('NO_QUALIFIED_DEFER_OR_ALTERNATIVE')
        st.update(phase='OBSERVATION_HOLD',nav_idle=True,nav_failed=True)
        wait(lambda:bool(st['gate'] and st['gate']['stopped'] and st['gate']['tool_stopped'] and st['gate'].get('nav_idle')),4)
        check_defer_guard(remaining,False)
        log('CANDIDATE_EXHAUSTION_EVIDENCE',point=p['id'],stage=stage,proof=proof)
        start=st['t']; query_count=0
        while st['t']-start<=45.:
            tick();bt=st['bt'] or {};action=bt.get('action');ev=bt.get('evidence') or {}
            if action in ('HUMAN','STOP') or localization_hold():
                raise RuntimeError('DEFER_SUPERSEDED_BY_SAFETY '+str(bt.get('reason_code')))
            if not (st['loc'] or {}).get('valid') or not (st['safety'] or {}).get('valid'):
                raise RuntimeError('DEFER_LOCALIZATION_OR_PERCEPTION_UNAVAILABLE')
            if ev.get('replans')!=st['replans'] or not ev.get('blocked'):continue
            if action=='DEFER':
                checks=check_defer_guard(remaining,True)
                log('CANDIDATE_DEFER_CONFIRMED',point=p['id'],next_point=remaining[0]['id'],stage=stage,
                    checks=checks,dispatch_id=bt['dispatch_id'],context_sequence=bt['context_sequence'],reason=bt['reason_code'],
                    bt_t=bt['t'],blocked_duration_s=bt['blocked_duration_s'],replans=st['replans'],new_planner_queries=query_count,
                    residual_quantity=p['quantity'],credited_quantity=0.,dose_meter=st['feedback']['meter'],global_no_path_claim=False)
                failed.append(dict(point=p['id'],state='DEFERRED',reason=bt['reason_code'],origin=stage,remaining=p['quantity'],confirmed=0.))
                st.update(nav_failed=False,phase='READY',replans=0,deferrals=st['deferrals']+1)
                return
            if action!='REPLAN':continue
            check_defer_guard(remaining,False)
            # Spend a retry only after the original planner actually handled a request for this same goal.
            goal=ComputePathToPose.Goal();goal.planner_id='GridBased';goal.use_start=False
            target=PoseStamped();target.header.frame_id='map';target.pose.position.x,target.pose.position.y=p['target'];target.pose.orientation.z=math.sin(p['yaw']/2);target.pose.orientation.w=math.cos(p['yaw']/2);goal.goal=target
            wait(planner.server_is_ready,3);handle=result(planner.send_goal_async(goal),4)
            if not handle.accepted:raise RuntimeError('DEFER_PLANNER_SERVICE_REJECTED')
            answer=result(handle.get_result_async(),10);query_count+=1
            log('CANDIDATE_SAME_POINT_PLAN_RESULT',point=p['id'],stage=stage,query=query_count,status=answer.status,poses=len(answer.result.path.poses),
                code=getattr(answer.result,'error_code',None),work_credit=False,local_dock_admission=False)
            st['replans']+=1
        raise TimeoutError('BOUNDED_NATIVE_DEFER_NOT_ADMITTED')
'''

def replace_one(text,old,new):
    if text.count(old)!=1:raise ValueError('Formal worker changed: '+old[:100])
    return text.replace(old,new,1)

def build_mission():
    s=SOURCE.read_text();s=replace_one(s,'import argparse,json,time,signal,math',"import argparse,json,time,signal,math,sys\nfrom pathlib import Path\nsys.path.insert(0,str(Path(__file__).resolve().parents[3]/'ws/src/task_platform/scripts'))\nfrom spray_defer_policy_r10 import selected_points,exhausted_dock,defer_admission")
    s=replace_one(s,"points=allpoints[:spec['point_limit']] if spec['point_limit'] else allpoints","points=selected_points(allpoints,spec['candidate_selected_point_ids'])")
    s=replace_one(s,'    def service(p):',DEFER_METHODS+'    def service(p):')
    s=replace_one(s,"log('LOCAL_DOCK_NAV_FAILED',point=p['id'])","log('LOCAL_DOCK_NAV_FAILED',point=p['id'],status=future.result().status,error_code=getattr(future.result().result,'error_code',None))")
    s=replace_one(s,"approach_ready=False\n            attempt=0","approach_ready=False\n            deferred=False;point_events_start=len(events)\n            attempt=0")
    s=replace_one(s,"elif action=='DEFER':st['deferrals']+=1;break", """elif action=='DEFER':
                    local=events[point_events_start:]
                    proof=dict(eligible=st['replans']>=2 and any(e['event']=='NAV_GOAL' for e in local),actual_replans=st['replans'],
                               sent_navigation_goals=sum(e['event']=='NAV_GOAL' for e in local),aborts=sum(e['event']=='NAV_TERMINAL_FAILURE' for e in local))
                    defer_point(p,index,'COARSE_NAVIGATION',proof);deferred=True;break""")
    s=replace_one(s,"            if not approach_ready:failed.append(dict(point=p['id'],reason='NAVIGATION_RESIDUAL'));continue\n            settle_and_redock(p)","""            if deferred:continue
            if not approach_ready:raise RuntimeError('NAVIGATION_RESIDUAL_WITHOUT_QUALIFIED_NATIVE_DEFER')
            try:settle_and_redock(p)
            except RuntimeError as ex:
                if str(ex)!='DOCKING_REALIGNMENT_EXHAUSTED':raise
                proof=exhausted_dock(events[point_events_start:],p['id'],cfg['docking_realign_max_attempts'])
                defer_point(p,index,'PRECISION_DOCK',proof);continue""")
    s=replace_one(s,"not_selected=[p['id'] for p in allpoints[len(points):]]","not_selected=[p['id'] for p in allpoints if p['id'] not in spec['candidate_selected_point_ids']]")
    s=replace_one(s,"    if error == repr(RuntimeError('LOCALIZATION_REVALIDATION_BOUND')):","""    gate=final.get('final_gate') or {}
    if error is None and failed and all(r.get('state')=='DEFERRED' for r in failed) and len(completed)+len(failed)==len(points):
        physically_closed=(gate.get('stopped') and gate.get('tool_stopped') and 0<=final['t']-gate.get('t',-100)<.15 and
             any(e['event']=='FINAL_TOOL_STOP' and e.get('state')=='STOPPED' for e in events) and not any(e['event']=='SHUTDOWN_UNCONFIRMED' for e in events))
        return 0 if physically_closed else 2
    if error == repr(RuntimeError('LOCALIZATION_REVALIDATION_BOUND')):""")
    BUILD.mkdir(parents=True,exist_ok=True);out=BUILD/'mission_spray_candidate.py';compile(s,str(out),'exec');out.write_text(s);return out

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('mode',choices=('prepare','run'));p.add_argument('--seconds',type=int,default=330);p.add_argument('--rtf',type=float,default=.55);p.add_argument('--natural',action='store_true');a=p.parse_args()
    worker=build_mission()
    if a.mode=='prepare':print(worker)
    else:
        from spray_defer_fixture_r10 import build_runtime
        from analyze_spray_defer_r10 import analyze
        run=build_runtime(worker,obstacle=not a.natural)(seconds=a.seconds,rtf=a.rtf)
        result=analyze(run);print(run,result['behavior_result'],result['precision_defer_demonstrated']);sys.exit(0 if result['behavior_result']=='PASS' else 2)
