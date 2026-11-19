"""Build a separate point-service candidate with auditable operator handoff and process exit."""
import argparse
import inspect
import json
import math
from pathlib import Path
import sys


def operational_docking_failure(message):
    direct={'DOCKING_REALIGNMENT_EXHAUSTED','NO_QUALIFIED_DEFER_OR_ALTERNATIVE',
            'BOUNDED_NATIVE_DEFER_NOT_ADMITTED','DEFER_LOCALIZATION_OR_PERCEPTION_UNAVAILABLE',
            'DEFER_PLANNER_SERVICE_REJECTED','NAVIGATION_RESIDUAL_WITHOUT_QUALIFIED_NATIVE_DEFER',
            'LOCAL_DOCK_OBSERVED_PATH_BLOCKED','LOCAL_DOCK_ODOMETRY_STALE','LOCAL_DOCK_ADMISSION_CHANGED',
            'LOCAL_DOCK_NAV_REJECTED','LOCAL_DOCK_SAFETY_HOLD','LOCALIZATION_REVALIDATION_BOUND'}
    return message in direct or message.startswith(('UNSAFE_TO_DEFER ','DEFER_SUPERSEDED_BY_SAFETY ',
                                                    'DOCKING_MANUAL_OR_SAFETY_HOLD ','LOCAL_DOCK_GEOMETRY_'))


def operator_request(point,records,active_tool,completed,original_count,version,reason,requested_t,session):
    past=[e for e in records if e.get('event')=='TOOL_RESULT' and e.get('point')==point['id'] and not e.get('replayed')]
    last=past[-1] if past else {};uncertain=bool(active_tool or last.get('uncertainty'))
    accounted=last.get('confirmed',0.) if not uncertain else last.get('confirmed')
    remaining=last.get('remaining',point['quantity']) if not uncertain else last.get('remaining')
    return dict(schema='SPRAY-OPERATOR-HANDOFF/1',session=session,terminal='HUMAN_REQUIRED',reason=reason,
        requested_t=requested_t,point=point['id'],original_key=version+':'+point['id'],requested=point['quantity'],
        previous_confirmed=accounted,residual=remaining,quantity_uncertain=uncertain,credit_added_by_this_handoff=0.,
        original_obligations=original_count,completed_at_request=len(completed),mission_passed=False,automatic_resume=False,
        operator_action='Inspect the pending target, localization and obstacle evidence; an explicit new task is required to resume.')


def handoff_result(snapshot):
    now=snapshot['t'];request=snapshot['request'];gate=snapshot['gate'] or {};odom=snapshot['odometry'] or {};fb=snapshot['feedback'] or {}
    events=snapshot['events'];tol=snapshot['limits'];f=request['requested_t'];session=request['session'];
    def fresh(row,max_age):return isinstance(row.get('t'),(int,float)) and 0<=now-row['t']<=max_age
    def slow(row,key,limit):return isinstance(row.get(key),(int,float)) and math.isfinite(row[key]) and abs(row[key])<limit
    tool_ack=next((e for e in reversed(events) if e.get('event')=='FINAL_TOOL_STOP'),{})
    checks=dict(known_operational_hold=operational_docking_failure(request['reason']),explicit_manual_terminal=request.get('terminal')=='HUMAN_REQUIRED',
        task_remains_incomplete=request.get('mission_passed') is False and snapshot['mission_passed'] is False,
        old_key_retained=bool(request.get('original_key')) and request.get('credit_added_by_this_handoff')==0.,
        no_live_navigation_or_tool_action=snapshot['actions_idle'] is True,
        fresh_gate_double_stop=fresh(gate,.15) and all(gate.get(k) is True for k in ('stopped','tool_stopped','nav_idle')),
        measured_chassis_stop=fresh(odom,.15) and slow(odom,'v',tol['max_v']) and slow(odom,'w',tol['max_w']),
        independent_tool_stopped=fresh(fb,tol['max_feedback_age_s']) and fb.get('sensor_epoch')==session and fb.get('tool_revision')==tol['tool_revision'] and
            fb.get('healthy') is True and fb.get('request_on') is False and fb.get('flow')==0. and slow(fb,'rpm_rad_s',tol['max_tool_rpm_rad_s']),
        explicit_stop_work_ack=tool_ack.get('state')=='STOPPED' and ('action_status' not in tool_ack or tool_ack['action_status']==4) and f<=tool_ack.get('t',-1)<=now and now-tool_ack.get('t',-100)<=tol['max_ack_age_s'],
        no_unconfirmed_shutdown=not any(e.get('event')=='SHUTDOWN_UNCONFIRMED' for e in events))
    return dict(task_terminal='HUMAN_REQUIRED',mission_passed=False,process_exit=0 if all(checks.values()) else 2,
                shutdown_verified=all(checks.values()),checks=checks,actions_authorized=False,stop_ack_action_status_recorded='action_status' in tool_ack)


def build():
    from spray_defer_candidate_r10 import build_mission,replace_one
    worker=build_mission();s=worker.read_text();methods='\n\n'.join(inspect.getsource(f) for f in (operational_docking_failure,operator_request,handoff_result))
    s=replace_one(s,'def main():',methods+'\n\ndef main():')
    s=replace_one(s,'    n.create_timer(.03,publish)',"""    from rclpy.qos import QoSProfile,DurabilityPolicy
    operator_pub=n.create_publisher(String,'/r10/operator_attention',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    handoff=None
    def persist_handoff(message):
        tmp=out/'operator_handoff.next';tmp.write_text(json.dumps(message,indent=2)+'\\n');tmp.replace(out/'operator_handoff.json')
        operator_pub.publish(String(data=json.dumps(message)))
    n.create_timer(.03,publish)""")
    s=replace_one(s,"pose=[p.position.x,p.position.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))])","pose=[p.position.x,p.position.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))],v=m.twist.twist.linear.x,w=m.twist.twist.angular.z)")
    s=replace_one(s,"    except BaseException as ex:error=repr(ex);log('TASK_STOP',error=error)","""    except BaseException as ex:
        error=repr(ex);log('TASK_STOP',error=error)
        if operational_docking_failure(str(ex)):
            handoff=operator_request(points[st['index']],events,active_tool is not None,completed,len(allpoints),cfg['version'],str(ex),st['t'],spec['session'])
            st.update(phase='STOPPING',nav_idle=active_nav is None,precision_translation_m=None)
            handoff['selected_alternatives_after_point']=[q['id'] for q in points[st['index']+1:]]
            handoff['precision_dock_evidence']=exhausted_dock(events,points[st['index']]['id'],cfg['docking_realign_max_attempts'])
            handoff['defer_denials']=[e for e in events if e['event']=='CANDIDATE_DEFER_DENIED' and e['t']>=handoff['requested_t']-2.]
            persist_handoff(handoff);log('HUMAN_ASSISTANCE_REQUESTED',point=handoff['point'],reason=handoff['reason'],work_credit=False)""")
    s=replace_one(s,"log('FINAL_TOOL_STOP',state=fin.result().result.state)","log('FINAL_TOOL_STOP',state=fin.result().result.state,action_status=fin.result().status)")
    s=replace_one(s,"        (out/'mission_result.json').write_text", """        if handoff is not None:
            limits=dict(max_v=cfg['vehicle']['safety']['stopped_v_mps'],max_w=cfg['vehicle']['safety']['stopped_w_rad_s'],max_feedback_age_s=toolcfg['feedback_age_sim_s'],max_tool_rpm_rad_s=toolcfg['stopped_max_rad_s'],max_ack_age_s=toolcfg['stop_timeout_s'],tool_revision=toolcfg['tool_revision'])
            closure=handoff_result(dict(t=st['t'],request=handoff,gate=st['gate'],odometry=local_odom,feedback=st['feedback'],events=events,limits=limits,mission_passed=final['passed'],actions_idle=active_nav is None and active_tool is None))
            handoff['shutdown']=closure;final.update(passed=False,terminal='HUMAN_REQUIRED',operator_handoff=handoff,control_process_exit=closure['process_exit'])
            persist_handoff(handoff);log('HUMAN_HANDOFF_CLOSED',point=handoff['point'],verified=closure['shutdown_verified'],process_exit=closure['process_exit'],mission_passed=False)
        (out/'mission_result.json').write_text""")
    s=replace_one(s,"    gate=final.get('final_gate') or {}\n    if error is None", "    if handoff is not None:return handoff['shutdown']['process_exit']\n    gate=final.get('final_gate') or {}\n    if error is None")
    out=worker.parent.parent/'spray_handoff_candidate'/'mission_spray_handoff.py';out.parent.mkdir(parents=True,exist_ok=True);compile(s,str(out),'exec');out.write_text(s);return out


def run_candidate(seconds,rtf,natural):
    import importlib.util
    from spray_defer_fixture_r10 import EXPERIMENTS
    fixture_path=Path(__file__).with_name('spray_defer_fixture_r10.py')
    spec=importlib.util.spec_from_file_location('isolated_handoff_fixture',fixture_path)
    fixture=importlib.util.module_from_spec(spec);spec.loader.exec_module(fixture)
    fixture.EXPERIMENTS=EXPERIMENTS.parent/'spray_operator_handoff'
    fixture.FIXTURE_SOURCES=[*fixture.FIXTURE_SOURCES,Path(__file__).resolve()]
    worker=build();case=fixture.build_runtime(worker,obstacle=not natural)(seconds=seconds,rtf=rtf)
    receipt=json.loads((case/'mission_result.json').read_text()) if (case/'mission_result.json').is_file() else {}
    runtime=json.loads((case/'runtime.json').read_text());handoff=receipt.get('operator_handoff') or {}
    observer=next((p.get('exit') for p in runtime.get('processes',[]) if p['role']=='observer'),None)
    status='NOT_EXERCISED'
    if handoff:
        verified=handoff.get('shutdown',{}).get('shutdown_verified') and observer==0 and receipt.get('passed') is False and runtime.get('passed')
        status='CONTROLLED_MANUAL_HOLD' if verified else 'UNVERIFIED_SHUTDOWN'
    from analyze_spray_r10 import analyze as evaluate
    from common import sha,dump
    quantitative=evaluate(case);checks=quantitative['checks'];selected=fixture.FOUR
    original_count=receipt.get('original_points');jobkeys=[key.split(':')[-1] for key in (j['key'] for j in quantitative.get('jobs',[]))]
    essential=('runtime_clean','map_database_unchanged','no_unexpected_contact','sensor_render_configuration_valid','four_real_near_sensors','map_pose_rmse_under_15cm','measured_final_stop','unique_actuator_owner','unique_tool_owner','unique_transform_owners','implementation_bound')
    closed_camera=json.loads((case/'video/recording.json').read_text()) if (case/'video/recording.json').is_file() else {}
    frames=[json.loads(l) for l in (case/'video/frames.jsonl').read_text().splitlines()] if (case/'video/frames.jsonl').is_file() else []
    gates=[json.loads(l) for l in (case/'control/gate.jsonl').read_text().splitlines()] if (case/'control/gate.jsonl').is_file() else []
    first_drive=next((g['t'] for g in gates if g['mode']=='DRIVE' and abs(g['v'])+abs(g['w'])>.003),None);end=receipt.get('t',1e20)
    active=[f for f in frames if (first_drive or 0)<=f['sim_t']<=end];gap=max((b['sim_t']-a['sim_t'] for a,b in zip(active,active[1:])),default=1e9);age=max((f['image_wall_age_s'] for f in active),default=1e9)
    camera_ok=bool(closed_camera.get('valid') and closed_camera.get('source')=='actual Gazebo camera' and closed_camera.get('encoder_exit')==0 and first_drive is not None and len(frames)==closed_camera['frames'] and frames[0]['sim_t']<=first_drive and frames[-1]['sim_t']>=end-.5 and gap<=1.0 and age<=2.5)
    four_pass=bool(receipt.get('passed') and original_count==56 and receipt.get('selected_points')==4 and set(selected)==set(jobkeys) and checks['feedback_and_independent_physics_agree'] and checks['idempotent_replay_no_extra_flow'] and all(checks[k] for k in essential) and camera_ok)
    result=dict(schema='SPRAY-OPERATOR-HANDOFF-LIVE/1',path=str(case),session=case.name,handoff_status=status,separately_confirmed_four_points=four_pass,observer_exit=observer,mission_passed=receipt.get('passed'),runtime_clean=runtime.get('passed'),
        selected=selected,completed=[j['point'] for j in receipt.get('completed',[])],original=original_count,formal_credit=False,manual_handoff=handoff or None,mission_error=receipt.get('error'),
        quantitative_safety={k:checks.get(k) for k in essential},map_xy_rmse_m=quantitative.get('map_xy_rmse_m'),camera_valid=camera_ok,max_active_gap_sim_s=gap,max_active_image_age_wall_s=age,
        actual_jobs=quantitative.get('jobs',[]),received_tool_and_physics_for_every_point=checks.get('feedback_and_independent_physics_agree'),
        source_sha256={n:sha(case/n) for n in ('spec.json','mission.json','mission_result.json','operator_handoff.json','runtime.json','mission_events.jsonl','video/full.mp4','video/recording.json') if (case/n).is_file()})
    dump(case/'candidate_handoff_acceptance.json',result)
    print(json.dumps({k:result[k] for k in ('path','handoff_status','separately_confirmed_four_points','observer_exit','mission_passed')}))
    return 0 if status=='CONTROLLED_MANUAL_HOLD' or four_pass else 2


def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('mode',choices=('prepare','decision','run'));ap.add_argument('--seconds',type=int,default=330);ap.add_argument('--rtf',type=float,default=.55);ap.add_argument('--natural',action='store_true');a=ap.parse_args()
    if a.mode=='prepare':print(build());return 0
    if a.mode=='run':return run_candidate(a.seconds,a.rtf,a.natural)
    d=handoff_result(json.load(sys.stdin));print(json.dumps(d));return d['process_exit']

if __name__=='__main__':raise SystemExit(main())
