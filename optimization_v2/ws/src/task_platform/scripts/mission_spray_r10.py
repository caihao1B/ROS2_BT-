"""Point-service dispatcher: Nav2 action -> measured stop -> tool action -> ledger."""
import argparse,json,time,signal,math
from progress_spray_r10 import NearGoalProgress
from pathlib import Path
import rclpy
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from nav2_msgs.action import NavigateToPose
from mission_interfaces.action import ToolTask
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String
from rosgraph_msgs.msg import Clock
from std_srvs.srv import Trigger
from docking_r10 import stable_dock

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path,required=True);a=ap.parse_args();out=a.out;spec=json.loads((out/'spec.json').read_text());cfg=json.loads((out/'mission.json').read_text());toolcfg=json.loads((out/'scene/tool.json').read_text());cluster=next(x for x in cfg['clusters'] if x['id']==spec['cluster']);allpoints=cluster['points'];points=allpoints[:spec['point_limit']] if spec['point_limit'] else allpoints
    rclpy.init();n=rclpy.create_node('r07_point_service_task',parameter_overrides=[rclpy.parameter.Parameter('use_sim_time',value=True)]);nav=ActionClient(n,NavigateToPose,'/navigate_to_pose');manager=n.create_client(Trigger,'/lifecycle_manager/is_active');tool=ActionClient(n,ToolTask,'/s10/tool_task');pub=n.create_publisher(String,'/r07/mission',10);hp=n.create_publisher(PoseWithCovarianceStamped,'/s07/rtabmap/initialpose',10)
    st=dict(t=0.,stop=False,phase='STARTUP',nav_idle=True,complete=False,nav_failed=False,replans=0,deferrals=0,alternative=True,bt=None,gate=None,loc=None,tool=None,feedback=None,index=0);events=[];completed=[];failed=[];active_nav=None;active_tool=None;begin=time.monotonic();hinted=False;replay=None;error=None
    def log(event,**d):
        row=dict(t=st['t'],event=event,**d);events.append(row)
        with (out/'mission_events.jsonl').open('a') as f:f.write(json.dumps(row)+'\n')
    n.create_subscription(Clock,'/clock',lambda m:st.update(t=m.clock.sec+m.clock.nanosec*1e-9),qos_profile_sensor_data)
    for topic,key in [('/r06/directive','bt'),('/r07/gate','gate'),('/r07/localization','loc'),('/s10/action/state','tool'),('/s10/tool/feedback','feedback')]:n.create_subscription(String,topic,lambda m,k=key:st.update({k:json.loads(m.data)}),20)
    def publish():
        d={k:st[k] for k in ['t','phase','nav_idle','complete','nav_failed','replans','deferrals','alternative','index']};pub.publish(String(data=json.dumps(d)));tmp=out/'live.tmp';tmp.write_text(json.dumps(dict(d,bt=st['bt'],gate=st['gate'],loc=st['loc'])));tmp.replace(out/'live.json')
    n.create_timer(.03,publish)
    def tick(allow_stop=False):
        nonlocal hinted
        rclpy.spin_once(n,timeout_sec=.01)
        if st['t']>8 and not hinted:
            h=spec['operator_hint'];m=PoseWithCovarianceStamped();m.header.frame_id='map';m.pose.pose.position.x=h[0];m.pose.pose.position.y=h[1];m.pose.pose.orientation.z=math.sin(h[2]/2);m.pose.pose.orientation.w=math.cos(h[2]/2);m.pose.covariance=[1. if i in [0,7,14] else .25 if i in [21,28,35] else 0. for i in range(36)];hp.publish(m);hinted=True;log('COARSE_PUBLIC_PLAN_HINT')
        if not allow_stop and (st['stop'] or st['t']>spec['sim_seconds'] or time.monotonic()-begin>spec['wall_budget']-30):raise TimeoutError('TASK_BOUND_OR_STOP_REQUEST')
    def wait(predicate,seconds=20,allow_stop=False):
        start=st['t'];wall=time.monotonic()
        while not predicate():
            tick(allow_stop)
            if st['t']-start>seconds or time.monotonic()-wall>max(20,seconds*4):raise TimeoutError('WAIT_DEADLINE')
    def result(future,seconds=20):wait(future.done,seconds);return future.result()
    def cancel_nav():
        nonlocal active_nav
        if active_nav is not None:
            fut=active_nav.cancel_goal_async();wait(fut.done,8,True);log('NAV_CANCEL_ACK',accepted=bool(fut.result() and fut.result().goals_canceling));end=active_nav.get_result_async();wait(end.done,8,True);active_nav=None
        st['nav_idle']=True;st['phase']='STOPPING';wait(lambda:bool(st['gate'] and st['gate']['stopped']),4,True)
    def tool_goal(p):
        g=ToolTask.Goal();g.operation=ToolTask.Goal.POINT_SERVICE;g.task_id='R10_'+cluster['id'];g.idempotency_key=cfg['version']+':'+p['id'];g.map_id=cfg['map_id'];g.map_revision=cfg['map_revision'];g.frame_epoch=cfg['frame_epoch'];g.plan_revision=cfg['version'];g.tool_revision=toolcfg['tool_revision'];g.target.header.frame_id='map';g.target.pose.position.x=p['target'][0];g.target.pose.position.y=p['target'][1];g.target.pose.orientation.w=1.;g.requested_quantity=p['quantity'];g.timeout_sim_s=20.;return g
    def settle_and_redock(p,force=False):
        nonlocal active_nav
        for attempt in range(3):
            st['phase']='STOPPING';wait(lambda:bool(st['gate'] and st['gate']['stopped'] and st['gate']['tool_stopped']),6)
            since=st['t'];wait(lambda:st['t']>=since+cfg['docking_settle_s'],4)
            history=[];stable=False;metrics={};observation_start=st['t']
            while st['t']-observation_start<8.:
                tick();loc=st['loc'] or {};gate=st['gate'] or {}
                if loc.get('valid') and -.03<=st['t']-loc.get('t',-100)<.15 and gate.get('stopped') and gate.get('tool_stopped') and 'pose' in loc:
                    if not history or loc['t']>history[-1][0]:history.append((loc['t'],loc['pose'][:2]))
                    history=[x for x in history if loc['t']-x[0]<=2.2];stable,metrics=stable_dock(history,p['target'])
                    if stable:break
                else:history=[]
            loc=st['loc'] or {};distance=math.dist(loc['pose'][:2],p['target']) if loc.get('valid') and 'pose' in loc else float('inf')
            log('DOCKING_SETTLED_CHECK',point=p['id'],distance_m=distance,attempt=attempt,stable=stable,stability=metrics)
            if stable and not force:return
            force=False
            if attempt>=cfg['docking_realign_max_attempts']:raise RuntimeError('DOCKING_REALIGNMENT_EXHAUSTED')
            wait(lambda:bool(st['bt'] and st['bt']['action']=='EXECUTE' and st['loc'] and st['loc']['valid']),12)
            g=NavigateToPose.Goal();g.pose.header.frame_id='map';g.pose.pose.position.x=p['target'][0];g.pose.pose.position.y=p['target'][1];g.pose.pose.orientation.z=math.sin(p['yaw']/2);g.pose.pose.orientation.w=math.cos(p['yaw']/2)
            st.update(phase='NAVIGATING',nav_idle=False);active_nav=result(nav.send_goal_async(g),10)
            if not active_nav.accepted:active_nav=None;st.update(nav_idle=True,phase='STOPPING');raise RuntimeError('REDOCK_NAV_REJECTED')
            log('REDOCK_NAV_GOAL',point=p['id'],distance_m=distance,attempt=attempt);future=active_nav.get_result_async();start=st['t']
            while not future.done():
                tick()
                if (st['bt'] or {}).get('action') in ['CANCEL','STOP','HUMAN','WAIT','REPLAN','DEFER'] or st['t']-start>45:
                    cancel_nav();raise RuntimeError('REDOCK_INTERRUPTED_OR_TIMEOUT')
            if future.result().status!=4:cancel_nav();raise RuntimeError('REDOCK_NAV_FAILED')
            active_nav=None;st.update(nav_idle=True,phase='STOPPING');log('REDOCK_NAV_SUCCESS',point=p['id'])
    def service(p):
        nonlocal active_tool
        wait(lambda:bool(st['gate'] and st['gate']['stopped'] and st['gate']['tool_stopped'] and st['bt'] and st['bt']['action']=='EXECUTE' and st['bt'].get('stopped_confirmation')),8)
        meter_before=(st['feedback'] or {}).get('meter');st['phase']='WORKING';active_tool=result(tool.send_goal_async(tool_goal(p)))
        if not active_tool.accepted:raise RuntimeError('TOOL_GOAL_REJECTED')
        future=active_tool.get_result_async();start=st['t'];cancelled=False
        while not future.done():
            tick();bt=st['bt'] or {}
            if bt.get('action') in ['CANCEL','STOP','HUMAN','WAIT'] and not cancelled:
                log('BT_CANCEL_TOOL',reason=bt.get('reason_code'));active_tool.cancel_goal_async();cancelled=True
            if st['t']-start>25:raise TimeoutError('TOOL_RESULT_DEADLINE')
        final=future.result();active_tool=None;r=final.result;data=dict(point=p['id'],state=r.state,reason=r.reason,confirmed=r.confirmed_quantity,remaining=r.remaining_quantity,uncertainty=r.uncertainty_pending,replayed=r.replayed,meter_before=meter_before,meter_after=(st['feedback'] or {}).get('meter'));log('TOOL_RESULT',**data);st['phase']='STOPPING';wait(lambda:bool(st['gate'] and st['gate']['tool_stopped']),5);return data
    def end(s,f):st['stop']=True
    for s in [signal.SIGINT,signal.SIGTERM]:signal.signal(s,end)
    (out/'observer_ready').write_text('Nav2 and tool action client; no actuator or truth access')
    try:
        wait(lambda:nav.server_is_ready() and tool.server_is_ready() and bool(st['loc'] and st['loc']['valid']) and bool(st['gate'] and st['gate']['tool_stopped']),75)
        wait(manager.service_is_ready,20)
        startup_t=st['t']
        while True:
            answer=result(manager.call_async(Trigger.Request()),5)
            if answer and answer.success:break
            if st['t']-startup_t>35:raise TimeoutError('NAV_LIFECYCLE_NOT_ACTIVE')
            pause=st['t'];wait(lambda:st['t']>pause+.3,2)
        log('NAV_LIFECYCLE_ACTIVE_VERIFIED')
        for index,p in enumerate(points):
            st.update(index=index,alternative=index+1<len(points),nav_failed=False,replans=0)
            succeeded=False
            attempt=0
            while attempt<3:
                st['phase']='READY';wait(lambda:bool(st['bt'] and st['bt']['action']=='EXECUTE'),15)
                g=NavigateToPose.Goal();g.pose.header.frame_id='map';g.pose.pose.position.x=p['target'][0];g.pose.pose.position.y=p['target'][1];g.pose.pose.orientation.z=math.sin(p['yaw']/2);g.pose.pose.orientation.w=math.cos(p['yaw']/2)
                st.update(phase='NAVIGATING',nav_idle=False);active_nav=result(nav.send_goal_async(g),10)
                if not active_nav.accepted:active_nav=None;st.update(nav_idle=True,nav_failed=True,phase='NAV_FAILED');log('NAV_REJECTED',point=p['id']);attempt+=1;continue
                log('NAV_GOAL',point=p['id'],target=p['target'],attempt=attempt);f=active_nav.get_result_async();nav_start=st['t'];interrupted=False;near_progress=NearGoalProgress(cfg['navigation_recovery'])
                while not f.done():
                    tick();bt=st['bt'] or {};action=bt.get('action')
                    if action in ['CANCEL','STOP','HUMAN','REPLAN','DEFER','WAIT'] and st['t']-nav_start>1.:
                        log('BT_INTERRUPTS_NAV',action=action,reason=bt.get('reason_code'));interrupted=True;cancel_nav();break
                    loc=st['loc'] or {};gate=st['gate'] or {};stalled=None
                    if loc.get('pose'):
                        qualified=bool(loc.get('valid') and -.03<=st['t']-loc.get('t',-100)<.15 and action=='EXECUTE')
                        stalled=near_progress.update(loc.get('t',st['t']),loc['pose'],p['target'],qualified,gate.get('v',0.))
                    if stalled:
                        log('NEAR_GOAL_PROGRESS_REPLAN',point=p['id'],attempt=attempt,evidence=stalled);cancel_nav();break
                    if st['t']-nav_start>120:cancel_nav();break
                if f.done() and f.result().status==4:
                    active_nav=None;st.update(nav_idle=True,phase='STOPPING');wait(lambda:bool(st['gate'] and st['gate']['stopped']),5);succeeded=True;log('NAV_SUCCESS',point=p['id']);break
                if active_nav is not None:cancel_nav()
                st.update(nav_failed=not interrupted,phase='OBSERVATION_HOLD' if interrupted else 'NAV_FAILED')
                if not interrupted:log('NAV_TERMINAL_FAILURE',status=f.result().status if f.done() else None,error_code=getattr(f.result().result,'error_code',None) if f.done() else None)
                choices=['EXECUTE','REPLAN','DEFER','HUMAN','STOP'] if interrupted else ['REPLAN','DEFER','HUMAN','STOP']
                wait(lambda:bool(st['bt'] and st['bt']['action'] in choices),50)
                action=st['bt']['action'];log('RETRY_DECISION',action=action,reason=st['bt'].get('reason_code'))
                if action=='EXECUTE':log('RESUME_AFTER_OBSERVATION_CLEAR',point=p['id']);continue
                if action=='REPLAN':attempt+=1;st['replans']+=1;st['nav_failed']=False;wait(lambda:bool(st['bt'] and st['bt']['action']=='EXECUTE'),5)
                elif action=='DEFER':st['deferrals']+=1;break
                else:raise RuntimeError('MANUAL_OR_SAFETY_HOLD '+str(st['bt'].get('reason_code')))
            if not succeeded:failed.append(dict(point=p['id'],reason='NAVIGATION_RESIDUAL'));continue
            settle_and_redock(p)
            r=service(p)
            if r['state']!='SUCCEEDED' and not r['uncertainty']:
                log('RESIDUAL_RETRY',point=p['id']);wait(lambda:bool(st['bt'] and st['bt']['action']=='EXECUTE'),15);wait(lambda:bool(st['tool'] and st['tool']['stopped_confirmed'] and st['feedback'] and st['feedback']['healthy']),8)
                if r['reason']=='NOT_DOCKED':settle_and_redock(p,force=True)
                else:settle_and_redock(p)
                r=service(p)
            if r['state']=='SUCCEEDED':
                completed.append(r)
                if replay is None:replay=service(p);log('IDEMPOTENT_REPLAY',result=replay)
            else:failed.append(r)
            if r['uncertainty']:raise RuntimeError('TOOL_STOP_UNCERTAIN')
        st.update(complete=not failed and len(completed)==len(points),phase='DONE',nav_idle=True);log('SUBSET_FINISHED',selected=len(points),completed=len(completed),original_total=len(allpoints));wait(lambda:bool(st['gate'] and st['gate']['stopped'] and st['gate']['tool_stopped']),5)
        ending=st['t'];wait(lambda:st['t']>ending+2,4)
    except BaseException as ex:error=repr(ex);log('TASK_STOP',error=error)
    finally:
        st['phase']='STOPPING'
        try:
            if active_nav:cancel_nav()
            if active_tool:
                f=active_tool.cancel_goal_async();wait(f.done,4,True)
            if tool.server_is_ready():
                g=ToolTask.Goal();g.operation=ToolTask.Goal.STOP_WORK;f=tool.send_goal_async(g);wait(f.done,4,True);handle=f.result()
                if handle and handle.accepted:fin=handle.get_result_async();wait(fin.done,4,True);log('FINAL_TOOL_STOP',state=fin.result().result.state)
            wait(lambda:bool(st['gate'] and st['gate']['stopped'] and st['gate']['tool_stopped']),5,True)
        except BaseException as ex:log('SHUTDOWN_UNCONFIRMED',error=repr(ex));error=error or repr(ex)
        ownership={topic:[dict(node=i.node_name,namespace=i.node_namespace,gid=list(i.endpoint_gid)) for i in n.get_publishers_info_by_topic(topic)] for topic in ['/tf','/s03/actuator/request','/s10/tool/command']}
        final=dict(ownership=ownership,final_gate=st['gate'],passed=error is None and st['complete'] and bool(replay and replay['replayed']),error=error,cluster=cluster['id'],selected_points=len(points),original_points=len(allpoints),completed=completed,failed=failed,not_selected=[p['id'] for p in allpoints[len(points):]],replay=replay,t=st['t'],wall_seconds=time.monotonic()-begin,scope='Explicitly selected original points; confirmed work and residual separate; full selection does not imply full completion')
        (out/'mission_result.json').write_text(json.dumps(final,indent=2));n.destroy_node();rclpy.try_shutdown()
    return 0 if final['passed'] or (final['final_gate'] and final['final_gate']['stopped'] and final['final_gate']['tool_stopped'] and error and any(k in error for k in ['WAIT_DEADLINE','TASK_BOUND','MANUAL_OR_SAFETY_HOLD'])) else 2
if __name__=='__main__':raise SystemExit(main())
