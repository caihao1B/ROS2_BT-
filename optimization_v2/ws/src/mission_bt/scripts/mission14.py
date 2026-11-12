"""Single mission/action owner. Planned geometry never earns work credit."""
from common14 import *
from collections import deque
import copy,subprocess
import rclpy
from rclpy.action import ActionClient
from std_msgs.msg import String
from geometry_msgs.msg import Point32
from mission_interfaces.action import ToolTask
from mission_interfaces.srv import LedgerCommand
from shapely.geometry import Polygon,shape,mapping
from shapely.affinity import translate,rotate
from shapely.ops import unary_union
from accounting14 import confirmed_points
from admission_recovery14 import AdmissionRecovery, observation_interruption

class Suspended(RuntimeError):pass

class Mission:
    def __init__(self,a,st,node,api):
        self.a,self.st,self.n,self.api=a,st,node,api
        self.c=read(a.out.parent/'scene/tool.json');self.p=cluster(a.vehicle);self.scope='FULL_CLUSTER' if a.case.startswith('nominal_') else 'SMOKE'
        self.directive={};self.last_directive=time.monotonic();self.busy=False;self.stopping=False;self.initialized=False
        self.working=False;self.key='';self.segment='TRANSIT';self.segment_id=''
        self.replans=0;self.deferrals=0;self.patches=0;self.blocked=False;self.unsafe=False;self.complete=False
        policy=yaml.safe_load((BASE/'config/decision_policy.yaml').read_text())
        self.admission_recovery=AdmissionRecovery(policy['clear_hold_s'],policy['max_block_s'])
        self.platform_ok=False
        self.tool_handle=None;self.tool_result=None;self.last_pub=0.;self.index=0;self.done=[];self.deferred=[];self.tool_results=[];self.error=None
        self.client=ActionClient(node,ToolTask,'/s10/tool_task');self.ledger=node.create_client(LedgerCommand,'/s10/ledger/command')
        self.context=node.create_publisher(String,'/s14/context',5);self.intent=node.create_publisher(String,'/s10/segment',5)
        self.fault=node.create_publisher(String,'/s10/test/fault',5)
        node.create_subscription(String,'/s14/directive',self.receive,5)
        node.create_subscription(String,'/s10/ledger/state',lambda m:st.update(ledger=json.loads(m.data)),10)
        node.create_subscription(String,'/s14/tree',lambda m:self.tree_log.write(m.data+'\n') if not self.tree_log.closed else None,5)
        self.tree_log=(a.out/'tree.jsonl').open('w')
        self.jobs=self.make_jobs();self.pending=deque(self.jobs)
        self.last_dispatch_token=None
        self.patch_process=None;self.patch_log=None;self.patch_output=None
        self.area=translate(shape(self.p['coverage_check']['covered']),xoff=-28,yoff=-14) if a.vehicle=='ackermann' else Polygon()
        if a.vehicle=='ackermann' and abs(self.area.area-self.p['coverage_check']['domain_area_m2'])>1e-6:raise ValueError('ORIGINAL_DOMAIN_REQUIRED_NO_DENOMINATOR_SHRINK')
        if a.case=='mow_smoke':
            from shapely.geometry import box
            self.area=translate(rotate(box(.75,-.25,2.25,.25),.15,origin=(0,0),use_radians=True),xoff=2.5,yoff=1.3)
        if a.case in ['ackermann_navigation_probe','ackermann_registered_route_probe']:self.area=Polygon()
        self.credit=Polygon();self.reason='NOT_STARTED';self.patch_candidate=None;self.patch_checked=False;self.executing_action='';self.platform_bad_since=None;self.injected=False;self.last_audit=-100.;self.planning=False
        dump(a.out/'obligations.json',dict(plan_revision=plan()['plan_revision'],cluster=self.p['zone']+'1',jobs=self.jobs,points=self.p.get('points',[]) if self.scope=='FULL_CLUSTER' else [],required_area_m2=self.area.area,scope=self.scope))

    def make_jobs(self):
        if self.a.case=='ackermann_navigation_probe':
            return [self.transit_job(self.p['segments'][4],'probe_B1_4')]
        if self.a.case=='ackermann_registered_route_probe':
            from binding14 import execution_cluster
            s=max((s for s in execution_cluster(self.a.vehicle)['segments'] if not s['tool_on']),key=lambda s:s['timing']['motion_seconds'])
            return [self.transit_job(s,'probe_registered_B1_transit',reference=True)]
        if self.a.case=='mow_smoke':
            route=[[2.5+d*math.cos(.15),1.3+d*math.sin(.15),.15] for d in np.linspace(0,2.,51)]
            return [dict(id='smoke_mow',kind='COVERAGE',poses=route,target=[0,0],task_id='smoke_mow',planned_s=21.,steering_events=[])]
        if self.a.case in ['decision_story','tool_loss','persistent_block','budget_exhaustion']:
            jobs=[]
            for i,d in enumerate([0.,.6]):
                x,y=2.5+d*math.cos(.15),1.3+d*math.sin(.15)
                jobs.append(dict(id=f'smoke_point_{i}',kind='POINT_SERVICE',poses=[[x,y,.15]],target=[x+.42*math.cos(.15),y+.42*math.sin(.15)],task_id=f'smoke_point_{i}',planned_s=10.,steering_events=[]))
            return jobs
        jobs=[]
        from binding14 import execution_cluster
        execution=execution_cluster(self.a.vehicle)
        for i,s in enumerate(execution['segments']):
            if self.a.vehicle=='diff' and s['kind']!='POINT_SERVICE':continue
            if self.a.vehicle=='ackermann' and s['kind'] not in ['COVERAGE','HEADLAND_WORK']:
                jobs.append(self.transit_job(s,f'B1_{i}',reference=bool(execution.get('execution_route_revision'))))
            else:jobs.extend(self.segment_jobs(s,f'{self.p["zone"]}1_{i}'))
        if self.a.case=='spray_registered_docking_probe':return [j for j in jobs if j['id'] in ['A1_47','A1_51']]
        if self.a.vehicle=='diff':jobs.append(dict(id='A1_return',kind='TRANSIT',poses=[[self.p['end'][0]-28,self.p['end'][1]-14,self.p['end'][2]]],planned_s=20,task_id='return',steering_events=[]))
        return jobs

    def transit_job(self,s,label,reference=False):
        job=dict(id=label,kind=s['kind'],poses=[[p[0]-28,p[1]-14,p[2]] for p in s['poses']],task_id=label,planned_s=s['timing']['motion_seconds'],steering_events=s['timing'].get('steering_events',[]),route_policy='QUALIFIED_REFERENCE' if reference else 'ORIGINAL_ENDPOINT_HYBRID_ASTAR_ON_OBSERVED_COSTMAP')
        if reference:job['reference_parts']=self.segment_jobs(s,label+'_reference')
        return job

    def segment_jobs(self,s,label):
        poses=s['poses'];cuts={0,len(poses)-1}
        if self.a.vehicle=='ackermann':
            cuts.update(e['index'] for e in s['timing'].get('steering_events',[]))
            arc=0.
            for i in range(1,len(poses)):
                arc+=math.dist(poses[i-1][:2],poses[i][:2])
                if arc>=8.:cuts.add(i);arc=0.
        cuts=sorted(cuts);parts=list(zip(cuts,cuts[1:])) if len(cuts)>1 else [(0,0)]
        return [dict(id=label+(f'_{n}' if len(parts)>1 else ''),kind=s['kind'],poses=[[p[0]-28,p[1]-14,p[2]] for p in poses[a:b+1]],target=[v-o for v,o in zip(s.get('target_xy',[0,0]),[28,14])],task_id=s.get('task_id',label),planned_s=s['timing'].get('motion_seconds',0)/len(parts),steering_events=s['timing'].get('steering_events',[])) for n,(a,b) in enumerate(parts)]

    def receive(self,m):
        self.directive=json.loads(m.data);self.last_directive=time.monotonic()

    def claim_directive(self,d,j):
        e=d.get('evidence',{})
        if e.get('obligation')!=j['id'] or e.get('completed_jobs')!=len(self.done) or e.get('replans')!=self.replans or e.get('deferrals')!=self.deferrals:return False
        if e.get('blocked')!=self.blocked:return False
        token=(d.get('id'),j['id'],len(self.done),self.replans,self.deferrals,self.patches)
        if token==self.last_dispatch_token:return False
        self.last_dispatch_token=token;return True

    def log(self,event,**kw):self.api['log'](event,**kw)
    def lease_fields(self):return dict(work=self.working,tool_key=self.key,segment=self.segment)
    def work_permission(self):
        t=self.st.get('tool') or {}
        return self.working and t.get('key')==self.key and t.get('active') and t.get('working_confirmed') and t.get('tool_on_requested')

    def pulse(self):
        now=time.monotonic();t=self.st['t'];tool=self.st.get('tool') or {};g=self.st.get('global_') or {};p=self.st.get('perception') or {};h=self.st.get('health') or {}
        platform_ok=g.get('navigation_allowed') and not g.get('plan_revalidation_required') and p.get('geometry_fresh') and p.get('boundary_valid') and p.get('max_speed',0)>0 and h.get('max_speed',0)>0
        self.platform_ok=bool(platform_ok)
        if self.admission_recovery.since is not None:
            qualified=bool(platform_ok and tool.get('stopped_confirmed') and not tool.get('active') and self.st.get('pose') is not None and self.st.get('odom') is not None and self.api['stopped']() and self.api['costmaps_ready']())
            recovered=self.admission_recovery.update(t,qualified)
            if recovered=='RESTORED':
                self.blocked=False
                self.log('observation_admission_restored',resumes=self.admission_recovery.resumes,clear_hold_s=self.admission_recovery.clear_hold_s,original_obligation=self.pending[0]['id'] if self.pending else None,no_work_credit=True)
        if platform_ok:self.platform_bad_since=None
        elif self.platform_bad_since is None:self.platform_bad_since=t
        persistent_loss=self.platform_bad_since is not None and t-self.platform_bad_since>1.
        if self.working and self.tool_result is not None and self.tool_result.done():
            self.working=False;self.segment='TRANSIT'
        if self.initialized and t-self.last_audit>=.5 and not self.stopping:
            self.last_audit=t;owners={topic:[x.node_name for x in self.n.get_publishers_info_by_topic(topic)] for topic in ['/s10/tool/command','/s10/tool_task/_action/status','/s14/directive','/s13/lease']}
            valid=all(owners[t]==[name] for t,name in [('/s10/tool/command','s10_tool_action_server'),('/s10/tool_task/_action/status','s10_tool_action_server'),('/s14/directive','s14_mission_bt'),('/s13/lease','s13_action_client')])
            self.log('mission_ownership',owners=owners,valid=valid,tool_active=tool.get('active',False))
            if not valid:self.unsafe=True
        if not self.injected and self.a.case=='tool_loss' and (self.st.get('ledger') or {}).get('confirmed_quantity',0)>.3:
            self.injected=True;self.fault.publish(String(data=json.dumps(dict(mode='DROP_FEEDBACK',until=t+5.))));self.log('fault_injected',mode='DROP_FEEDBACK',duration_s=5.)
        if not self.injected and self.a.case in ['decision_story','persistent_block'] and len(self.done)>=1 and self.st.get('odom') is not None and (self.a.case=='persistent_block' or abs(self.st['odom'].twist.twist.linear.x)>.03):
            self.injected=True;dump(self.a.out/'runtime_request.json',dict(id='observed_source_loss',operation='localization_loss',t=t));self.log('fault_injected',mode='SOURCE_PROCESS_PAUSED',duration_s=90. if self.a.case=='persistent_block' else 5.)
        if now-self.last_pub>.035:
            self.last_pub=now
            self.intent.publish(String(data=json.dumps(dict(t=t,segment=self.segment,segment_id=self.segment_id,move=self.working))))
            next_min=0.;budget_evidence=dict(source='NO_PENDING_OBLIGATION',seconds=0.)
            if self.pending:
                from budget14 import estimate
                budget_evidence=estimate(self.pending[0],self.api['xy']() if self.st.get('pose') is not None else None,t,self.busy,self.st.get('nav_plan'),live_replan=self.executing_action in ['REPLAN','DETOUR']);next_min=budget_evidence['seconds']
            costmap_ready=bool(self.st.get('pose') is not None and self.st.get('odom') is not None and self.api['costmaps_ready']())
            self.context.publish(String(data=json.dumps(dict(admission_recovery=self.admission_recovery.since is not None,admission_recovery_exhausted=self.admission_recovery.exhausted,admission_resumes=self.admission_recovery.resumes,admission_resume_limit=self.admission_recovery.max_resumes,costmaps_ready=costmap_ready,t=t,idle=not self.busy,busy_action=self.executing_action if self.busy else '',initialized=self.initialized,remaining_s=self.a.seconds-t,next_min_s=next_min,budget_estimate=budget_evidence,unsafe=self.unsafe,blocked=self.blocked or (self.initialized and persistent_loss and not self.stopping),platform_valid=bool(platform_ok),risk=0.,replans=self.replans,deferrals=self.deferrals,patches=self.patches,alternative=len(self.pending)>1,complete=self.complete,residual_only=not self.pending and not self.complete and self.patch_checked and not self.planning,patch_valid=self.patch_candidate is not None,patch_s=(self.patch_candidate or {}).get('cost_s',1e6),obligation=self.pending[0]['id'] if self.pending else None,completed_jobs=len(self.done),required_jobs=len(self.jobs),global_reason=g.get('reason'),map_reason=g.get('map_reason'),gnss_reason=g.get('gnss_reason'),innovation_m=g.get('innovation_m'),perception_reason=p.get('reason'),perception_max_speed=p.get('max_speed'),local_odom_max_speed=h.get('max_speed'),tool_phase=tool.get('phase'),tool_stopped=tool.get('stopped_confirmed'),stale=False))))
        if self.busy and not self.stopping:
            if now-self.last_directive>.5:raise Suspended('BT_WATCHDOG')
            if self.directive.get('action') in ['CANCEL','STOP','WAIT','DEFER']:
                raise Suspended(self.directive.get('reason_code','BT_WATCHDOG'))

    def wait(self,pred,sim=20,wall=80):return self.api['wait'](pred,sim,wall)
    def spin(self):self.api['tick']()

    def tool_stop(self):
        self.working=False;self.segment='TRANSIT'
        if not self.client.server_is_ready():return False
        g=ToolTask.Goal();g.operation=3;g.timeout_sim_s=3.
        sf=self.client.send_goal_async(g);self.wait(sf.done,4,12)
        if not sf.result().accepted:return False
        rf=sf.result().get_result_async();self.wait(rf.done,5,15)
        ok=rf.result().result.state=='STOPPED' and not rf.result().result.uncertainty_pending
        if self.tool_result is not None:
            self.wait(self.tool_result.done,4,12);self.collect_tool()
        self.key='';return ok

    def halt(self):
        self.stopping=True
        try:
            self.cancel_patch()
            self.api['cancel']();ok=self.tool_stop();self.wait(self.api['stopped'],4,12)
            if not ok:self.unsafe=True
            return ok
        finally:self.busy=False;self.stopping=False;self.save()

    def cancel_patch(self):
        if self.patch_process is not None:
            if self.patch_process.poll() is None:
                self.patch_process.terminate()
                try:self.patch_process.wait(timeout=.5)
                except subprocess.TimeoutExpired:self.patch_process.kill();self.patch_process.wait(timeout=.5)
            self.patch_process=None
        if self.patch_log is not None:self.patch_log.close();self.patch_log=None
        self.planning=False

    def start_patch(self):
        self.patch_checked=True;self.planning=True;p=self.st['pose'].pose.pose
        directory=self.a.out/'patches';directory.mkdir(exist_ok=True);request=directory/f'request_{self.patches}.json';self.patch_output=directory/f'result_{self.patches}.json'
        from binding14 import execution_cluster
        dump(request,dict(plan_revision=self.c['plan_revision'],cluster=execution_cluster(self.a.vehicle),residual=mapping(self.area.difference(self.credit)),pose_map=[*self.api['xy'](),yaw(p.orientation)],out=str(directory),attempt=self.patches))
        self.patch_log=(directory/f'worker_{self.patches}.log').open('w')
        self.patch_process=subprocess.Popen([sys.executable,str(PKG/'scripts/residual14.py'),str(request),str(self.patch_output)],stdout=self.patch_log,stderr=subprocess.STDOUT)
        self.patch_deadline=time.monotonic()+40.
        self.log('patch_planning_started',pid=self.patch_process.pid)

    def poll_patch(self):
        if self.patch_process is not None and time.monotonic()>self.patch_deadline:
            self.cancel_patch();self.log('patch_planning_failed',reason='WALL_BUDGET_40S');return
        if self.patch_process is None or self.patch_process.poll() is None:return
        code=self.patch_process.returncode;self.cancel_patch()
        if code: self.log('patch_planning_failed',exit_code=code);return
        result=read(self.patch_output)
        if result['plan_revision']!=self.c['plan_revision']:raise ValueError('PATCH_PLAN_VERSION_CHANGED')
        self.patch_candidate=result['candidate'];self.log('patch_candidate',candidate=self.patch_candidate)

    def tool_goal(self,j):
        g=ToolTask.Goal();g.operation=1 if self.a.vehicle=='diff' else 2;g.task_id=j['task_id'];g.idempotency_key=self.c['plan_revision']+':'+j['id'];g.segment_id=j['id']
        for k in ['map_id','map_revision','frame_epoch','plan_revision','tool_revision']:setattr(g,k,self.c[k])
        g.target.header.frame_id='map';g.target.pose.orientation.w=1.;g.timeout_sim_s=150.
        if g.operation==1:g.target.pose.position.x=float(j['target'][0]);g.target.pose.position.y=float(j['target'][1]);g.requested_quantity=2.
        else:
            g.target_area.points=[Point32(x=float(np.float32(x)),y=float(np.float32(y)),z=0.) for x,y in list(self.area.exterior.coords)[:-1]]
            transport_area=Polygon([(p.x,p.y) for p in g.target_area.points]).area
            if abs(transport_area-self.area.area)>1e-3:raise ValueError('TOOL_AREA_TRANSPORT_PRECISION')
            g.requested_quantity=transport_area
        return g

    def start_tool(self,j):
        self.segment='COVERAGE' if self.a.vehicle=='ackermann' else 'POINT_SERVICE';self.segment_id=j['id'];g=self.tool_goal(j);self.key=g.idempotency_key
        self.api['delay'](.2);sf=self.client.send_goal_async(g);self.wait(sf.done,4,12)
        if not sf.result().accepted:raise RuntimeError('TOOL_GOAL_REJECTED')
        self.tool_handle=sf.result();self.tool_result=self.tool_handle.get_result_async();self.log('tool_goal',id=j['id'],key=self.key,target=j.get('target'),operation=g.operation)

    def collect_tool(self):
        if self.tool_result is None or not self.tool_result.done():return
        r=self.tool_result.result().result
        record=dict(key=self.key,state=r.state,reason=r.reason,quantity=r.confirmed_quantity,remaining=r.remaining_quantity,uncertain=r.uncertainty_pending,residual=json.loads(r.residual_geojson))
        self.tool_results.append(record);self.log('tool_result',**record)
        req=LedgerCommand.Request();req.request_json=json.dumps(dict(op='snapshot',key=self.key));f=self.ledger.call_async(req);self.wait(f.done,4,12);snap=json.loads(f.result().response_json)
        if snap.get('credit_geojson'):self.credit=unary_union([self.credit,shape(snap['credit_geojson'])]).intersection(self.area)
        self.tool_result=None;self.tool_handle=None
        if r.uncertainty_pending:self.unsafe=True;raise RuntimeError('UNCERTAIN_WORK_MANUAL_RECONCILIATION_REQUIRED')
        return record

    def execute_job(self,j,replan=False):
        self.api['revalidate']()
        if j['kind'] in ['POINT_SERVICE','TRANSIT'] and self.a.vehicle=='diff':
            pose=j['poses'][-1];route=[[*self.api['xy'](),pose[2]],pose]
            result=self.api['follow'](j['id'],route,nav=True)
            if result['status']!=4:raise RuntimeError('NAV_FAILED '+str(result.get('error_code')))
            if j['kind']=='POINT_SERVICE':
                self.start_tool(j);self.wait(self.tool_result.done,30,90);r=self.collect_tool()
                if r['state']!='SUCCEEDED':raise RuntimeError('POINT_NOT_CONFIRMED '+r['reason'])
            return
        route=j['poses'];work=j['kind'] in ['COVERAGE','HEADLAND_WORK']
        if self.a.vehicle=='ackermann' and not work:
            if j.get('route_policy')=='QUALIFIED_REFERENCE' and not replan:
                self.log('navigation_policy',id=j['id'],action='FollowPath',controller='RPP',reason='MEASURED_REFERENCE_VALIDATED_STOP_AT_CURVATURE_TRANSITIONS',parts=len(j['reference_parts']))
                for part in j['reference_parts']:
                    poses=part['poses'];dh=(poses[1][2]-poses[0][2]+math.pi)%(2*math.pi)-math.pi
                    self.api['align'](float(np.clip(dh/max(.001,math.dist(poses[0][:2],poses[1][:2])),-1/2.11,1/2.11)))
                    result=self.api['follow'](part['id'],poses,nav=False)
                    if result['status']!=4:raise RuntimeError('REFERENCE_TRANSIT_FAILED '+str(result.get('error_code')))
                    self.log('reference_part_complete',id=part['id'],original_endpoint=j['poses'][-1])
                return
            self.log('navigation_policy',id=j['id'],action='NavigateToPose',planner='SmacPlannerHybrid',controller='RPP',reason='NON_WORK_CONNECTOR_REQUIRES_LIVE_COLLISION_AND_CURVATURE_PLAN',original_endpoint=route[-1],minimum_turning_radius_m=2.11)
            self.api['align'](0.)
            result=self.api['follow'](j['id'],route,nav=True)
            if result['status']!=4:raise RuntimeError('NAV_FAILED '+str(result.get('error_code')))
            return
        if work and math.dist(self.api['xy'](),route[0][:2])>.25:
            raise RuntimeError('WORK_ENTRY_NOT_REACHED_TOOL_REMAINS_OFF')
        if self.a.vehicle=='ackermann' and len(route)>1:
            dh=(route[1][2]-route[0][2]+math.pi)%(2*math.pi)-math.pi
            self.api['align'](float(np.clip(dh/max(.001,math.dist(route[0][:2],route[1][:2])),-1/2.11,1/2.11)))
        if work:
            self.start_tool(j);self.wait(lambda:(self.st.get('tool') or {}).get('working_confirmed') or self.tool_result.done(),10,35)
            if self.tool_result.done():raise RuntimeError('MOWER_START_FAILED')
            self.working=True
        result=self.api['follow'](j['id'],route,nav=replan and not work)
        if work:
            self.stopping=True
            try:self.tool_stop()
            finally:self.stopping=False
        if result['status']!=4:raise RuntimeError('NAV_FAILED '+str(result.get('error_code')))

    def save(self):
        points=[j for j in self.jobs if j['kind']=='POINT_SERVICE'];point_account=confirmed_points(self.jobs,self.tool_results,self.c['plan_revision']);count=point_account['confirmed']
        residual=self.area.difference(self.credit)
        data=dict(t=self.st['t'],state='SUCCEEDED' if self.complete else 'PARTIAL',reason=self.reason,case=self.a.case,scope=self.scope,cluster=self.p['zone']+'1',plan_revision=self.c['plan_revision'],required_points=len(points),confirmed_points=count,remaining_point_keys=point_account['remaining'],required_area_m2=self.area.area,confirmed_area_m2=self.credit.area,coverage_fraction=self.credit.area/self.area.area if self.area.area else None,done=self.done,pending=list(self.pending),deferred=self.deferred,tool_results=self.tool_results,residual=mapping(residual),credit=mapping(self.credit),uncertainty_pending=self.unsafe)
        features=[dict(type='Feature',properties=dict(state='UNCOMPLETED',denominator_m2=self.area.area),geometry=mapping(residual))] if self.area.area else []
        for j in points:
            key=self.c['plan_revision']+':'+j['id']
            if key in point_account['remaining']:features.append(dict(type='Feature',properties=dict(state='UNCOMPLETED',task_id=j['task_id'],key=key,required_quantity=2.),geometry=dict(type='Point',coordinates=j['target'])))
        dump(self.a.out/'mission.json',data);dump(self.a.out/'residual.geojson',dict(type='FeatureCollection',features=features))
        return data

    def run(self):
        self.save()
        while self.st['t']<self.a.seconds-8:
            self.spin();self.poll_patch();d=self.directive;action=d.get('action','WAIT');self.reason=d.get('reason_code','WAITING_FOR_BT')
            if not self.initialized and self.client.server_is_ready() and self.api['admitted']() and self.api['costmaps_ready']():self.initialized=True;self.log('mission_admitted');continue
            if action in ['STOP','COMPLETE']:
                # Initial zero-clock discovery may briefly have no context.
                if self.st['t']<2 and self.reason=='INPUT_STALE':continue
                self.halt();break
            if action in ['WAIT','CANCEL'] or not self.initialized:continue
            if action=='DEFER':
                if self.pending and self.claim_directive(d,self.pending[0]):
                    j=self.pending.popleft();self.deferred.append(j);self.deferrals+=1;self.blocked=False;self.save();self.api['delay'](1.)
                continue
            if not self.pending:
                s=self.save();self.complete=not self.deferred and (len(self.done)==len(self.jobs) if self.a.case in ['ackermann_navigation_probe','ackermann_registered_route_probe'] else (s['confirmed_points']==s['required_points']) if self.a.vehicle=='diff' else s['coverage_fraction']>=.95)
                if not self.complete and self.a.vehicle=='ackermann' and not self.patch_checked:
                    self.start_patch()
                if action=='PATCH' and self.patch_candidate:
                    self.patches+=1
                    for i,segment in enumerate(self.patch_candidate['segments']):self.pending.extend(self.segment_jobs(segment,f'patch_{self.patches}_{i}'))
                    self.patch_candidate=None;self.patch_checked=False
                continue
            j=self.pending[0]
            if action=='REPLAN' and self.replans>=2:continue
            if not self.claim_directive(d,j):continue
            self.busy=True;self.executing_action=action;self.blocked=False
            try:
                if action in ['REPLAN','DETOUR']:self.replans+=1;self.log('replan_attempt',attempt=self.replans,obligation=j['id'])
                self.execute_job(j,replan=action in ['REPLAN','DETOUR']);self.pending.popleft();self.done.append(j['id']);self.blocked=False;self.busy=False;self.save()
                if self.a.case=='decision_story' and len(self.done)>=2:self.reason='SMOKE_LIMIT_NO_FULL_CLUSTER_CLAIM';self.halt();return True
            except (Suspended,RuntimeError,TimeoutError,ValueError) as exc:
                recover_observations=isinstance(exc,Suspended) and observation_interruption(str(exc),self.platform_ok,self.st.get('perception') or {})
                interrupted_t=self.st['t']
                self.log('mission_action_interrupted',obligation=j['id'],error=str(exc),failure_class='OBSERVATION_ADMISSION' if recover_observations else 'ACTION_OR_OBSTRUCTION');self.halt();self.blocked=True
                if recover_observations and not self.unsafe:
                    self.admission_recovery.begin(interrupted_t)
                    self.log('observation_admission_wait',obligation=j['id'],max_wait_s=self.admission_recovery.max_block_s,resumes=self.admission_recovery.resumes,max_resumes=self.admission_recovery.max_resumes)
        stopped=self.halt();saved=self.save()
        # The story's acceptance target is the demonstrated decision/stop chain;
        # its ledger still explicitly says PARTIAL unless all obligations finish.
        story_ok=self.a.case=='decision_story' and saved['confirmed_points']>=1 and self.injected and (self.replans>0 or self.admission_recovery.resumes>0) and stopped
        if story_ok:self.log('smoke_checks_complete',full_cluster_claim=False,ledger_state=saved['state'])
        return self.complete or story_ok

    def shutdown(self):
        self.stopping=True;self.busy=False
        # Final STOP uses the existing discovered client and keeps feedback processing alive.
        old=self.st['stop'];self.st['stop']=False
        try:
            if self.st['t']<self.a.seconds-6:self.halt()
        except BaseException as e:self.error=repr(e)
        finally:self.st['stop']=old;self.save();self.tree_log.close()
