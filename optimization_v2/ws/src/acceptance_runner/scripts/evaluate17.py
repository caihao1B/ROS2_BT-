"""Independent recorded truth scoring, original denominators, no online feedback."""
from common17 import *
from statistics17 import quantiles
import numpy as np,subprocess

def evaluate(out,spec,monitor):
    out=Path(out);runtime=read(out/'runtime.json');mission=read(out/'client/mission.json') if (out/'client/mission.json').exists() else {}
    client=read(out/'client/result.json') if (out/'client/result.json').exists() else {}
    identity=runtime.get('seed')==spec['seed'] and runtime.get('vehicle')==spec['vehicle'] and client.get('controller')==spec['controller']
    if spec.get('scope')=='FULL_CLUSTER':identity=identity and runtime.get('seconds')==1800 and mission.get('plan_revision')==runtime.get('regional_binding',{}).get('plan_revision')
    truth=rows(out/'evaluation/truth.jsonl');events=rows(out/'client/events.jsonl')
    contacts=read(out/'evaluation/contacts.json') if (out/'evaluation/contacts.json').exists() else None
    tool=runtime.get('tool_shutdown',{}).get('state')=='STOPPED'
    times=[t['t'] for t in truth];physical=rows(out/'tool_evaluation/physical.jsonl')
    last=max(times+[runtime.get('sim_seconds_observed',0)]+[t['t'] for t in physical])
    evidence=dict(clock=times+[last],contacts=contacts,tool_stopped=tool,limit_s=runtime['seconds']);dump(out/'monitor_input.json',evidence)
    run=subprocess.run([str(monitor),str(out/'monitor_input.json'),str(out/'monitor.json')],capture_output=True,text=True,timeout=20)
    measured=read(out/'monitor.json') if (out/'monitor.json').exists() else dict(passed=False,error=run.stderr,exit_code=run.returncode)
    admitted=next((e['t'] for e in events if e['event']=='mission_admitted'),None)
    obs=[r for r in rows(out/'online/observations.jsonl') if r.get('record_type')=='pose' and admitted is not None and r['t']>=admitted and times and times[0]<=r['t']<=times[-1]]
    errors=[];heading=[]
    if obs:
        t=np.array([x['t'] for x in obs]);xy=np.array([x['pose'][:2] for x in obs]);actual=np.array([x['xyz'][:2] for x in truth])-[28,14]
        aligned=np.column_stack([np.interp(t,times,actual[:,i]) for i in range(2)]);errors=np.linalg.norm(xy-aligned,axis=1).tolist()
        q=np.array([x['pose'][3:] for x in obs]);yaw=np.arctan2(2*(q[:,3]*q[:,2]+q[:,0]*q[:,1]),1-2*(q[:,1]**2+q[:,2]**2));actualyaw=np.interp(t,times,np.unwrap([x['rpy'][2] for x in truth]));heading=np.abs(np.arctan2(np.sin(yaw-actualyaw),np.cos(yaw-actualyaw))).tolist()
    plan=read(BASE/'plans/validated/mission_plan.json');c=plan['zones'][spec['cluster'][0]]['plans'][int(spec['cluster'][1])-1]
    required_points=len(c.get('visited',[]));required_area=c.get('coverage_check',{}).get('domain_area_m2',0.)
    full=mission.get('scope')=='FULL_CLUSTER' and mission.get('cluster')==spec['cluster']
    denominator=mission.get('required_points')==required_points and abs(mission.get('required_area_m2',0)-required_area)<1e-6
    complete=full and denominator and mission.get('state')=='SUCCEEDED' and (mission.get('confirmed_points')==required_points if required_points else mission.get('coverage_fraction',0)>=.95)
    inspection=spec.get('operation')=='inspection';inspection_proof=None
    if inspection:
        contract=read(out/'client/inspection_contract17.json') if (out/'client/inspection_contract17.json').exists() else {}
        required_points=0;required_area=0.
        denominator=bool(contract.get('required_jobs')) and mission.get('required_points')==0 and mission.get('required_area_m2')==0
        complete=denominator and mission.get('scope')=='INSPECTION' and mission.get('state')=='SUCCEEDED' and set(mission.get('done',[]))==set(contract.get('required_jobs',[])) and not mission.get('tool_results')
    if inspection:
        from inspection_audit17 import audit
        inspection_proof=audit(contract,truth,admitted,spec['vehicle']);dump(out/'inspection_audit17.json',inspection_proof)
        complete=complete and inspection_proof['passed']
    cleanup=read(out/'cleanup14.json') if (out/'cleanup14.json').exists() else dict(survivors=['UNVERIFIED'])
    sys.path.insert(0,str(BASE/'ws/src/navigation_adapter/scripts'))
    from analysis13 import evaluate as navigation_audit
    navigation=navigation_audit(out)
    audits=[r for r in read(out/'online/result.json').get('audits',[]) if r.get('navigation_active')] if (out/'online/result.json').exists() else []
    owners=bool(audits) and all(r['request']==['s03_test_driver'] and r['actuator']==['s03_drive_bridge'] and r['nav']==['controller_server'] and r['lease']==['s13_action_client'] and sorted(r['tf'])==['s05_local_odometry','s08_global_pose_manager'] for r in audits)
    safety=identity and measured['passed'] and owners and runtime.get('database_unchanged') and not cleanup.get('survivors') and all(navigation[k] for k in ['command_limits_ok','truth_isolated','stop_thresholds_ok'])
    loc=bool(errors) and np.quantile(errors,.95)<=.15 and np.quantile(heading,.95)<=.05
    resources=read(out/'resources.json');rtf=(resources[-1]['t']-resources[0]['t'])/(resources[-1]['wall']-resources[0]['wall']) if len(resources)>1 and resources[-1]['wall']>resources[0]['wall'] else None
    result=dict(case=spec['id'],cluster=spec['cluster'],seed=spec['seed'],status='PASS' if complete and safety and loc else ('PARTIAL' if safety and loc and denominator and not complete else 'FAIL'),complete=complete,safety=safety,localization_pass=bool(loc),original_denominator=denominator,required_points=required_points,confirmed_points=mission.get('confirmed_points'),required_area_m2=required_area,confirmed_area_m2=mission.get('confirmed_area_m2'),coverage_fraction=mission.get('coverage_fraction'),sim_seconds_including_stop=last,wall_seconds=runtime.get('wall_seconds'),rtf=rtf,position_error_m=quantiles(errors),yaw_error_rad=quantiles(heading),monitor=measured,reason=mission.get('reason',runtime.get('error')),peak_rss_bytes=max([r['owned_rss_bytes'] for r in resources],default=None),process_exit_codes={r['role']:r.get('exit_code') for r in runtime.get('commands',[])})
    result['navigation_contract']={k:navigation[k] for k in ['command_limits_ok','truth_isolated','stop_thresholds_ok','max_stop_delay_s','compute_p95_ms','effective_compute_hz_sim','compute_p95_target_met']}
    result['runtime_identity_valid']=identity
    if inspection_proof is not None:result['inspection_trajectory_audit']=inspection_proof
    dump(out/'metrics17.json',result)
    obligation=read(out/'client/obligations.json') if (out/'client/obligations.json').exists() else {}
    remaining=set(mission.get('remaining_point_keys',[]));version=mission.get('plan_revision','')
    credited={r['key'] for r in mission.get('tool_results',[]) if r.get('state')=='SUCCEEDED' and r.get('quantity',0)>=2 and not r.get('uncertain',True)}
    points=[dict(xy=j['target'],id=j['id'],state='CONFIRMED' if version+':'+j['id'] in credited else 'UNCOMPLETED') for j in obligation.get('jobs',[]) if j['kind']=='POINT_SERVICE']
    dump(out/'replay17.json',dict(trace=obs[::max(1,len(obs)//600)],events=[e for e in events if e['event']!='mission_ownership'],mission=mission,points=points,truth_online=False))
    return result
