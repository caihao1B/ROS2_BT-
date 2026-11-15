"""Independent post-run tool/pose/contact assessment, never imported online."""
from common import *
from analyze_r02 import rows
import numpy as np,math,sqlite3

def analyze(out):
    runtime=read(out/'runtime.json');spec=read(out/'spec.json');cfg=read(out/'mission.json');mission=read(out/'mission_result.json') if (out/'mission_result.json').exists() else {};checks=dict(runtime_clean=runtime['passed'],map_database_unchanged=runtime['original_db_unchanged'],mission_completed=mission.get('passed',False),qualified_three_points=mission.get('selected_points',0)>=3)
    physical=rows(out/'tool_evaluation/physical.jsonl') if (out/'tool_evaluation/physical.jsonl').exists() else []
    truth=rows(out/'evaluation/truth.jsonl');refs=rows(out/'reference/reference.jsonl') if (out/'reference/reference.jsonl').exists() else [];contacts=read(out/'evaluation/contacts.json')
    unsafe_contacts={k:v for k,v in contacts.items() if 'vehicle::' in k and not ('wheel::' in k and 'ground::body::ground' in k)}
    checks['no_unexpected_contact']=not unsafe_contacts
    server_log=(out/'server.log').read_text();checks['sensor_render_configuration_valid']=not any(x in server_log for x in ['Vertical FOV of internal camera exceeds','XML Attribute[type] in element[noise]'])
    near=rows(out/'near/near.jsonl');checks['four_real_near_sensors']=bool(near) and sum(len(x.get('near_sources',[]))==4 for x in near)/len(near)>.9
    rt=np.array([x['t'] for x in truth]);xy=np.array([x['xyz'][:2] for x in truth]);err=[]
    for row in refs:
        if row.get('valid') and 'pose' in row:
            estimate=np.array(row['pose'][:2])+[28,14];expected=np.array([np.interp(row['odom_t'],rt,xy[:,k]) for k in range(2)]);err.append([row['t'],np.linalg.norm(estimate-expected)])
    rmse=float(np.sqrt(np.mean(np.array(err)[:,1]**2))) if err else None;checks['map_pose_rmse_under_15cm']=rmse is not None and rmse<.15
    db=out/'ledger/ledger.sqlite';jobs=[];accounting=[]
    if db.exists():
        conn=sqlite3.connect('file:'+str(db)+'?mode=ro',uri=True)
        for key,status,q,uncertain,g in conn.execute('SELECT key,status,quantity,uncertain,goal FROM jobs'):
            goal=json.loads(g);actual=[x for x in physical if x['key']==key and x['flow']>.01];dose=0.
            for a,b in zip(physical,physical[1:]):
                if a['key']==key and b['key']==key and 0<b['t']-a['t']<.1:dose+=min(a['flow'],b['flow'])*(b['t']-a['t'])
            errors=[math.dist(np.array(x['tool_xyz'][:2])-[28,14],goal['target'][:2]) for x in actual]
            row=dict(key=key,state=status,confirmed=q,uncertain=bool(uncertain),physical_dose=dose,maximum_physical_docking_error_m=max(errors) if errors else None,requested=goal['requested_quantity']);jobs.append(row);accounting.append(status=='SUCCEEDED' and not uncertain and q>=goal['requested_quantity'] and q<=dose+.10 and errors and max(errors)<.20)
        conn.close()
    checks['feedback_and_independent_physics_agree']=bool(jobs) and len(jobs)==mission.get('selected_points') and all(accounting)
    replay=mission.get('replay') or {};checks['idempotent_replay_no_extra_flow']=bool(replay.get('replayed') and replay.get('meter_before') is not None and abs(replay['meter_after']-replay['meter_before'])<=.01)
    checks['measured_final_stop']=bool(mission.get('final_gate') and mission['final_gate']['stopped'] and mission['final_gate']['tool_stopped']) and bool(physical) and abs(physical[-1]['rpm_rad_s'])<.1 and physical[-1]['flow']==0
    graph=mission.get('ownership',{});checks['unique_actuator_owner']=[x['node'] for x in graph.get('/s03/actuator/request',[])]==['r07_actuator_gate'];checks['unique_tool_owner']=[x['node'] for x in graph.get('/s10/tool/command',[])]==['s10_tool_action_server'];checks['unique_transform_owners']=set(x['node'] for x in graph.get('/tf',[]))=={'s05_local_odometry','s08_global_pose_manager'} and len(graph.get('/tf',[]))==2
    checks['implementation_bound']=bool(spec.get('implementation_bindings')) and all((ROOT/p).is_file() and sha(ROOT/p)==h for p,h in spec.get('implementation_bindings',{}).items());checks['probe_source_bound']=spec.get('online_probe_sha256')==sha(PKG/'scripts/mission_r07.py');checks['runtime_source_bound']=spec.get('runtime_sha256')==sha(PKG/'scripts/runtime_r07.py');checks['task_version_bound']=cfg['version']==read(edition('R07')/'mission.json')['version']
    if spec['case']=='tool_jam':
        events=rows(out/'mission_events.jsonl');checks['real_fault_and_residual_retry']=bool((out/'fault_injection.json').exists() and any(x['event']=='RESIDUAL_RETRY' for x in events))
    result=dict(passed=all(checks.values()),checks={k:bool(v) for k,v in checks.items()},case=spec['case'],cluster=spec['cluster'],map_xy_rmse_m=rmse,jobs=jobs,unexpected_contacts=unsafe_contacts,task_result=mission,wall_seconds=runtime['wall_seconds'],original_obligations_preserved=True,formal_credit=False,scope='Three actual point services, tool feedback accounting, idempotent replay and fault retry; full 56-point cluster still pending R10')
    dump(out/'analysis_r07.json',result);np.savez(out/'errors.npz',map_error=np.asarray(err),truth_xy=xy,truth_t=rt);return result

def figures(paths):
    import matplotlib;matplotlib.use('Agg');import matplotlib.pyplot as plt
    fig,axes=plt.subplots(1,2,figsize=(12,5))
    for case,p in paths.items():
        d=np.load(p/'errors.npz');xy=d['truth_xy'];e=d['map_error'];axes[0].plot(xy[:,0],xy[:,1],label=case)
        if len(e):axes[1].plot(e[:,0],e[:,1],label=case)
    axes[0].axis('equal');axes[0].set(title='Actual point-service vehicle trajectory',xlabel='Site X [m]',ylabel='Site Y [m]');axes[1].set(title='Online map estimate / offline truth evaluation',xlabel='Simulation seconds',ylabel='XY error [m]')
    for ax in axes:ax.grid(alpha=.3);ax.legend()
    fig.tight_layout();p=APP/'results/R07/task.png';fig.savefig(p,dpi=150);plt.close(fig);return[p]
