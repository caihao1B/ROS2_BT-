"""Serial, bounded S14 acceptance. A safe refusal is not a completed cluster."""
from common14 import *
from common11 import files,row
import argparse,datetime,fcntl,signal,subprocess,traceback
from runtime14 import run_case
from analysis14 import evaluate,assess
from report14 import render
import checkpoint14
sys.path.insert(0,str(BASE/'ws/src/project_baseline/scripts'))
from session_owner import save_session,stop_session
RESULTS=BASE/'results/S14';SESSION=BASE/'runtime/S14.json'

def sources():
    return sorted(set(files(PKG)+files(BASE/'lessons/S14')+files(BASE/'stages/S14')+files(BASE/'ws/install/mission_bt')+[BASE/'scripts/build_s14.sh',BASE/'behavior_trees/mission.xml',BASE/'config/decision_policy.yaml',BASE/'config/nominal_missions.yaml',BASE/'config/s14_survey_stations.json',BASE/'contracts/decision_reason_codes.yaml']+files(BASE/'plans/validated')+files(BASE/'plans/s14_registered')+files(BASE/'maps/s14_regions')))

def command(out,name,args,timeout=120):
    start=time.monotonic();r=subprocess.run(args,cwd=ROOT,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,timeout=timeout);(out/(name+'.log')).write_text(r.stdout)
    with (out/'commands.jsonl').open('a') as f:f.write(json.dumps(dict(name=name,args=args,exit_code=r.returncode,wall_s=time.monotonic()-start))+'\n')
    if r.returncode:raise RuntimeError(name+' failed: '+str(out/(name+'.log')))

def receipt(out,data):
    from docs14 import write_results
    write_results(out,data)
    artifacts=files(out)+files(ROOT/'docs/optimization_v2/S14')+files(BASE/'lessons/S14')
    r=dict(schema_version=1,step='S14',plan_revision='20261109.1',run_id=data['run_id'],checked_at_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),automated_status=data['status'],manual_status='NOT_CHECKED',reason=data.get('error','Actual Gazebo, Nav2 and feedback-bound ledger evidence; full-cluster results kept separate.'),check_exit_code=0 if data['status']=='PASS' else 1,checks=data['checks'],report_path=str((out/'index.html').relative_to(ROOT)),source_files=[row(p) for p in sources()],artifacts=[row(p) for p in sorted(set(artifacts))],dependency_receipts={s:sha(BASE/f'results/{s}/latest.json') for s in ['S10','S13']})
    dump(RESULTS/'latest.json',r)

def main():
    ap=argparse.ArgumentParser();ap.add_argument('mode',choices=['demo','test','stop']);ap.add_argument('--case',choices=['decision_story','nominal_spray_cluster','nominal_mow_cluster','ackermann_navigation_probe','ackermann_registered_route_probe','spray_registered_docking_probe']);ap.add_argument('--suite',choices=['acceptance']);ap.add_argument('--seed',type=int,default=41);ap.add_argument('--sim-seconds',type=float,default=1800);ap.add_argument('--wall-timeout',type=float,default=10800);ap.add_argument('--headless',action='store_true');ap.add_argument('--resume-from',action='append',type=Path,default=[]);a=ap.parse_args()
    if a.mode=='stop':
        r=stop_session(SESSION,40);print(json.dumps(r));return 0 if r['status'] in ['STOPPED','NOT_RUNNING'] else 1
    if a.seed!=41 or not all(math.isfinite(v) and v>0 for v in [a.sim_seconds,a.wall_timeout]) or a.sim_seconds>1800:ap.error('seed 41, finite positive budgets, sim <=1800 required')
    if a.mode=='test' and (a.suite!='acceptance' or a.case):ap.error('test requires --suite acceptance')
    if a.mode=='demo' and (not a.case or a.suite):ap.error('demo requires --case')
    lock=(ROOT/'native_20261108/runs/session.lock').open('a')
    try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    except BlockingIOError:print('已有项目仿真占用互斥锁');return 2
    run_id=a.mode+'_'+datetime.datetime.now().strftime('%Y%m%d_%H%M%S')+'_'+str(os.getpid());out=RESULTS/run_id;out.mkdir(parents=True)
    if os.getpgrp()!=os.getpid():os.setsid()
    save_session(SESSION,str(Path(__file__).resolve()),run_id);record=read(SESSION);record.update(ros_domain=119,raw_domain=121,evaluation_domain=120,run_directory=str(out),seed=41);dump(SESSION,record)
    def interrupt(s,f):
        if s==signal.SIGALRM:raise TimeoutError('S14_WALL_BUDGET')
        raise KeyboardInterrupt
    for sig in [signal.SIGINT,signal.SIGTERM,signal.SIGALRM]:signal.signal(sig,interrupt)
    signal.setitimer(signal.ITIMER_REAL,max(.1,a.wall_timeout-5));start=time.monotonic()
    data=dict(run_id=run_id,status='PARTIAL',mode=a.mode,seed=41,cases={},checks={f'S14-C{i:02}':'NOT_RUN' for i in range(1,5)},command=sys.argv,manual_status='NOT_CHECKED')
    try:
        for s in ['S10','S13']:command(out,'prerequisite_'+s,['bash',str(BASE/'stepctl.sh'),'check',s],150)
        plan();command(out,'build',['bash',str(BASE/'scripts/build_s14.sh')],180)
        command(out,'terrain_plane',[str(BASE/'ws/install/mission_bt/lib/mission_bt/terrain_plane_test'),str(PKG/'tests/fixtures/sparse_ground_ring.json')],10)
        command(out,'local_surface',[str(BASE/'ws/install/mission_bt/lib/mission_bt/local_surface_test'),str(PKG/'tests/fixtures/local_surface.json')],10)
        command(out,'risk_evidence',[str(BASE/'ws/install/mission_bt/lib/mission_bt/risk_evidence_test'),str(PKG/'tests/fixtures/stale_risk_evidence.json')],10)
        command(out,'collision_volume',[sys.executable,str(PKG/'tests/collision_volume14.py')],30)
        command(out,'context_clock',[str(BASE/'ws/install/mission_bt/lib/mission_bt/context_clock_test')],10)
        command(out,'path_retention',[str(BASE/'ws/install/mission_bt/lib/mission_bt/path_retention_test')],10)
        cfg=yaml.safe_load((BASE/'config/decision_policy.yaml').read_text());dump(out/'policy.json',cfg)
        command(out,'lesson_build',['c++','-std=c++17','-O2','-I'+str(PKG/'include'),str(BASE/'lessons/S14/lesson.cpp'),'-o',str(out/'lesson_cpp')],60)
        command(out,'lesson_cpp',[str(out/'lesson_cpp'),str(out/'policy.json')],10);command(out,'lesson_python',[sys.executable,str(BASE/'lessons/S14/lesson.py')],10);data['policy_tests_passed']=True
        command(out,'async_protocol',[sys.executable,str(PKG/'tests/async_protocol.py'),str(out/'async_protocol')],30)
        command(out,'dispatch_protocol',[sys.executable,str(PKG/'tests/dispatch_protocol14.py')],30)
        command(out,'admission_recovery',[sys.executable,str(PKG/'tests/admission_recovery14.py')],10)
        command(out,'navigation_contract',[sys.executable,str(PKG/'tests/navigation_contract14.py')],30)
        command(out,'budget_progress',[sys.executable,str(PKG/'tests/budget_progress14.py')],15)
        command(out,'routing_contract',[sys.executable,str(PKG/'tests/routing_contract14.py')],45)
        command(out,'checkpoint_contract',[sys.executable,str(PKG/'tests/checkpoint_contract14.py')],30)
        command(out,'transport_precision',[sys.executable,str(PKG/'tests/transport_precision.py')],30)
        command(out,'ledger_boundary',[sys.executable,str(PKG/'tests/ledger_boundary14.py')],30)
        command(out,'residual_async',[sys.executable,str(PKG/'tests/residual_async14.py')],50)
        command(out,'cleanup_guard',[sys.executable,str(PKG/'tests/cleanup_guard.py'),str(out/'cleanup_guard')],20)
        command(out,'regional_contracts',[sys.executable,str(PKG/'tests/regional_contracts14.py')],45)
        command(out,'dependency_versions',['dpkg-query','-W','ros-jazzy-behaviortree-cpp','ros-jazzy-nav2-controller','ros-jazzy-rtabmap-slam'],10)
        dependencies={x:sha(BASE/f'results/{x}/latest.json') for x in ['S10','S13']};binding={str(p.relative_to(ROOT)):sha(p) for p in sources()};dump(out/'source_snapshot.json',binding);dump(out/'input_bindings.json',dict(plan_revision=plan()['plan_revision'],map_manifest=read(BASE/'maps/frozen/manifest.json'),regional_atlas=read(BASE/'maps/s14_regions/manifest.json'),registered_plan_revision=read(BASE/'plans/s14_registered/mission_plan.json')['plan_revision'],nominal=yaml.safe_load((BASE/'config/nominal_missions.yaml').read_text())))
        if a.mode=='test':
            sys.path.insert(0,str(PKG/'tests'))
            from registered_probe14 import assess as assess_registration
            data['regional_regression']={}
            for spec in ['A','B','fault_A','low_obstacle']:
                probe_out=out/'regional_regression'/spec
                print('S14 REGIONAL '+spec,flush=True)
                run_case(probe_out,'zone_a','regional_probe_'+spec,90 if 'fault' in spec else 35,300 if 'fault' in spec else 150,kind='ackermann' if spec=='B' else 'diff',probe=True,survey=True)
                data['regional_regression'][spec]=assess_registration(probe_out)
                if not data['regional_regression'][spec]['passed']:raise ValueError('REGIONAL_REGRESSION_FAILED '+spec)
        matrix=[('decision_story','diff',300),('mow_smoke','ackermann',180),('nominal_spray_cluster','diff',1800),('nominal_mow_cluster','ackermann',1800),('budget_exhaustion','diff',25),('persistent_block','diff',180),('tool_loss','diff',180)] if a.mode=='test' else [(a.case,'ackermann' if a.case in ['nominal_mow_cluster','ackermann_navigation_probe','ackermann_registered_route_probe'] else 'diff',a.sim_seconds)]
        for case,kind,seconds in matrix:
            reused=checkpoint14.reuse(a.resume_from,out/case,case,binding,dependencies) if a.mode=='test' else None
            if reused:
                result=evaluate(out/case)
                if not checkpoint14.complete(case,result):raise ValueError('CHECKPOINT_REEVALUATION_FAILED '+case)
                with (out/'commands.jsonl').open('a') as f:f.write(json.dumps(dict(name=case,operation='verified_evidence_reuse',source=reused,exit_code=0,ran_simulation_in_this_invocation=False))+'\n')
                result['reused_from']=reused;data['cases'][case]=result;dump(out/'result.json',data);render(out,data);continue
            remaining=a.wall_timeout-(time.monotonic()-start)-30
            if remaining<45:raise TimeoutError('INSUFFICIENT_WALL_BUDGET')
            print('S14 RUN '+case,flush=True);t0=time.monotonic();r=run_case(out/case,'zone_a',case,min(a.sim_seconds,seconds),remaining,41,a.mode=='demo' and not a.headless,kind)
            with (out/'commands.jsonl').open('a') as f:f.write(json.dumps(dict(name=case,call='runtime14.run_case',sim_limit=min(a.sim_seconds,seconds),wall_s=time.monotonic()-t0,exit_code=0 if r.get('success') else 1))+'\n')
            data['cases'][case]=evaluate(out/case)
            if binding!={str(p.relative_to(ROOT)):sha(p) for p in sources()}:raise ValueError('SOURCE_CHANGED_DURING_CASE')
            checkpoint14.save(out/case,case,data['cases'][case],binding,dependencies)
            dump(out/'result.json',data);render(out,data)
        if binding!={str(p.relative_to(ROOT)):sha(p) for p in sources()}:raise ValueError('SOURCE_CHANGED_DURING_RUN')
        if a.mode=='test':assess(data)
        else:data['status']='PASS' if all(c['passed'] for c in data['cases'].values()) else 'PARTIAL'
        if data['status']!='PASS':data['error']='本轮存在未完成义务或未通过检查；安全停车不等于全簇完成。详见各案例定位拒绝原因和固定分母。'
    except KeyboardInterrupt:data.update(status='PARTIAL',error='本轮中断，已请求停车和保存残余。')
    except Exception as e:data.update(status='FAIL',error=repr(e));(out/'exception.log').write_text(traceback.format_exc());print(traceback.format_exc(),flush=True)
    finally:
        signal.setitimer(signal.ITIMER_REAL,0);data['wall_seconds']=time.monotonic()-start;data['next_step_verdict']='可以执行下一步 S15' if a.mode=='test' and data['status']=='PASS' else '暂不能凭本轮结果执行下一步 S15';dump(out/'result.json',data);render(out,data)
        try:
            if a.mode=='test':receipt(out,data)
            else:
                dump(RESULTS/'last_demo.json',dict(run_id=run_id,status=data['status'],report_path=str((out/'index.html').relative_to(ROOT))))
                if not (RESULTS/'latest.json').exists():data['status']='NOT_RUN';receipt(out,data)
        finally:SESSION.unlink(missing_ok=True);lock.close()
    if a.mode=='demo' and not a.headless:subprocess.Popen(['xdg-open',str(out/'index.html')],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL,start_new_session=True)
    print(json.dumps(dict(status=data['status'],report=str(out/'index.html'),next=data['next_step_verdict']),ensure_ascii=False));return 0 if data['status']=='PASS' else 1
if __name__=='__main__':raise SystemExit(main())
