"""Bounded S12 executor: generate, independently verify, report, then publish offline plan."""
import argparse,datetime,fcntl,signal,traceback,resource,shutil,gzip
from common12 import *
from planning12 import plan_zone
from connector12 import plan_connector
from contracts12 import verify_selected,bind_mission,check_binding,version_bindings
from products12 import export_plan,validate_products
from faults12 import run_faults
from report12 import render
sys.path.insert(0,str(BASE/'ws/src/project_baseline/scripts'))
from session_owner import save_session,stop_session
RESULTS=BASE/'results/S12';SESSION=BASE/'runtime/S12.json'

def sources():return sorted(set([ROOT/p for p in version_bindings()]+files(BASE/'lessons/S12')+files(BASE/'references/S12')))
def snapshot():return {str(p.relative_to(ROOT)):sha(p) for p in sources()}

def command(out,name,args,timeout):
    start=time.monotonic();p=subprocess.Popen(args,cwd=ROOT,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,start_new_session=True)
    try:log,_=p.communicate(timeout=timeout)
    except BaseException:
        os.killpg(p.pid,signal.SIGTERM)
        try:log,_=p.communicate(timeout=5)
        except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);log,_=p.communicate()
        (out/(name+'.log')).write_text(log or '');raise
    (out/(name+'.log')).write_text(log)
    with (out/'commands.jsonl').open('a') as f:f.write(json.dumps(dict(name=name,args=args,exit_code=p.returncode,wall_s=time.monotonic()-start))+'\n')
    if p.returncode:raise RuntimeError(name+' failed; see '+str(out/(name+'.log')))

def compact(out):
    # Keep reconstructible raw candidates compressed; do not touch older stages.
    records=[]
    for folder in ['planning','connectors','faults']:
        for p in (out/folder).rglob('*.json'):
            if not (p.name.endswith('_result.json') or p.name.endswith('_request.json') or p.name=='plan.json'):continue
            if p.stat().st_size<30000:continue
            raw=p.read_bytes();target=p.with_suffix(p.suffix+'.gz')
            with gzip.GzipFile(filename=str(target),mode='wb',mtime=0,compresslevel=1) as f:f.write(raw)
            records.append(dict(original=str(p.relative_to(out)),compressed=str(target.relative_to(out)),uncompressed_sha256=__import__('hashlib').sha256(raw).hexdigest(),original_bytes=len(raw),compressed_bytes=target.stat().st_size));p.unlink()
    dump(out/'compression_manifest.json',records)

def receipt(out,data):
    from docs12 import write_results
    write_results(out,data)
    artifacts=files(out)+files(ROOT/'docs/optimization_v2/S12')+files(BASE/'lessons/S12')
    if data['status']=='PASS':artifacts+=files(BASE/'plans/validated')
    r=dict(schema_version=1,step='S12',plan_revision='20261109.1',run_id=data['run_id'],checked_at_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),automated_status=data['status'],manual_status='NOT_CHECKED',reason=data.get('error','Six bounded, connected offline work plans; original oversized input refused; geometry/kinematics/time, fault cases, C++/Python lessons and browser interactions verified. Online execution NOT_RUN.'),check_exit_code=0 if data['status']=='PASS' else 1,checks=data['checks'],report_path=str((out/'index.html').relative_to(ROOT)),source_files=[row(p) for p in sources()],artifacts=[row(p) for p in sorted(set(artifacts))],dependency_receipts={'S11':sha(BASE/'results/S11/latest.json')})
    dump(RESULTS/'latest.json',r)

def main():
    ap=argparse.ArgumentParser();ap.add_argument('mode',choices=['test','demo','stop']);ap.add_argument('--case',choices=['plan_animation','infeasible_and_stale']);ap.add_argument('--suite',choices=['acceptance']);ap.add_argument('--seed',type=int,default=41);ap.add_argument('--sim-seconds',type=float,default=300);ap.add_argument('--wall-timeout',type=float,default=1800);ap.add_argument('--headless',action='store_true');a=ap.parse_args()
    if a.mode=='stop':
        r=stop_session(SESSION,20);print(json.dumps(r));return 0 if r['status'] in ['STOPPED','NOT_RUNNING'] else 1
    if a.mode=='test' and (a.suite!='acceptance' or a.case):ap.error('test requires acceptance, no case')
    if a.mode=='demo' and (not a.case or a.suite):ap.error('demo requires case, no suite')
    if a.seed!=41:ap.error('this frozen revision requires seed 41')
    if not all(math.isfinite(x) and x>0 for x in [a.sim_seconds,a.wall_timeout]):ap.error('positive finite budgets required')
    lock=(ROOT/'native_20261108/runs/session.lock').open('a')
    try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    except BlockingIOError:print('已有项目会话持有锁');return 2
    run_id=a.mode+'_'+datetime.datetime.now().strftime('%Y%m%d_%H%M%S')+'_'+str(os.getpid());out=RESULTS/run_id;out.mkdir(parents=True)
    if os.getpgrp()!=os.getpid():os.setsid()
    save_session(SESSION,str(Path(__file__).resolve()),run_id);session=read(SESSION);session.update(offline=True,ros_domain=None,gz_partition=None,seed=a.seed,map_id=read(BASE/'maps/prior/map_metadata.json')['map_id']);dump(SESSION,session)
    def interrupted(s,f):
        if s==signal.SIGALRM:raise TimeoutError('S12_WALL_BUDGET')
        raise KeyboardInterrupt
    for s in [signal.SIGALRM,signal.SIGINT,signal.SIGTERM]:signal.signal(s,interrupted)
    signal.setitimer(signal.ITIMER_REAL,max(.1,a.wall_timeout-4));start=time.monotonic();mission=None;faults=None
    data=dict(run_id=run_id,status='PARTIAL',mode=a.mode,case=a.case,seed=a.seed,command=sys.argv,sim_clock='N/A_OFFLINE',requested_sim_budget_not_used=a.sim_seconds,wall_budget_s=a.wall_timeout,checks={f'S12-C{i:02}':'NOT_RUN' for i in range(1,5)},attempts={},next_step_verdict='暂不能执行下一步：本轮正式验收未完成',online_execution='NOT_RUN',manual_status='NOT_CHECKED')
    try:
        command(out,'prerequisite_S11',['bash',str(BASE/'stepctl.sh'),'check','S11'],90);validate_partition_bundle(BASE/'plans/candidate')
        if a.mode=='test':command(out,'build',['bash',str(BASE/'scripts/build_s12.sh')],150)
        command(out,'dependencies',[sys.executable,str(PKG/'tests/dependencies.py'),str(out/'environment_verified.json')],15)
        binding=snapshot();dump(out/'inputs.json',dict(source_hashes=binding,seed=a.seed,dependency_receipts={'S11':sha(BASE/'results/S11/latest.json')}));c=config()
        if c['k']!=3 or c['online_execution_allowed']:raise ValueError('OFFLINE_THREE_CLUSTER_CONTRACT')
        if a.mode=='test':
            command(out,'unit_tests',[sys.executable,str(PKG/'tests/test_planner.py')],60)
            command(out,'lesson_cpp',[str(BASE/'ws/build/coverage_planner/planner_lesson_s12')],15)
            command(out,'lesson_python',[sys.executable,str(BASE/'lessons/S12/lesson.py')],15)
            chosen={}
            for zone in ['A','B']:
                result=plan_zone(zone,c,out/'planning');data['attempts'][zone]=result['attempts'];dump(out/'progress.json',dict(zone=zone,status=result['status'],attempts=data['attempts']))
                if result['selected'] is None:raise ValueError('NO_FEASIBLE_TASK_REVISION '+zone)
                chosen[zone]=result['selected']
            print('Independent geometry, time and coverage recheck',flush=True);data['verification']=verify_selected(chosen,c)
            if not data['verification']['passed']:raise ValueError('INDEPENDENT_RECHECK_FAILED')
            connectors=plan_connector(chosen,c,out/'connectors')
            if not all(p['passed'] for p in connectors):raise ValueError('CONNECTOR_PLAN_FAILED')
            if snapshot()!=binding:raise ValueError('INPUTS_CHANGED_DURING_PLANNING')
            mission=bind_mission(chosen,connectors,c);export_plan(out/'validated',mission);validate_products(out/'validated')
        else:
            command(out,'valid_formal_receipt',['bash',str(BASE/'stepctl.sh'),'check','S12'],90);validate_products(BASE/'plans/validated');mission=read(BASE/'plans/validated/mission_plan.json');check_binding(mission)
            data['replay_of']=read(RESULTS/'latest.json')['run_id'];data['attempts']=read(RESULTS/data['replay_of']/'result.json')['attempts'];data['demo_scope']='version-checked formal plan replay; fault cases rerun; no new formal receipt'
        faults=run_faults(mission,c,out/'faults');data['faults']=faults
        if not faults['passed']:raise ValueError('FAULT_REFUSAL_TEST_FAILED')
        data.update(status='PASS',plan_revision=mission['plan_revision'],task_revision=mission['task_revision'],zones={z:dict(area_m2=d['task_area_m2'],original_area_m2=d['original_area_m2'],scale=d['scale'],point_count=len(d['points']),iteration=d['iteration'],plans=[{k:p[k] for k in ['zone','cluster','vehicle','method','timing','coverage_check']} for p in d['plans']]) for z,d in mission['zones'].items()},connector_metrics=[{k:p[k] for k in ['vehicle','direction','timing','passed','centerline_deviation_m']} for p in mission['connector_routes']])
        if a.mode=='test':data['checks']={f'S12-C{i:02}':'PASS' for i in range(1,5)}
        render(out,data,mission,faults);command(out,'browser_check',[sys.executable,str(PKG/'tests/report_check.py'),str(out/'index.html')],50)
        if snapshot()!=binding:raise ValueError('INPUTS_CHANGED_DURING_RUN')
        check_binding(mission)
        if a.mode=='test':
            target=BASE/'plans/validated';target.mkdir(parents=True,exist_ok=True)
            for p in files(out/'validated'):
                if p.name!='manifest.json':shutil.copy2(p,target/p.name)
            shutil.copy2(out/'validated/manifest.json',target/'manifest.json');validate_products(target)
            data['next_step_verdict']='可以执行下一步 S13——Nav2 控制器与三类导航动作；六簇离线计划及 S08/S09 前置有效。S12 不代表在线任务执行通过；须落实停车转向、定位完整性、反馈账本与 1800 s 退出。'
        else:data['next_step_verdict']='演示已完成；正式状态以 stepctl check S12 与 next S12 为准。'
    except KeyboardInterrupt:data.update(status='PARTIAL',error='用户中断离线规划；无车辆或工具命令，已保存进度')
    except Exception as e:data.update(status='FAIL',error=repr(e));(out/'exception.log').write_text(traceback.format_exc());print(traceback.format_exc(),flush=True)
    finally:
        signal.setitimer(signal.ITIMER_REAL,0)
        try:
            if data['status']!='PASS':data['checks']={f'S12-C{i:02}':'FAIL' if data['status']=='FAIL' else 'NOT_RUN' for i in range(1,5)};data['next_step_verdict']='暂不能执行下一步：本轮验收未完整通过'
            if data['status']=='PASS':compact(out)
            data['wall_seconds']=time.monotonic()-start;data['peak_process_rss_mib']=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/1024
            dump(out/'result.json',data);render(out,data,mission,faults)
            if a.mode=='test':receipt(out,data)
            else:dump(RESULTS/'last_demo.json',dict(run_id=run_id,status=data['status'],report_path=str((out/'index.html').relative_to(ROOT))))
        finally:SESSION.unlink(missing_ok=True);lock.close()
    if a.mode=='demo' and not a.headless:subprocess.Popen(['xdg-open',str(out/'index.html')],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL,start_new_session=True)
    print(json.dumps(dict(status=data['status'],report=str(out/'index.html'),next=data['next_step_verdict']),ensure_ascii=False));return 0 if data['status']=='PASS' else 1
if __name__=='__main__':raise SystemExit(main())
