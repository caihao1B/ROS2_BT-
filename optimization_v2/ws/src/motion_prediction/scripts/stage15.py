"""Actual S15 experiments, failure tests, immutable checkpoints and receipts."""
from common15 import *
import argparse,datetime,time,os,signal,fcntl,subprocess,traceback,shutil
import yaml
add14()
from common11 import files,row
sys.path.insert(0,str(BASE/'ws/src/project_baseline/scripts'))
from session_owner import save_session,stop_session
from runtime15 import run_case
from faults15 import channel_loss
from dataset15 import validate as validate_dataset
from analysis15 import evaluate_case,choose_default
from report15 import render
RESULTS=BASE/'results/S15';SESSION=BASE/'runtime/S15.json'

def sources():
    return sorted(set(files(PKG)+files(GUARD)+files(BASE/'ws/install/motion_prediction')+files(BASE/'ws/install/prediction_risk_guard')+files(BASE/'stages/S15')+files(BASE/'lessons/S15')+[BASE/'scripts/build_s15.sh',BASE/'scripts/install_s15_dependencies.sh',BASE/'contracts/prediction15.yaml']+[BASE/'config'/x for x in ['prediction.yaml','model_lock.yaml','prediction_requirements.lock','prediction_wheels.lock','rviz/prediction.rviz']]))

def command(out,name,args,timeout=120):
    start=time.monotonic();r=subprocess.run(args,cwd=ROOT,capture_output=True,text=True,timeout=timeout);(out/(name+'.log')).write_text(r.stdout+r.stderr)
    with (out/'commands.jsonl').open('a') as f:f.write(json.dumps(dict(name=name,args=args,exit_code=r.returncode,wall_s=time.monotonic()-start))+'\n')
    if r.returncode:raise RuntimeError(name+' failed; see '+str(out/(name+'.log')))

def save_checkpoint(out,spec,binding,deps):
    runtime=read(out/'runtime.json');metrics=read(out/'metrics15.json')
    if runtime.get('error') or not metrics['passed']:return
    dump(out/'checkpoint15.json',dict(schema='S15-EXPERIMENT-CHECKPOINT/1',spec=spec,source_snapshot=binding,dependencies=deps,experiment_finished=True,mission_completed=metrics['completed'],evidence={str(p.relative_to(out)):sha(p) for p in files(out) if p.name!='checkpoint15.json'}))

def reuse(parents,out,spec,binding,deps):
    for parent in parents:
        p=(parent/out.name).resolve()
        if not p.is_relative_to(RESULTS) or p==out.resolve():raise ValueError('Unsafe checkpoint path')
        c=p/'checkpoint15.json'
        if not c.exists():continue
        proof=read(c)
        if proof.get('spec')!=spec or proof.get('source_snapshot')!=binding or proof.get('dependencies')!=deps or not proof.get('experiment_finished'):continue
        if any(not (p/k).resolve().is_relative_to(p) or not (p/k).is_file() or sha(p/k)!=h for k,h in proof['evidence'].items()):continue
        out.mkdir(parents=True)
        for name in proof['evidence']:
            dest=out/name;dest.parent.mkdir(parents=True,exist_ok=True);os.link(p/name,dest)
        dump(out/'checkpoint15.json',dict(proof,reused_from=str(p)))
        print('S15 VERIFIED EXPERIMENT CHECKPOINT '+str(p),flush=True)
        return read(out/'metrics15.json')

def receipt(out,data):
    docs=ROOT/'docs/optimization_v2/S15';docs.mkdir(parents=True,exist_ok=True)
    content='# S15 本轮实测结果\n\n'+f"状态：{data['status']}。人工：NOT_CHECKED。运行：`{data['run_id']}`。\n\n"
    content+='本步为差速两点动态基准和仿真观测预测评估，不替代 S14 全簇、S17 六簇或阿克曼动态中断验收。\n\n'
    content+='```json\n'+json.dumps({k:data.get(k) for k in ['checks','selection','cases','error']},ensure_ascii=False,indent=2)+'\n```\n\n'
    content+='证据：['+str(out/'index.html')+']('+str(out/'index.html')+')。命令/退出码见同目录 commands.jsonl。\n\nGazebo 在停车、工具关闭和暂停后清理仍可能返回 −11；进程存活、任务结果、碰撞和退出码分别记录，未声称修复这一上游退出问题。\n'
    (docs/'RESULTS.md').write_text(content)
    artifacts=files(out)+files(docs)+files(BASE/'lessons/S15')+files(BASE/'datasets/prediction')
    result=dict(schema_version=1,step='S15',plan_revision='20261109.1',run_id=data['run_id'],checked_at_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),automated_status=data['status'],manual_status='NOT_CHECKED',reason=data.get('error') or 'Actual observation data, real pinned-model inference and closed-loop A/B; disabled candidates retained honestly.',check_exit_code=0 if data['status']=='PASS' else 1,checks=data['checks'],report_path=str((out/'index.html').relative_to(ROOT)),source_files=[row(p) for p in sources()],artifacts=[row(p) for p in sorted(set(artifacts))],dependency_receipts={'S14':sha(BASE/'results/S14/latest.json')})
    dump(RESULTS/'latest.json',result)

def assess(data,cfg):
    cases=data['cases'];normal=[x for x in cases.values() if x['fault']=='none'];faults=[x for x in cases.values() if x['fault']!='none']
    data['checks']['S15-C01']='PASS' if data.get('dataset_valid') and data.get('inference',{}).get('future_prefix_invariant') else 'FAIL'
    data['checks']['S15-C02']='PASS' if data.get('inference',{}).get('passed') and all(data['offline']['summary']['test'][m]['windows']>=cfg['selection']['minimum_test_windows'] for m in ['imm','learned']) and sum(x['learned_person_forecasts'] for x in normal)>0 else 'FAIL'
    matched=all(len({x['fixture_sha'] for x in normal if x['scenario']==scenario})==1 and len({x['benchmark_task_sha'] for x in normal if x['scenario']==scenario})==1 for scenario in ['turning','stop_go','yield','occlusion'])
    data['matched_ab_geometry']=matched
    data['checks']['S15-C03']='PASS' if len(normal)==12 and matched and all(x['safe'] for x in normal) and any(x['completed'] for x in normal if x['method']=='no_prediction') and data.get('selection') else 'FAIL'
    data['checks']['S15-C04']='PASS' if data.get('risk_contracts') and len(faults)==3 and all(x['passed'] and x['fault_observed'] and x['fault_window_fallback'] for x in faults) else 'FAIL'
    data['status']='PASS' if all(x=='PASS' for x in data['checks'].values()) else 'FAIL'

def main():
    ap=argparse.ArgumentParser();ap.add_argument('mode',choices=['demo','test','stop']);ap.add_argument('--case',choices=['prediction_turning','prediction_timeout']);ap.add_argument('--suite',choices=['acceptance']);ap.add_argument('--seed',type=int,default=41);ap.add_argument('--sim-seconds',type=float,default=300);ap.add_argument('--wall-timeout',type=float,default=2400);ap.add_argument('--headless',action='store_true');ap.add_argument('--resume-from',action='append',type=Path,default=[]);a=ap.parse_args()
    if a.mode=='stop':
        stopped=stop_session(SESSION,40);print(json.dumps(stopped));return 0 if stopped['status'] in ['STOPPED','NOT_RUNNING'] else 1
    if a.seed!=41 or not 0<a.sim_seconds<=300 or not 0<a.wall_timeout<=14400:ap.error('seed41; 0<sim<=300; 0<wall<=14400 required')
    if a.mode=='test' and (a.suite!='acceptance' or a.case or a.sim_seconds<100):ap.error('test requires --suite acceptance and sim-seconds >=100 for fixed cases')
    if a.mode=='demo' and (not a.case or a.suite):ap.error('demo requires --case')
    lock=(ROOT/'native_20261108/runs/session.lock').open('a')
    try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    except BlockingIOError:print('已有项目仿真运行，S15 拒绝并发');return 2
    run_id=a.mode+'_'+datetime.datetime.now().strftime('%Y%m%d_%H%M%S')+'_'+str(os.getpid());out=RESULTS/run_id;out.mkdir(parents=True)
    if os.getpgrp()!=os.getpid():os.setsid()
    save_session(SESSION,str(Path(__file__).resolve()),run_id)
    session=read(SESSION);session.update(ros_domain=119,evaluation_domain=120,raw_domain=121,run_directory=str(out),seed=41);dump(SESSION,session)
    def stop(s,f):
        if s==signal.SIGALRM:raise TimeoutError('S15_WALL_BUDGET_CHECKPOINT_RETAINED')
        raise KeyboardInterrupt
    for s in [signal.SIGINT,signal.SIGTERM,signal.SIGALRM]:signal.signal(s,stop)
    signal.setitimer(signal.ITIMER_REAL,max(.1,a.wall_timeout-5));start=time.monotonic()
    data=dict(run_id=run_id,status='PARTIAL',mode=a.mode,seed=41,manual_status='NOT_CHECKED',scope='S15_PREDICTION_AND_TWO_POINT_CLOSED_LOOP',cases={},checks={f'S15-C{i:02}':'NOT_RUN' for i in range(1,5)},command=sys.argv)
    try:
        command(out,'prerequisite_S14',['bash',str(BASE/'stepctl.sh'),'check','S14'],180)
        command(out,'build',['bash',str(BASE/'scripts/build_s15.sh')],120)
        cfg=yaml.safe_load((BASE/'config/prediction.yaml').read_text());dump(out/'policy.json',yaml.safe_load((BASE/'config/decision_policy.yaml').read_text()));dump(out/'prediction.json',cfg)
        command(out,'inference',[str(BASE/'venvs/prediction/bin/python'),str(PKG/'tests/inference15.py'),str(out/'inference.json')],90);data['inference']=read(out/'inference.json')
        command(out,'risk_contract_build',['c++','-std=c++17','-O2','-UNDEBUG','-I'+str(GUARD/'include'),'-I'+str(S14/'include'),str(GUARD/'tests/contracts15.cpp'),'-o',str(out/'risk_contract')],50)
        command(out,'risk_contract',[str(out/'risk_contract'),str(out/'prediction.json'),str(out/'policy.json')],10);data['risk_contracts']=True
        command(out,'lesson_build',['c++','-std=c++17','-O2','-UNDEBUG','-I'+str(GUARD/'include'),str(BASE/'lessons/S15/lesson.cpp'),'-o',str(out/'lesson_cpp')],40)
        command(out,'lesson_cpp',[str(out/'lesson_cpp'),str(out/'prediction.json')],10);command(out,'lesson_python',[str(BASE/'venvs/prediction/bin/python'),str(BASE/'lessons/S15/lesson.py')],10)
        dataset=read(BASE/'datasets/prediction/manifest.json');data['dataset_valid']=validate_dataset(dataset);data['dataset_manifest_sha']=sha(BASE/'datasets/prediction/manifest.json')
        command(out,'offline_scoring',[str(BASE/'venvs/prediction/bin/python'),str(PKG/'scripts/evaluate15.py'),'--out',str(out)],180);data['offline']=read(out/'offline_metrics.json')
        binding={str(p.relative_to(ROOT)):sha(p) for p in sources()};deps=dict(S14=sha(BASE/'results/S14/latest.json'),dataset=sha(BASE/'datasets/prediction/manifest.json'));dump(out/'source_snapshot.json',binding);dump(out/'input_bindings.json',deps)
        matrix=[dict(scenario=s,method=m,fault='none',seconds=100) for s in ['turning','stop_go','yield','occlusion'] for m in ['no_prediction','imm','learned']]+[dict(scenario='turning',method='learned',fault=f,seconds=100) for f in ['timeout','channel_loss','uncertainty']]
        if a.mode=='demo':matrix=[dict(scenario='turning',method='learned',fault='timeout' if a.case=='prediction_timeout' else 'none',seconds=min(a.sim_seconds,100))]
        for spec in matrix:
            name=spec['scenario']+'_'+spec['method']+'_'+spec['fault'];case=out/name
            cached=reuse(a.resume_from,case,spec,binding,deps) if a.mode=='test' else None
            if cached:
                with (out/'commands.jsonl').open('a') as f:f.write(json.dumps(dict(name=name,call='verified immutable checkpoint reuse',spec=spec,reused_from=read(case/'checkpoint15.json')['reused_from'],experiment_exit_code=0,mission_completed=cached['completed']))+'\n')
                data['cases'][name]=cached;dump(out/'result.json',data);render(out,data);continue
            remaining=a.wall_timeout-(time.monotonic()-start)-25
            if remaining<180:raise TimeoutError('INSUFFICIENT_WALL_BUDGET; resume verified completed experiments')
            print('S15 RUN '+name,flush=True);ts=time.monotonic()
            if spec['fault']=='channel_loss':r=channel_loss(case,spec['scenario'],spec['method'],min(a.sim_seconds,spec['seconds']),min(360,remaining),a.mode=='demo' and not a.headless)
            else:r=run_case(case,spec['scenario'],spec['method'],'test',min(a.sim_seconds,spec['seconds']),min(360,remaining),a.mode=='demo' and not a.headless,fault=spec['fault'])
            with (out/'commands.jsonl').open('a') as f:f.write(json.dumps(dict(name=name,call='actual Gazebo runtime15',spec=spec,wall_s=time.monotonic()-ts,task_exit_code=0 if r.get('success') else 1))+'\n')
            data['cases'][name]=evaluate_case(case)
            if binding!={str(p.relative_to(ROOT)):sha(p) for p in sources()}:raise ValueError('SOURCE_CHANGED_DURING_EXPERIMENT')
            save_checkpoint(case,spec,binding,deps);dump(out/'result.json',data);render(out,data)
        if a.mode=='test':data['selection']=choose_default(data['cases'],data['offline'],cfg);assess(data,cfg)
        else:data['status']='PASS' if all(c['passed'] for c in data['cases'].values()) else 'FAIL'
    except (KeyboardInterrupt,TimeoutError) as e:data.update(status='PARTIAL',error=repr(e))
    except Exception as e:data.update(status='FAIL',error=repr(e));(out/'exception.log').write_text(traceback.format_exc());print(traceback.format_exc(),flush=True)
    finally:
        signal.setitimer(signal.ITIMER_REAL,0);data['wall_seconds']=time.monotonic()-start;data['next_step_verdict']='可以执行下一步 S16' if a.mode=='test' and data['status']=='PASS' else '暂不能凭本轮进入 S16';dump(out/'result.json',data);render(out,data)
        try:
            if a.mode=='test':receipt(out,data)
            else:dump(RESULTS/'last_demo.json',dict(run_id=run_id,status=data['status'],report_path=str((out/'index.html').relative_to(ROOT))))
        finally:SESSION.unlink(missing_ok=True);lock.close()
    if a.mode=='demo' and not a.headless:subprocess.Popen(['xdg-open',str(out/'index.html')],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL,start_new_session=True)
    print(json.dumps(dict(status=data['status'],report=str(out/'index.html'),next=data['next_step_verdict']),ensure_ascii=False));return 0 if data['status']=='PASS' else 1
if __name__=='__main__':raise SystemExit(main())
