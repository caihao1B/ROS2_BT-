"""S11 offline bounded executor. No ROS graph, no vehicle or tool commands."""
import argparse,datetime,fcntl,signal,subprocess,time,traceback,resource,shutil
from common11 import *
from geometry11 import *
from partition11 import solve
from products11 import domain_record,export_bundle,validate_bundle
from report11 import render
sys.path.insert(0,str(BASE/'ws/src/project_baseline/scripts'))
from session_owner import save_session,stop_session
RESULTS=BASE/'results/S11';SESSION=BASE/'runtime/S11.json'
def sources():
    return files(PKG)+inputs()+[BASE/'scripts/build_s11.sh',BASE/'stages/S11/run.sh',BASE/'config/s11_dependency_lock.json',BIN]+files(BASE/'references/S11')
def snapshot():return {str(p.relative_to(ROOT)):sha(p) for p in sources() if p.exists()}
def command(out,name,args,timeout):
    start=time.monotonic();p=subprocess.Popen(args,cwd=ROOT,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,start_new_session=True)
    try:
        log,_=p.communicate(timeout=timeout)
    except BaseException:
        os.killpg(p.pid,signal.SIGTERM)
        try:log,_=p.communicate(timeout=5)
        except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);log,_=p.communicate()
        (out/(name+'.log')).write_text(log or '');raise
    (out/(name+'.log')).write_text(log)
    with (out/'commands.jsonl').open('a') as f:f.write(json.dumps(dict(name=name,args=args,exit_code=p.returncode,wall_s=time.monotonic()-start))+'\n')
    if p.returncode:raise RuntimeError(name+' failed; see '+str(out/(name+'.log')))

def receipt(out,data):
    docs=ROOT/'docs/optimization_v2/S11';lines=['# S11 实测结果','',f'本轮 `{data["run_id"]}`：自动 **{data["status"]}**；人工 NOT_CHECKED。',data['next_step_verdict'],'',f'[交互报告]({out}/index.html) · [完整指标]({out}/result.json)','',f'离线运行，无仿真时钟；墙钟 {data["wall_seconds"]:.2f} 秒。矩阵 {len(data["records"])} 组。计划状态 PROVISIONAL，执行许可 false。','', '| 区域/算法（基础 seed 41） | 最大初估/min | 不均衡比 | 加权切边 | 分区/ms |','|---|---:|---:|---:|---:|']
    for r in data['records']:
        if r['variant']=='base' and r['seed']==41:
            m=r['metrics'];lines.append(f'| {r["zone"]}/{r["algorithm"]} | {m["max_estimate_s"]/60:.3f} | {m["imbalance"]:.5f} | {m["weighted_cut"]} | {m["partition_wall_s"]*1000:.2f} |')
    lines+=['','本步证明任务分区的完整、连通和确定性，不证明机器人可沿实际路线覆盖全部区域。底盘高速度测试不能覆盖 S08/S10 当前有效限速 0.12 m/s；本轮初估超预算，割草全区域按物理刀幅计算的乐观下界也超过三簇 × 30 分钟。没有提高限速、增大 k 或丢弃边缘任务来改变结果。','', 'METIS 并非所有指标都胜出：以表中实际最大负载、切边和计算时间为准。DARP 仅接口及适用条件测试，算法未运行。','', '包络只做保守可达性筛查；未通过的边界点和面积仍保留任务归属。S02 先验米制坐标未直接等同 S07 SLAM map，需要后续配准。连接路/B 区已有定位地图问题未在本步修改。','', '完整正常/失败测试、重复种子重算、C++/Python 教学例及浏览器交互检查见 commands.jsonl、unit_tests.log、matrix.json、browser_check.json。本步未安装新系统包。','', f'检查：{json.dumps(data["checks"],ensure_ascii=False)}']
    if data.get('error'):lines+=['',data['error']]
    (docs/'RESULTS.md').write_text('\n'.join(lines)+'\n')
    artifacts=files(out)+files(docs)+files(BASE/'lessons/S11')
    if data['status']=='PASS':artifacts+=files(BASE/'plans/candidate')
    history=RESULTS/'development_summary.json'
    if history.exists():artifacts.append(history)
    r=dict(schema_version=1,step='S11',plan_revision='20261109.1',run_id=data['run_id'],checked_at_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),automated_status=data['status'],manual_status='NOT_CHECKED',reason=data.get('error','36 组几何分簇、确定性复算、正常/失败回归和真实浏览器检查；工时仅初估且当前超预算'),check_exit_code=0 if data['status']=='PASS' else 1,checks=data['checks'],report_path=str((out/'index.html').relative_to(ROOT)),source_files=[row(p) for p in sorted(set(sources())) if p.exists()],artifacts=[row(p) for p in sorted(set(artifacts))],dependency_receipts={s:sha(BASE/f'results/{s}/latest.json') for s in ['S02','S03','S10']})
    dump(RESULTS/'latest.json',r)

def main():
    ap=argparse.ArgumentParser();ap.add_argument('mode',choices=['demo','test','stop']);ap.add_argument('--case',choices=['partition_base','partition_variants']);ap.add_argument('--suite',choices=['acceptance']);ap.add_argument('--seed',type=int,default=41);ap.add_argument('--sim-seconds',type=float,default=300);ap.add_argument('--wall-timeout',type=float,default=1200);ap.add_argument('--headless',action='store_true');a=ap.parse_args()
    if a.mode=='stop':
        r=stop_session(SESSION,20);print(json.dumps(r));return 0 if r['status'] in ['STOPPED','NOT_RUNNING'] else 1
    if a.mode=='test' and (a.suite!='acceptance' or a.case or a.seed!=41):ap.error('test requires acceptance seed 41, no case')
    if a.mode=='demo' and (not a.case or a.suite):ap.error('demo requires case, no suite')
    if not (0<=a.seed<2**30 and math.isfinite(a.wall_timeout) and a.wall_timeout>0 and math.isfinite(a.sim_seconds) and a.sim_seconds>0):ap.error('finite positive budgets and supported seed required')
    lock=(ROOT/'native_20261108/runs/session.lock').open('a')
    try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    except BlockingIOError:print('已有项目会话持有锁');return 2
    run_id=a.mode+'_'+datetime.datetime.now().strftime('%Y%m%d_%H%M%S')+'_'+str(os.getpid());out=RESULTS/run_id;out.mkdir(parents=True)
    if os.getpgrp()!=os.getpid():os.setsid()
    save_session(SESSION,str(Path(__file__).resolve()),run_id)
    session=read(SESSION);session.update(offline=True,ros_domain=None,gz_partition=None,seed=a.seed,map_id=read(BASE/'maps/prior/map_metadata.json')['map_id']);dump(SESSION,session)
    def interrupted(s,f):
        if s==signal.SIGALRM:raise TimeoutError('S11_WALL_BUDGET')
        raise KeyboardInterrupt
    for s in [signal.SIGINT,signal.SIGTERM,signal.SIGALRM]:signal.signal(s,interrupted)
    signal.setitimer(signal.ITIMER_REAL,a.wall_timeout);start=time.monotonic()
    data=dict(run_id=run_id,mode=a.mode,seed=a.seed,status='PARTIAL',sim_clock='N/A_OFFLINE',requested_sim_budget_not_used=a.sim_seconds,records=[],domains={},checks={f'S11-C{i:02}':'NOT_RUN' for i in range(1,5)},next_step_verdict='暂不能执行下一步：正式矩阵未完成',command=sys.argv)
    try:
        for s in ['S02','S03','S10']:command(out,'prerequisite_'+s,['bash',str(BASE/'stepctl.sh'),'check',s],90)
        command(out,'build',['bash',str(BASE/'scripts/build_s11.sh')],120)
        command(out,'dependencies',[sys.executable,str(PKG/'tests/dependencies.py'),str(out/'environment_verified.json')],15)
        binding=dict(input_hashes={str(p.relative_to(ROOT)):sha(p) for p in inputs()},source_hashes=snapshot(),dependency_receipts={s:sha(BASE/f'results/{s}/latest.json') for s in ['S02','S03','S10']});dump(out/'inputs.json',binding)
        if a.mode=='test':
            command(out,'unit_tests',[sys.executable,str(PKG/'tests/test_partition.py')],45)
            command(out,'lesson_compile',['c++','-std=c++17','-O2',str(BASE/'lessons/S11/lesson.cpp'),'-lmetis','-o',str(BASE/'ws/build/coverage_partition/lesson_s11')],30)
            command(out,'lesson_cpp',[str(BASE/'ws/build/coverage_partition/lesson_s11')],10)
            command(out,'lesson_python',[sys.executable,str(BASE/'lessons/S11/lesson.py')],15)
        c=parameters()
        if c['k']!=3:raise ValueError('EXPERIMENT_REQUIRES_EXACTLY_THREE')
        variants=['base'] if a.case=='partition_base' else ['base','notch','holes'];seeds=[a.seed] if a.case=='partition_base' else [a.seed+i for i in range(3)];graphs={};repro=[]
        for variant in variants:
            for zone in ['A','B']:
                meta=load_domain(zone,variant,c);g=build_graph(meta['domain'],c,meta);key=zone+'_'+variant;graphs[key]=g;data['domains'][key]=domain_record(meta)
                for seed in seeds:
                    for algo in ['metis','kmeans']:
                        name=f'{key}_{algo}_{seed}';r=solve(meta,g,c,algo,seed,out/name);data['records'].append(r)
                        if a.mode=='test':
                            repeat=solve(meta,g,c,algo,seed,out/'repro'/name)
                            same=all(r[k]==repeat[k] for k in ['assignment_sha256','clusters','verification'])
                            repro.append(dict(case=name,passed=same,assignment_sha256=r['assignment_sha256']))
                            if not same:raise ValueError('REPRODUCIBILITY_FAILED '+name)
                        print(f'{name}: geometry PASS, max estimate {r["metrics"]["max_estimate_s"]/60:.2f} min',flush=True)
                dump(out/'progress.json',dict(completed=len(data['records']),last=key))
        dump(out/'matrix.json',dict(cases=[dict(zone=r['zone'],variant=r['variant'],seed=r['seed'],algorithm=r['algorithm'],verification=r['verification'],metrics=r['metrics']) for r in data['records']],reproducibility=repro))
        if snapshot()!=binding['source_hashes']:raise ValueError('SOURCES_CHANGED_DURING_RUN')
        if a.mode=='test' and not (len(data['records'])==36 and len(repro)==36 and all(x['passed'] for x in repro)):raise ValueError('MATRIX_INCOMPLETE')
        data['status']='PASS';data['checks']={f'S11-C{i:02}':'PASS' for i in range(1,5)} if a.mode=='test' else data['checks']
        if a.mode=='test':export_bundle(out/'candidate',data['records'],graphs,data['domains'],41,binding);validate_bundle(out/'candidate')
        render(out,data);command(out,'browser_check',[sys.executable,str(PKG/'tests/report_check.py'),str(out/'index.html')],45)
        if a.mode=='test':
            target=BASE/'plans/candidate';target.mkdir(parents=True,exist_ok=True)
            for p in files(out/'candidate'):
                if p.name!='partition_manifest.json':shutil.copy2(p,target/p.name)
            shutil.copy2(out/'candidate/partition_manifest.json',target/'partition_manifest.json');validate_bundle(target)
            data['next_step_verdict']='可以执行下一步 S12——点服务和条带离线路径与工时闭环；S11 几何前置通过，当前候选工时超预算，需在 S12 解决后才能允许任务执行。'
        else:data['next_step_verdict']='演示不覆盖正式验收；以 stepctl next S11 为准。'
    except KeyboardInterrupt:data.update(status='PARTIAL',error='中断离线计算；无车辆/工具命令，已完成结果保留')
    except Exception as e:data.update(status='FAIL',error=repr(e));(out/'exception.log').write_text(traceback.format_exc());print(traceback.format_exc(),flush=True)
    finally:
        signal.setitimer(signal.ITIMER_REAL,0)
        try:
            if data['status']!='PASS':
                data['checks']={f'S11-C{i:02}':data['status'] if data['status']=='FAIL' else 'NOT_RUN' for i in range(1,5)};data['next_step_verdict']='暂不能执行下一步：本轮正式验收未完整通过'
            data['wall_seconds']=time.monotonic()-start;data['peak_process_rss_mib']=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/1024
            dump(out/'result.json',data);render(out,data)
            if a.mode=='test':receipt(out,data)
            else:dump(RESULTS/'last_demo.json',dict(run_id=run_id,status=data['status'],report_path=str((out/'index.html').relative_to(ROOT))))
        finally:SESSION.unlink(missing_ok=True);lock.close()
    if a.mode=='demo' and not a.headless:subprocess.Popen(['xdg-open',str(out/'index.html')],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL,start_new_session=True)
    print(json.dumps(dict(status=data['status'],report=str(out/'index.html'),next=data['next_step_verdict']),ensure_ascii=False));return 0 if data['status']=='PASS' else 1
if __name__=='__main__':raise SystemExit(main())
