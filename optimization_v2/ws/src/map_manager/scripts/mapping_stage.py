"""Public S06 execution, measured acceptance, version binding and owned shutdown."""
import argparse,datetime,fcntl,gzip,os,signal,subprocess,sys,time,traceback,shutil
from s06_common import *
from mapping_export import export,verify_manifest,command
from mapping_analysis import analyze
from mapping_report import render
# Frozen runtimes modify sys.path. Import stage-local modules before loading them.
from mapping_runtime import run_case
sys.path.insert(0,str(BASE/'ws/src/project_baseline/scripts'))
from session_owner import save_session,stop_session
RESULTS=BASE/'results/S06';SESSION=BASE/'runtime/S06.json'
def files(path):return [p for p in path.rglob('*') if p.is_file() and '__pycache__' not in p.parts]
def row(p):return dict(path=str(p.relative_to(ROOT)),sha256=digest(p))
def compact(out):
    for p in out.rglob('*.jsonl'):
        with p.open('rb') as src,gzip.open(str(p)+'.gz','wb',compresslevel=6) as dst:shutil.copyfileobj(src,dst)
        p.unlink()
def aggregate(cases):
    required={'mapping_full','mapping_loop','weak_texture'};complete=required<=set(cases)
    groups={1:['completed','database_reopened','real_sensor_payloads','real_3d_height','sensor_sync','truth_isolation','unique_tf_and_actuator'],2:['loop_closure','false_loop_rejected','actual_before_after_event'],3:['alignment_measured','map_quality','dynamic_measurement','route_completed','no_obstacle_contact','resource_bound','gravity_observed'],4:['pause_save_resume','physical_stop_thresholds','pause_really_stops_collection','save_then_more_nodes','version_bound']}
    checks={f'S06-C{k:02}':'PASS' if complete and all(all(cases[c]['metrics']['checks'].get(x,False) for x in keys) for c in ['mapping_full','mapping_loop']) else 'FAIL' for k,keys in groups.items()}
    if not cases.get('weak_texture',{}).get('metrics',{}).get('passed',False):checks['S06-C02']='FAIL'
    if not cases.get('mapping_full',{}).get('metrics',{}).get('checks',{}).get('both_zones_and_connector',False):checks['S06-C03']='FAIL'
    if not cases.get('mapping_full',{}).get('metrics',{}).get('checks',{}).get('measured_terrain_surfaces',False):checks['S06-C01']='FAIL'
    return checks,complete and all(v=='PASS' for v in checks.values()) and all(x['metrics'].get('passed',False) for x in cases.values())
def receipt(out,data):
    docs=ROOT/'docs/optimization_v2/S06';lines=['# S06 本轮实测结果','',f'运行 `{data["run_id"]}`：**{data["status"]}**。{data["next_step_verdict"]}。人工查看 NOT_CHECKED。','',f'[交互三维报告]({out}/index.html) · [完整指标]({out}/result.json)','']
    lines+=['- '+k+'：'+v for k,v in data['checks'].items()]
    if data.get('error'):lines+=['','本轮失败／中断原因：'+data['error']]
    lines+=['','| 案例 | 通过 | 关键帧 | 视觉／邻近回环 | 优化前／后 ATE m | 对齐 P95 前／后 m | 动态点剔除率 |','|---|---|---:|---:|---:|---:|---:|']
    evidence=[]
    for name,c in data['cases'].items():
        m=c['metrics'];val=lambda k,field:m.get(k,{}).get(field,float('nan'))
        fmt=lambda x:f'{x:.3f}' if math.isfinite(x) else 'N/A'
        label=('诊断通过；建图不合格' if name=='weak_texture' else 'PASS') if m.get('passed') else 'FAIL'
        lines.append(f'| {name} | {label} | {m.get("nodes",0)} | {m.get("visual_loops",0)}/{m.get("proximity_loops",0)} | {fmt(val("raw_trajectory","ate_rmse_m"))}/{fmt(val("optimized_trajectory","ate_rmse_m"))} | {fmt(val("raw_alignment","p95_m"))}/{fmt(val("optimized_alignment","p95_m"))} | {100*val("dynamic_residue","dynamic_rejection_fraction"):.1f}% |')
        evidence.append(f'- `{name}`：[metrics.json]({ROOT/c["path"]}/metrics.json)，[真实进程命令和退出码]({ROOT/c["path"]}/runtime.json)。')
    lines+=['']+evidence
    if data.get('final_review'):
        lines+=['','补充复核：默认 GUI 演示完成；最终 RViz 显示配置已在实际窗口查看；空重叠点云统计边界回归通过；报告交互重新检查。人工状态仍指用户尚未手工验收。',f'[复核记录]({out}/final_validation.json) · [最终 RViz 截图]({out}/rviz_verified.png) · [无损日志整理]({out}/retention_summary.json)']
    lines+=['','正常案例逐项门槛在 metrics.json；原场景弱纹理案例只考察诊断、数据链和安全行为，不能称为成功建图。','', 'ATE 使用启动 5 秒时的单一坐标原点；没有逐段拟合或真值初始化。回访点云误差仅代表该配对的可重叠可见表面，并非整张地图每处精度。动态残留标签仅由离线评分器使用；线上没有仿真物体 ID。','', '原始 S01–S05 未更改。地图仍处于 MAPPING；S07 的冻结、不同初始位置重定位未在本步宣称通过。','', '生产参数、源代码、二进制在采集开始时记录 capture_inputs.json。导出会拒绝采集期间变化的运行输入。运行命令包含种子 41，完整导出命令和退出码见 export_commands.jsonl.gz。','', '观测 JSONL 无损压缩；真实数据库与 PLY 保留。短程调试得到的初期失败见 development_summary.json，不作为本轮通过数据。']
    (docs/'RESULTS.md').write_text('\n'.join(lines)+'\n')
    source=files(PKG)+files(BASE/'references/S06')+[BASE/'config'/x for x in ['slam_mapping.yaml','routes/mapping_routes.yaml','s06_dependency_lock.json','rviz/mapping.rviz']]+[BASE/'scripts'/x for x in ['build_s06.sh','install_s06_dependencies.sh']]+[BASE/'stages/S06/run.sh']+[BASE/'ws/install/map_manager/lib/map_manager'/x for x in ['mapping_manager','graph_probe']]
    artifacts=files(out)+files(docs)+files(BASE/'lessons/S06')
    for c in data['cases'].values():
        path=ROOT/c['path']
        if not path.is_relative_to(out):artifacts+=files(path)
    for p in [RESULTS/'development_summary.json']:
        if p.exists():artifacts.append(p)
    for p in (BASE/'maps/mapping_sessions').glob(data['run_id']+'/*.json'):artifacts.append(p)
    r=dict(schema_version=1,step='S06',plan_revision='20261109.1',run_id=data['run_id'],checked_at_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),automated_status=data['status'],manual_status='NOT_CHECKED',reason=data.get('error','真实三维数据库、回环和错误候选、对齐误差、暂停/保存/继续及版本绑定'),check_exit_code=0 if data['status']=='PASS' else 1,checks=data['checks'],report_path=str((out/'index.html').relative_to(ROOT)),source_files=[row(p) for p in sorted(set(source))],artifacts=[row(p) for p in sorted(set(artifacts))],dependency_receipts={'S05':digest(BASE/'results/S05/latest.json')})
    dump(RESULTS/'latest.json',r)
def main():
    p=argparse.ArgumentParser();p.add_argument('mode',choices=['test','demo','stop']);p.add_argument('--suite',choices=['acceptance']);p.add_argument('--case',choices=['mapping_loop','loop_comparison']);p.add_argument('--seed',type=int,default=41);p.add_argument('--sim-seconds',type=float,default=900);p.add_argument('--wall-timeout',type=float,default=3600);p.add_argument('--headless',action='store_true');p.add_argument('--resume',type=Path,help='Reuse one finished mapping_full capture only after every capture input hash matches');a=p.parse_args()
    if a.mode=='stop':
        result=stop_session(SESSION,60);print(json.dumps(result));return 0 if result['status'] in ['STOPPED','NOT_RUNNING'] else 1
    if a.seed!=41:p.error('S06 qualification table is frozen at seed 41')
    if not math.isfinite(a.sim_seconds) or a.sim_seconds<=0 or not math.isfinite(a.wall_timeout):p.error('finite positive time budgets required')
    if a.mode=='test' and a.case:p.error('--case is only valid for demo')
    if a.mode=='demo' and (a.suite or a.resume):p.error('--suite/--resume are only valid for test')
    if a.mode=='test' and (a.suite!='acceptance' or a.sim_seconds<900):p.error('test requires --suite acceptance and >=900 sim-second per-case budget')
    if a.mode=='demo' and not a.case:p.error('--case required')
    if a.mode=='demo' and a.case=='mapping_loop' and a.sim_seconds<140:p.error('mapping loop requires >=140 sim seconds including lifecycle test')
    if a.wall_timeout<30:p.error('wall timeout must be >=30')
    if a.case=='loop_comparison':
        r=read(RESULTS/'latest.json');report=ROOT/r['report_path']
        if not report.exists():raise FileNotFoundError(report)
        if not a.headless:subprocess.Popen(['xdg-open',str(report)],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL,start_new_session=True)
        print(json.dumps(dict(action_status='PASS',source_automated_status=r['automated_status'],simulation_clock='N/A',source_run=r['run_id'],report=str(report)),ensure_ascii=False));return 0
    if subprocess.run(['bash',str(BASE/'stepctl.sh'),'check','S05'],cwd=ROOT).returncode:return 4
    lock=(ROOT/'native_20261108/runs/session.lock').open('a')
    try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    except BlockingIOError:print('已有项目仿真持有共享会话锁。');return 2
    if os.getpgrp()!=os.getpid():os.setsid()
    run_id=a.mode+'_'+datetime.datetime.now().strftime('%Y%m%d_%H%M%S')+'_'+str(os.getpid());out=RESULTS/run_id;out.mkdir(parents=True);save_session(SESSION,str(Path(__file__).resolve()),run_id)
    session=read(SESSION);session.update(pgid=os.getpgrp(),sid=os.getsid(0),domains={'online':119,'evaluation':120,'raw':121},seed=a.seed);dump(SESSION,session)
    def interrupt(s,f):raise KeyboardInterrupt
    signal.signal(signal.SIGINT,interrupt);signal.signal(signal.SIGTERM,interrupt);start=time.monotonic()
    data=dict(run_id=run_id,mode=a.mode,seed=a.seed,status='PARTIAL',checks={f'S06-C{i:02}':'NOT_RUN' for i in range(1,5)},cases={},next_step_verdict='暂不能执行下一步：S06 未完整通过',command=sys.argv)
    def collect(name,resume=None):
        case_out=resume.resolve() if resume else out/name
        if not case_out.is_relative_to(ROOT):raise ValueError('capture must remain inside the local project root')
        if not resume and (case_out/'mapping.db').exists():raise ValueError('new capture refuses to overwrite an existing database')
        seconds=900 if name=='mapping_full' else 140
        if resume:
            rt=read(case_out/'runtime.json')
            if not rt['success'] or rt['case']!=name or rt['seed']!=a.seed or rt['seconds']<seconds:raise ValueError('resume capture is incomplete/wrong case/seed/budget')
            for item in read(case_out/'capture_inputs.json'):
                if digest(ROOT/item['path'])!=item['sha256']:raise ValueError('resume source changed: '+item['path'])
            data['resumed_capture']=str(case_out)
        else:
            remaining=a.wall_timeout-(time.monotonic()-start)-60
            if remaining<120:raise TimeoutError('not enough wall time for next case')
            print('RUN '+name,flush=True);run_case(case_out,'diff',name,seconds,min(remaining,seconds*2+90),a.seed,a.mode=='demo' and not a.headless)
        export(case_out);m=analyze(case_out);data['cases'][name]=dict(path=str(case_out.relative_to(ROOT)),metrics=m);dump(out/'progress.json',data);print(('PASS ' if m['passed'] else 'FAIL ')+name+' '+str({k:v for k,v in m['checks'].items() if not v}),flush=True)
        if name=='mapping_full':
            session_dir=BASE/'maps/mapping_sessions'/run_id;session_dir.mkdir(parents=True,exist_ok=True);dump(session_dir/'manifest.json',read(case_out/'map_manifest.json'))
    try:
        command(out,'build',['bash',str(BASE/'scripts/build_s06.sh')],120)
        command(out,'contracts',[sys.executable,str(PKG/'tests/contracts.py')],60)
        command(out,'lesson_compile',['g++','-std=c++17','-O2','-I/usr/include/eigen3',str(BASE/'lessons/S06/lesson.cpp'),'-o',str(BASE/'ws/build/map_manager/lesson_s06')],60)
        command(out,'lesson_cpp',[str(BASE/'ws/build/map_manager/lesson_s06')],20);command(out,'lesson_python',['/usr/bin/python3',str(BASE/'lessons/S06/lesson.py')],20)
        if a.mode=='test':
            collect('mapping_full',a.resume);collect('mapping_loop');collect('weak_texture');data['checks'],passed=aggregate(data['cases']);data['status']='PASS' if passed else 'FAIL'
            if passed:data['next_step_verdict']='可以执行下一步 S07：S06 全部检查通过且 S05 有效；不自动执行'
        else:
            collect('mapping_loop');data['status']='PASS' if data['cases']['mapping_loop']['metrics']['passed'] else 'FAIL';data['next_step_verdict']='演示独立记录；正式验收状态请用 stepctl next S06 查看'
    except KeyboardInterrupt:data['status']='PARTIAL';data['error']='用户中断；已停车并请求数据库正常保存'
    except Exception as exc:data['status']='FAIL';data['error']=repr(exc);(out/'exception.log').write_text(traceback.format_exc());print(traceback.format_exc(),flush=True)
    finally:
        try:
            data['wall_seconds']=time.monotonic()-start;dump(out/'result.json',data)
            try:
                render(out,data)
                if data['cases']:command(out,'browser_check',[sys.executable,str(PKG/'tests/report_check.py'),str(out/'index.html')],45)
            except Exception as exc:data.update(status='FAIL',error='report: '+repr(exc),next_step_verdict='暂不能执行下一步：可视报告验证失败')
            for c in data['cases'].values():compact(ROOT/c['path'])
            dump(out/'result.json',data)
            if a.mode=='test':receipt(out,data)
            else:dump(RESULTS/'last_demo.json',dict(run_id=run_id,status=data['status'],report_path=str((out/'index.html').relative_to(ROOT))))
        finally:SESSION.unlink(missing_ok=True);lock.close()
    print(json.dumps(dict(status=data['status'],report=str(out/'index.html'),next=data['next_step_verdict']),ensure_ascii=False));return 0 if data['status']=='PASS' else 1
if __name__=='__main__':raise SystemExit(main())
