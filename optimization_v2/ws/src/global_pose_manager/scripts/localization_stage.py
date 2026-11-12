"""S07 public bounded executor, real failure matrix, report and versioned receipt."""
import argparse,datetime,fcntl,gzip,os,signal,subprocess,sys,time,traceback,shutil,importlib.util
from s07_common import *
from localization_analysis import analyze
from localization_report import render
from localization_runtime import run_case
sys.path.insert(0,str(BASE/'ws/src/project_baseline/scripts'))
from session_owner import save_session,stop_session
RESULTS=BASE/'results/S07';SESSION=BASE/'runtime/S07.json'

def files(path):return [p for p in path.rglob('*') if p.is_file() and '__pycache__' not in p.parts]
def row(p):return dict(path=str(p.relative_to(ROOT)),sha256=digest(p))
def compact(out):
    for p in out.rglob('*.jsonl'):
        with p.open('rb') as src,gzip.open(str(p)+'.gz','wb',compresslevel=6) as dst:shutil.copyfileobj(src,dst)
        p.unlink()
def command(out,name,args,timeout):
    r=subprocess.run(args,cwd=ROOT,capture_output=True,text=True,timeout=timeout);(out/(name+'.log')).write_text(r.stdout+r.stderr)
    with (out/'commands.jsonl').open('a') as f:f.write(json.dumps(dict(name=name,args=args,exit_code=r.returncode))+'\n')
    if r.returncode:raise RuntimeError(name+' failed; see '+str(out/(name+'.log')))

def aggregate(data):
    cases=data['cases'];norm=['zone_a','connector','zone_b'];complete=set(norm+['restart_occlusion','wrong_initial'])<=set(cases)
    def ok(names,keys):return all(n in cases and all(cases[n]['metrics']['checks'].get(k,False) for k in keys) for n in names)
    conditions={1:ok(norm,['completed','localized_with_multiple_matches','position_and_heading_accuracy','acquisition_bounded','coarse_prior_not_truth_seed','diagnostic_motion_observed']),2:ok(list(cases),['frozen_database_unchanged','bundle_valid','unique_tf_and_actuator','capture_versions_match']),3:ok(list(cases),['confirmation_hysteresis','no_unadmitted_drive','truth_isolation','sensor_based_map_agreement']) and ok(['restart_occlusion'],['actual_process_restart','localized_after_restart','localized_after_occlusion','occlusion_delivered','lost_state_observed','physical_fault_stops']) and ok(['wrong_initial'],['wrong_seed_not_blindly_admitted','wrong_seed_diagnosed']),4:data.get('contracts',{}).get('passed',False)}
    checks={f'S07-C{k:02}':'PASS' if complete and v else 'FAIL' for k,v in conditions.items()};passed=complete and all(v=='PASS' for v in checks.values()) and all(c['metrics']['passed'] for c in cases.values());return checks,passed

def receipt(out,data):
    docs=ROOT/'docs/optimization_v2/S07';docs.mkdir(parents=True,exist_ok=True)
    lines=['# S07 本轮实测结果','',f'运行 `{data["run_id"]}`：**{data["status"]}**。{data["next_step_verdict"]}。用户人工验收 NOT_CHECKED。','',f'[交互报告]({out}/index.html) · [全部指标与命令]({out}/result.json)','']+['- '+k+'：'+v for k,v in data['checks'].items()]
    if data.get('error'):lines+=['','未通过原因：'+data['error']]
    lines+=['','| 案例 | 结论 | 首获许可 / s | XY RMSE / m | 航向 RMSE / ° | 首获定位后准入比例 |','|---|---|---:|---:|---:|---:|']
    for name,c in data['cases'].items():
        m=c['metrics'];fmt=lambda k:'N/A' if m.get(k) is None else f'{m[k]:.3f}'
        lines.append(f'| {name} | {"PASS" if m["passed"] else "FAIL"} | {fmt("first_localized_sim_s")} | {fmt("xy_rmse_m")} | {fmt("yaw_rmse_deg")} | {100*m.get("admission_fraction_after_first_fix",0):.1f}% |')
    show_time=lambda value:'N/A' if value is None else f'{value:.3f}'
    fault=data['cases'].get('restart_occlusion',{}).get('metrics',{})
    if fault:
        lines+=['','重启后重新获得许可：'+show_time(fault.get('restart_recovery_sim_s'))+' 仿真秒；遮挡解除后重新获得许可：'+show_time(fault.get('occlusion_recovery_sim_s'))+' 仿真秒。']
        for name,v in fault.get('physical_stops',{}).items():lines.append(f'- {name}：许可关闭后 {show_time(v.get("delay_sim_s"))} s 停至原速度门槛；从故障触发计 {show_time(v.get("fault_to_stop_sim_s"))} s；故障前确实在运动={v.get("moving_before_fault")}。')
    if data.get('final_review'):lines+=['',f'[最后交付复核]({out}/final_validation.json)：实际 GUI 截图检查、版本拒绝公共入口、浏览器交互与仿真时间加权统计已复核。用户人工验收仍单独记录。']
    lines+=['','准入比例按首获可靠定位后的全部时段计算，包含故障；低于 100% 说明当前观测有间歇失效并触发停车，不宣称已达到连续商用可用率。上述精度使用冻结 S06 坐标原点，不做真值轨迹拟合。传感器匹配需粗略操作员位置先验，不能解释为无先验任意位置定位。初始姿态仅用于仿真器；独立真值只供离线评分。','',f'地图 `{verify_bundle()["map_id"]}` 的实际数据库、点云、占据图和粗地形由 manifest 内容哈希绑定；在线 LOCALIZATION 不保存长期地图。每例 runtime.json 记录真实进程命令、退出码、DB 前后哈希、重启事件；metrics.json 记录停车时间、动作准入和 TF 所有权。','',f'版本拒绝共 {len(data.get("contracts",{}).get("cases",[]))} 项检查：[contracts.json]({out}/contracts/contracts.json)。它们使用诊断契约 fixture；S12 真实覆盖计划尚未实现，需再做端到端绑定测试。','', '自动检查 PASS 与用户人工验收分开记录。地图之外的区域、任意初始航向、重遮挡、长期场景变化不在本次成功范围。粗地形只是观测近似，S09 尚未判定通行性。','', '阶段命令、种子 41、实际依赖版本、学习例和浏览器测试退出码均在回执中；各案例完整输入哈希必须和当前文件一致，才允许断点复用。原 S01–S06 源码与回执未改动。']
    (docs/'RESULTS.md').write_text('\n'.join(lines)+'\n')
    source=files(PKG)+files(BASE/'references/S07')+[BASE/'config'/x for x in ['slam_localization.yaml','map_contract.yaml','relocalization_cases.yaml','s07_dependency_lock.json','rviz/localization.rviz']]+[BASE/'scripts/build_s07.sh',BASE/'stages/S07/run.sh',BASE/'ws/install/global_pose_manager/lib/global_pose_manager/global_pose_manager']
    artifacts=files(out)+files(docs)+files(BASE/'lessons/S07')+files(BASE/'maps/frozen')
    for c in data['cases'].values():
        p=ROOT/c['path']
        if not p.is_relative_to(out):artifacts+=files(p)
    for p in [RESULTS/'development_summary.json',BASE/'results/S06/manual_acceptance.json']:
        if p.exists():artifacts.append(p)
    r=dict(schema_version=1,step='S07',plan_revision='20261109.1',run_id=data['run_id'],checked_at_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),automated_status=data['status'],manual_status='NOT_CHECKED',reason=data.get('error','冻结地图只读、三位置传感器重定位、实际重启/遮挡停车恢复、版本准入及学习材料'),check_exit_code=0 if data['status']=='PASS' else 1,checks=data['checks'],report_path=str((out/'index.html').relative_to(ROOT)),source_files=[row(p) for p in sorted(set(source))],artifacts=[row(p) for p in sorted(set(artifacts))],dependency_receipts={'S06':digest(BASE/'results/S06/latest.json')});dump(RESULTS/'latest.json',r)

def main():
    p=argparse.ArgumentParser();p.add_argument('mode',choices=['test','demo','stop']);p.add_argument('--suite',choices=['acceptance']);p.add_argument('--case',choices=['restart_relocalize','wrong_map']);p.add_argument('--seed',type=int,default=41);p.add_argument('--sim-seconds',type=float,default=600);p.add_argument('--wall-timeout',type=float,default=1800);p.add_argument('--headless',action='store_true');p.add_argument('--resume',type=Path,help='Resume finished cases only when every actual runtime input hash matches');a=p.parse_args()
    if a.mode=='stop':
        r=stop_session(SESSION,60);print(json.dumps(r));return 0 if r['status'] in ['STOPPED','NOT_RUNNING'] else 1
    if a.seed!=41:p.error('qualification table frozen at seed 41')
    if not all(math.isfinite(x) and x>0 for x in [a.sim_seconds,a.wall_timeout]):p.error('finite positive budgets required')
    if a.mode=='test' and (a.suite!='acceptance' or a.case or a.sim_seconds<455):p.error('test requires --suite acceptance and >=455 total simulation seconds')
    if a.mode=='demo' and (not a.case or a.suite or a.resume):p.error('demo requires --case and cannot use --suite/--resume')
    if a.case=='restart_relocalize' and a.sim_seconds<285:p.error('restart and three positions need >=285 total simulation seconds')
    if a.wall_timeout<60:p.error('wall timeout must be >=60')
    if subprocess.run(['bash',str(BASE/'stepctl.sh'),'check','S06'],cwd=ROOT).returncode:return 4
    verify_bundle();lock=(ROOT/'native_20261108/runs/session.lock').open('a')
    try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    except BlockingIOError:print('已有项目仿真持有共享会话锁。');return 2
    if os.getpgrp()!=os.getpid():os.setsid()
    run_id=a.mode+'_'+datetime.datetime.now().strftime('%Y%m%d_%H%M%S')+'_'+str(os.getpid());out=RESULTS/run_id;out.mkdir(parents=True);save_session(SESSION,str(Path(__file__).resolve()),run_id);session=read(SESSION);session.update(pgid=os.getpgrp(),sid=os.getsid(0),domains={'online':119,'evaluation':120,'raw':121},seed=a.seed,map_id=verify_bundle()['map_id']);dump(SESSION,session)
    def interrupt(s,f):raise KeyboardInterrupt
    signal.signal(signal.SIGINT,interrupt);signal.signal(signal.SIGTERM,interrupt);start=time.monotonic();data=dict(run_id=run_id,mode=a.mode,seed=a.seed,status='PARTIAL',checks={f'S07-C{i:02}':'NOT_RUN' for i in range(1,5)},cases={},next_step_verdict='暂不能执行下一步：S07 未完整通过',command=sys.argv)
    def collect(name,pose_name,case,seconds):
        candidate=a.resume.resolve()/name if a.resume else None
        if a.resume and (a.resume/'result.json').exists():
            prior=read(a.resume/'result.json')
            if name in prior.get('cases',{}):candidate=ROOT/prior['cases'][name]['path']
        if candidate is not None:candidate=candidate.resolve()
        reuse=candidate is not None and (candidate/'runtime.json').exists();case_out=candidate if reuse else out/name
        if reuse:
            if not case_out.is_relative_to(ROOT):raise ValueError('capture outside project')
            rt=read(case_out/'runtime.json')
            if not rt['success'] or rt['pose_name']!=pose_name or rt['case']!=case or rt['seconds']<seconds or rt['seed']!=a.seed:raise ValueError('resume case/seed/budget mismatch')
            if read(case_out/'capture_inputs.json')!=capture_inputs():raise ValueError('resume runtime inputs changed')
            data.setdefault('resumed_captures',{})[name]=str(case_out.relative_to(ROOT));print('RESUME '+name,flush=True)
        else:
            remaining=a.wall_timeout-(time.monotonic()-start)-45
            if remaining<seconds+30:raise TimeoutError('insufficient wall budget for next case')
            print('RUN '+name,flush=True);run_case(case_out,pose_name,case,seconds,min(remaining,seconds*2+70),a.seed,a.mode=='demo' and not a.headless)
        m=analyze(case_out);data['cases'][name]=dict(path=str(case_out.relative_to(ROOT)),metrics=m);dump(out/'progress.json',data);print(('PASS ' if m['passed'] else 'FAIL ')+name+' '+str({k:v for k,v in m['checks'].items() if not v}),flush=True)
    try:
        command(out,'dependencies',[sys.executable,str(PKG/'tests/dependencies.py'),str(out/'environment_verified.json')],30)
        command(out,'build',['bash',str(BASE/'scripts/build_s07.sh')],120)
        command(out,'contracts',[sys.executable,str(PKG/'tests/contracts.py'),str(out/'contracts')],40);data['contracts']=read(out/'contracts/contracts.json')
        command(out,'refinement_contracts',[sys.executable,str(PKG/'tests/refinement.py')],20)
        command(out,'manager_contracts',['/usr/bin/python3',str(PKG/'tests/manager_contracts.py'),str(out/'manager_contracts')],35)
        command(out,'lesson_compile',['g++','-std=c++17','-O2','-I/usr/include/eigen3',str(BASE/'lessons/S07/lesson.cpp'),'-o',str(BASE/'ws/build/global_pose_manager/lesson_s07')],45)
        command(out,'lesson_cpp',[str(BASE/'ws/build/global_pose_manager/lesson_s07')],10);command(out,'lesson_python',['/usr/bin/python3',str(BASE/'lessons/S07/lesson.py')],10)
        if a.mode=='test':
            for name in ['zone_a','connector','zone_b']:collect(name,name,'normal',80)
            collect('restart_occlusion','zone_a','restart_occlusion',125);collect('wrong_initial','zone_a','wrong_initial',90);data['checks'],passed=aggregate(data);data['status']='PASS' if passed else 'FAIL'
            if passed:data['next_step_verdict']='可以执行下一步 S08：S07 四项自动验收通过且 S06 有效；不自动执行'
        elif a.case=='restart_relocalize':
            collect('restart_occlusion','zone_a','restart_occlusion',125);collect('connector','connector','normal',80);collect('zone_b','zone_b','normal',80);data['status']='PASS' if all(x['metrics']['passed'] for x in data['cases'].values()) else 'FAIL';data['next_step_verdict']='独立演示记录；正式前置状态用 stepctl next S07 检查'
        else:data['status']='PASS' if data['contracts']['passed'] else 'FAIL';data['next_step_verdict']='离线版本拒绝演示，无仿真时钟；正式前置状态用 stepctl next S07 检查'
    except KeyboardInterrupt:data.update(status='PARTIAL',error='用户中断；已停车并关闭本会话进程')
    except Exception as exc:data.update(status='FAIL',error=repr(exc));(out/'exception.log').write_text(traceback.format_exc());print(traceback.format_exc(),flush=True)
    finally:
        try:
            data['wall_seconds']=time.monotonic()-start;dump(out/'result.json',data)
            try:render(out,data);command(out,'browser_check',[sys.executable,str(PKG/'tests/report_check.py'),str(out/'index.html')],45)
            except Exception as exc:data.update(status='FAIL',error='report: '+repr(exc),next_step_verdict='暂不能执行下一步：报告检查失败')
            for c in data['cases'].values():compact(ROOT/c['path'])
            compact(out);dump(out/'result.json',data)
            if a.mode=='test':receipt(out,data)
            else:
                dump(RESULTS/'last_demo.json',dict(run_id=run_id,status=data['status'],report_path=str((out/'index.html').relative_to(ROOT))))
                if not a.headless:subprocess.Popen(['xdg-open',str(out/'index.html')],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL,start_new_session=True)
        finally:SESSION.unlink(missing_ok=True);lock.close()
    print(json.dumps(dict(status=data['status'],report=str(out/'index.html'),next=data['next_step_verdict']),ensure_ascii=False));return 0 if data['status']=='PASS' else 1
if __name__=='__main__':raise SystemExit(main())
