"""S03 real acceptance, derived qualifications, versioned receipts and manual demos."""
import argparse,datetime,fcntl,hashlib,json,os,signal,subprocess,sys,time,traceback
from pathlib import Path
from runtime import ROOT,BASE,run_case
from analyze import analyze,read
from report import render
sys.path.insert(0,str(BASE/'ws/src/project_baseline/scripts'));from session_owner import save_session,stop_session
PKG=Path(__file__).resolve().parents[1];DESC=BASE/'ws/src/robot_description';RESULTS=BASE/'results/S03';SESSION=BASE/'runtime/S03.json'
def interrupt(sig,frame):raise KeyboardInterrupt
def files(path):return [p for p in path.rglob('*') if p.is_file() and '__pycache__' not in p.parts]
def file_row(p):return dict(path=str(p.relative_to(ROOT)),sha256=hashlib.sha256(p.read_bytes()).hexdigest())
def dump(p,d):p.parent.mkdir(parents=True,exist_ok=True);p.write_text(json.dumps(d,ensure_ascii=False,indent=2)+'\n')
def command(out,name,args,timeout=120):
    start=time.monotonic()
    with (out/(name+'.log')).open('w') as log:
        p=subprocess.Popen(args,stdout=log,stderr=subprocess.STDOUT,cwd=ROOT)
        try:rc=p.wait(timeout=timeout)
        except BaseException:
            p.send_signal(signal.SIGINT)
            try:p.wait(timeout=5)
            except subprocess.TimeoutExpired:p.kill();p.wait()
            raise
    with (out/'commands.jsonl').open('a') as f:f.write(json.dumps(dict(name=name,argv=args,exit_code=rc,wall_seconds=time.monotonic()-start))+'\n')
    if rc:raise RuntimeError(f'{name} failed: {out/name}.log')
def qualify(data):
    result=dict(schema='S03-QUALIFIED/1',scope='simulator model only; fresh stage receipt required',map_version=read(BASE/'maps/prior/map_metadata.json')['map_version'],vehicles={})
    for kind in ['diff','ackermann']:
        from generate import config
        c=config(kind);bench=data['cases'][kind+'_bench'];speeds=[float(k.split('_',1)[1]) for k,v in bench['motion'].items() if k.startswith('straight_') and v['passed']]
        turns=[v['radius_m'] for k,v in bench['motion'].items() if k.startswith('minimum_radius') and v['passed']]
        result['vehicles'][kind]=dict(vehicle_revision=c['revision'],config_hash=c['config_hash'],validated_flat_speeds_mps=sorted(speeds),selected_work_mps=c['qualification_candidates']['work_mps'],selected_flat_transit_mps=max(speeds),connector_mps=c['qualification_candidates']['connector_mps'],dynamic_avoidance='NOT_QUALIFIED; actors parked for chassis trials',slope_qualification=dict(degrees=5,max_speed_mps=.4),physical_min_radius_m=turns if turns else 0,planning_min_radius_m=math.ceil(max(turns)*1.05*100)/100 if turns else 0,footprint=c['footprint'],tool_profile=c['tool'],higher_speeds='NOT_TESTED',steeper_slopes='NOT_TESTED',real_hardware='NOT_CALIBRATED')
    return result
def receipt(out,data):
    docs=ROOT/'docs/optimization_v2/S03';docs.mkdir(parents=True,exist_ok=True)
    text=f'# S03 实测结果\n\n运行 `{data["run_id"]}`：**{data["status"]}**。\n\n**{data["next_step_verdict"]}**。人工状态 NOT_CHECKED。\n\n'+ '\n'.join(f'- {k}：{v}' for k,v in data['checks'].items())+f'\n\n[交互报告]({out}/index.html) · [本轮全部指标]({out}/result.json)。本轮所有构建、教学和原停止门限回归命令见 commands.jsonl，各仿真案例中的 runtime.json 保存实际命令、域、PID、退出码和耗时。\n\n'
    if data.get('error'):text+='未完成原因：'+data['error']+'\n\n'
    if data.get('qualified_profiles'):
        text+='## 本轮合格参数\n\n| 底盘 | 平地转场 | 连接路 | 坡面 | 规划最小半径 | 1.2 m/s 停车时间/距离 |\n|---|---|---|---|---|---|\n'
        for kind,c in data['qualified_profiles']['vehicles'].items():
            stop=data['cases'][kind+'_bench']['stops']['stop_1.2']
            text+=f'| {kind} | {c["selected_flat_transit_mps"]} m/s | {c["connector_mps"]} m/s | 5°、0.4 m/s | {c["planning_min_radius_m"]} m | {stop["delay_sim_s"]:.3f} s / {stop["distance_m"]:.3f} m |\n'
        text+='\n动态对象在 S03 试验副本中停放在道路外，动态避障尚未认证。\n\n'
    if data.get('delivery_checks'):text+='交付检查：'+json.dumps(data['delivery_checks'],ensure_ascii=False)+'\n\n'
    for name,m in data['cases'].items():
        text+=f'- {name}：'+('PASS' if m.get('passed') else 'FAIL')
        for k in ['cross_track_max_m','endpoint_error_m','min_footprint_road_margin_m','max_pitch_deg']:
            if k in m:text+=f'；{k}={m[k]:.5f}'
        text+='\n'
    text+='\n开发试验发现：名义轮距直接驱动会产生接触模型偏差；S02 原坡面朝内绕序不能正确支持轮胎。已保留校准依据、修复坡面并重验 S02。最终资格只来自本轮通过的数据。5°以外坡面、更高速度、实机电机制动和工具作业效果不属于本次通过范围。\n';(docs/'RESULTS.md').write_text(text)
    sources=set(files(PKG)+files(DESC)+files(BASE/'config/vehicles'));sources.update([BASE/'config/tool_profiles.yaml',BASE/'scripts/build_s03.sh',BASE/'stages/S03/run.sh',BASE/'env.sh',BASE/'stepctl.py',BASE/'stepctl.sh',BASE/'ws/install/chassis_bridge/lib/libs03_drive.so',ROOT/'native_20261108/ros2_ws/src/dino_control/config/control.json',ROOT/'field95/stop_observer.py'])
    calibration=read(BASE/'config/vehicles/diff_calibration.json')
    calibration_truth=ROOT/calibration['source_run']/'evaluation/truth.jsonl'
    if hashlib.sha256(calibration_truth.read_bytes()).hexdigest()!=calibration['source_truth_sha256']:
        raise ValueError('calibration evidence has changed')
    artifacts=files(out)+files(docs)+files(BASE/'lessons/S03')+[calibration_truth,ROOT/calibration['source_run']/'metrics.json']
    for run in data.get('delivery_runs',[]):artifacts+=files(ROOT/run)
    r=dict(schema_version=1,step='S03',plan_revision='20261109.1',run_id=data['run_id'],checked_at_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),automated_status=data['status'],manual_status='NOT_CHECKED',reason=data.get('error','物理关节、轮编码器、道路净空与停止故障矩阵实测；仅仿真模型资格'),check_exit_code=0 if data['status']=='PASS' else 1,checks=data['checks'],report_path=str((out/'index.html').relative_to(ROOT)),source_files=[file_row(p) for p in sorted(sources) if p.exists()],artifacts=[file_row(p) for p in sorted(set(artifacts))],dependency_receipts={'S02':file_row(BASE/'results/S02/latest.json')['sha256']})
    dump(RESULTS/'latest.tmp',r);(RESULTS/'latest.tmp').replace(RESULTS/'latest.json')
def main():
    p=argparse.ArgumentParser();p.add_argument('mode',choices=['test','demo','stop']);p.add_argument('--suite',choices=['acceptance']);p.add_argument('--case',choices=['diff_connector','ackermann_connector','watchdog_stop']);p.add_argument('--seed',type=int,default=41);p.add_argument('--sim-seconds',type=float,default=300);p.add_argument('--wall-timeout',type=float,default=1800);p.add_argument('--gui',action='store_true');p.add_argument('--headless',action='store_true');a=p.parse_args()
    if a.mode=='stop':
        r=stop_session(SESSION);print(json.dumps(r));return 0 if r['status'] in ['STOPPED','NOT_RUNNING'] else 1
    if a.mode=='test' and a.sim_seconds<210:p.error('acceptance requires >=210 simulation seconds per benchmark case')
    if a.mode=='demo' and not a.case:p.error('--case required')
    if a.wall_timeout<30:p.error('wall timeout >=30 required')
    if subprocess.run(['bash',str(BASE/'stepctl.sh'),'check','S02'],cwd=ROOT).returncode:return 4
    lock=(ROOT/'native_20261108/runs/session.lock').open('a')
    try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    except BlockingIOError:print('已有项目仿真持有共享会话锁。');return 2
    run_id=a.mode+'_'+datetime.datetime.now().strftime('%Y%m%d_%H%M%S')+'_'+str(os.getpid());out=RESULTS/run_id;out.mkdir(parents=True)
    save_session(SESSION,str(Path(__file__).resolve()),run_id);session=read(SESSION);session.update(seed=a.seed,map_version=read(BASE/'maps/prior/map_metadata.json')['map_version'],plan_revision='20261109.1',online_domain=119,evaluation_domain=120,gazebo_partition_prefix='s03_'+run_id);dump(SESSION,session)
    signal.signal(signal.SIGINT,interrupt);signal.signal(signal.SIGTERM,interrupt);start=time.monotonic()
    data=dict(run_id=run_id,kind=a.mode,seed=a.seed,status='PARTIAL',checks={f'S03-C{i:02}':'NOT_RUN' for i in range(1,5)},cases={},next_step_verdict='暂不能执行下一步：S03 尚未完整通过')
    def run(kind,case):
        name=kind+'_'+case;remaining=a.wall_timeout-(time.monotonic()-start)-30
        if remaining<30:raise TimeoutError('stage wall budget exhausted')
        print('RUN '+name,flush=True);r=run_case(out/name,kind,case,a.sim_seconds,min(remaining,max(120,a.sim_seconds*1.5)),(a.mode=='demo' and not a.headless) or (a.gui and case=='connector'),1 if a.mode=='demo' else 2,a.seed)
        try:m=analyze(out/name)
        except Exception as exc:m=dict(passed=False,error=repr(exc),checks={'complete_measurements':False});dump(out/name/'metrics.json',m)
        data['cases'][name]=m;dump(out/'progress.json',data);print(('PASS ' if m['passed'] else 'FAIL ')+name+' '+str({k:v for k,v in m.get('checks',{}).items() if not v}),flush=True)
        if not m['passed']:raise RuntimeError(name+' did not pass; inspect its measured metrics')
        return m
    try:
        command(out,'units',[sys.executable,str(PKG/'tests/unit_checks.py')]);command(out,'build',['bash',str(BASE/'scripts/build_s03.sh')])
        if a.mode=='demo':
            kind,case=('ackermann','command_loss') if a.case=='watchdog_stop' else (a.case.split('_')[0],'connector');run(kind,case);data['status']='PASS';data['next_step_verdict']='演示完成；请用 stepctl next S03 检查正式验收回执'
        else:
            command(out,'legacy_stop_tests',['/usr/bin/python3','-m','unittest','field95.test_stop_observer','field95.test_stop_acceptance'])
            lessons=BASE/'lessons/S03';command(out,'lesson_cpp_build',['g++','-std=c++17','-Wall','-Wextra',str(lessons/'lesson.cpp'),'-o',str(out/'lesson_cpp')]);command(out,'lesson_cpp',[str(out/'lesson_cpp')]);command(out,'lesson_python',[sys.executable,str(lessons/'lesson.py')])
            for case in ['connector','slope','bench']:
                for kind in ['diff','ackermann']:run(kind,case)
            run('diff','dock');data['checks']['S03-C01']='PASS';data['checks']['S03-C02']='PASS'
            for kind in ['diff','ackermann']:
                for case in ['command_loss','stale_timeout','cancel','actuator_loss','adapter_loss','feedback_loss','pause','reset','duplicate_owner']:run(kind,case)
            data['checks']['S03-C03']='PASS';data['qualified_profiles']=qualify(data);dump(BASE/'config/vehicles/qualified_profiles.json',data['qualified_profiles']);data['checks']['S03-C04']='PASS';data['status']='PASS';data['next_step_verdict']='可以执行下一步 S04：两种底盘、模型参数与停止故障矩阵通过，S02 前置有效'
    except KeyboardInterrupt:data['status']='PARTIAL';data['error']='运行被中断；已发送停止并清理本会话'
    except Exception as exc:data['status']='FAIL';data['error']=repr(exc);(out/'exception.log').write_text(traceback.format_exc());print(traceback.format_exc(),flush=True)
    finally:
        try:
            data['wall_seconds']=time.monotonic()-start;dump(out/'result.json',data)
            try:render(out,data)
            except Exception as exc:
                data['status']='FAIL';data['error']='report failed: '+repr(exc);data['next_step_verdict']='暂不能执行下一步：报告生成失败';dump(out/'result.json',data);(out/'index.html').write_text('<meta charset="utf-8"><h1>S03 FAIL</h1><p>请查看 result.json</p>')
            if a.mode=='test':receipt(out,data)
            else:dump(RESULTS/'last_demo.json',dict(run_id=run_id,status=data['status'],report_path=str((out/'index.html').relative_to(ROOT))))
        finally:SESSION.unlink(missing_ok=True);lock.close()
    print(json.dumps(dict(status=data['status'],report=str(out/'index.html'),next=data['next_step_verdict']),ensure_ascii=False),flush=True);return 0 if data['status']=='PASS' else 1
if __name__=='__main__':
    import math
    sys.path.insert(0,str(DESC/'scripts'))
    raise SystemExit(main())
