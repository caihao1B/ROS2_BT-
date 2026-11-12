"""Public bounded S05 executor; honest measured acceptance and version-bound receipts."""
import argparse,datetime,fcntl,hashlib,os,signal,subprocess,sys,time,traceback,gzip,shutil
from support import *
from analyze import analyze
from report import render
from runtime import run_case
sys.path.insert(0,str(BASE/'ws/src/project_baseline/scripts'))
from session_owner import save_session,stop_session
RESULTS=BASE/'results/S05';SESSION=BASE/'runtime/S05.json'
def files(path):return [p for p in path.rglob('*') if p.is_file() and '__pycache__' not in p.parts]
def row(p):return dict(path=str(p.relative_to(ROOT)),sha256=hashlib.sha256(p.read_bytes()).hexdigest())
def interrupt(s,f):raise KeyboardInterrupt

def command(out,name,args,timeout=120):
    start=time.monotonic()
    with (out/(name+'.log')).open('w') as log:
        proc=subprocess.Popen(args,cwd=ROOT,stdout=log,stderr=subprocess.STDOUT)
        try:rc=proc.wait(timeout=timeout)
        except BaseException:
            proc.send_signal(signal.SIGINT)
            try:proc.wait(timeout=5)
            except subprocess.TimeoutExpired:proc.kill();proc.wait()
            raise
    with (out/'commands.jsonl').open('a') as f:f.write(json.dumps(dict(name=name,args=args,exit_code=rc,wall_seconds=time.monotonic()-start))+'\n')
    if rc:raise RuntimeError(name+' failed; see log')

def compact_evidence(out):
    # Preserve every observation losslessly; bounded runtime no longer needs JSONL writers.
    count=before=after=0
    for p in out.glob('*/**/*.jsonl'):
        if p.parent==out:continue
        before+=p.stat().st_size;target=Path(str(p)+'.gz')
        with p.open('rb') as src,gzip.open(target,'wb',compresslevel=6) as dst:shutil.copyfileobj(src,dst)
        after+=target.stat().st_size;p.unlink();count+=1
    return dict(files=count,before_bytes=before,after_bytes=after,lossless=True)

def receipt(out,d):
    docs=ROOT/'docs/optimization_v2/S05';text=f'# S05 实测结果\n\n运行 `{d["run_id"]}`：**{d["status"]}**。{d["next_step_verdict"]}。人工查看 NOT_CHECKED。\n\n'
    text+='\n'.join(f'- {k}：{v}' for k,v in d['checks'].items())+'\n\n'
    text+=f'[交互报告]({out}/index.html) · [汇总指标]({out}/result.json)。完整进程命令、PID、退出码在各案例 runtime.json；各门槛和未通过项在 metrics.json。\n\n'
    if d.get('error'):text+='失败/中断：'+d['error']+'\n\n'
    text+='| 案例 | 状态 | 物理路程 m | 轮速 ATE m | 轮速+IMU ATE m | 融合 ATE m | RTF |\n|---|---|---:|---:|---:|---:|---:|\n'
    for k,m in d['cases'].items():
        score=lambda key:m.get('trajectories',{}).get(key,{}).get('ate_rmse_m',float('nan'))
        text+=f'| {k} | {"PASS" if m.get("passed") else "FAIL"} | {m.get("physical_distance_m",0):.2f} | {score("wheel"):.3f} | {score("wheel_imu"):.3f} | {score("fused"):.3f} | {m.get("rtf",0):.3f} |\n'
    for k,m in d['cases'].items():
        if m.get('faults'):text+='\n'+k+' 故障指标：\n```json\n'+json.dumps(m['faults'],ensure_ascii=False,indent=2)+'\n```\n\n'
    if d.get('targeted_validation'):text+='\n补充复验说明：'+d['targeted_validation']['reason']+'\n\n原失败案例保留在 superseded 中；它们没有被改写成 PASS。补充命令和退出码在 commands.jsonl；完整启动命令仍在各案例 runtime.json。\n'
    if d.get('evidence_compaction'):text+='\n观测 JSONL 已无损 gzip 压缩；评分器直接支持解压读取，未丢弃采样。\n'
    text+='\n固定同一起点对齐；全程 ATE 与 20–50 s 打滑窗口分开统计。没有按段拟合误差。真实 IMU/轮编码器/激光观测进入估计，domain 120 真值只用于离线评分。默认不使用 GNSS、地图回环或视觉定位。\n\n在线可查量是输入质量、协方差、轮速与独立激光速度一致性残差；实际 EKF 内部创新另由链接已安装 C++ 引擎的有限探针展示，不冒充完整实车 EKF 内部创新日志。\n\n局限：15 状态普通 EKF，没有在线 IMU 偏置状态；静止启动校准；无逐点激光时间戳，未做运动去畸变；稀疏农田可能停车，不能视为全场定位保证。RTAB-Map 自身配准参数选择及被否定方案见 TECHNICAL.md。\n'
    (docs/'RESULTS.md').write_text(text)
    sources=files(PKG)+[BASE/'config'/k for k in ['local_estimator.yaml','icp_odometry.yaml','tf_ownership.yaml','localization_policy.yaml','s05_dependency_lock.json']]+[BASE/'config/rviz/odometry.rviz',BASE/'scripts/build_s05.sh',BASE/'scripts/install_s05_dependencies.sh',BASE/'stages/S05/run.sh']+files(BASE/'references/S05')+[BASE/'ws/install/localization_guard/lib/localization_guard'/k for k in ['health_guard','ekf_probe']]
    arts=files(out)+files(docs)+files(BASE/'lessons/S05')
    if (RESULTS/'development_summary.json').exists():arts.append(RESULTS/'development_summary.json')
    for delivered in d.get('delivery_runs',[]):arts+=files(ROOT/delivered)
    r=dict(schema_version=1,step='S05',plan_revision='20261109.1',run_id=d['run_id'],checked_at_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),automated_status=d['status'],manual_status='NOT_CHECKED',reason=d.get('error','固定案例真实里程计、连续性、输入质量、退化和 epoch 检查'),check_exit_code=0 if d['status']=='PASS' else 1,checks=d['checks'],report_path=str((out/'index.html').relative_to(ROOT)),source_files=[row(p) for p in sorted(set(sources))],artifacts=[row(p) for p in sorted(set(arts))],dependency_receipts={'S04':row(BASE/'results/S04/latest.json')['sha256']})
    dump(RESULTS/'latest.json',r)

def aggregate(cases):
    required={k+'_'+c for k in ['diff','ackermann'] for c in ['loop','faults','slope','imu_loss']}|{'diff_reset'}
    groups={1:['completed','monotonic_stamps','sole_tf_owner','correct_tf_tree'],2:['covariance_finite_psd','innovation_observable','gravity_compensation','planning_projection'],3:['three_trajectories','odom_continuous','physical_motion','no_obstacle_contact'],4:['truth_isolation','pure_icp','sole_actuator_owner','resource_bounds']}
    checks={f'S05-C{i:02}':'PASS' if required<=set(cases) and all(all(cases[n].get('checks',{}).get(k,False) for k in keys) for n in required) else 'FAIL' for i,keys in groups.items()}
    def need(code,name,keys):
        if not all(cases.get(name,{}).get('checks',{}).get(k,False) for k in keys):checks[code]='FAIL'
    need('S05-C01','diff_reset',['new_epoch_on_clock_rewind','reset_service_succeeded','new_epoch_initial_stop','new_epoch_recovers','all_estimators_reset','post_reset_accuracy'])
    for k in ['diff','ackermann']:
        need('S05-C02',k+'_slope',['measured_3d_slope'])
        need('S05-C03',k+'_loop',['nominal_accuracy','completed_loop'])
        need('S05-C03',k+'_slope',['slope_progress'])
        need('S05-C03',k+'_faults',['fusion_improves_slip'])
        need('S05-C04',k+'_faults',['icp_degradation_interface','imu_loss_interface','icp_physical_stop','imu_physical_stop','fault_changes_observations'])
        need('S05-C04',k+'_imu_loss',['independent_imu_stop_interface','independent_imu_physical_stop','moving_before_imu_loss','physical_moving_before_imu_loss'])
    passed=all(v=='PASS' for v in checks.values()) and all(cases[n].get('passed',False) for n in required if n in cases)
    return checks,passed

def main():
    p=argparse.ArgumentParser();p.add_argument('mode',choices=['test','demo','stop']);p.add_argument('--suite',choices=['acceptance']);p.add_argument('--case',choices=['odometry_loop','slip_and_icp_loss']);p.add_argument('--seed',type=int,default=41);p.add_argument('--sim-seconds',type=float,default=300);p.add_argument('--wall-timeout',type=float,default=1500);p.add_argument('--vehicle',choices=['diff','ackermann'],default='diff');p.add_argument('--gui',action='store_true');p.add_argument('--headless',action='store_true');a=p.parse_args()
    if a.mode=='stop':
        r=stop_session(SESSION);print(json.dumps(r));return 0 if r['status'] in ['STOPPED','NOT_RUNNING'] else 1
    if a.seed!=41:p.error('S05 qualified fault table is frozen at seed 41')
    if a.mode=='test' and a.sim_seconds<110:p.error('acceptance requires at least 110 sim seconds budget per case')
    if a.mode=='demo' and (not a.case or a.sim_seconds<(110 if a.case=='slip_and_icp_loss' else 120)):p.error('demo requires --case and enough time for the complete closed loop/fault sequence')
    if a.wall_timeout<60:p.error('wall-timeout must be >=60')
    if subprocess.run(['bash',str(BASE/'stepctl.sh'),'check','S04'],cwd=ROOT).returncode:return 4
    lock=(ROOT/'native_20261108/runs/session.lock').open('a')
    try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    except BlockingIOError:print('已有项目仿真持有共享会话锁。');return 2
    if os.getpgrp()!=os.getpid():os.setsid()
    run_id=a.mode+'_'+datetime.datetime.now().strftime('%Y%m%d_%H%M%S')+'_'+str(os.getpid());out=RESULTS/run_id;out.mkdir(parents=True);save_session(SESSION,str(Path(__file__).resolve()),run_id)
    session=read(SESSION);session.update(pgid=os.getpgrp(),sid=os.getsid(0),seed=a.seed,config_hashes={k:row(BASE/'config'/k)['sha256'] for k in ['local_estimator.yaml','icp_odometry.yaml','localization_policy.yaml']},domains={'online':119,'evaluation':120,'raw':121});dump(SESSION,session)
    signal.signal(signal.SIGINT,interrupt);signal.signal(signal.SIGTERM,interrupt);start=time.monotonic();data=dict(run_id=run_id,kind=a.mode,seed=a.seed,status='PARTIAL',checks={f'S05-C{i:02}':'NOT_RUN' for i in range(1,5)},cases={},next_step_verdict='暂不能执行下一步：S05 未完整通过')
    def run(kind,case):
        name=kind+'_'+case;seconds=min(a.sim_seconds,dict(loop=120,faults=110,slope=70,reset=32,imu_loss=45)[case]);remaining=a.wall_timeout-(time.monotonic()-start)-30
        if remaining<50:raise TimeoutError('remaining wall budget insufficient')
        print('RUN '+name,flush=True);run_case(out/name,kind,case,seconds,min(remaining,max(150,seconds*2.3)),a.seed,(a.mode=='demo' and not a.headless) or (a.gui and kind=='diff' and case=='loop'))
        try:m=analyze(out/name)
        except Exception as exc:m=dict(passed=False,error=repr(exc));dump(out/name/'metrics.json',m)
        data['cases'][name]=m;dump(out/'progress.json',data);print(('PASS ' if m['passed'] else 'FAIL ')+name+' '+str({k:v for k,v in m.get('checks',{}).items() if not v}),flush=True)
    try:
        command(out,'build',['bash',str(BASE/'scripts/build_s05.sh')]);command(out,'math_checks',[sys.executable,str(PKG/'tests/math_checks.py')]);command(out,'lesson_cpp',[str(BASE/'ws/install/localization_guard/lib/localization_guard/ekf_probe'),str(out/'ekf_internal_trace.txt')]);command(out,'lesson_python',[sys.executable,str(BASE/'lessons/S05/lesson.py')])
        if a.mode=='demo':
            run(a.vehicle,'loop' if a.case=='odometry_loop' else 'faults');data['status']='PASS' if all(m['passed'] for m in data['cases'].values()) else 'FAIL';data['next_step_verdict']='演示结果不覆盖正式验收；请用 stepctl next S05 查看'
        else:
            run('diff','reset')
            for case in ['slope','loop','faults','imu_loss']:
                for kind in ['diff','ackermann']:run(kind,case)
            data['checks'],passed=aggregate(data['cases'])
            data['status']='PASS' if passed else 'FAIL'
            if data['status']=='PASS':data['next_step_verdict']='可以执行下一步 S06：S05 固定矩阵通过且 S04 有效；不自动执行'
    except KeyboardInterrupt:data['status']='PARTIAL';data['error']='用户中断；已停止本会话并保存输出'
    except Exception as exc:data['status']='FAIL';data['error']=repr(exc);(out/'exception.log').write_text(traceback.format_exc());print(traceback.format_exc(),flush=True)
    finally:
        try:
            data['wall_seconds']=time.monotonic()-start;dump(out/'result.json',data)
            try:
                render(out,data)
                if data['cases']:command(out,'browser_check',['/usr/bin/python3',str(PKG/'tests/report_check.py'),str(out/'index.html')],45)
            except Exception as exc:data.update(status='FAIL',error='report: '+repr(exc));dump(out/'result.json',data)
            data['evidence_compaction']=compact_evidence(out);dump(out/'result.json',data)
            if a.mode=='test':receipt(out,data)
            else:dump(RESULTS/'last_demo.json',dict(run_id=run_id,status=data['status'],report_path=str((out/'index.html').relative_to(ROOT))))
        finally:SESSION.unlink(missing_ok=True);lock.close()
    print(json.dumps(dict(status=data['status'],report=str(out/'index.html'),next=data['next_step_verdict']),ensure_ascii=False));return 0 if data['status']=='PASS' else 1
if __name__=='__main__':raise SystemExit(main())
