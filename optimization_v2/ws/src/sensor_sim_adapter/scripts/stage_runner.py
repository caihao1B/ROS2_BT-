"""S04 public executor and receipts from measured observations."""
import argparse,datetime,fcntl,hashlib,os,signal,subprocess,sys,time,traceback
from common import *
from runtime import run_case
from analyze import analyze
from report import render
sys.path.insert(0,str(BASE/'ws/src/project_baseline/scripts'))
from session_owner import save_session,stop_session
RESULTS=BASE/'results/S04';SESSION=BASE/'runtime/S04.json'
def interrupt(s,f):raise KeyboardInterrupt
def files(path):return [p for p in path.rglob('*') if p.is_file() and '__pycache__' not in p.parts]
def row(p):return dict(path=str(p.relative_to(ROOT)),sha256=hashlib.sha256(p.read_bytes()).hexdigest())
def command(out,name,args,timeout=120):
    start=time.monotonic()
    with (out/(name+'.log')).open('w') as log:
        p=subprocess.Popen(args,cwd=ROOT,stdout=log,stderr=subprocess.STDOUT)
        try:rc=p.wait(timeout=timeout)
        except BaseException:
            p.send_signal(signal.SIGINT)
            try:p.wait(timeout=5)
            except subprocess.TimeoutExpired:p.kill();p.wait()
            raise
    with (out/'commands.jsonl').open('a') as f:f.write(json.dumps(dict(name=name,args=args,exit_code=rc,wall_seconds=time.monotonic()-start))+'\n')
    if rc:raise RuntimeError(name+' failed; see log')
def receipt(out,data):
    docs=ROOT/'docs/optimization_v2/S04';text=f'# S04 实测结果\n\n运行 `{data["run_id"]}`：**{data["status"]}**。\n\n{data["next_step_verdict"]}。人工目视状态 NOT_CHECKED。\n\n'
    text+='\n'.join(f'- {k}：{v}' for k,v in data['checks'].items())+'\n\n'
    text+=f'[交互报告]({out}/index.html) · [完整指标]({out}/result.json)。实际命令及退出码在 commands.jsonl，各场景 runtime.json 包含完整进程命令、域、PID、退出码。\n\n'
    if data.get('error'):text+='失败/中断原因：'+data['error']+'\n\n'
    text+='| 案例 | 状态 | RTF | 峰值进程 RSS GiB |\n|---|---|---|---|\n'
    for name,m in data['cases'].items():text+=f'| {name} | {"PASS" if m.get("passed") else "FAIL"} | {m.get("rtf",0):.3f} | {m.get("peak_owned_rss_bytes",0)/2**30:.2f} |\n'
    text+='\n| 案例 | 激光 Hz | 左/右目 Hz | IMU / 轮速 / GNSS Hz | 录包 MiB |\n|---|---|---|---|---|\n'
    for name,m in data['cases'].items():
        hz=lambda k:m.get('rates',{}).get(k,{}).get('sim_hz',0)
        text+=f'| {name} | {hz("points"):.2f} | {hz("left"):.2f} / {hz("right"):.2f} | {hz("imu"):.1f} / {hz("wheel"):.1f} / {hz("gnss"):.1f} | {m.get("bag_disk_bytes",0)/2**20:.2f} |\n'
    text+='\n故障案例频率只统计故障前稳态。双目配对最大时间差、定位/感知支路实收频率、CPU/GPU 资源及掉帧缺口见完整指标。录包已实际解码回读。\n\n'
    for name,m in data['cases'].items():
        if m.get('faults'):text+=f'`{name}` 的实际故障指标：\n```json\n'+json.dumps(m['faults'],ensure_ascii=False,indent=2)+'\n```\n\n'
    if data.get('delivery_checks'):text+='附加公开交付验证：\n```json\n'+json.dumps(data['delivery_checks'],ensure_ascii=False,indent=2)+'\n```\n\n'
    text+='\n保持原始目标：雷达 32×1024/10 Hz、双目 640×360/15 Hz、IMU 200 Hz、轮速 100 Hz、GNSS 5 Hz。未静默降配。此前单队列方案未达高频目标；已分离视觉与惯性生成进程并异步保存 PNG。延迟恢复后的过时包丢弃并记数。\n\n本步只验证观测和接口，不代表融合定位/SLAM/动态避障通过。GNSS 假固定解及轮滑为受控测量失配；源文件、有效配置及真值隔离边界见 TECHNICAL.md。\n'
    docs.mkdir(parents=True,exist_ok=True);(docs/'RESULTS.md').write_text(text)
    sources=files(PKG)+[BASE/'config'/name for name in ['sensors.yaml','fault_profiles.yaml','data_contract.yaml','sensor_replay_qos.yaml']]+[BASE/'config/rviz/sensors.rviz',BASE/'scripts/build_s04.sh',BASE/'stages/S04/run.sh',BASE/'ws/install/sensor_sim_adapter/lib/libs04_joint_observer.so']
    artifacts=files(out)+files(docs)+files(BASE/'lessons/S04')
    for run in data.get('delivery_runs',[]):artifacts+=files(ROOT/run)
    r=dict(schema_version=1,step='S04',plan_revision='20261109.1',run_id=data['run_id'],checked_at_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),automated_status=data['status'],manual_status='NOT_CHECKED',reason=data.get('error','五类真实观测、频率、故障注入、隔离和资源边界实测'),check_exit_code=0 if data['status']=='PASS' else 1,checks=data['checks'],report_path=str((out/'index.html').relative_to(ROOT)),source_files=[row(p) for p in sorted(set(sources)) if p.exists()],artifacts=[row(p) for p in sorted(set(artifacts))],dependency_receipts={'S03':row(BASE/'results/S03/latest.json')['sha256']})
    dump(RESULTS/'latest.json',r)
def main():
    p=argparse.ArgumentParser();p.add_argument('mode',choices=['test','demo','stop']);p.add_argument('--suite',choices=['acceptance']);p.add_argument('--case',choices=['sensor_panel','sensor_faults']);p.add_argument('--seed',type=int,default=41);p.add_argument('--sim-seconds',type=float,default=300);p.add_argument('--wall-timeout',type=float,default=1800);p.add_argument('--gui',action='store_true');p.add_argument('--headless',action='store_true');a=p.parse_args()
    if a.mode=='stop':
        r=stop_session(SESSION);print(json.dumps(r));return 0 if r['status'] in ['STOPPED','NOT_RUNNING'] else 1
    if a.mode=='test' and a.sim_seconds<90:p.error('acceptance requires >=90 simulation seconds budget per case')
    if a.mode=='demo' and not a.case:p.error('--case required')
    if a.mode=='demo' and a.sim_seconds<(85 if a.case=='sensor_faults' else 20):p.error('not enough time for requested observations/fault sequence')
    if subprocess.run(['bash',str(BASE/'stepctl.sh'),'check','S03'],cwd=ROOT).returncode:return 4
    lock=(ROOT/'native_20261108/runs/session.lock').open('a')
    try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    except BlockingIOError:print('已有项目仿真持有共享会话锁。');return 2
    run_id=a.mode+'_'+datetime.datetime.now().strftime('%Y%m%d_%H%M%S')+'_'+str(os.getpid());out=RESULTS/run_id;out.mkdir(parents=True);save_session(SESSION,str(Path(__file__).resolve()),run_id)
    session=read(SESSION);session.update(seed=a.seed,config_hash=config_hash(),plan_revision='20261109.1',domains={'online':119,'evaluation':120,'raw':121});dump(SESSION,session)
    signal.signal(signal.SIGINT,interrupt);signal.signal(signal.SIGTERM,interrupt);start=time.monotonic();data=dict(run_id=run_id,kind=a.mode,seed=a.seed,status='PARTIAL',checks={f'S04-C{i:02}':'NOT_RUN' for i in range(1,5)},cases={},next_step_verdict='暂不能执行下一步：S04 未完整通过')
    def run(kind,profile):
        name=kind+'_'+profile;seconds=min(a.sim_seconds,90 if profile=='sensor_faults' else 60 if a.mode=='demo' else 50);remaining=a.wall_timeout-(time.monotonic()-start)-30
        if remaining<40:raise TimeoutError('remaining wall budget insufficient')
        print('RUN '+name,flush=True);run_case(out/name,kind,profile,seconds,min(remaining,max(150,seconds*2)),a.seed,(a.mode=='demo' and not a.headless) or (a.gui and kind=='diff'))
        try:m=analyze(out/name)
        except Exception as exc:m=dict(passed=False,error=repr(exc));dump(out/name/'metrics.json',m)
        data['cases'][name]=m;dump(out/'progress.json',data);print(('PASS ' if m['passed'] else 'FAIL ')+name+' '+str({k:v for k,v in m.get('checks',{}).items() if not v}),flush=True)
        if not m['passed']:raise RuntimeError(name+' failed; inspect measured metrics')
    try:
        command(out,'units',[sys.executable,str(PKG/'tests/unit_checks.py')]);command(out,'build',['bash',str(BASE/'scripts/build_s04.sh')]);command(out,'legacy_sensor_geometry',['/usr/bin/python3',str(ROOT/'native_20261108/ros2_ws/src/dino_sensors/tests/test_geometry.py')])
        lessons=BASE/'lessons/S04';command(out,'lesson_cpp_build',['g++','-std=c++17','-Wall','-Wextra',str(lessons/'lesson.cpp'),'-o',str(out/'lesson_cpp')]);command(out,'lesson_cpp',[str(out/'lesson_cpp')]);command(out,'lesson_python',[sys.executable,str(lessons/'lesson.py')])
        if a.mode=='demo':run('diff','sensor_faults' if a.case=='sensor_faults' else 'nominal');data['status']='PASS';data['next_step_verdict']='演示通过；正式步骤状态请用 stepctl next S04 核对'
        else:
            for profile in ['nominal','sensor_faults']:
                for kind in ['diff','ackermann']:run(kind,profile)
            data['checks']={k:'PASS' for k in data['checks']};data['status']='PASS';data['next_step_verdict']='可以执行下一步 S05：多传感器观测、频率与误差注入通过，S03 前置有效'
    except KeyboardInterrupt:data['status']='PARTIAL';data['error']='用户中断；已停车并清理本会话'
    except Exception as exc:data['status']='FAIL';data['error']=repr(exc);(out/'exception.log').write_text(traceback.format_exc());print(traceback.format_exc(),flush=True)
    finally:
        try:
            data['wall_seconds']=time.monotonic()-start;dump(out/'result.json',data)
            try:
                render(out,data)
                if data['status']=='PASS':
                    command(out,'browser_check',['/usr/bin/python3',str(PKG/'tests/report_check.py'),str(out/'index.html')],45)
                    data['browser_check']=read(out/'browser_check.json');dump(out/'result.json',data)
                    if not all((ROOT/'docs/optimization_v2/S04'/f).exists() for f in ['TECHNICAL.md','MANUAL_CHECK.md','CODE_WALKTHROUGH.md']):raise RuntimeError('required learning/manual documents absent')
            except Exception as exc:
                data.update(status='FAIL',error='report failed: '+repr(exc),next_step_verdict='暂不能执行下一步：报告失败');dump(out/'result.json',data);(out/'index.html').write_text('<meta charset="utf-8"><h1>S04 FAIL</h1><p>See result.json</p>')
            if a.mode=='test':receipt(out,data)
            else:dump(RESULTS/'last_demo.json',dict(run_id=run_id,status=data['status'],report_path=str((out/'index.html').relative_to(ROOT))))
        finally:SESSION.unlink(missing_ok=True);lock.close()
    print(json.dumps(dict(status=data['status'],report=str(out/'index.html'),next=data['next_step_verdict']),ensure_ascii=False));return 0 if data['status']=='PASS' else 1
if __name__=='__main__':raise SystemExit(main())
