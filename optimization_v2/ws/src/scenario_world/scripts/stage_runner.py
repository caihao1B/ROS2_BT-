"""S02 stage entry. Actual scene probes, no prewritten PASS receipt."""
import argparse,datetime,fcntl,hashlib,json,os,signal,subprocess,sys,time,traceback,shutil
from pathlib import Path
from generate_scene import ROOT,BASE,generate,dump
from runtime import run_case
from validate_scene import offline,observed,reproducibility,read
from report import render
sys.path.insert(0,str(BASE/'ws/src/project_baseline/scripts'))
from session_owner import save_session,stop_session
PKG=Path(__file__).resolve().parents[1];RESULTS=BASE/'results/S02';SESSION=BASE/'runtime/S02.json'
def interrupt(sig,frame):raise KeyboardInterrupt
def file_row(p):return dict(path=str(p.relative_to(ROOT)),sha256=hashlib.sha256(p.read_bytes()).hexdigest())
def files(p):return [x for x in p.rglob('*') if x.is_file() and '__pycache__' not in x.parts]
def command(out,name,args,timeout=90):
    start=time.monotonic()
    with (out/(name+'.log')).open('w') as log:
        p=subprocess.Popen(args,stdout=log,stderr=subprocess.STDOUT,cwd=ROOT)
        try:rc=p.wait(timeout=timeout)
        except BaseException:
            p.send_signal(signal.SIGINT)
            try:p.wait(timeout=5)
            except subprocess.TimeoutExpired:p.kill();p.wait()
            raise
    row=dict(name=name,argv=args,exit_code=rc,wall_seconds=time.monotonic()-start)
    with (out/'commands.jsonl').open('a') as f:f.write(json.dumps(row)+'\n')
    if rc:raise RuntimeError(f'{name} failed; see {out/name}.log')
def write_receipt(out,data):
    docs=ROOT/'docs/optimization_v2/S02'
    (docs/'RESULTS.md').write_text('# S02 本轮结果\n\n'+f'运行 `{data["run_id"]}`：**{data["status"]}**。\n\n'+f'**{data["next_step_verdict"]}**。人工查看尚未记录（NOT_CHECKED）。\n\n'+ '\n'.join(f'- {k}：{v}' for k,v in data['checks'].items())+f'\n\n[本轮交互报告]({out}/index.html) · [完整实测数据]({out}/result.json)。\n\n'+
        '基础区面积 1650 / 1140 m²；连接中心线约 88.48 m、名义宽 6 m。粗坡度先验与精细真实坡面分开。三种变体未预切物理簇。\n\n'+
        '本步验证范围：世界加载、几何对齐、实体接触、动态重复性、临时传感器成像及 ROS 数据流隔离。未验证底盘通行、30 分钟作业、真实视觉识别和完整融合导航。物体由理想速度执行器驱动，不是行人交互/动力学预测模型；噪声和失效工况在后续传感器阶段补齐。\n')
    source=set(files(PKG))|set(files(BASE/'assets'))|set(files(BASE/'maps/prior'))|set(files(BASE/'evaluation'))
    source.update([BASE/'config/scenes.yaml',BASE/'config/sensor_config.json',BASE/'scripts/build_s02.sh',BASE/'stages/S02/run.sh',BASE/'env.sh',BASE/'stepctl.py',BASE/'stepctl.sh',BASE/'ws/install/scenario_world/lib/libs02_motion.so'])
    artifacts=files(docs)+files(BASE/'lessons/S02')+files(out)
    receipt=dict(schema_version=1,step='S02',plan_revision='20261109.1',run_id=data['run_id'],checked_at_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),automated_status=data['status'],manual_status='NOT_CHECKED',reason=data.get('error','实测几何、传感器、碰撞、重复性与隔离检查；底盘净空留 S03'),check_exit_code=0 if data['status']=='PASS' else 1,checks=data['checks'],report_path=str((out/'index.html').relative_to(ROOT)),source_files=[file_row(p) for p in sorted(source) if p.exists()],artifacts=[file_row(p) for p in sorted(set(artifacts))],dependency_receipts={'S01':file_row(BASE/'results/S01/latest.json')['sha256']})
    tmp=RESULTS/'latest.tmp';dump(tmp,receipt);tmp.replace(RESULTS/'latest.json')
def main():
    p=argparse.ArgumentParser();p.add_argument('mode',choices=['test','demo','stop']);p.add_argument('--suite',choices=['acceptance']);p.add_argument('--case',choices=['world_overview','prior_vs_unknown']);p.add_argument('--seed',type=int,default=41);p.add_argument('--sim-seconds',type=float,default=300);p.add_argument('--wall-timeout',type=float,default=1200);p.add_argument('--variant',choices=['base','notch','holes'],default='base');p.add_argument('--view',choices=['overview','ground'],default='overview');p.add_argument('--headless',action='store_true');a=p.parse_args()
    if a.mode=='stop':
        result=stop_session(SESSION);print(json.dumps(result));return 0 if result['status'] in ['STOPPED','NOT_RUNNING'] else 1
    if a.sim_seconds<=0 or a.wall_timeout<20:p.error('positive simulation duration and wall timeout >=20 required')
    if a.mode=='test' and a.sim_seconds<24:p.error('acceptance needs at least 24 simulation seconds per long case')
    if a.mode=='demo' and not a.case:p.error('--case required for demo')
    deps=subprocess.run(['bash',str(BASE/'stepctl.sh'),'check','S01'],cwd=ROOT)
    if deps.returncode:return deps.returncode
    lockpath=ROOT/'native_20261108/runs/session.lock';lockpath.parent.mkdir(parents=True,exist_ok=True);lock=lockpath.open('a')
    try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    except BlockingIOError:print('已有项目仿真持有共享会话锁，请先停止该会话。');return 2
    run_id=a.mode+'_'+datetime.datetime.now().strftime('%Y%m%d_%H%M%S')+'_'+str(os.getpid());out=RESULTS/run_id;out.mkdir(parents=True)
    save_session(SESSION,str(Path(__file__).resolve()),run_id)
    session=read(SESSION);session.update(seed=a.seed,variant=a.variant,map_version='s02.2',online_ros_domain=119,evaluation_ros_domain=120,gazebo_partition_prefix='s02_'+run_id,plan_revision='20261109.1');dump(SESSION,session)
    signal.signal(signal.SIGINT,interrupt);signal.signal(signal.SIGTERM,interrupt)
    data=dict(run_id=run_id,kind=a.mode,seed=a.seed,status='PARTIAL',checks={f'S02-C{i:02}':'NOT_RUN' for i in range(1,5)},metrics={},next_step_verdict='暂不能执行下一步：本轮验收尚未完整通过')
    chosen=out/'base_a';start=time.monotonic()
    def run(name,variant='base',seconds=18,fixture=False,gui=False,view='overview'):
        remaining=a.wall_timeout-(time.monotonic()-start)-15
        if remaining<25:raise TimeoutError('stage wall budget exhausted')
        print(f'RUN {name}: {seconds} simulation seconds, {variant}, GUI={gui}',flush=True)
        r=run_case(out/name,a.seed,variant,min(seconds,a.sim_seconds),min(remaining,max(seconds*3+30,90)),fixture,gui,view)
        if not r['success']:raise RuntimeError(f'{name} failed: '+str(r.get('error',r.get('observer_exit_codes'))))
        return observed(out/name)
    try:
        command(out,'build',['bash',str(BASE/'scripts/build_s02.sh')],120)
        import shapely
        dump(out/'dependency_versions.json',dict(python=sys.version,shapely=shapely.__version__,ros=os.environ.get('ROS_DISTRO'),gazebo_major=8,scene_config_sha256=file_row(BASE/'config/scenes.yaml')['sha256']))
        if a.mode=='demo':
            chosen=out/'demo';data['metrics']['demo']=run('demo',a.variant,a.sim_seconds,gui=a.case=='world_overview' and not a.headless,view=a.view)
            data['status']='PASS';data['next_step_verdict']='本次是演示；是否可进入 S03 请运行 stepctl next S02 检查正式回执'
        else:
            generate(BASE,a.seed,'base')
            data['metrics']['geometry']=offline(out/'geometry')
            lessons=BASE/'lessons/S02';command(out,'lesson_cpp_build',['g++','-std=c++17','-Wall','-Wextra',str(lessons/'lesson.cpp'),'-o',str(out/'lesson_cpp')])
            command(out,'lesson_cpp',[str(out/'lesson_cpp')]);command(out,'lesson_python',[sys.executable,str(lessons/'lesson.py')])
            for name,variant,seconds in [('base_a','base',24),('base_b','base',24),('notch','notch',8),('holes','holes',8)]:data['metrics'][name]=run(name,variant,seconds)
            data['checks']['S02-C01']='PASS';data['checks']['S02-C02']='PASS'
            data['metrics']['reproducibility']=reproducibility(out/'base_a',out/'base_b')
            data['metrics']['contact']=run('contact',seconds=12,fixture=True)
            # Both views are actual Gazebo windows captured in bounded independent runs.
            if os.environ.get('DISPLAY'):
                for view in ['overview','ground']:data['metrics'][view]=run(view,seconds=12,gui=True,view=view)
            else:raise RuntimeError('desktop unavailable: actual overview and ground GUI evidence still required')
            primary=data['metrics']['base_a'];contact=data['metrics']['contact']
            objects=['unknown_low_0','unknown_low_1','unknown_low_2','ped_0','equipment']
            c3=data['metrics']['reproducibility']['passed'] and min(primary['actor_displacement_m'].values())>.5 and all(primary['lidar_max_hits'][n]>=5 for n in objects) and all(primary['camera_color_pixels'][n]>=3 for n in objects) and contact['actor_fixture_contact_count']>0 and contact['fixture_displacement_m']>.05
            data['checks']['S02-C03']='PASS' if c3 else 'FAIL'
            c4=all(data['metrics'][n]['online_isolation'] for n in ['base_a','base_b','notch','holes','contact','overview','ground']) and data['metrics']['geometry']['prior_independent_of_unknown_seed']
            data['checks']['S02-C04']='PASS' if c4 else 'FAIL'
            data['status']='PASS' if all(v=='PASS' for v in data['checks'].values()) else 'FAIL'
            data['next_step_verdict']='可以执行下一步 S03：S02 场景与先验分层自动验收通过；S03 将验证两种底盘与连接路净空' if data['status']=='PASS' else '暂不能执行下一步：S02 必需检查未全通过'
    except KeyboardInterrupt:data['status']='PARTIAL';data['error']='用户/超时中断，已关闭本会话进程'
    except Exception as exc:data['status']='FAIL';data['error']=repr(exc);(out/'exception.log').write_text(traceback.format_exc());print(traceback.format_exc(),flush=True)
    finally:
        try:
            data['wall_seconds']=time.monotonic()-start;dump(out/'result.json',data)
            try:
                render(out,data,chosen)
                if a.mode=='test' and data['status']=='PASS':
                    command(out,'browser_check',['/usr/bin/python3',str(PKG/'tests/report_check.py'),str(out/'index.html')],40)
                    data['metrics']['browser']=read(out/'browser_check.json')
                    lifecycle=BASE/'setup_logs/s02_lifecycle/result.json'
                    if lifecycle.exists():
                        shutil.copy2(lifecycle,out/'lifecycle_check.json');data['metrics']['lifecycle']=read(lifecycle)
                    dump(out/'result.json',data);render(out,data,chosen)
            except Exception as exc:
                data['status']='FAIL';data['error']='report validation: '+repr(exc);data['checks']['S02-C02']='FAIL';data['next_step_verdict']='暂不能执行下一步：可视报告验证失败'
                dump(out/'result.json',data)
                (out/'index.html').write_text('<!doctype html><meta charset="utf-8"><h1>S02 FAIL</h1><p>报告验证失败，请查看 result.json 和 browser_check.log。</p>')
            if a.mode=='test':write_receipt(out,data)
            else:dump(RESULTS/'last_demo.json',dict(run_id=run_id,status=data['status'],report_path=str((out/'index.html').relative_to(ROOT))))
        finally:
            SESSION.unlink(missing_ok=True);lock.close()
    print(json.dumps(dict(status=data['status'],report=str(out/'index.html'),next=data['next_step_verdict']),ensure_ascii=False),flush=True)
    if a.mode=='demo' and a.case=='prior_vs_unknown' and data['status']=='PASS':subprocess.Popen(['xdg-open',str(out/'index.html')],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
    return 0 if data['status']=='PASS' else 1
if __name__=='__main__':raise SystemExit(main())
