#!/usr/bin/env python3
"""S01 implementation and reproducible acceptance. Robot motion is never requested here."""
import argparse, datetime, fcntl, html, json, os, re, shutil, signal, subprocess, sys, time
from pathlib import Path
from baseline_audit import ROOT, BASE, NATIVE, audit, file_row, tree_files
from session_owner import save_session, stop_session

PKG=Path(__file__).resolve().parents[1]
RESULTS=BASE/'results/S01'
SESSION=BASE/'runtime/S01.json'
MARKER=str(Path(__file__).resolve())
BINDIR=BASE/'ws/install/project_baseline/lib/project_baseline'

def interrupt(sig, frame): raise KeyboardInterrupt

def legacy_env():
    env=os.environ.copy()
    for k in ['AMENT_PREFIX_PATH','CMAKE_PREFIX_PATH','COLCON_PREFIX_PATH','PYTHONPATH','LD_LIBRARY_PATH']:
        env.pop(k,None)
    env['PATH']='/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin'
    env['ROS_DOMAIN_ID']='118'
    return env

def stop_child(p):
    if p.poll() is None:
        p.send_signal(signal.SIGINT)
        try:p.wait(timeout=16)
        except subprocess.TimeoutExpired:
            p.terminate()
            try:p.wait(timeout=4)
            except subprocess.TimeoutExpired:p.kill();p.wait(timeout=3)

class Commands:
    def __init__(self,out):self.out=out;self.rows=[]
    def run(self,name,argv,timeout=90,expected=0,env=None):
        print(f'RUN {name}',flush=True)
        start=time.monotonic();file=self.out/(name+'.log')
        with file.open('w') as log:
            p=subprocess.Popen(argv,cwd=ROOT,env=env,stdout=log,stderr=subprocess.STDOUT)
            interrupted=False
            try:rc=p.wait(timeout=timeout)
            except subprocess.TimeoutExpired:stop_child(p);rc=124
            except KeyboardInterrupt:stop_child(p);rc=130;interrupted=True
            except BaseException:stop_child(p);raise
        row={'name':name,'argv':argv,'exit_code':rc,'expected_exit_code':expected,
             'wall_seconds':round(time.monotonic()-start,3),'status':'PASS' if rc==expected else 'FAIL',
             'log':file.name}
        self.rows.append(row)
        (self.out/'commands.json').write_text(json.dumps(self.rows,ensure_ascii=False,indent=2)+'\n')
        print(f'{row["status"]} {name} exit={rc} ({row["wall_seconds"]}s)',flush=True)
        if rc!=expected:print(file.read_text(errors='replace')[-2000:],flush=True)
        if interrupted:raise KeyboardInterrupt
        return row['status']=='PASS'
    def native(self,name,command,timeout=90,expected=0):
        import shlex
        return self.run(name,['bash','--noprofile','--norc','-c',
            f'source {NATIVE}/scripts/env.sh\nexec '+shlex.join(command)],timeout,expected,legacy_env())

def figures(out):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    result=out/'farm/result.json'
    if not result.exists():return
    d=json.loads(result.read_text());hz=d.get('received_wall_hz',{})
    if hz:
        fig,ax=plt.subplots(figsize=(10,4.5)); names=list(hz)
        ax.barh([s.replace('/dino/','') for s in names],list(hz.values()),color='#237c9b')
        ax.set_xlabel('Received messages / wall second (not configured simulation frequency)')
        ax.set_title(f"Stationary sensor baseline | RTF {d.get('clock_rtf',0):.2f}")
        fig.tight_layout();fig.savefig(out/'sensor_rates.svg');plt.close(fig)
    points=out/'farm/last_scan_points.json'
    if points.exists():
        import numpy as np
        scans=json.loads(points.read_text());fig,axes=plt.subplots(1,2,figsize=(10,4))
        for ax,topic in zip(axes,['/dino/lidar/front/scan','/dino/lidar/rear/scan']):
            a=np.array(scans.get(topic,[]))
            if a.size:ax.scatter(a[:,1]*np.cos(a[:,0]),a[:,1]*np.sin(a[:,0]),s=2)
            ax.set_title(topic);ax.set_aspect('equal');ax.set_xlabel('sensor x / m');ax.set_ylabel('sensor y / m')
        fig.tight_layout();fig.savefig(out/'scan_observations.svg');plt.close(fig)

def render(out,data,rows):
    figures(out)
    stop_proof=BASE/'setup_logs/stop_integration.json'
    if stop_proof.exists() and data['kind']=='test':
        shutil.copy2(stop_proof,out/'stop_integration.json')
    esc=lambda v:html.escape(str(v))
    cards=''.join(f'<li><b>{esc(k)}</b>: {esc(v)}</li>' for k,v in data['checks'].items())
    command_rows=''.join(f'<tr><td>{esc(r["name"])}</td><td>{r["status"]}</td><td>{r["exit_code"]}</td><td>{r["wall_seconds"]}</td><td><a href="{r["log"]}">日志</a></td></tr>' for r in rows)
    images=''.join(f'<figure><img src="{p.name}"><figcaption>{esc(p.name)} · 本轮实测观测</figcaption></figure>' for p in out.glob('*.svg'))
    screen=out/'farm/gazebo_window.png'
    if not screen.exists() and data['kind']=='test':
        for prior in sorted(RESULTS.glob('demo_*/farm/result.json'),reverse=True):
            evidence=json.loads(prior.read_text())
            image_path=prior.parent/'gazebo_window.png'
            if evidence.get('status')=='PASS' and evidence.get('checks',{}).get('gui_survived') and image_path.exists():
                shutil.copy2(image_path,out/'gui_verified.png')
                (out/'gui_evidence_source.json').write_text(json.dumps({'source_run':str(prior.parent.parent.relative_to(ROOT)),
                    'source_result':str(prior.relative_to(ROOT)),'gui':'ogre','observation':'3D viewport, independent GUI demo'},indent=2))
                images+='<figure><img src="gui_verified.png"><figcaption>单独 GUI 演示的实测三维视窗；来源见 gui_evidence_source.json，非本轮无界面测试截图。</figcaption></figure>'
                break
    if screen.exists():images+='<figure><img src="farm/gazebo_window.png"><figcaption>本轮 Gazebo 窗口截图；人工验收仍单独记录。</figcaption></figure>'
    facts=json.loads((out/'baseline_manifest.json').read_text()) if (out/'baseline_manifest.json').exists() else {}
    changes=facts.get('source_comparison',{}).get('changed',[])
    body=f'''<!doctype html><html lang="zh-CN"><meta charset="utf-8"><title>S01 基线验收</title>
<style>body{{max-width:1120px;margin:32px auto;padding:0 24px;font:16px/1.7 sans-serif;color:#173047;background:#f5f8fa}}h1{{font-size:30px}}section{{background:white;padding:24px;margin:20px 0;border-radius:12px}}table{{border-collapse:collapse;width:100%}}td,th{{border-bottom:1px solid #dce4e9;padding:9px;text-align:left}}img{{max-width:100%}}pre{{white-space:pre-wrap;overflow-wrap:anywhere}}.status{{font-size:22px;color:#176d70}}</style>
<h1>第 1 步 原项目基线与隔离工作区</h1><p class="status">{esc(data['status'])} · {esc(data['kind'])}</p>
<p>{esc(data['run_id'])} | 人工查看：NOT_CHECKED | 下一步判断：{esc(data['next_step_verdict'])}</p>
<section><h2>检查结果</h2><ul>{cards}</ul><p>静态/组件基线通过不表示完整自主喷洒或割草通过。</p></section>
<section><h2>直观观测</h2>{images or '<p>本案例是离线依赖审计；农田观测请运行 legacy_farm。</p>'}</section>
<section><h2>实际命令与退出码</h2><table><tr><th>检查</th><th>结论</th><th>退出码</th><th>现实秒</th><th>证据</th></tr>{command_rows}</table></section>
<section><h2>源码与依赖</h2><p>旧入口绑定 native_20261108/ros2_ws；新开发仅使用 optimization_v2/ws。旧 BT 完整任务入口已隔离，C++ 核心回归由新工作区编译执行。</p><p>两份源码差异 {len(changes)} 项：</p><pre>{esc(chr(10).join(changes))}</pre>
<a href="baseline_manifest.json">完整路径、硬件及资源哈希</a> · <a href="dependency_lock.yaml">依赖锁</a> · <a href="commands.json">实际命令</a> · <a href="result.json">结果 JSON</a></section>
<section><h2>后续范围</h2><p>S02 建立连续不规则三维世界。RTAB-Map 留到 S05/S06 安装；S01 只验证 F2C v2 基本 C++ 接口，不宣称完整 opennav_coverage 适配已验收。原 Dino 的 300 Nm 派生仿真转向参数不等于实机标定。</p></section></html>'''
    (out/'index.html').write_text(body)

def write_receipt(out,data):
    docs=ROOT/'docs/optimization_v2/S01';docs.mkdir(parents=True,exist_ok=True)
    (docs/'RESULTS.md').write_text('# 第 1 步实测结果\n\n'+f'运行：`{data["run_id"]}`。状态：**{data["status"]}**。\n\n'+
        f'下一步判断：**{data["next_step_verdict"]}**。人工查看尚未记录，不代替用户目视判断。\n\n'+
        '\n'.join(f'- {k}：{v}' for k,v in data['checks'].items())+'\n\n'+
        f'可视报告：[打开本轮报告]({out}/index.html)。实际命令、退出码、耗时与日志见 [commands.json]({out}/commands.json)。\n\n'+
        '本步仅验静态传感器/环境/组件/依赖接口。完整动态导航、覆盖、3D 雷达与 RTK 等后续功能不在本步通过范围。\n')
    source_files=set(tree_files(PKG))
    source_files.update(tree_files(BASE/'config'))
    source_files.update(tree_files(BASE/'scripts'))
    source_files.update(tree_files(BASE/'tests'))
    source_files.update([BASE/'env.sh',BASE/'stages/S01/run.sh',BASE/'stepctl.py',BASE/'stepctl.sh'])
    # Hash the assets themselves; an unchanged manifest must not hide changed meshes.
    manifest=json.loads((out/'baseline_manifest.json').read_text())
    source_files.update(ROOT/x['path'] for x in manifest['runtime_files'])
    for name in ['baseline_bt','dependency_probe','isolation_publisher']:source_files.add(BINDIR/name)
    artifacts=set(tree_files(out)); artifacts.update(tree_files(docs));artifacts.update(tree_files(BASE/'lessons/S01'))
    payload={'schema_version':1,'step':'S01','plan_revision':json.loads((BASE/'plan/steps.json').read_text())['revision'],
             'run_id':data['run_id'],'checked_at_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),
             'automated_status':data['status'],'manual_status':'NOT_CHECKED','reason':data.get('error','Real acceptance evidence; no GUI-user approval inferred.'),
             'check_exit_code':0 if data['status']=='PASS' else 1,'checks':data['checks'],
             'report_path':str((out/'index.html').relative_to(ROOT)),
             'source_files':[file_row(p) for p in sorted(source_files)],
             'artifacts':[file_row(p) for p in sorted(artifacts)],'dependency_receipts':{},
             'next_step':{'step':'S02','ready':data['status']=='PASS','verdict':data['next_step_verdict']}}
    temp=RESULTS/'latest.tmp';temp.write_text(json.dumps(payload,ensure_ascii=False,indent=2)+'\n');temp.replace(RESULTS/'latest.json')

def accept(args,out,c):
    success={}
    success['build']=c.run('build',['bash',str(BASE/'scripts/build.sh')],300)
    success['audit']=audit(out,freeze=True)['status']=='PASS'
    success['dependency']=c.run('f2c_metis_api',[str(BINDIR/'dependency_probe')])
    success['bad_width']=c.run('invalid_width',[str(BINDIR/'dependency_probe'),'0'],expected=2)
    success['bt']=c.run('bt_core',['bash',str(NATIVE/'task_bt/run.sh'),'core-test'])
    success['bt_isolated']=c.run('old_bt_guard',['bash',str(NATIVE/'task_bt/run.sh'),'adapter.py'],expected=64)
    source=NATIVE/'ros2_ws/src'
    for package,test in [('dino_control','test_kinematics.py'),('dino_sensors','test_geometry.py'),('dino_slam','test_contract.py'),('dino_navigation','test_navigation_math.py')]:
        success[package]=c.native(package,['/usr/bin/python3',str(source/package/'tests'/test)])
    success['estimator_guard']=c.native('estimator_guard',['/usr/bin/python3',str(source/'dino_sensors/tests/test_estimator_guard.py'),str(out/'estimator_guard')],60)
    # Deliberately contaminate all common overlay variables, then prove env.sh resets them.
    code="import json,os; from ament_index_python.packages import get_package_prefix; p=get_package_prefix('project_baseline'); vals={k:os.getenv(k,'') for k in ['AMENT_PREFIX_PATH','CMAKE_PREFIX_PATH','PYTHONPATH','LD_LIBRARY_PATH']}; assert all('s01_forbidden' not in x and 'native_20261108' not in x for x in vals.values()); assert os.getenv('ROS_DOMAIN_ID')=='119'; print(json.dumps(dict(prefix=p,domain=119,paths=vals)))"
    import shlex
    env=os.environ.copy()
    for k in ['AMENT_PREFIX_PATH','CMAKE_PREFIX_PATH','PYTHONPATH','LD_LIBRARY_PATH']:env[k]='/tmp/s01_forbidden_overlay'
    success['clean_env']=c.run('clean_environment',['bash','--noprofile','--norc','-c',f'source {BASE}/env.sh\nexec /usr/bin/python3 -c '+shlex.quote(code)],env=env)
    publog=(out/'publisher.log').open('w');pub=subprocess.Popen([str(BINDIR/'isolation_publisher')],stdout=publog,stderr=subprocess.STDOUT)
    try:
        old=os.environ.copy();old['ROS_DOMAIN_ID']='118'
        success['domain_separate']=c.run('domain_118_separate',['/usr/bin/python3',str(PKG/'scripts/isolation_check.py'),'--expect-empty','--duration','2'],env=old)
        success['domain_transport']=c.run('domain_119_transport',['/usr/bin/python3',str(PKG/'scripts/isolation_check.py')],15)
    finally:stop_child(pub);publog.close()
    lockpath=NATIVE/'runs/session.lock';lockpath.parent.mkdir(exist_ok=True)
    with lockpath.open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        success['mutual_exclusion']=c.run('concurrent_native_refused',['bash',str(NATIVE/'run.sh'),'model','--seconds','15','--headless'],expected=2)
        success['mutual_exclusion'] &= '已有一个原生预览/检查在运行' in (out/'concurrent_native_refused.log').read_text()
    success['ownership']=c.run('session_and_index_tests',['/usr/bin/python3','-m','unittest','discover','-s',str(BASE/'tests'),'-v'])
    for scene,mode,seconds in [('farm','check',35),('model','check-model',25)]:
        success[scene]=c.run(scene+'_smoke',['bash',str(NATIVE/'run.sh'),mode,'--seconds',str(seconds),'--sim-seconds',str(args.sim_seconds),'--out',str(out/scene)],120,env=legacy_env())
    success['lessons']=c.run('lessons',['bash',str(BASE/'lessons/S01/run.sh')])
    (out/'component_checks.json').write_text(json.dumps(success,ensure_ascii=False,indent=2)+'\n')
    return {'S01-C01':all(success[k] for k in ['bt','dino_control','dino_sensors','dino_slam','dino_navigation','estimator_guard','farm','model']),
            'S01-C02':all(success[k] for k in ['build','clean_env','domain_separate','domain_transport','audit']),
            'S01-C03':all(success[k] for k in ['audit','bt_isolated','bt']),
            'S01-C04':all(success[k] for k in ['mutual_exclusion','ownership','dependency','bad_width','lessons'])}

def demo(args,out,c):
    result=audit(out)
    if args.case=='dependency_audit':
        c.run('f2c_metis_api',[str(BINDIR/'dependency_probe')])
    else:
        # Shared native lock is acquired by the legacy wrapper, so it also excludes old previews.
        argv=['bash',str(NATIVE/'run.sh'),'check','--seconds',str(min(210,args.wall_timeout-25)),
              '--sim-seconds',str(args.sim_seconds),'--out',str(out/'farm')]
        if not args.headless:argv+=['--gui']
        c.run('farm_demo',argv,args.wall_timeout-10,env=legacy_env())
    return {'DEMO_AUDIT':result['status']=='PASS','DEMO_COMMANDS':all(r['status']=='PASS' for r in c.rows)}

def main():
    p=argparse.ArgumentParser();p.add_argument('command',choices=['test','demo','stop'])
    p.add_argument('--case',choices=['legacy_farm','dependency_audit'],default='legacy_farm')
    p.add_argument('--suite',choices=['acceptance'],default='acceptance');p.add_argument('--seed',type=int,choices=[41],default=41)
    p.add_argument('--sim-seconds',type=int,default=300);p.add_argument('--wall-timeout',type=int,default=1200)
    p.add_argument('--headless',action='store_true');args=p.parse_args()
    if args.command=='stop':
        result=stop_session(SESSION);print(json.dumps(result,ensure_ascii=False));return 0 if result['status'] in ['STOPPED','NOT_RUNNING'] else 1
    if not 1<=args.sim_seconds<=1800 or args.wall_timeout<45:p.error('需要有效仿真上限且现实超时不少于 45 秒')
    BASE.joinpath('runtime').mkdir(exist_ok=True)
    lock=(BASE/'runtime/S01.lock').open('a')
    try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    except BlockingIOError:print('S01_BUSY：本步已有会话，未启动第二实例。',file=sys.stderr);return 9
    run_id=args.command+'_'+datetime.datetime.now().strftime('%Y%m%d_%H%M%S')+'_'+str(os.getpid())
    out=RESULTS/run_id;out.mkdir(parents=True)
    save_session(SESSION,MARKER,run_id)
    signal.signal(signal.SIGINT,interrupt);signal.signal(signal.SIGTERM,interrupt)
    c=Commands(out);data={'run_id':run_id,'kind':args.command,'seed':args.seed,'checks':{},'status':'FAIL'}
    print(f'结果目录：{out}',flush=True)
    try:
        checks=accept(args,out,c) if args.command=='test' else demo(args,out,c)
        data['checks']={k:'PASS' if v else 'FAIL' for k,v in checks.items()}
        data['status']='PASS' if all(checks.values()) else 'FAIL'
    except KeyboardInterrupt:data.update(status='PARTIAL',error='User interruption; only owned subprocesses were stopped.')
    except Exception as e:data.update(status='FAIL',error=repr(e));print(repr(e),file=sys.stderr)
    finally:
        data['next_step_verdict']='可以执行第 2 步' if args.command=='test' and data['status']=='PASS' else '本次结果不足以准入下一步；以有效自动验收回执为准'
        (out/'result.json').write_text(json.dumps(data,ensure_ascii=False,indent=2)+'\n')
        try:
            render(out,data,c.rows)
            if args.command=='test':write_receipt(out,data)
        finally:
            SESSION.unlink(missing_ok=True);lock.close()
        print(json.dumps(data,ensure_ascii=False,indent=2),flush=True)
        print('可视报告：'+str(out/'index.html'),flush=True)
    return 0 if data['status']=='PASS' else 1

if __name__=='__main__':raise SystemExit(main())
