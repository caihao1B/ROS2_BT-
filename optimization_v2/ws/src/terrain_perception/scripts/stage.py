"""S09 bounded executor with actual observations, receipts and dependency chain."""
import argparse,datetime,fcntl,signal,subprocess,traceback
from common09 import *
from runtime import run_case,capture09,s08_inputs
from analysis09 import analyze
from report09 import render
from localization_stage import files,row,compact,command
sys.path.insert(0,str(BASE/'ws/src/project_baseline/scripts'))
from session_owner import save_session,stop_session
RESULTS=BASE/'results/S09';SESSION=BASE/'runtime/S09.json'
def receipt(out,d):
 docs=ROOT/'docs/optimization_v2/S09';docs.mkdir(parents=True,exist_ok=True);lines=['# S09 实测结果','',f'本轮 `{d["run_id"]}`：**{d["status"]}**；用户人工状态 NOT_CHECKED。',d['next_step_verdict'],'',f'[离线交互报告]({out}/index.html) · [完整指标]({out}/result.json)','', '| 案例 | 底盘 | 通过 | 地形 P95/ms | 检测 P95/ms | 清除/s |','|---|---|---|---:|---:|---:|']
 details=[]
 for name,c in d['cases'].items():
  m=c['metrics'];lines.append(f'| {name} | {m["vehicle"]} | {m["passed"]} | {m["terrain_latency_ms_p95"]:.2f} | {m["detector_latency_ms"]["p95"]} | {m["clearing"]["clear_delay_sim_s"]} |');details+=['',f'`{name}` 未通过项：'+str([k for k,v in m['checks'].items() if not v]),'']
 lines+=details
 lines+=['','GUI 复核发现 S08 诊断点在 A/B 作业区外，已明确增加 S09 诊断通行区并补充实际 Nav2 空闲地面检查，重新运行全部四个场景。原 A/B 边界和冻结地图未修改。车辆驻停进行传感器/代价层验收；未运行 S13 闭环控制。局部窗口随 odom 移动；未知格不自动转为空闲。几何不依赖视觉类别。行人采用 CC0 静态步态网格、真实刚体运动，未模拟人体关节步态。','', '轻量检测器仍有树木误分类和短时漏识别；语义不是安全放行依据。每例 dynamic 指标公开实际可见帧、识别人帧数、ID 切换。长期遮挡超时后生成新 ID，不宣称永久身份。','', 'S08 已知连接路/B 区地图变形仍存在；本步地形风险在局部 odom 下计算。全地图精细导航仍需后续修订地图并复验。坡度上限沿用 S03 已验证 5°，台阶/粗糙度为保守工程候选，不代表实体底盘爬越认证。','',f'检查：`{json.dumps(d["checks"],ensure_ascii=False)}`。命令/退出码见 commands.jsonl.gz；运行 PID、时钟、输入哈希、地图前后哈希见每例 runtime.json / s09_inputs.json。依赖锁与模型哈希已核验；本步未新增系统包。']
 if d.get('error'):lines+=['', '失败原因：'+d['error']]
 (docs/'RESULTS.md').write_text('\n'.join(lines)+'\n');source=files(PKG)+files(FUSION)+[BASE/'config'/x for x in ['traversability.yaml','costmap.yaml','perception_test_area.geojson','detector_lock.yaml','s09_dependency_lock.json','perception_assets.lock.json','rviz/perception.rviz']]+[BASE/'scripts/build_s09.sh',BASE/'stages/S09/run.sh']+files(BASE/'assets/perception')+[BASE/'ws/install/terrain_perception/lib'/x for x in ['libobserved_risk_layer.so','libs09_actor.so','terrain_perception/terrain_node']];artifacts=files(out)+files(docs)+files(BASE/'lessons/S09')+files(BASE/'references/S09')
 for c in d['cases'].values():
  p=ROOT/c['path']
  if not p.is_relative_to(out):artifacts+=files(p)
 for extra in ['development_summary.json','manual_entrypoint_validation.json']:
  p=RESULTS/extra
  if p.exists():
   artifacts.append(p)
   if extra=='manual_entrypoint_validation.json':
    for trial in read(p).get('runs',[]):
     q=(ROOT/trial['path']).resolve()
     if not q.is_relative_to(RESULTS.resolve()):raise ValueError('manual evidence outside S09')
     artifacts+=files(q)
 r=dict(schema_version=1,step='S09',plan_revision='20261109.1',run_id=d['run_id'],checked_at_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),automated_status=d['status'],manual_status='NOT_CHECKED',reason=d.get('error','带噪点云地形、双目/视觉关联、实际 Nav2 风险层与失效清除实测'),check_exit_code=0 if d['status']=='PASS' else 1,checks=d['checks'],report_path=str((out/'index.html').relative_to(ROOT)),source_files=[row(p) for p in sorted(set(source))],artifacts=[row(p) for p in sorted(set(artifacts))],dependency_receipts={'S08':digest(BASE/'results/S08/latest.json')});dump(RESULTS/'latest.json',r)
def main():
 p=argparse.ArgumentParser();p.add_argument('mode',choices=['test','demo','stop']);p.add_argument('--suite',choices=['acceptance']);p.add_argument('--case',choices=['low_obstacles_and_slope','occluded_pedestrian','camera_loss']);p.add_argument('--vehicle',choices=['diff','ackermann'],default='diff');p.add_argument('--seed',type=int,default=41);p.add_argument('--sim-seconds',type=float,default=300);p.add_argument('--wall-timeout',type=float,default=1800);p.add_argument('--headless',action='store_true');p.add_argument('--resume',type=Path);a=p.parse_args()
 if a.mode=='stop':r=stop_session(SESSION,60);print(json.dumps(r));return 0 if r['status'] in ['STOPPED','NOT_RUNNING'] else 1
 if a.seed!=41:p.error('qualification seed=41')
 if not all(math.isfinite(x) and x>0 for x in [a.sim_seconds,a.wall_timeout]):p.error('finite positive budgets required')
 if a.mode=='test' and (a.suite!='acceptance' or a.case or a.sim_seconds<300):p.error('test requires acceptance and >=300 total sim seconds')
 if a.mode=='demo' and (not a.case or a.suite or a.resume or a.sim_seconds<65):p.error('demo requires case, >=65 sim seconds, no suite/resume')
 if a.wall_timeout<120:p.error('wall timeout >=120')
 if subprocess.run(['bash',str(BASE/'stepctl.sh'),'check','S08'],cwd=ROOT).returncode:return 4
 lock=(ROOT/'native_20261108/runs/session.lock').open('a')
 try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
 except BlockingIOError:print('项目会话锁被占用');return 2
 if os.getpgrp()!=os.getpid():os.setsid()
 run_id=a.mode+'_'+datetime.datetime.now().strftime('%Y%m%d_%H%M%S')+'_'+str(os.getpid());out=RESULTS/run_id;out.mkdir(parents=True);save_session(SESSION,str(Path(__file__).resolve()),run_id);start=time.monotonic();data=dict(run_id=run_id,status='PARTIAL',mode=a.mode,seed=a.seed,command=sys.argv,checks={f'S09-C{i:02}':'NOT_RUN' for i in range(1,5)},cases={},next_step_verdict='暂不能执行下一步：S09 未完整通过')
 def interrupt(s,f):raise KeyboardInterrupt
 signal.signal(signal.SIGINT,interrupt);signal.signal(signal.SIGTERM,interrupt)
 def collect(name,case,seconds,kind):
  candidate=None
  if a.resume:
   prior=read(a.resume/'result.json');candidate=(ROOT/prior['cases'][name]['path']).resolve() if name in prior.get('cases',{}) else None
  if candidate:
   if not candidate.is_relative_to(RESULTS):raise ValueError('resume outside S09')
   rt=read(candidate/'runtime.json')
   from s07_common import capture_inputs
   if not rt['success'] or rt['case']!=case or rt['vehicle']!=kind or rt['seed']!=a.seed or rt['seconds']<seconds or read(candidate/'s09_inputs.json')!=capture09() or read(candidate/'s08_inputs.json')!=s08_inputs() or read(candidate/'capture_inputs.json')!=capture_inputs():raise ValueError('resume versions/case/budget mismatch')
   case_out=candidate;print('RESUME '+name,flush=True)
  else:
   case_out=out/name;remaining=a.wall_timeout-(time.monotonic()-start)-45
   if remaining<seconds+45:raise TimeoutError('insufficient remaining wall budget')
   print('RUN '+name,flush=True);run_case(case_out,'zone_a',case,seconds,min(remaining,seconds*2+90),a.seed,a.mode=='demo' and not a.headless,kind)
  m=analyze(case_out);data['cases'][name]=dict(path=str(case_out.relative_to(ROOT)),metrics=m);dump(out/'progress.json',data);print(json.dumps(dict(case=name,passed=m['passed'],failed=[k for k,v in m['checks'].items() if not v])),flush=True)
 try:
  command(out,'dependencies',['/usr/bin/python3',str(PKG/'tests/dependencies.py'),str(out/'environment_verified.json')],30);command(out,'build',['bash',str(BASE/'scripts/build_s09.sh')],120)
  if a.mode=='test':
   command(out,'authorized_boundaries',[sys.executable,str(PKG/'tests/boundaries.py')],20)
   command(out,'tracking_projection',['/usr/bin/python3',str(FUSION/'tests/algorithms.py')],20)
   command(out,'nav2_layer',['/usr/bin/python3',str(PKG/'tests/nav2_layer.py'),str(out/'component_nav2')],60)
   command(out,'terrain_time',['/usr/bin/python3',str(PKG/'tests/terrain_time.py'),str(out/'component_terrain')],45)
   command(out,'lesson_compile',['g++','-std=c++17','-O2','-I'+str(PKG/'include'),str(BASE/'lessons/S09/lesson.cpp'),'-o',str(BASE/'ws/build/terrain_perception/lesson_s09')],30);command(out,'lesson_cpp',[str(BASE/'ws/build/terrain_perception/lesson_s09')],10);command(out,'lesson_python',['/usr/bin/python3',str(BASE/'lessons/S09/lesson.py')],10)
   for name,case,seconds,kind in [('low_diff','low_obstacles_and_slope',60,'diff'),('low_ackermann','low_obstacles_and_slope',60,'ackermann'),('occluded_pedestrian','occluded_pedestrian',75,'diff'),('camera_loss','camera_loss',75,'diff')]:collect(name,case,seconds,kind)
   cs=data['cases'];allpass=all(c['metrics']['passed'] for c in cs.values());checks={1:all(cs[k]['metrics']['checks']['all_low_detected'] for k in ['low_diff','low_ackermann']),2:all(cs[k]['metrics']['checks']['slopes'] for k in ['low_diff','low_ackermann']) and all(c['metrics']['checks']['actual_nav2_layers'] and c['metrics']['checks']['authorized_boundary'] for c in cs.values()),3:cs['occluded_pedestrian']['metrics']['passed'] and read(out/'component_terrain/result.json')['passed'],4:all(c['metrics']['checks']['temporary_clearing'] for c in cs.values()) and cs['camera_loss']['metrics']['passed'] and read(out/'component_nav2/result.json')['passed']};data['checks']={f'S09-C{k:02}':'PASS' if v else 'FAIL' for k,v in checks.items()};data['status']='PASS' if allpass and all(checks.values()) else 'FAIL'
  else:collect(a.case,a.case,a.sim_seconds-10,a.vehicle);data['status']='PASS' if data['cases'][a.case]['metrics']['passed'] else 'FAIL'
 except KeyboardInterrupt:data.update(status='PARTIAL',error='中断；本会话已停车收尾')
 except Exception as e:data.update(status='FAIL',error=repr(e));(out/'exception.log').write_text(traceback.format_exc());print(traceback.format_exc(),flush=True)
 finally:
  try:
   data['wall_seconds']=time.monotonic()-start
   try:
    render(out,data)
    if data['cases']:command(out,'browser_check',[sys.executable,str(PKG/'tests/report_check.py'),str(out/'index.html')],45)
   except Exception as e:data.update(status='FAIL',error='Report: '+repr(e))
   if a.mode=='test' and data['status']=='PASS':data['next_step_verdict']='可以执行下一步 S10：S09 四项自动验收通过，S08 有效；未自动实施 S10，闭环导航由 S13 验证'
   if a.mode=='demo':data['next_step_verdict']='独立演示；正式状态见 stepctl next S09'
   for c in data['cases'].values():compact(ROOT/c['path'])
   compact(out);dump(out/'result.json',data)
   if a.mode=='test':receipt(out,data)
   else:
    dump(RESULTS/'last_demo.json',dict(run_id=run_id,status=data['status'],report_path=str((out/'index.html').relative_to(ROOT))))
    if not a.headless:subprocess.Popen(['xdg-open',str(out/'index.html')],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL,start_new_session=True)
  finally:SESSION.unlink(missing_ok=True);lock.close()
 print(json.dumps(dict(status=data['status'],report=str(out/'index.html'),next=data['next_step_verdict']),ensure_ascii=False));return 0 if data['status']=='PASS' else 1
if __name__=='__main__':raise SystemExit(main())
