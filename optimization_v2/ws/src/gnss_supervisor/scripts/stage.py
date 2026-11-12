"""Public S08 executor; bounded serial simulator, own session and evidence chain."""
import argparse,datetime,fcntl,signal,subprocess,traceback,gzip,shutil
from common08 import *
from runtime import run_case,s08_inputs
from analysis08 import analyze
from report08 import render
from localization_stage import files,row,compact,command
sys.path.insert(0,str(BASE/'ws/src/project_baseline/scripts'))
from session_owner import save_session,stop_session
RESULTS=BASE/'results/S08';SESSION=BASE/'runtime/S08.json'
def receipt(out,d):
 docs=ROOT/'docs/optimization_v2/S08';docs.mkdir(parents=True,exist_ok=True)
 text=['# S08 实测结果','',f'本轮 `{d["run_id"]}`：**{d["status"]}**。人工状态 NOT_CHECKED。',d['next_step_verdict'],'',f'[交互报告]({out}/index.html) · [原始指标和命令]({out}/result.json)','', '全部坐标在冻结 map 下评分，不拟合真值轨迹。基线为 A 区诊断运动，不能推广为全地图商业性能。', '', '| 案例 | 位置 P95 / m | 航向 P95 / rad | 准入时间占比 | 通过 |','|---|---:|---:|---:|---|']
 stops=[]
 for name,c in d['cases'].items():
  m=c['metrics'];text.append(f'| {name} | {m["xy_p95_m"]} | {m["yaw_p95_rad"]} | {m["admission_fraction"]:.3f} | {m["passed"]} |')
  if m['physical_stops']:stops.append('\n停车实测 `'+name+'`：`'+json.dumps(m['physical_stops'],ensure_ascii=False)+'`\n')
 text+=stops
 if (RESULTS/'manual_entrypoint_validation.json').exists():text+=['',f'[默认 GUI 演示与实时话题检查记录]({RESULTS}/manual_entrypoint_validation.json)：由执行代理核验，用户人工状态仍为 NOT_CHECKED。']
 if d.get('error'):text+=['','失败原因：'+d['error']]
 text+=['','故障阶段同样报告误差；另存全部已发布位姿的 P95，避免只显示准入窗口。未初始化时没有位姿，记录首次准入时间而不伪造零误差。地图失配为明确的观测接口注入；不是复杂视觉误匹配发生概率的评估。','', '冻结地图的连接路和 B 区在 S07 已发现空间不均匀偏差，单个刚体配准无法消除。后续在这些区域开展精细作业前需要地图质量修订和版本重新验收。本步未修改 S01–S07 源码和回执。','',f'自动检查：`{json.dumps(d["checks"],ensure_ascii=False)}`。每例 runtime.json 保存真实 PID、命令、退出码、地图前后哈希；s08_inputs.json 绑定实际捕获版本。测试停止仅收尾本会话。', '', '参考实现使用已安装 Jazzy/RTAB-Map/robot_localization，无新增系统包。实际 dpkg 版本在 environment_verified.json；C++/Python 教学、真实节点组件测试、浏览器交互检查的退出码在 commands.jsonl.gz。']
 (docs/'RESULTS.md').write_text('\n'.join(text)+'\n')
 source=files(PKG)+[BASE/'config'/x for x in ['gnss_alignment.yaml','localization_supervisor.yaml','fault_cases/localization.yaml','s08_dependency_lock.json']]+[BASE/'scripts/build_s08.sh',BASE/'stages/S08/run.sh',BASE/'ws/install/gnss_supervisor/lib/gnss_supervisor/gnss_supervisor']
 artifacts=files(out)+files(docs)+files(BASE/'lessons/S08')+files(BASE/'references/S08')
 for c in d['cases'].values():
  p=ROOT/c['path']
  if not p.is_relative_to(out):artifacts+=files(p)
 if (RESULTS/'development_summary.json').exists():artifacts.append(RESULTS/'development_summary.json')
 extra=RESULTS/'manual_entrypoint_validation.json'
 if extra.exists():
  artifacts.append(extra)
  for trial in read(extra).get('runs',[]):
   path=(ROOT/trial['path']).resolve()
   if not path.is_relative_to(RESULTS.resolve()):raise ValueError('manual evidence outside S08')
   artifacts+=files(path)
 r=dict(schema_version=1,step='S08',plan_revision='20261109.1',run_id=d['run_id'],checked_at_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),automated_status=d['status'],manual_status='NOT_CHECKED',reason=d.get('error','GNSS 配准、主参考选择、偏移监督及实际故障停车恢复'),check_exit_code=0 if d['status']=='PASS' else 1,checks=d['checks'],report_path=str((out/'index.html').relative_to(ROOT)),source_files=[row(p) for p in sorted(set(source))],artifacts=[row(p) for p in sorted(set(artifacts))],dependency_receipts={'S07':digest(BASE/'results/S07/latest.json')});dump(RESULTS/'latest.json',r)
def main():
 p=argparse.ArgumentParser();p.add_argument('mode',choices=['test','demo','stop']);p.add_argument('--suite',choices=['acceptance']);p.add_argument('--case',choices=['gnss_outage','false_fixed_and_loss','map_degradation']);p.add_argument('--seed',type=int,default=41);p.add_argument('--sim-seconds',type=float,default=300);p.add_argument('--wall-timeout',type=float,default=1800);p.add_argument('--headless',action='store_true');p.add_argument('--resume',type=Path);a=p.parse_args()
 if a.mode=='stop':r=stop_session(SESSION,60);print(json.dumps(r));return 0 if r['status'] in ['STOPPED','NOT_RUNNING'] else 1
 if a.seed!=41:p.error('qualification seed is 41')
 if not all(math.isfinite(x) and x>0 for x in [a.sim_seconds,a.wall_timeout]):p.error('positive finite budgets required')
 if a.mode=='test' and (a.suite!='acceptance' or a.case or a.sim_seconds<300):p.error('acceptance requires --suite acceptance, total sim budget >=300')
 if a.mode=='demo' and (not a.case or a.suite or a.resume or a.sim_seconds<85):p.error('demo requires case, >=85 sim seconds, no suite/resume')
 if a.wall_timeout<120:p.error('wall timeout >=120')
 if subprocess.run(['bash',str(BASE/'stepctl.sh'),'check','S07'],cwd=ROOT).returncode:return 4
 validate();lock=(ROOT/'native_20261108/runs/session.lock').open('a')
 try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
 except BlockingIOError:print('项目会话锁被占用');return 2
 if os.getpgrp()!=os.getpid():os.setsid()
 run_id=a.mode+'_'+datetime.datetime.now().strftime('%Y%m%d_%H%M%S')+'_'+str(os.getpid());out=RESULTS/run_id;out.mkdir(parents=True);save_session(SESSION,str(Path(__file__).resolve()),run_id);start=time.monotonic();data=dict(run_id=run_id,status='PARTIAL',mode=a.mode,seed=a.seed,command=sys.argv,checks={f'S08-C{i:02}':'NOT_RUN' for i in range(1,5)},cases={},next_step_verdict='暂不能执行下一步：S08 未完整通过')
 def interrupt(s,f):raise KeyboardInterrupt
 signal.signal(signal.SIGINT,interrupt);signal.signal(signal.SIGTERM,interrupt)
 def collect(name,seconds):
  candidate=None
  if a.resume:
   prior=read(a.resume/'result.json');candidate=(ROOT/prior['cases'][name]['path']).resolve() if name in prior.get('cases',{}) else None
  if candidate:
   if not candidate.is_relative_to(ROOT):raise ValueError('resume outside project')
   rt=read(candidate/'runtime.json')
   if not rt['success'] or rt['case']!=name or rt['seed']!=a.seed or rt['seconds']<seconds or read(candidate/'s08_inputs.json')!=s08_inputs():raise ValueError('resume versions/case/budget mismatch')
   from s07_common import capture_inputs
   if read(candidate/'capture_inputs.json')!=capture_inputs():raise ValueError('upstream runtime inputs changed')
   print('RESUME '+name,flush=True);case_out=candidate
  else:
   case_out=out/name;remaining=a.wall_timeout-(time.monotonic()-start)-45
   if remaining<seconds+45:raise TimeoutError('insufficient remaining wall budget')
   print('RUN '+name,flush=True);run_case(case_out,'zone_a',name,seconds,min(remaining,seconds*2+70),a.seed,a.mode=='demo' and not a.headless)
  m=analyze(case_out);data['cases'][name]=dict(path=str(case_out.relative_to(ROOT)),metrics=m);dump(out/'progress.json',data);print(json.dumps(dict(case=name,passed=m['passed'],failed=[k for k,v in m['checks'].items() if not v])),flush=True)
 try:
  command(out,'dependencies',['/usr/bin/python3',str(PKG/'tests/dependencies.py'),str(out/'environment_verified.json')],30)
  command(out,'build',['bash',str(BASE/'scripts/build_s08.sh')],120)
  if a.mode=='test':
   command(out,'coordinates',['/usr/bin/python3',str(PKG/'tests/coordinates.py')],20)
   command(out,'supervisor',['/usr/bin/python3',str(PKG/'tests/supervisor.py'),str(out/'components')],90)
   command(out,'lesson_compile',['g++','-std=c++17','-O2','-I/usr/include/eigen3',str(BASE/'lessons/S08/lesson.cpp'),'-o',str(BASE/'ws/build/gnss_supervisor/lesson_s08')],45);command(out,'lesson_cpp',[str(BASE/'ws/build/gnss_supervisor/lesson_s08')],10);command(out,'lesson_python',['/usr/bin/python3',str(BASE/'lessons/S08/lesson.py')],10)
  if a.mode=='test':
   for name,seconds in [('normal',60),('gnss_outage',70),('false_fixed_and_loss',75),('map_degradation',75)]:collect(name,seconds)
   cs=data['cases'];checks={1:cs['normal']['metrics']['checks']['baseline_accuracy'],2:all(cs[n]['metrics']['passed'] for n in ['gnss_outage','false_fixed_and_loss','map_degradation']),3:all(all(c['metrics']['checks'][k] for k in ['unique_tf_and_actuator','no_global_feedback','truth_isolation','tf_edges']) for c in cs.values()),4:read(out/'components/result.json')['passed'] and all(c['metrics']['checks']['no_unadmitted_drive'] for c in cs.values())};data['checks']={f'S08-C{k:02}':'PASS' if v else 'FAIL' for k,v in checks.items()};data['status']='PASS' if all(checks.values()) and all(c['metrics']['passed'] for c in cs.values()) else 'FAIL'
  else:collect(a.case,75);data['status']='PASS' if data['cases'][a.case]['metrics']['passed'] else 'FAIL'
 except KeyboardInterrupt:data.update(status='PARTIAL',error='中断；本会话已停车收尾')
 except Exception as e:data.update(status='FAIL',error=repr(e));(out/'exception.log').write_text(traceback.format_exc());print(traceback.format_exc(),flush=True)
 finally:
  try:
   data['wall_seconds']=time.monotonic()-start
   try:
    render(out,data)
    if data['cases']:command(out,'browser_check',[sys.executable,str(PKG/'tests/report_check.py'),str(out/'index.html')],45)
    else:dump(out/'browser_check.json',dict(status='NOT_RUN',reason='No completed capture; empty interrupted/failed run report'))
   except Exception as e:data.update(status='FAIL',error='Report: '+repr(e))
   if a.mode=='test' and data['status']=='PASS':data['next_step_verdict']='可以执行下一步 S09：S08 四项自动验收通过，S07 有效；全地图精细作业仍需修订已知局部地图畸变，不自动执行 S09'
   if a.mode=='demo':data['next_step_verdict']='独立演示；正式状态请运行 stepctl next S08'
   for c in data['cases'].values():compact(ROOT/c['path'])
   compact(out);dump(out/'result.json',data)
   if a.mode=='test':receipt(out,data)
   else:
    dump(RESULTS/'last_demo.json',dict(run_id=run_id,status=data['status'],report_path=str((out/'index.html').relative_to(ROOT))))
    if not a.headless:subprocess.Popen(['xdg-open',str(out/'index.html')],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL,start_new_session=True)
  finally:SESSION.unlink(missing_ok=True);lock.close()
 print(json.dumps(dict(status=data['status'],report=str(out/'index.html'),next=data['next_step_verdict']),ensure_ascii=False));return 0 if data['status']=='PASS' else 1
if __name__=='__main__':raise SystemExit(main())
