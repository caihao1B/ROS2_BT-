"""S10 bounded public executor, physical experiments, fault tests and versioned receipts."""
import argparse,datetime,fcntl,signal,subprocess,traceback
from common10 import *
from runtime10 import run_case,capture09,s08_inputs
from analysis10 import analyze
from report10 import render
from localization_stage import files,row,compact,command
sys.path.insert(0,str(BASE/'ws/src/project_baseline/scripts'))
from session_owner import save_session,stop_session
RESULTS=BASE/'results/S10';SESSION=BASE/'runtime/S10.json'
def sources():return files(PKG)+files(LEDGER)+files(BASE/'ws/src/mission_interfaces')+[BASE/'config'/x for x in ['tool_contract.yaml','s10_dependency_lock.json','tool_profiles.yaml']]+[BASE/'scripts/build_s10.sh',BASE/'stages/S10/run.sh',BASE/'contracts/future_tool_adapter.md']+[BASE/'ws/install/tool_sim/lib'/x for x in ['libs10_tool.so','tool_sim/tool_action_server']]+files(BASE/'references/S10')
def capture10():
 # Cache only simulator/online runtime inputs. Report/checker code is rerun, never cached.
 excluded={'stage.py','report10.py','analysis10.py','inspect_ledger.py','build_s10.sh','run.sh','future_tool_adapter.md'}
 paths=[p for p in sources() if p.name not in excluded and 'tests' not in p.parts and 'references' not in p.parts]
 return {str(p.relative_to(ROOT)):digest(p) for p in paths}
def runtime_matches(saved):
 current=capture10()
 return all(saved.get(k)==v for k,v in current.items())
def receipt(out,d):
 docs=ROOT/'docs/optimization_v2/S10';lines=['# S10 实测结果','',f'本轮 `{d["run_id"]}`：**{d["status"]}**；人工 NOT_CHECKED。',d['next_step_verdict'],'',f'[离线交互报告]({out}/index.html) · [完整指标]({out}/result.json)','', '| 案例 | 实测仿真秒 | 车辆移动/m | 账本观测数 | 峰值/GiB | 自动 |','|---|---:|---:|---:|---:|---|']
 for name,c in d['cases'].items():
  m=c['metrics'];lines.append(f'| {name} | {m["sim_seconds"]} | {m["distance_m"]:.3f} | {m["credited_samples"]} | {m["peak_rss_gib"]:.2f} | {m["passed"]} |')
 lines+=['','每例 metrics.json 的 comparisons 给出同键在线确认量、独立物理作用量和超记量；物理作用包括账本未确认的启动/停车尾迹。以独立观测为准，不拿命令当完成。','', '实际范围为 A 区附近诊断点、短割草条带；目标基于在线估计选取，目标条件为诊断预设。近景识别、机械臂、液体/草体物理和完整自主作业没有实现，不由本结果推定通过。','', '沿用 S03 尺寸/停止门限、S04 传感器误差、S05–S08 定位与 S09 感知。感知短时不可用时工具关闭并等候，期间不计量；没有放宽原停车/反馈门限。失联与重启中断保留不确定标记，不能盲目重复喷洒。','', '已知连接路/B 区地图变形仍需后续修复复验；本步未扩大到这些区域。在线账本不使用真值；独立评分在 domain 120，输入与 PID/退出码在每例 runtime.json。','', '组件测试使用明确标注的合成观测，真实启动 C++ Action server 和 SQLite 节点测试取消、进程重启、时钟、版本、失联；16 项 Python 生产账本回归包括事务失败与 WAL 异常退出恢复；原项目工具交接回归 4 项。不能把组件合成输入当 Gazebo 物理性能。','', f'检查：`{json.dumps(d["checks"],ensure_ascii=False)}`；运行命令及退出码见 commands.jsonl.gz。本步未安装新系统包。']
 if d.get('error'):lines+=['',d['error']]
 (docs/'RESULTS.md').write_text('\n'.join(lines)+'\n');artifacts=files(out)+files(docs)+files(BASE/'lessons/S10')
 for c in d['cases'].values():
  p=ROOT/c['path']
  if not p.is_relative_to(out):artifacts+=files(p)
 for name in ['development_summary.json','manual_entrypoint_validation.json']:
  p=RESULTS/name
  if p.exists():artifacts.append(p)
 r=dict(schema_version=1,step='S10',plan_revision='20261109.1',run_id=d['run_id'],checked_at_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),automated_status=d['status'],manual_status='NOT_CHECKED',reason=d.get('error','真实工具反馈驱动的抽象动作、事务账本、去重/部分恢复、独立物理评分和故障联锁'),check_exit_code=0 if d['status']=='PASS' else 1,checks=d['checks'],report_path=str((out/'index.html').relative_to(ROOT)),source_files=[row(p) for p in sorted(set(sources()))],artifacts=[row(p) for p in sorted(set(artifacts))],dependency_receipts={s:digest(BASE/f'results/{s}/latest.json') for s in ['S08','S09']});dump(RESULTS/'latest.json',r)
def main():
 p=argparse.ArgumentParser();p.add_argument('mode',choices=['test','demo','stop']);p.add_argument('--suite',choices=['acceptance']);p.add_argument('--case',choices=['point_service_states','continuous_work_and_retry']);p.add_argument('--seed',type=int,default=41);p.add_argument('--sim-seconds',type=float,default=300);p.add_argument('--wall-timeout',type=float,default=1500);p.add_argument('--headless',action='store_true');p.add_argument('--resume',type=Path);a=p.parse_args()
 if a.mode=='stop':r=stop_session(SESSION,60);print(json.dumps(r));return 0 if r['status'] in ['STOPPED','NOT_RUNNING'] else 1
 if a.seed!=41:p.error('qualified seed=41')
 if not all(math.isfinite(x) and x>0 for x in [a.sim_seconds,a.wall_timeout]):p.error('finite positive budgets required')
 if a.mode=='test' and (a.suite!='acceptance' or a.case or a.sim_seconds<300):p.error('test: acceptance, >=300 sim budget')
 if a.mode=='demo' and (not a.case or a.suite or a.resume or a.sim_seconds<100):p.error('demo: case, >=100 sim budget, no suite/resume')
 if a.wall_timeout<120:p.error('wall >=120')
 for s in ['S08','S09']:
  if subprocess.run(['bash',str(BASE/'stepctl.sh'),'check',s],cwd=ROOT).returncode:return 4
 lock=(ROOT/'native_20261108/runs/session.lock').open('a')
 try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
 except BlockingIOError:print('已有项目会话持有锁');return 2
 if os.getpgrp()!=os.getpid():os.setsid()
 run_id=a.mode+'_'+datetime.datetime.now().strftime('%Y%m%d_%H%M%S')+'_'+str(os.getpid());out=RESULTS/run_id;out.mkdir(parents=True);save_session(SESSION,str(Path(__file__).resolve()),run_id);start=time.monotonic();data=dict(run_id=run_id,status='PARTIAL',mode=a.mode,seed=a.seed,command=sys.argv,checks={f'S10-C{i:02}':'NOT_RUN' for i in range(1,5)},cases={},next_step_verdict='暂不能执行下一步：S10 验收尚未通过')
 def interrupt(s,f):raise KeyboardInterrupt
 signal.signal(signal.SIGINT,interrupt);signal.signal(signal.SIGTERM,interrupt)
 def collect(name,seconds,kind):
  candidate=None
  if a.resume:
   prior=read(a.resume/'result.json');candidate=(ROOT/prior['cases'][name]['path']).resolve() if name in prior.get('cases',{}) else None
  if candidate:
   if not candidate.is_relative_to(RESULTS):raise ValueError('resume path outside results')
   if not runtime_matches(read(candidate/'s10_inputs.json')) or read(candidate/'s09_inputs.json')!=capture09() or read(candidate/'s08_inputs.json')!=s08_inputs():raise ValueError('resume source changed')
   from s07_common import capture_inputs
   if read(candidate/'capture_inputs.json')!=capture_inputs():raise ValueError('resume upstream runtime inputs changed')
   rt=read(candidate/'runtime.json')
   if not rt['success'] or rt['vehicle']!=kind or rt['case']!=name or rt['seed']!=41:raise ValueError('resume case mismatch')
   case_out=candidate
  else:
   case_out=out/name;case_out.mkdir();dump(case_out/'s10_inputs.json',capture10());remaining=a.wall_timeout-(time.monotonic()-start)-35
   if remaining<150:raise TimeoutError('remaining wall budget too small')
   print('RUN '+name,flush=True)
   try:run_case(case_out,'zone_a',name,seconds,min(remaining,seconds*2.5+65),a.seed,a.mode=='demo' and not a.headless,kind)
   except KeyboardInterrupt:
    if (case_out/'runtime.json').exists() and (case_out/'ledger/summary.json').exists():
     m=analyze(case_out);data['cases'][name]=dict(path=str(case_out.relative_to(ROOT)),metrics=m);dump(out/'progress.json',data)
    raise
  m=analyze(case_out);data['cases'][name]=dict(path=str(case_out.relative_to(ROOT)),metrics=m);dump(out/'progress.json',data);print(json.dumps(dict(case=name,passed=m['passed'],failed=[k for k,v in m['checks'].items() if not v])),flush=True)
 try:
  command(out,'dependencies',[sys.executable,str(PKG/'tests/dependencies.py'),str(out/'environment_verified.json')],20);command(out,'build',['bash',str(BASE/'scripts/build_s10.sh')],180)
  if a.mode=='test':
   command(out,'ledger_tests',[sys.executable,str(LEDGER/'tests/test_ledger.py')],30);command(out,'original_handoff',[sys.executable,'-m','unittest','field95.test_tool_handoff','-v'],20);command(out,'action_contract',[sys.executable,str(PKG/'tests/action_contract.py'),str(out/'component_action')],120)
   command(out,'lesson_compile',['g++','-std=c++17','-O2',str(BASE/'lessons/S10/lesson.cpp'),'-o',str(BASE/'ws/build/tool_sim/lesson_s10')],30);command(out,'lesson_cpp',[str(BASE/'ws/build/tool_sim/lesson_s10')],10);command(out,'lesson_python',[sys.executable,str(BASE/'lessons/S10/lesson.py')],20)
   collect('point_service_states',150,'diff');collect('continuous_work_and_retry',140,'ackermann')
   if any(not runtime_matches(read(ROOT/c['path']/'s10_inputs.json')) for c in data['cases'].values()):raise RuntimeError('Source changed during acceptance; rerun current version')
   ok=all(c['metrics']['passed'] for c in data['cases'].values()) and read(out/'component_action/result.json')['passed'];data['checks']={f'S10-C{i:02}':'PASS' if ok else 'FAIL' for i in range(1,5)};data['status']='PASS' if ok else 'FAIL'
  else:collect(a.case,min(a.sim_seconds-5,150 if a.case=='point_service_states' else 140),'diff' if a.case=='point_service_states' else 'ackermann');data['status']='PASS' if all(c['metrics']['passed'] for c in data['cases'].values()) else 'FAIL'
 except KeyboardInterrupt:data.update(status='PARTIAL',error='中断；本会话发出 STOP_WORK/底盘停车并保存账本，未确认停止内容重启后保持待核查')
 except Exception as e:data.update(status='FAIL',error=repr(e));(out/'exception.log').write_text(traceback.format_exc());print(traceback.format_exc(),flush=True)
 finally:
  try:
   data['wall_seconds']=time.monotonic()-start
   try:
    render(out,data)
    if data['cases']:command(out,'browser_check',[sys.executable,str(PKG/'tests/report_check.py'),str(out/'index.html')],45)
   except Exception as e:data.update(status='FAIL',error='Report: '+repr(e))
   if a.mode=='test' and data['status']=='PASS':
    ready=all(subprocess.run(['bash',str(BASE/'stepctl.sh'),'check',s],cwd=ROOT,stdout=subprocess.DEVNULL).returncode==0 for s in ['S02','S03']);data['next_step_verdict']='可以执行下一步 S11——按工作量自动划分三个连通簇：S10 自动通过且 S02/S03 前置有效' if ready else '暂不能执行下一步：S11 的 S02/S03 前置失效'
   if a.mode=='demo':data['next_step_verdict']='本轮为独立演示；正式状态见 stepctl next S10'
   for c in data['cases'].values():compact(ROOT/c['path'])
   compact(out);dump(out/'result.json',data)
   if a.mode=='test':receipt(out,data)
   else:
    dump(RESULTS/'last_demo.json',dict(run_id=run_id,status=data['status'],report_path=str((out/'index.html').relative_to(ROOT))))
    if not a.headless:subprocess.Popen(['xdg-open',str(out/'index.html')],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL,start_new_session=True)
  finally:SESSION.unlink(missing_ok=True);lock.close()
 print(json.dumps(dict(status=data['status'],report=str(out/'index.html'),next=data['next_step_verdict']),ensure_ascii=False));return 0 if data['status']=='PASS' else 1
if __name__=='__main__':raise SystemExit(main())
