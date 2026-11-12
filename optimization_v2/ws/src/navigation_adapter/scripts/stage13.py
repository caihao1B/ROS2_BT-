"""Bounded S13 executor: serial real Gazebo runs; never infer success from launches."""
from common13 import *
import argparse,datetime,fcntl,signal,traceback,subprocess
from runtime13 import run_case
from analysis13 import evaluate
from report13 import render
sys.path.insert(0,str(BASE/'ws/src/project_baseline/scripts'))
from session_owner import save_session,stop_session
RESULTS=BASE/'results/S13';SESSION=BASE/'runtime/S13.json'

def sources():
 paths=files(PKG)+files(BASE/'lessons/S13')+files(BASE/'references/S13')+files(BASE/'stages/S13')+[BASE/'scripts/build_s13.sh']
 paths += [BASE/'config'/x for x in ['controller_profiles.yaml','inspection_route.yaml','navigation_scope.yaml','nav2_diff.yaml','nav2_ackermann.yaml','rviz/navigation.rviz','connector_registration.json']]
 paths += files(BASE/'ws/install/navigation_adapter')
 if (BASE/'config/connector_registration.json').exists():
  reg=read(BASE/'config/connector_registration.json')
  paths += [ROOT/s['source']/'sources/events.jsonl' for s in reg['train']+reg['heldout']]
 return sorted(p for p in set(paths) if p.is_file())

def command(out,name,args,timeout=120):
 start=time.monotonic();r=subprocess.run(args,cwd=ROOT,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,timeout=timeout);(out/(name+'.log')).write_text(r.stdout)
 with (out/'commands.jsonl').open('a') as f:f.write(json.dumps(dict(name=name,args=args,exit_code=r.returncode,wall_s=time.monotonic()-start))+'\n')
 if r.returncode:raise RuntimeError(name+' failed; see '+str(out/(name+'.log')))

def receipt(out,data):
 from docs13 import write_results
 write_results(out,data);artifacts=files(out)+files(ROOT/'docs/optimization_v2/S13')+files(BASE/'lessons/S13')
 r=dict(schema_version=1,step='S13',plan_revision='20261109.1',run_id=data['run_id'],checked_at_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),automated_status=data['status'],manual_status='NOT_CHECKED',reason=data.get('error','Real controller/action/localization-loss/inspection evidence; tool OFF, no whole-cluster work claim.'),check_exit_code=0 if data['status']=='PASS' else 1,checks=data['checks'],report_path=str((out/'index.html').relative_to(ROOT)),source_files=[row(p) for p in sources()],artifacts=[row(p) for p in sorted(set(artifacts))],dependency_receipts={s:sha(BASE/f'results/{s}/latest.json') for s in ['S08','S09','S12']});dump(RESULTS/'latest.json',r)

def assess(data):
 c=data['cases'];bench=['diff_DWB','diff_MPPI','diff_MPPI_repeat_2','diff_MPPI_repeat_3','ackermann_RPP','ackermann_MPPI'];inspect=['inspection_diff','inspection_ackermann']
 def all_cases(names,predicate):return all(n in c and predicate(c[n]) for n in names)
 c01=all_cases(bench,lambda r:r['completed'] and r['command_limits_ok'])
 s=c.get('action_semantics',{});f=c.get('localization_loss',{});ids={}
 for e in s.get('actions',[]):
  if e['event']=='action_result':ids[e.get('id','').split('_resume_')[0]]=e
 semantics=all(ids.get(i,{}).get('status')==4 for i in ['resume_navigate','preempt_follow','after_restart','controller_switch']) and all(ids.get(i,{}).get('status')==6 for i in ['invalid_controller','unreachable_goal']) and all(ids.get(i,{}).get('status')==5 for i in ['cancel_navigate','cancel_follow']) and all(any(e.get('event')=='cancel_request' and e.get('id','').startswith(i) for e in s.get('actions',[])) for i in ['cancel_navigate','cancel_follow']) and any(e.get('event')=='nav_lifecycle_active_verified' for e in s.get('actions',[])) and any(e.get('event')=='preempted_result' and e.get('status')!=4 for e in s.get('actions',[]))
 c02=bool(s.get('passed') and s.get('nav_restart_verified') and s.get('restart_handshake_verified') and f.get('passed') and f.get('fault_injection_verified') and semantics and all(r['ownership_ok'] and r['truth_isolated'] for r in c.values()))
 c03=bool(data.get('near_field_check',{}).get('passed')) and all_cases(bench+inspect,lambda r:r['passed'] and r['compute_p95_target_met'] and (r['effective_compute_hz_sim'] or 0)>=20)
 c04=all_cases(inspect,lambda r:r['passed'] and r['checkpoint_observations_current'] and len(r['checkpoints'])>=3)
 data['checks']={f'S13-C{i:02}':'PASS' if x else 'FAIL' for i,x in enumerate([c01,c02,c03,c04],1)};data['status']='PASS' if all([c01,c02,c03,c04]) else 'FAIL'

def main():
 ap=argparse.ArgumentParser();ap.add_argument('mode',choices=['test','demo','stop']);ap.add_argument('--case',choices=['diff_point_and_inspect','ackermann_follow_path']);ap.add_argument('--suite',choices=['acceptance']);ap.add_argument('--seed',type=int,default=41);ap.add_argument('--sim-seconds',type=float,default=600);ap.add_argument('--wall-timeout',type=float,default=3600);ap.add_argument('--headless',action='store_true');a=ap.parse_args()
 if a.mode=='stop':
  r=stop_session(SESSION,30);print(json.dumps(r));return 0 if r['status'] in ['STOPPED','NOT_RUNNING'] else 1
 if a.mode=='test' and (a.suite!='acceptance' or a.case):ap.error('test requires --suite acceptance')
 if a.mode=='demo' and (not a.case or a.suite):ap.error('demo requires --case')
 if a.seed!=41 or not all(math.isfinite(x) and x>0 for x in [a.sim_seconds,a.wall_timeout]):ap.error('seed 41 and positive finite budgets required')
 lock=(ROOT/'native_20261108/runs/session.lock').open('a')
 try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
 except BlockingIOError:print('已有项目仿真占用互斥锁');return 2
 run_id=a.mode+'_'+datetime.datetime.now().strftime('%Y%m%d_%H%M%S')+'_'+str(os.getpid());out=RESULTS/run_id;out.mkdir(parents=True)
 if os.getpgrp()!=os.getpid():os.setsid()
 save_session(SESSION,str(Path(__file__).resolve()),run_id);record=read(SESSION);record.update(ros_domain=119,raw_domain=121,evaluation_domain=120,seed=a.seed,run_directory=str(out));dump(SESSION,record)
 def interrupt(s,f):
  if s==signal.SIGALRM:raise TimeoutError('S13_WALL_BUDGET')
  raise KeyboardInterrupt
 for sig in [signal.SIGINT,signal.SIGTERM,signal.SIGALRM]:signal.signal(sig,interrupt)
 signal.setitimer(signal.ITIMER_REAL,max(.1,a.wall_timeout-5));start=time.monotonic();data=dict(run_id=run_id,status='PARTIAL',mode=a.mode,seed=a.seed,cases={},checks={f'S13-C{i:02}':'NOT_RUN' for i in range(1,5)},command=sys.argv,manual_status='NOT_CHECKED',tool_work_credit=0)
 try:
  for step in ['S08','S09','S12']:command(out,'prerequisite_'+step,['bash',str(BASE/'stepctl.sh'),'check',step],120)
  m=plan();dump(out/'input_bindings.json',dict(plan_revision=m['plan_revision'],source_files=[row(p) for p in sources()],frozen_map_manifest=read(BASE/'maps/frozen/manifest.json'),task_scope=yaml.safe_load((BASE/'config/navigation_scope.yaml').read_text())))
  if a.mode=='test':
   command(out,'build',['bash',str(BASE/'scripts/build_s13.sh')],180);command(out,'unit_tests',[sys.executable,str(PKG/'tests/test_contracts.py')],30);command(out,'dependencies',[sys.executable,str(PKG/'tests/dependencies.py'),str(out/'environment_verified.json')],30)
   command(out,'discovery',[sys.executable,str(PKG/'tests/discovery_check.py'),str(out/'discovery')],75)
   command(out,'startup_clock',[sys.executable,str(PKG/'tests/startup_check.py'),str(out/'startup_clock')],75)
   command(out,'restart_handshake',[sys.executable,str(PKG/'tests/restart_check.py'),str(out/'restart_handshake')],150)
   command(out,'lesson_build',['c++','-std=c++17','-O2',str(BASE/'lessons/S13/lesson.cpp'),'-o',str(out/'lesson_cpp')],30);command(out,'lesson_cpp',[str(out/'lesson_cpp')],10);command(out,'lesson_python',['python3',str(BASE/'lessons/S13/lesson.py')],10)
   matrix=[('action_semantics','diff','DWB','semantics',400),('diff_DWB','diff','DWB','smoke',140),('diff_MPPI','diff','MPPI','smoke',140),('diff_MPPI_repeat_2','diff','MPPI','smoke',140),('diff_MPPI_repeat_3','diff','MPPI','smoke',140),('ackermann_RPP','ackermann','RPP','smoke',160),('ackermann_MPPI','ackermann','MPPI','smoke',160),('localization_loss','diff','DWB','localization_loss',220),('inspection_diff','diff','DWB','diff_point_and_inspect',260),('inspection_ackermann','ackermann','RPP','inspect',260)]
  else:matrix=[(a.case,'diff' if a.case.startswith('diff') else 'ackermann','DWB' if a.case.startswith('diff') else 'RPP',a.case,a.sim_seconds)]
  binding={str(p.relative_to(ROOT)):sha(p) for p in sources()};dump(out/'source_snapshot.json',binding)
  if a.mode=='test':
   from sensor_check13 import evaluate_sensor
   print('S13 RUN near_field_obstacle',flush=True);run_case(out/'near_field_obstacle','zone_a','near_obstacle',65,200,a.seed,False,'diff','DWB');data['near_field_check']=evaluate_sensor(out/'near_field_obstacle');dump(out/'result.json',data)
  for name,kind,controller,case,seconds in matrix:
   remaining=a.wall_timeout-(time.monotonic()-start)-30
   if remaining<45:raise TimeoutError('INSUFFICIENT_REMAINING_WALL_BUDGET')
   print('S13 RUN '+name,flush=True);run_case(out/name,'zone_a',case,min(a.sim_seconds,seconds),min(remaining,seconds*3+100),a.seed,a.mode=='demo' and not a.headless,kind,controller);data['cases'][name]=evaluate(out/name);dump(out/'result.json',data);render(out,data)
  if {str(p.relative_to(ROOT)):sha(p) for p in sources()}!=binding:raise ValueError('SOURCE_CHANGED_DURING_TEST')
  plan()
  if a.mode=='test':assess(data)
  else:data['status']='PASS' if all(r['passed'] for r in data['cases'].values()) else 'FAIL'
  if data['status']!='PASS':data['error']='部分真实闭环或门限未通过，见逐案例动作结果与指标；尚未满足进入 S14 的前置条件。'
  render(out,data);command(out,'browser_check',[sys.executable,str(PKG/'tests/report_check.py'),str(out/'index.html')],50)
 except KeyboardInterrupt:data.update(status='PARTIAL',error='本轮中断；已请求停车、关闭工具并保留剩余路线。')
 except Exception as e:data.update(status='FAIL',error=repr(e));(out/'exception.log').write_text(traceback.format_exc());print(traceback.format_exc(),flush=True)
 finally:
  signal.setitimer(signal.ITIMER_REAL,0);data['wall_seconds']=time.monotonic()-start;data['next_step_verdict']='可以执行下一步 S14' if a.mode=='test' and data['status']=='PASS' else '暂不能凭本轮结果进入 S14；以有效完整自动回执为准。';dump(out/'result.json',data);render(out,data)
  try:
   if a.mode=='test':receipt(out,data)
   else:dump(RESULTS/'last_demo.json',dict(run_id=run_id,status=data['status'],report_path=str((out/'index.html').relative_to(ROOT))))
  finally:SESSION.unlink(missing_ok=True);lock.close()
 if a.mode=='demo' and not a.headless:subprocess.Popen(['xdg-open',str(out/'index.html')],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL,start_new_session=True)
 print(json.dumps(dict(status=data['status'],report=str(out/'index.html'),next=data['next_step_verdict']),ensure_ascii=False));return 0 if data['status']=='PASS' else 1
if __name__=='__main__':raise SystemExit(main())
