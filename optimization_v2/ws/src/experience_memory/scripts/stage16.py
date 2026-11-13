"""Bounded S16 executor. Serial simulator, actual metrics, auditable resume."""
from common16 import *
import argparse,datetime,time,os,signal,fcntl,subprocess,traceback,shutil
from collections import Counter
from common15 import add14
add14()
from common11 import files,row
from stage15 import command
from dataset15 import validate as validate_dataset
from analysis15 import evaluate_case,rows
from runtime16 import run_case
from report16 import render
sys.path.insert(0,str(BASE/'ws/src/project_baseline/scripts'))
from session_owner import save_session,stop_session
RESULTS=BASE/'results/S16';SESSION=BASE/'runtime/S16.json'

def sources():
 return sorted(set(files(PKG)+files(BASE/'ws/install/experience_memory')+files(BASE/'stages/S16')+files(BASE/'lessons/S16')+[BASE/'scripts/build_s16.sh',BASE/'config/memory.yaml',BASE/'contracts/experience_schema.sql',BASE/'evaluation/memory_ablation.yaml']))
def measure(case):
 m=evaluate_case(case);forecasts=rows(case/'prediction/forecasts.jsonl');good=[x for x in forecasts if 'memory' in x];ds=[d for x in good for d in x['memory']['decisions']];runtime=read(case/'runtime.json')
 m['memory']=dict(frames=len(good),**{k:sum(x['memory'][k] for x in good) for k in ['feature_ms','query_ms','maintenance_ms','inference_ms','observe_ms','learned_calls','imm_calls']},wall_ms=sum(x['wall_ms'] for x in good),cpu_ms=sum(x['cpu_ms'] for x in good),queries=sum(d['lookup']['reason']!='NOT_QUERIED' for d in ds),hits=sum(d['lookup']['hit'] for d in ds),reasons=dict(Counter(d['lookup']['reason'] for d in ds)),db=read(case/'prediction/memory.audit.json') if (case/'prediction/memory.audit.json').exists() else None)
 m['memory_fault']=runtime.get('memory_fault','none');m['injected_version_rejected']=any(x.get('injected_fault')=='MAP_VERSION_MISMATCH' and any(d['lookup']['reason']=='VERSION_MISMATCH' and not d['lookup']['hit'] and d['selected']=='learned' for d in x.get('memory',{}).get('decisions',[])) for x in forecasts)
 dump(case/'metrics16.json',m);return m

def checkpoint(case,spec,binding,deps,snapshot):
 m=read(case/'metrics16.json');runtime=read(case/'runtime.json')
 if not m['safe'] or runtime.get('error'):return
 dump(case/'checkpoint16.json',dict(spec=spec,sources=binding,deps=deps,snapshot=snapshot,experiment_finished=True,evidence={str(p.relative_to(case)):sha(p) for p in files(case) if p.name!='checkpoint16.json'}))
def reuse(parents,case,spec,binding,deps,snapshot):
 for parent in parents:
  p=(parent/case.name).resolve()
  if not p.is_relative_to(RESULTS) or p==case.resolve():raise ValueError('UNSAFE_RESUME')
  if not (p/'checkpoint16.json').is_file():continue
  c=read(p/'checkpoint16.json')
  if any(c.get(k)!=v for k,v in dict(spec=spec,sources=binding,deps=deps,snapshot=snapshot).items()):continue
  if any(not (p/k).resolve().is_relative_to(p) or not (p/k).is_file() or sha(p/k)!=v for k,v in c['evidence'].items()):continue
  case.mkdir(parents=True)
  for k in c['evidence']:
   dest=case/k;dest.parent.mkdir(parents=True,exist_ok=True);os.link(p/k,dest)
  dump(case/'checkpoint16.json',dict(c,reused_from=str(p)));return read(case/'metrics16.json')
 return None

def choose(data,offline):
 cfg=yaml.safe_load((BASE/'evaluation/memory_ablation.yaml').read_text());m=offline['metrics']['test'];a=m['event'];b=m['cache'];saving=1-b['cpu_ms']/a['cpu_ms'];reasons=[]
 if saving<cfg['minimum_cpu_saving_fraction']:reasons.append('NO_NET_CPU_GAIN')
 if b['weighted_ade_m']>max(a['weighted_ade_m']*cfg['max_ade_ratio_vs_event'],a['weighted_ade_m']+cfg['max_ade_increase_m']):reasons.append('ACCURACY_REGRESSION')
 if offline['test_scored_hits']<cfg['minimum_test_hits']:reasons.append('INSUFFICIENT_TEST_HITS')
 if offline['wrong_hit_fraction'] is None or offline['wrong_hit_fraction']>cfg['max_wrong_hit_fraction']:reasons.append('WRONG_HIT_OR_INSUFFICIENT_EVIDENCE')
 normal=[x for x in data['cases'].values() if x['memory_fault']=='none'];by={(x['scenario'],x['method']):x for x in normal};ratios=[]
 for scenario in cfg['closed_loop']:
  e=by.get((scenario,'event'));c=by.get((scenario,'cache'))
  if not e or not c or not e['completed'] or not c['completed']:reasons.append('INCOMPLETE_PAIRED_TASK:'+scenario)
  else:ratios.append(c['sim_seconds_including_stop']/e['sim_seconds_including_stop'])
 if ratios and sum(ratios)/len(ratios)>cfg['max_task_time_ratio_vs_event']:reasons.append('TASK_TIME_REGRESSION')
 exposures=sum(c['risk_misses']['eligible'] for c in normal if c['method']=='cache')
 if exposures<cfg['minimum_conflict_exposures']:reasons.append('INSUFFICIENT_CONFLICT_EXPOSURE')
 if any(not c['safe'] for c in data['cases'].values()):reasons.append('SAFETY_FAILURE')
 s15=read(BASE/'results/S15/latest.json');previous=read(ROOT/s15['report_path'].replace('index.html','result.json'))
 if previous['selection']['default']=='no_prediction':reasons.append('S15_PREDICTIVE_POLICIES_NOT_QUALIFIED')
 return dict(enabled=False,recommendation='disable' if reasons else 'candidate_only_requires_explicit_config_review',reasons=reasons,default_config_enabled=config()['enabled'],cpu_saving_fraction_vs_event=saving,wall_saving_fraction_vs_event=1-b['wall_ms']/a['wall_ms'],paired_task_time_ratio=sum(ratios)/len(ratios) if ratios else None,measured_conflict_exposures=exposures,scope='four short scenes; no six-cluster or real-world qualification')

def receipt(out,data):
 docs=ROOT/'docs/optimization_v2/S16'
 body='# S16 本轮实测结果\n\n'+f"自动：{data['status']}；人工：NOT_CHECKED。运行 `{data['run_id']}`。\n\n"
 body+='四种场景的冻结观测三组回放，以及同场景两点任务闭环；不替代 S17 六簇。训练库不包含测试未来标签。原 S15/S14 安全门限保持。\n\n'
 body+='```json\n'+json.dumps({k:data.get(k) for k in ['checks','selection','offline_summary','cases','error']},ensure_ascii=False,indent=2)+'\n```\n\n'
 body+='[实际可视报告]('+str(out/'index.html')+')；命令、退出码见同目录 commands.jsonl。Gazebo 停车后的清理退出码单列，−11 不冒充正常退出。\n'
 (docs/'RESULTS.md').write_text(body)
 artifacts=files(out)+files(docs)+files(BASE/'lessons/S16')
 if (out/'offline/inputs.json').exists():artifacts += [ROOT/k for k in read(out/'offline/inputs.json')]
 r=dict(schema_version=1,step='S16',plan_revision='20261109.1',run_id=data['run_id'],checked_at_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),automated_status=data['status'],manual_status='NOT_CHECKED',reason=data.get('error') or 'Actual three-way replay, bounded cache rejection/restart and Gazebo task measurements; default disabled.',check_exit_code=0 if data['status']=='PASS' else 1,checks=data['checks'],report_path=str((out/'index.html').relative_to(ROOT)),source_files=[row(p) for p in sources()],artifacts=[row(p) for p in sorted(set(artifacts))],dependency_receipts={'S15':sha(BASE/'results/S15/latest.json')})
 dump(RESULTS/'latest.json',r)

def main():
 p=argparse.ArgumentParser();p.add_argument('mode',choices=['test','demo','stop']);p.add_argument('--case',choices=['memory_three_way','memory_wrong_match']);p.add_argument('--suite',choices=['acceptance']);p.add_argument('--seed',type=int,default=41);p.add_argument('--sim-seconds',type=float,default=300);p.add_argument('--wall-timeout',type=float,default=2400);p.add_argument('--headless',action='store_true');p.add_argument('--resume-from',type=Path,action='append',default=[]);a=p.parse_args()
 if a.mode=='stop':
  stopped=stop_session(SESSION,40);print(json.dumps(stopped));return 0 if stopped['status'] in ['STOPPED','NOT_RUNNING'] else 1
 if a.seed!=41 or not 100<=a.sim_seconds<=300 or not 0<a.wall_timeout<=14400:p.error('seed41; 100<=sim<=300; 0<wall<=14400')
 if a.mode=='test' and (a.suite!='acceptance' or a.case):p.error('test requires --suite acceptance')
 if a.mode=='demo' and (not a.case or a.suite):p.error('demo requires --case')
 lock=(ROOT/'native_20261108/runs/session.lock').open('a')
 try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
 except BlockingIOError:print('已有项目仿真，S16 拒绝并发');return 2
 run_id=a.mode+'_'+datetime.datetime.now().strftime('%Y%m%d_%H%M%S')+'_'+str(os.getpid());out=RESULTS/run_id;out.mkdir(parents=True)
 if os.getpgrp()!=os.getpid():os.setsid()
 save_session(SESSION,str(Path(__file__).resolve()),run_id);session=read(SESSION);session.update(run_directory=str(out),ros_domain=119,evaluation_domain=120,seed=41);dump(SESSION,session)
 def stop(s,f):
  if s==signal.SIGALRM:raise TimeoutError('S16_WALL_BUDGET; verified checkpoints retained')
  raise KeyboardInterrupt
 for s in [signal.SIGINT,signal.SIGTERM,signal.SIGALRM]:signal.signal(s,stop)
 signal.setitimer(signal.ITIMER_REAL,max(.1,a.wall_timeout-5));start=time.monotonic()
 data=dict(run_id=run_id,status='PARTIAL',mode=a.mode,seed=41,command=sys.argv,manual_status='NOT_CHECKED',cases={},checks={f'S16-C{i:02}':'NOT_RUN' for i in range(1,5)})
 try:
  command(out,'prerequisite_S15',['bash',str(BASE/'stepctl.sh'),'check','S15'],300)
  command(out,'build',['bash',str(BASE/'scripts/build_s16.sh')],90)
  command(out,'contracts',[str(BASE/'venvs/prediction/bin/python'),str(PKG/'tests/contracts16.py'),str(out/'contracts.json')],30)
  command(out,'lesson_python',[str(BASE/'venvs/prediction/bin/python'),str(BASE/'lessons/S16/lesson.py')],30)
  dump(out/'prediction.json',yaml.safe_load((BASE/'config/prediction.yaml').read_text()))
  command(out,'lesson_build',['c++','-std=c++17','-O2','-UNDEBUG','-I'+str(PKG/'include'),'-I'+str(PKG.parent/'prediction_risk_guard/include'),str(BASE/'lessons/S16/lesson.cpp'),'-o',str(out/'lesson_cpp')],45)
  command(out,'lesson_cpp',[str(out/'lesson_cpp'),str(out/'prediction.json')],10)
  validate_dataset(read(BASE/'datasets/prediction/manifest.json'))
  binding={str(p.relative_to(ROOT)):sha(p) for p in sources()};deps=dict(S15=sha(BASE/'results/S15/latest.json'),dataset=sha(BASE/'datasets/prediction/manifest.json'));dump(out/'source_snapshot.json',binding);dump(out/'input_bindings.json',deps)
  restored=False
  for parent in a.resume_from:
   parent=parent.resolve()
   if not parent.is_relative_to(RESULTS):raise ValueError('UNSAFE_RESUME')
   proof=parent/'offline_checkpoint.json'
   if not proof.exists():continue
   c=read(proof)
   if c['sources']!=binding or c['deps']!=deps:continue
   if any(not (parent/k).is_file() or sha(parent/k)!=v for k,v in c['evidence'].items()):continue
   shutil.copytree(parent/'offline',out/'offline');restored=True;break
  if not restored:command(out,'frozen_replay',[str(BASE/'venvs/prediction/bin/python'),str(PKG/'scripts/evaluate16.py'),'--out',str(out/'offline')],600)
  offline=read(out/'offline/offline.json');data['offline_summary']=offline['metrics'];snapshot=out/'offline/initial.sqlite';snapshot_sha=sha(snapshot)
  if any(sha(ROOT/k)!=v for k,v in read(out/'offline/inputs.json').items()):raise ValueError('INPUTS_CHANGED')
  dump(out/'offline_checkpoint.json',dict(sources=binding,deps=deps,evidence={str(p.relative_to(out)):sha(p) for p in files(out/'offline')}))
  cfg=yaml.safe_load((BASE/'evaluation/memory_ablation.yaml').read_text());matrix=[dict(scenario='turning',group='cache',fault='wrong_match')]+[dict(scenario=s,group=g,fault='none') for s in cfg['closed_loop'] for g in ['full','event','cache']]
  if a.mode=='demo':matrix=[dict(scenario='turning',group=g,fault='none') for g in ['full','event','cache']] if a.case=='memory_three_way' else [dict(scenario='turning',group='cache',fault='wrong_match')]
  for spec in matrix:
   name=spec['scenario']+'_'+spec['group']+'_'+spec['fault'];case=out/name
   cached=reuse(a.resume_from,case,spec,binding,deps,snapshot_sha) if a.mode=='test' else None
   if cached:data['cases'][name]=cached;dump(out/'result.json',data);render(out,data);continue
   remaining=a.wall_timeout-(time.monotonic()-start)-25
   if remaining<150:raise TimeoutError('INSUFFICIENT_WALL_BUDGET; resume completed cases')
   print('S16 RUN '+name,flush=True);ts=time.monotonic()
   r=run_case(case,spec['scenario'],spec['group'],snapshot,100,min(360,remaining),a.mode=='demo' and not a.headless,spec['fault'])
   with (out/'commands.jsonl').open('a') as f:f.write(json.dumps(dict(name=name,call='actual Gazebo runtime16',spec=spec,wall_s=time.monotonic()-ts,task_exit_code=0 if r.get('success') else 1))+'\n')
   data['cases'][name]=measure(case)
   if binding!={str(p.relative_to(ROOT)):sha(p) for p in sources()}:raise ValueError('SOURCE_CHANGED_DURING_EXPERIMENT')
   checkpoint(case,spec,binding,deps,snapshot_sha);dump(out/'result.json',data);render(out,data)
  if a.mode=='test':
   normal=[x for x in data['cases'].values() if x['memory_fault']=='none'];faults=[x for x in data['cases'].values() if x['memory_fault']!='none'];contracts=read(out/'contracts.json');matched=all(len({x['fixture_sha'] for x in normal if x['scenario']==s})==1 and len({x['benchmark_task_sha'] for x in normal if x['scenario']==s})==1 for s in cfg['closed_loop'])
   data['selection']=choose(data,offline)
   data['checks']['S16-C01']='PASS' if offline['fair_schedules'] and matched and len(normal)==12 and all(x['memory']['frames']>0 for x in normal) else 'FAIL'
   data['checks']['S16-C02']='PASS' if contracts['passed'] and len(faults)==1 and faults[0]['injected_version_rejected'] and all(x['safe'] for x in data['cases'].values()) else 'FAIL'
   data['checks']['S16-C03']='PASS' if contracts['checks']['bounded_growth'] and contracts['checks']['growth_restart'] and all(x['memory']['db'] and x['memory']['db']['bytes']<=config()['max_store_bytes'] for x in data['cases'].values()) else 'FAIL'
   data['checks']['S16-C04']='PASS' if not data['selection']['enabled'] and not config()['enabled'] and data['selection']['reasons'] else 'FAIL'
   data['status']='PASS' if all(x=='PASS' for x in data['checks'].values()) else 'FAIL'
  else:data['status']='PASS' if all(x['safe'] and (x['injected_version_rejected'] if a.case=='memory_wrong_match' else True) for x in data['cases'].values()) else 'FAIL'
  render(out,data)
  command(out,'browser_interaction',['/usr/bin/python3',str(PKG/'tests/browser16.py'),str(out)],50)
 except (KeyboardInterrupt,TimeoutError) as exc:data.update(status='PARTIAL',error=repr(exc))
 except Exception as exc:data.update(status='FAIL',error=repr(exc));(out/'exception.log').write_text(traceback.format_exc());print(traceback.format_exc(),flush=True)
 finally:
  signal.setitimer(signal.ITIMER_REAL,0);data['wall_seconds']=time.monotonic()-start;data['next_step_verdict']='可以执行下一步 S17' if a.mode=='test' and data['status']=='PASS' else '暂不能凭本轮进入 S17';dump(out/'result.json',data);render(out,data)
  try:
   if a.mode=='test':receipt(out,data)
   else:dump(RESULTS/'last_demo.json',dict(run_id=run_id,status=data['status'],report_path=str((out/'index.html').relative_to(ROOT))))
  finally:SESSION.unlink(missing_ok=True);lock.close()
 if a.mode=='demo' and not a.headless:subprocess.Popen(['xdg-open',str(out/'index.html')],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL,start_new_session=True)
 print(json.dumps(dict(status=data['status'],report=str(out/'index.html'),next=data['next_step_verdict']),ensure_ascii=False));return 0 if data['status']=='PASS' else 1
if __name__=='__main__':raise SystemExit(main())
