from common import *
import signal,time
from policy_r06 import build

def protocol(task):
 out=APP/'results/R06'/(task+'_'+stamp());out.mkdir(parents=True);env=os.environ.copy();env.update(ROS_DOMAIN_ID='123',OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1');cfg=edition('R06')/(task+'.json');log=(out/'supervisor.log').open('w');p=subprocess.Popen([str(APP/'build/r06/r06_supervisor'),'--ros-args','-p','use_sim_time:=true','-p','config_file:='+str(cfg),'-p','tree_file:='+str(edition('R06')/'common.xml'),'-p','log_file:='+str(out/'decisions.jsonl')],env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True);forced=False;code=None
 try:
  with (out/'fixture.log').open('w') as f:r=subprocess.run(['/usr/bin/python3',str(PKG/'scripts/protocol_r06.py'),'--out',str(out),'--config',str(cfg)],env=env,stdout=f,stderr=subprocess.STDOUT,timeout=100);code=r.returncode
 finally:
  if p.poll() is None:p.send_signal(signal.SIGINT)
  try:p.wait(timeout=8)
  except subprocess.TimeoutExpired:forced=True;p.kill();p.wait(timeout=3)
  log.close();dump(out/'runtime.json',dict(supervisor_exit=p.returncode,fixture_exit=code,forced=forced,passed=p.returncode==0 and code==0 and not forced,context_scope='synthetic protocol fixture; never actuator commands'))
 dump(APP/'results/R06'/(task+'_latest.json'),dict(path=str(out.relative_to(ROOT))));return out

def plots(paths):
 import matplotlib;matplotlib.use('Agg');import matplotlib.pyplot as plt
 fig,ax=plt.subplots(figsize=(14,5));actions=['EXECUTE','FOLLOW','DETOUR','WAIT','CANCEL','REPLAN','DEFER','STOP','HUMAN','COMPLETE']
 for i,(task,p) in enumerate(paths.items()):
  data=[json.loads(l) for l in (p/'decisions.jsonl').read_text().splitlines()];ax.plot([d['t'] for d in data],[actions.index(d['action'])+i*.12 for d in data],label=task,drawstyle='steps-post',alpha=.8)
 ax.set_yticks(range(len(actions)),actions);ax.set(xlabel='Fixture simulated seconds (ROS clock)',title='R06 real BT runtime / declared context cases');ax.grid(alpha=.3);ax.legend();fig.tight_layout();p=APP/'results/R06/decisions.png';fig.savefig(p,dpi=160);plt.close(fig);return[p]

def main(command,args):
 require('R05');out=APP/'results/R06';out.mkdir(parents=True,exist_ok=True)
 if command=='build':
  build();b=APP/'build/r06';subprocess.run(['cmake','-S',str(PKG/'r06_backend'),'-B',str(b)],check=True);subprocess.run(['cmake','--build',str(b),'--parallel','2'],check=True);return 0
 if command=='test':
  ok=True
  for t in args or TASKS:
   if t not in TASKS:raise ValueError('UNKNOWN_TASK')
   p=protocol(t);r=read(p/'protocol_result.json');ok &=r['passed'];print(json.dumps(dict(task=t,passed=r['passed'],failed=[x['name'] for x in r['cases'] if not x['passed']])),flush=True)
  return 0 if ok else 2
 if command!='finalize':raise ValueError('USE build / test / finalize / check / open')
 paths={t:ROOT/read(out/(t+'_latest.json'))['path'] for t in TASKS};tests={}
 for t in TASKS:
  p=subprocess.run([str(APP/'build/r06/r06_policy_test'),str(edition('R06')/(t+'.json')),str(edition('R06')/'common.xml')],capture_output=True,text=True,timeout=20);tests[t]=dict(exit=p.returncode,stdout=p.stdout,stderr=p.stderr)
 dump(out/'native_tests.json',tests);results={t:read(p/'protocol_result.json') for t,p in paths.items()};summary=dict(passed=all(r['passed'] and read(paths[t]/'runtime.json')['passed'] and tests[t]['exit']==0 for t,r in results.items()),tasks=results,native_tests=tests,scope='Common BT priority, explanation, hysteresis, bounded retries and cancellation/acknowledgement protocol. Real sensors and vehicles independently qualified R02-R05; task integration pending R07-R09',next='R07 point service integration')
 images=plots(paths);report=render('R06','R06 / explainable common BT protocol',summary,images);sources=list((PKG/'scripts').glob('*_r06.py'))+list((PKG/'r06_backend').glob('*'))+list((ROOT/'docs/three_tasks/R06').glob('*.md'));files=list(edition('R06').rglob('*'))+[APP/'results/R05/latest.json',out/'native_tests.json',report,*images,*list((APP/'build/r06').glob('r06_*'))]
 for p in paths.values():files+=list(p.rglob('*'))
 d=seal('R06',summary,files,sources,'R07');print(json.dumps(dict(status=d['automated_status'],next=d['next_step'])));return 0 if summary['passed'] else 2
