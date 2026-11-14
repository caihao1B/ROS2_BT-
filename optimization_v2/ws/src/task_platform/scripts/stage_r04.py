from common import *
from dynamics_r04 import build,phases,evaluate,figures
from probe_r02 import run

def main(command,args):
 require('R03');out=APP/'results/R04';out.mkdir(parents=True,exist_ok=True)
 if command=='build':build();return 0
 if command=='test':
  ok=True
  for task in args or TASKS:
   if task not in TASKS:raise ValueError('UNKNOWN_TASK')
   c=read(edition('R04')/'vehicles'/f'{task}.json');p=run(task,phases(c),stage='R04',config=c);r=evaluate(p);ok &=r['passed'];print(json.dumps(dict(task=task,passed=r['passed'],failed=[k for k,v in r['checks'].items() if not v])),flush=True)
  return 0 if ok else 2
 if command!='finalize':raise ValueError('USE build / test [task] / finalize / check / open')
 paths={t:ROOT/read(out/(t+'_latest.json'))['path'] for t in TASKS};results={t:evaluate(p) for t,p in paths.items()};passed=all(r['passed'] for r in results.values())
 qualified=dict(stage='R04',passed=passed,vehicles={t:dict(vehicle_hash=read(edition('R04')/'vehicles'/f'{t}.json')['config_hash'],flat_transit_max_mps=.8,selected_transit_mps=.6,selected_work_mps=.5 if t=='mowing' else .4,reverse_mps=.25,slope_qualified=False,full_navigation_qualified=False,analysis_sha256=sha(paths[t]/'analysis_r04.json')) for t in TASKS},upstream_plan_sha256=sha(edition('R03')/'mission.json'))
 dump(edition('R04')/'qualification.json',qualified)
 with (out/'tests.log').open('w') as log:r=subprocess.run([sys.executable,str(PKG/'tests/test_r04.py')],stdout=log,stderr=subprocess.STDOUT,timeout=30)
 summary=dict(passed=passed and r.returncode==0,vehicles=results,tests_exit=r.returncode,tests=4,next='R05 shared sensors and localization',scope='Flat physics and watchdog qualification only; original operational speed limits unchanged')
 images=figures(paths);report=render('R04','R04 / speed, braking and holdout turning',summary,images)
 sources=list((PKG/'scripts').glob('*_r04.py'))+list((PKG/'tests').glob('*_r04.py'))+list((ROOT/'docs/three_tasks/R04').glob('*.md'))
 files=list(edition('R04').rglob('*'))+[APP/'results/R03/latest.json',report,out/'tests.log',*images]
 for p in paths.values():files+=list(p.rglob('*'))
 d=seal('R04',summary,files,sources,'R05');print(json.dumps(dict(status=d['automated_status'],next=d['next_step'])));return 0 if summary['passed'] else 2
