from common import *
from rig_r05 import build
from runtime_r05 import run
from analyze_r05 import analyze,figures

def main(command,args):
 require('R04');out=APP/'results/R05';out.mkdir(parents=True,exist_ok=True)
 if command=='build':build();return 0
 if command=='test':
  ok=True
  for t in args or TASKS:
   if t not in TASKS:raise ValueError('UNKNOWN_TASK')
   r=analyze(run(t));ok &= r['passed'];print(json.dumps(dict(task=t,passed=r['passed'],failed=[k for k,v in r['checks'].items() if not v])),flush=True)
  return 0 if ok else 2
 if command!='finalize':raise ValueError('USE build / test [task] / finalize / check / open')
 paths={t:ROOT/read(out/(t+'_latest.json'))['path'] for t in TASKS};results={t:analyze(p) for t,p in paths.items()}
 with (out/'tests.log').open('w') as log:r=subprocess.run([sys.executable,str(PKG/'tests/test_r05.py')],stdout=log,stderr=subprocess.STDOUT,timeout=90)
 summary=dict(passed=all(r['passed'] for r in results.values()) and r.returncode==0,vehicles=results,tests_exit=r.returncode,next='R06 common BT and motion supervision',scope='Rig/map localization qualification at three declared stations; autonomous tasks not accepted')
 images=figures(paths);report=render('R05','R05 / shared sensor and registered map / RTAB local odometry',summary,images)
 sources=list((PKG/'scripts').glob('*_r05.py'))+list((PKG/'tests').glob('*_r05.py'))+list((ROOT/'docs/three_tasks/R05').glob('*.md'));files=list(edition('R05').rglob('*'))+[APP/'results/R04/latest.json',out/'tests.log',report,*images,BASE/'maps/frozen/manifest.json']
 for p in paths.values():files+=list(p.rglob('*'))
 d=seal('R05',summary,files,sources,'R06');print(json.dumps(dict(status=d['automated_status'],next=d['next_step'])));return 0 if summary['passed'] else 2
