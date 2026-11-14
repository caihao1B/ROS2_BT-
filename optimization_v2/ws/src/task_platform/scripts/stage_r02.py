from common import *
from geometry_r02 import generate
from probe_r02 import run
from analyze_r02 import analyze,static_contracts,figures

def main(command,args):
 require('R01');out=APP/'results/R02';out.mkdir(parents=True,exist_ok=True)
 if command=='build':generate();return 0
 if command=='test':
  tasks=args or TASKS
  for task in tasks:
   if task not in TASKS:raise ValueError('UNKNOWN_TASK')
   analyze(run(task))
  return 0
 if command!='finalize':raise ValueError('USE build / test [task] / finalize / check / open')
 runs={t:analyze(ROOT/read(out/(t+'_latest.json'))['path']) for t in TASKS};static=static_contracts();dump(out/'static_contracts.json',static)
 with (out/'tests.log').open('w') as log:r=subprocess.run([sys.executable,str(PKG/'tests/test_r02.py')],stdout=log,stderr=subprocess.STDOUT,timeout=30)
 summary=dict(passed=all(x['passed'] for x in runs.values()) and all(x['passed'] for x in static.values()) and r.returncode==0,tests_exit=r.returncode,tests=6,vehicles={t:{k:v for k,v in x.items() if k!='runtime'} for t,x in runs.items()},static=static,scope='Primitive physical geometry, turning, forward/reverse and stop; navigation and work not accepted here',next='R03 task routes and headlands')
 images=figures(runs);report=render('R02','R02 / three physical chassis contracts',summary,images)
 sources=[p for p in PKG.rglob('*') if p.is_file() and '__pycache__' not in p.parts and ('_r02' in p.name or p.name in ['common.py','dispatch.py','VehicleDrive.cc','CMakeLists.txt','package.xml'])]+[APP/'advance.sh']+list((ROOT/'docs/three_tasks/R02').glob('*.md'))
 files=list(edition('R02').rglob('*'))+[out/'static_contracts.json',out/'tests.log',report,*images,APP/'build/platform/libr02_drive.so',APP/'results/R01/latest.json']
 files += [BASE/'assets/worlds/irregular_farm.sdf',ROOT/'ros2_ws/src/dino_description/models/dino/model.sdf',BASE/'ws/src/robot_description/scripts/generate.py',BASE/'ws/src/chassis_bridge/scripts/evaluator.py']+list((BASE/'maps/prior').glob('*.geojson'))
 for task in TASKS:
  p=ROOT/read(out/(task+'_latest.json'))['path'];files.extend(x for x in p.rglob('*') if x.is_file())
 d=seal('R02',summary,files,sources,'R03');print(json.dumps(dict(stage='R02',status=d['automated_status'],next=d['next_step'])));return 0 if summary['passed'] else 2
