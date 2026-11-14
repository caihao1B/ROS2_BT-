"""Independent material work can be inspected while earlier full-job acceptance is partial."""
from common import *
from visual_r11 import assets,apply,signature
from xml.etree import ElementTree as E

def main(command,args):
 require('R09');out=APP/'results/R11';out.mkdir(parents=True,exist_ok=True)
 if command=='build':
  assets()
  for task in TASKS:
   source=edition('R11')/task/'original';source.mkdir(parents=True,exist_ok=True)
   if task=='spray':
    from scene_spray_r10 import compose
   elif task=='mowing':
    from scene_mowing_r10 import compose
   else:
    from scene_r08 import compose
   compose(source,task,'r11_'+task,rtf=.4)
   apply(source/'world.sdf',source.parent/'world.sdf',task)
  return 0
 if command=='test':
  from preview_r11 import run
  for task in (args or list(TASKS)):
   if task not in TASKS:raise ValueError('UNKNOWN_TASK')
   run(task)
  return 0
 if command!='finalize':raise ValueError('build / test [task] / finalize / check / open')
 cases={};pending=[]
 with (out/'tests.log').open('w') as f:tests=subprocess.run([str(BASE/'venvs/scene/bin/python'),'-m','unittest','discover','-s',str(PKG/'tests'),'-p','*_r11.py','-v'],stdout=f,stderr=subprocess.STDOUT,timeout=20)
 if tests.returncode:pending.append('material contract component tests failed')
 for task in TASKS:
  path=out/task/'comparison.json'
  cases[task]=read(path) if path.exists() else dict(passed=False,reason='NOT_RUN')
  if cases[task].get('passed'):
   c=cases[task]['material_contract'];base=edition('R11')/task
   current=dict(source=sha(base/'original/world.sdf')==c['source_sha256'],destination=sha(base/'world.sdf')==c['destination_sha256'],assets=all((ROOT/p).is_file() and sha(ROOT/p)==h for p,h in c['assets'].items()),paired_scene_bindings=all(cases[task]['variants'][v]['scene_sha256']==c[k] for v,k in [('original','source_sha256'),('material','destination_sha256')]),geometry=signature(E.parse(base/'original/world.sdf').getroot())==signature(E.parse(base/'world.sdf').getroot())==c['geometry_sensor_physics_signature'])
   cases[task]=dict(cases[task],current_evidence_checks=current,passed=all(current.values()))
  if not cases[task]['passed']:pending.append(task+' material/render benchmark incomplete')
 predecessor=check('R10')
 if not predecessor['passed']:pending.append('R10 full-job acceptance remains '+predecessor.get('status','NOT_RUN'))
 summary=dict(passed=not pending,components_passed=all(x['passed'] for x in cases.values()),cases=cases,pending=pending,scope='Material-only assets and paired parked rendering. Autonomous sensor qualification belongs to R12 recordings; no full-job extrapolation.')
 report=render('R11','R11 / 三任务材质与性能对照',summary,[out/t/'material/preview.png' for t in TASKS if (out/t/'material/preview.png').exists()]);files=list(edition('R11').rglob('*'))+list(out.rglob('*'));sources=list((PKG/'scripts').glob('*_r11.py'))+list((PKG/'tests').glob('*_r11.py'))+list((ROOT/'docs/three_tasks/R11').glob('*.md'));seal('R11',summary,[p for p in files if p.name!='latest.json' and '__pycache__' not in p.parts],sources,'R12');print(json.dumps(summary,ensure_ascii=False));return 0 if summary['passed'] else 2
