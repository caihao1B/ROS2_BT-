from common import *
from prepare_r08 import build
from runtime_r08 import run
from analyze_r08 import analyze
import time

def main(command,args):
 require('R07');out=APP/'results/R08';out.mkdir(parents=True,exist_ok=True)
 if command=='probe':
  p=run(case='probe',probe_arc=float(args[0]) if args else 37.7,seconds=60);d=read(p/'mission_result.json');d['runtime_clean']=read(p/'runtime.json')['passed'];print(json.dumps(dict(path=str(p),**d)));return 0 if d['passed'] and d['runtime_clean'] else 2
 if command=='build':
  from terrain_r08 import build_terrain,build_navigation
  build();build_terrain();build_navigation();return 0
 if command=='test':
  case=args[0] if args else 'nominal';limit=float(args[1]) if len(args)>1 else (15. if case in ['leader','detour'] else 20. if case=='blocked' else 0.)
  if case not in ['nominal','leader','detour','blocked'] or limit<0:raise ValueError('test nominal|leader|detour|blocked [metres; 0=original entire connector]')
  p=run(distance_limit=limit,case=case,seconds=600 if limit==0 else 240);r=analyze(p);print(json.dumps(dict(passed=r['passed'],path=str(p),failed=[k for k,v in r['checks'].items() if not v])));return 0 if r['passed'] else 2
 if command!='finalize':raise ValueError('build / test / finalize / check / open')
 with (out/'tests.log').open('w') as f:
  unit=subprocess.run([str(BASE/'venvs/scene/bin/python'),'-m','unittest','discover','-s',str(PKG/'tests'),'-p','*_r08.py','-v'],cwd=ROOT,stdout=f,stderr=subprocess.STDOUT,timeout=40)
 native=subprocess.run([str(APP/'build/r08/r08_policy_test'),str(edition('R06')/'inspection.json'),str(PKG/'r08_backend/common.xml')],capture_output=True,text=True,timeout=10);dump(out/'native_tests.json',dict(exit=native.returncode,output=native.stdout,stderr=native.stderr))
 paths={case:ROOT/read(out/('inspection_'+case+'_latest.json'))['path'] for case in ['nominal','leader','detour','blocked'] if (out/('inspection_'+case+'_latest.json')).exists()};results={k:analyze(p) for k,p in paths.items()};pending=[]
 if not results.get('nominal',{}).get('full_connector_passed'):pending.append('Original complete 88.48 m connector has not passed.')
 for case in ['leader','detour','blocked']:
  if not results.get(case,{}).get('passed'):pending.append(case+' has not passed under current bindings.')
 if unit.returncode or native.returncode:pending.append('Component or native BT checks failed.')
 import matplotlib;matplotlib.use('Agg')
 import matplotlib.pyplot as plt
 import numpy as np
 fig,axes=plt.subplots(1,2,figsize=(13,5));cfg=read(edition('R08')/'mission.json');route=np.asarray(cfg['route_world']);axes[0].plot(route[:,0],route[:,1],'k--',label='Original connector')
 for case,p in paths.items():
  z=np.load(p/'trajectory.npz');axes[0].plot(z['xy'][:,0],z['xy'][:,1],label=case);line=np.asarray([r['xyz'][:2] for r in __import__('analyze_r02').rows(p/'evaluation/truth.jsonl')]);dist=np.r_[0,np.cumsum(np.linalg.norm(np.diff(line,axis=0),axis=1))];axes[1].plot(z['t'],dist,label=case)
 axes[0].axis('equal');axes[0].set(xlabel='Site X [m]',ylabel='Site Y [m]',title='Actual trajectories / authorized original route');axes[1].set(xlabel='Simulation seconds',ylabel='Measured travel [m]',title='Actual travel; blocked hold is not completion')
 for a in axes:a.grid(alpha=.2);a.legend()
 fig.tight_layout();figure=out/'inspection.png';fig.savefig(figure,dpi=150);plt.close(fig)
 summary=dict(passed=not pending,cases=results,pending=pending,component_tests_passed=unit.returncode==0,native_bt_passed=native.returncode==0,scope='Original complete connector plus declared follow, observed detour and bounded blocked-hold cases; no formal S17 coverage credit');report=render('R08','R08 / observed inspection and bounded obstacle interaction',summary,[figure]);files=list(edition('R08').rglob('*'))+[APP/'results/R07/latest.json',out/'tests.log',out/'native_tests.json',APP/'build/r08/r08_shutdown_probe',report,figure]
 for p in paths.values():files+=list(p.rglob('*'))
 sources=list((PKG/'scripts').glob('*_r08.py'))+list((ROOT/'docs/three_tasks/R08').glob('*.md'))+list((PKG/'tests').glob('*_r08.py'))+list((PKG/'r08_backend').glob('*'))+list((PKG/'r08_nav').glob('*'));d=seal('R08',summary,files,sources,'R09');print(json.dumps(dict(status=d['automated_status'],pending=pending)));return 0 if summary['passed'] else 2
