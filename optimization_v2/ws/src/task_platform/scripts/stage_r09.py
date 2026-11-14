from common import *
from prepare_r09 import build
from runtime_r09 import run
from analyze_r09 import analyze

def main(command,args):
 require('R08');out=APP/'results/R09';out.mkdir(parents=True,exist_ok=True)
 if command=='build':build();return 0
 if command=='test':
  case=args[0] if args else 'nominal';limit=int(args[1]) if len(args)>1 else 5
  if case not in ['nominal','blocked'] or limit<0:raise ValueError('test nominal|blocked [segment_count; 0=all original B1 segments]')
  p=run(case=case,segment_limit=limit,seconds=1800 if limit==0 else 420);d=analyze(p);print(json.dumps(dict(passed=d['passed'],path=str(p),failed=[k for k,v in d['checks'].items() if not v])));return 0 if d['passed'] else 2
 if command!='finalize':raise ValueError('build / test / finalize / check / open')
 with (out/'tests.log').open('w') as f:r=subprocess.run([str(BASE/'venvs/scene/bin/python'),'-m','unittest','discover','-s',str(PKG/'tests'),'-p','*_r09.py','-v'],cwd=ROOT,stdout=f,stderr=subprocess.STDOUT,timeout=45)
 paths={case:ROOT/read(out/('mowing_'+case+'_latest.json'))['path'] for case in ['nominal','blocked'] if (out/('mowing_'+case+'_latest.json')).exists()};cases={k:analyze(v) for k,v in paths.items()};pending=[k+' has not passed' for k in ['nominal','blocked'] if not cases.get(k,{}).get('passed')]
 if r.returncode:pending.append('component checks failed')
 import matplotlib;matplotlib.use('Agg')
 import matplotlib.pyplot as plt
 import numpy as np
 fig,axes=plt.subplots(1,2,figsize=(14,6));ax=axes[0];cfg=read(edition('R09')/'mission.json')
 for s in cfg['clusters'][0]['segments']:
  p=np.asarray(s['poses_world']);ax.plot(p[:,0],p[:,1],color='.75',lw=.7)
 for case,path in paths.items():
  z=np.load(path/'trajectory.npz');ax.plot(z['xy'][:,0],z['xy'][:,1],label=case)
 ax.axis('equal');ax.grid(alpha=.3);ax.legend();ax.set(title='Dino actual motion / original B1 route',xlabel='Site X [m]',ylabel='Site Y [m]')
 if 'nominal' in paths:
  from shapely.geometry import shape
  from shapely.geometry.polygon import orient
  from matplotlib.path import Path as DrawingPath
  from matplotlib.patches import PathPatch
  colors={'selected_target':'#becbda','confirmed_feedback':'#2b9b70','residual':'#e7a646'}
  for feature in read(paths['nominal']/'coverage.geojson')['features']:
   geometry=shape(feature['geometry']);parts=list(geometry.geoms) if hasattr(geometry,'geoms') else [geometry];labeled=False
   for part in parts:
    if part.is_empty or part.geom_type!='Polygon':continue
    part=orient(part,sign=1.);vertices=[];codes=[]
    for ring in [part.exterior,*part.interiors]:
     xy=np.asarray(ring.coords)+[28,14];vertices.extend(xy.tolist());codes.extend([DrawingPath.MOVETO]+[DrawingPath.LINETO]*(len(xy)-2)+[DrawingPath.CLOSEPOLY])
    axes[1].add_patch(PathPatch(DrawingPath(vertices,codes),facecolor=colors[feature['properties']['kind']],edgecolor='none',alpha=.9,label=None if labeled else feature['properties']['kind']));labeled=True
  axes[1].autoscale_view()
 axes[1].axis('equal');axes[1].grid(alpha=.3);axes[1].legend();axes[1].set(title='Selected work / feedback credit / residual',xlabel='Site X [m]',ylabel='Site Y [m]');fig.tight_layout();figure=out/'mowing.png';fig.savefig(figure,dpi=150);plt.close(fig)
 summary=dict(passed=not pending,cases=cases,pending=pending,scope='Selected original swaths and headland segments, real feedback accounting, bounded blocked hold; no full B1 or formal S17 credit');report=render('R09','R09 / Ackermann mowing and feedback ledger',summary,[figure]);files=list(edition('R09').rglob('*'))+[APP/'results/R08/latest.json',out/'tests.log',figure,report]
 for path in paths.values():files+=list(path.rglob('*'))
 sources=list((PKG/'scripts').glob('*_r09.py'))+list((PKG/'tests').glob('*_r09.py'))+list((ROOT/'docs/three_tasks/R09').glob('*.md'));d=seal('R09',summary,files,sources,'R10');print(json.dumps(dict(status=d['automated_status'],pending=pending)));return 0 if not pending else 2
