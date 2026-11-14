from common import *
from prepare_r08 import build
from runtime_r08 import run
from analyze_r08 import analyze

def main(command,args):
 require('R07');out=APP/'results/R08';out.mkdir(parents=True,exist_ok=True)
 if command=='build':build();return 0
 if command=='test':
  case=args[0] if args else 'nominal';limit=float(args[1]) if len(args)>1 else (15. if case=='leader' else 0.)
  if case not in ['nominal','leader','blocked'] or limit<0:raise ValueError('test nominal|leader|blocked [metres; 0=original entire connector]')
  p=run(distance_limit=limit,case=case,seconds=420 if limit==0 else 240);r=analyze(p);print(json.dumps(dict(passed=r['passed'],path=str(p),failed=[k for k,v in r['checks'].items() if not v])));return 0 if r['passed'] else 2
 if command!='finalize':raise ValueError('build / test / finalize / check / open')
 paths={case:ROOT/read(out/('inspection_'+case+'_latest.json'))['path'] for case in ['nominal','leader','blocked'] if (out/('inspection_'+case+'_latest.json')).exists()};results={k:analyze(p) for k,p in paths.items()};pending=['Observed live detour validation and its real obstacle case remain unimplemented/unqualified.']
 if not results.get('nominal',{}).get('full_connector_passed'):pending.append('Original complete 88.48 m connector not yet passed.')
 if not results.get('leader',{}).get('passed'):pending.append('Observed leader follow/stop/resume not yet passed.')
 summary=dict(passed=False,cases=results,pending=pending,next=None,scope='R08 working checkpoint; no downstream admission until original behavior scope passes');report=render('R08','R08 / independent inspection working checkpoint',summary,[]);files=list(edition('R08').rglob('*'))+[APP/'results/R07/latest.json',out/'tests.log',report]
 for p in paths.values():files+=list(p.rglob('*'))
 sources=list((PKG/'scripts').glob('*_r08.py'))+list((ROOT/'docs/three_tasks/R08').glob('*.md'))+list((PKG/'tests').glob('*_r08.py'));d=seal('R08',summary,files,sources,'R09');print(json.dumps(dict(status=d['automated_status'],pending=pending)));return 2
