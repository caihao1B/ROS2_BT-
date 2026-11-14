"""Separate task acceptance; protected component receipts and full-job evidence differ."""
from common import *
from prepare_r10 import build
from runtime_spray_r10 import run as spray_run
from analyze_spray_r10 import analyze as spray_analyze
from runtime_r09 import run as mow_run
from analyze_r09 import analyze as mow_analyze
from analyze_r08 import analyze as inspect_analyze

def main(command,args):
 require('R09');out=APP/'results/R10';out.mkdir(parents=True,exist_ok=True)
 if command=='build':build();return 0
 if command=='test':
  case=args[0] if args else 'spray_full_A1'
  if case=='spray_full_A1':p=spray_run(cluster='A1',case='nominal',point_limit=0,seconds=1800);d=spray_analyze(p)
  elif case=='spray_tool_jam':p=spray_run(cluster='A1',case='tool_jam',point_limit=3,seconds=240);d=spray_analyze(p)
  elif case=='mowing_full_B1':p=mow_run(cluster='B1',case='nominal',segment_limit=0,seconds=1800);d=mow_analyze(p)
  else:raise ValueError('test spray_full_A1 | spray_tool_jam | mowing_full_B1; inspection and unchanged blocked cases use sealed R08/R09 evidence')
  dump(out/(case+'_latest.json'),dict(path=str(p.relative_to(ROOT)),passed=d['passed']));print(json.dumps(dict(case=case,passed=d['passed'],path=str(p),failed=[k for k,v in d['checks'].items() if not v])));return 0 if d['passed'] else 2
 if command!='finalize':raise ValueError('build / test / finalize / check / open')
 paths={};cases={};pending=[]
 for name in ['spray_full_A1','spray_tool_jam','mowing_full_B1']:
  f=out/(name+'_latest.json')
  if f.exists():
   p=ROOT/read(f)['path'];paths[name]=p;cases[name]=(mow_analyze if name.startswith('mowing') else spray_analyze)(p)
  else:pending.append(name+' not run')
 for name,stage,case,fn in [('inspection_full_connector','R08','nominal',inspect_analyze),('inspection_follow_stop_resume','R08','leader',inspect_analyze),('inspection_detour','R08','detour',inspect_analyze),('inspection_persistent_block','R08','blocked',inspect_analyze),('mowing_persistent_block','R09','blocked',mow_analyze)]:
  sealed=read(APP/'results'/stage/'latest.json');record=sealed['summary']['cases'][case];task=record['task_result'];# Locate the exact sealed artifact, never an arbitrary latest successful sample.
  analysis_name='analysis_r08.json' if stage=='R08' else 'analysis_r09.json'
  candidates=[(ROOT/x['path']).parent for x in sealed['artifacts'] if x['path'].endswith('/'+analysis_name)]
  p=next((p for p in candidates if read(p/'spec.json')['case']==case),None)
  if p is None:pending.append(name+' sealed evidence missing');continue
  paths[name]=p;cases[name]=fn(p)
 for name,d in cases.items():
  if not d['passed']:pending.append(name+' failed current evidence checks')
 d=cases.get('spray_full_A1',{}).get('task_result',{})
 if not(d.get('passed') and d.get('selected_points')==d.get('original_points') and d.get('selected_points',0)>=56):pending.append('Full original A1 not completed; partial point demo does not replace it')
 if not cases.get('inspection_full_connector',{}).get('full_connector_passed'):pending.append('Full original connector missing')
 d=cases.get('mowing_full_B1',{}).get('task_result',{})
 if not(d.get('passed') and d.get('selected_segments')==d.get('original_segments') and d.get('selected_segments',0)>5):pending.append('Full original B1 traversal not completed; partial swaths do not replace it')
 table=[]
 for name,d in cases.items():
  task=d['task_result'];table.append(dict(case=name,passed=d['passed'],simulation_seconds=task.get('t'),wall_seconds=task.get('wall_seconds'),terminal=task.get('terminal','COMPLETED' if task.get('passed') else 'INCOMPLETE'),reason=task.get('manual_reason') or task.get('error'),selected_points=task.get('selected_points'),original_points=task.get('original_points'),confirmed_points=len(task.get('completed',[])) if 'completed' in task else None,selected_segments=task.get('selected_segments'),original_segments=task.get('original_segments'),confirmed_segments=len(task.get('completed_segments',[])) if 'completed_segments' in task else None,confirmed_area_m2=d.get('selected_confirmed_union_m2'),residual_area_m2=d.get('selected_residual_m2'),remaining_original_route_m=task.get('remaining_original_m'),scope=d['scope'],path=str(paths[name].relative_to(ROOT))))
 summary=dict(passed=not pending,cases=table,pending=pending,original_obligations=read(edition('R10')/'catalog.json')['original_denominators'],formal_credit=False,original_S17='PARTIAL',original_S18='PARTIAL',scope='Three separately run task acceptance cases plus declared bounded faults; no sixty-run matrix, no A2/A3/B2/B3 or cutting-effectiveness claim')
 dump(out/'case_summary.json',summary);report=render('R10','R10 / separate task acceptance and residuals',summary,[]);files=list(edition('R10').rglob('*'))+[report,out/'case_summary.json',APP/'results/R09/latest.json',APP/'results/R08/latest.json']
 for p in paths.values():files+=list(p.rglob('*'))
 sources=list((PKG/'scripts').glob('*_r10.py'))+list((ROOT/'docs/three_tasks/R10').glob('*.md'));d=seal('R10',summary,files,sources,'R11');print(json.dumps(dict(status=d['automated_status'],pending=pending)));return 0 if not pending else 2
