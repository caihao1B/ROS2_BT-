"""Separate task acceptance; protected component receipts and full-job evidence differ."""
from common import *
from prepare_r10 import build,build_mowing,build_spray
from runtime_spray_r10 import run as spray_run
from analyze_spray_r10 import analyze as spray_analyze
from runtime_mowing_r10 import run as mow_run
from analyze_mowing_r10 import analyze as mow_analyze
from analyze_r09 import analyze as component_mow_analyze
from analyze_r08 import analyze as inspect_analyze

def main(command,args):
 require('R09');out=APP/'results/R10';out.mkdir(parents=True,exist_ok=True)
 if command=='build':build();return 0
 if command=='build-mowing':build_mowing();return 0
 if command=='build-spray':build_spray();return 0
 if command=='import-recording':
  names={'spray_tool_jam':('spray','tool_jam',3),'spray_full_A1':('spray_full_A1','nominal',0),'mowing_full_B1':('mowing_full_B1','nominal',0)};name=args[0];alias,expected_case,limit=names[name];record=read(APP/'results/R12'/(alias+'_latest.json'));p=ROOT/record['path'];spec=read(p/'spec.json');field='segment_limit' if name.startswith('mowing') else 'point_limit'
  if spec['case']!=expected_case or spec[field]!=limit:raise ValueError('RECORDED_CASE_SCOPE_MISMATCH')
  d=(mow_analyze if name.startswith('mowing') else spray_analyze)(p)
  dump(out/(name+'_latest.json'),dict(path=str(p.relative_to(ROOT)),passed=d['passed'],source='same-version real recorded case; reanalyzed, not inherited video credit'));print(json.dumps(dict(case=name,passed=d['passed'],path=str(p))));return 0 if d['passed'] else 2
 if command=='test':
  case=args[0] if args else 'spray_full_A1'
  if case=='global_filter_guards':
   env=os.environ.copy();env['ROS_DOMAIN_ID']='133';return subprocess.run(['/usr/bin/python3',str(PKG/'scripts/probe_global_r10.py')],cwd=ROOT,env=env).returncode
  if case=='ledger_clock':
   env=os.environ.copy();env['ROS_DOMAIN_ID']='134';return subprocess.run([str(BASE/'venvs/scene/bin/python'),str(PKG/'scripts/probe_ledger_clock_r10.py')],cwd=ROOT,env=env).returncode
  if case in ['record_spray_full_A1','record_mowing_full_B1','record_mowing_startup_perception_loss']:
   from stage_r12 import record
   case=case.removeprefix('record_');p,d,video=record(case)
   # Same original task and quantitative gates, with separately bound material/camera assets.
   dump(p/'r10_recorded_test.json',dict(case=case,scope=('original first five segments with real START_WORK perception fault' if case=='mowing_startup_perception_loss' else 'original full task')+'; actual R12 observer camera simultaneously recorded',video_acceptance=video))
  elif case=='spray_full_A1':p=spray_run(cluster='A1',case='nominal',point_limit=0,seconds=1800);d=spray_analyze(p)
  elif case=='spray_tool_jam':p=spray_run(cluster='A1',case='tool_jam',point_limit=3,seconds=240);d=spray_analyze(p)
  elif case=='mowing_startup_perception_loss':p=mow_run(cluster='B1',case='startup_perception_loss',segment_limit=5,seconds=420);d=mow_analyze(p)
  elif case=='mowing_headland':p=mow_run(cluster='B1',case='nominal',segment_limit=5,seconds=420);d=mow_analyze(p)
  elif case=='mowing_full_B1':p=mow_run(cluster='B1',case='nominal',segment_limit=0,seconds=1800);d=mow_analyze(p)
  else:raise ValueError('test spray_full_A1 | spray_tool_jam | mowing_full_B1; inspection and unchanged blocked cases use sealed R08/R09 evidence')
  dump(out/(case+'_latest.json'),dict(path=str(p.relative_to(ROOT)),passed=d['passed']));print(json.dumps(dict(case=case,passed=d['passed'],path=str(p),failed=[k for k,v in d['checks'].items() if not v])));return 0 if d['passed'] else 2
 if command!='finalize':raise ValueError('build / test / finalize / check / open')
 paths={};cases={};pending=[]
 with (out/'tests.log').open('w') as f:component=subprocess.run([str(BASE/'venvs/scene/bin/python'),'-m','unittest','discover','-s',str(PKG/'tests'),'-p','*_r10.py','-v'],stdout=f,stderr=subprocess.STDOUT,timeout=30)
 if component.returncode:pending.append('R10 grouping component checks failed')
 probe_path=out/'global_filter_probe/result.json';probe=read(probe_path) if probe_path.exists() else {}
 if not (probe.get('passed') and probe.get('binary_sha256')==sha(APP/'build/r10_global/r10_global_manager') and probe.get('policy_sha256')==sha(edition('R10')/'spray/manager.json')):pending.append('R10 current global correction native guard probe missing or failed')
 clock_path=out/'ledger_clock_probe/result.json';clock_probe=read(clock_path) if clock_path.exists() else {}
 if not (clock_probe.get('passed') and clock_probe.get('policy_sha256')==sha(edition('R10')/'mowing/tool.json') and all(c['source_sha256']==sha(PKG/'scripts'/('ledger_r09.py' if c['variant']=='original' else 'ledger_mowing_r10.py')) for c in clock_probe.get('cases',[]))):pending.append('R10 current ledger heartbeat clock probe missing or failed')
 for name in ['spray_full_A1','spray_tool_jam','mowing_full_B1','mowing_startup_perception_loss']:
  f=out/(name+'_latest.json')
  if f.exists():
   p=ROOT/read(f)['path'];paths[name]=p;cases[name]=(mow_analyze if name.startswith('mowing') else spray_analyze)(p)
  else:pending.append(name+' not run')
 for name,stage,case,fn in [('inspection_full_connector','R08','nominal',inspect_analyze),('inspection_follow_stop_resume','R08','leader',inspect_analyze),('inspection_detour','R08','detour',inspect_analyze),('inspection_persistent_block','R08','blocked',inspect_analyze),('mowing_persistent_block','R09','blocked',component_mow_analyze)]:
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
 for row in table:
  notes={name:read(paths[row['case']]/name) for name in ['repair_stop_note.json','timebox_reserve_stop.json'] if (paths[row['case']]/name).exists()}
  if notes:row['termination_context']=notes
  live_path=paths[row['case']]/'live.json'
  if live_path.exists():
   live=read(live_path);row['terminal_snapshot']=dict(bt_reason=(live.get('bt') or {}).get('reason_code'),localization_reason=(live.get('loc') or {}).get('reason'),gate_reasons=(live.get('gate') or {}).get('reasons'),scope='Last snapshot for context, not an independent root-cause conclusion')
  if row['case']=='mowing_full_B1':
   events=[json.loads(line) for line in (paths[row['case']]/'mission_events.jsonl').read_text().splitlines()];row['endpoint_recovery_successes']=[e for e in events if e['event']=='ENDPOINT_RECOVERY_SUCCESS'];row['observation_resume_count']=sum(e['event']=='ENDPOINT_OBSERVATION_RESUME' for e in events)
 summary=dict(passed=not pending,global_policy_probe=probe,ledger_clock_probe=clock_probe,cases=table,pending=pending,original_obligations=read(edition('R10')/'catalog.json')['original_denominators'],formal_credit=False,original_S17='PARTIAL',original_S18='PARTIAL',scope='Three separately run task acceptance cases plus declared bounded faults; no sixty-run matrix, no A2/A3/B2/B3 or cutting-effectiveness claim')
 dump(out/'case_summary.json',summary);images=[p for p in [out/'r10_overview.png',out/'repair_docking.png',out/'repair_start_work.png'] if p.exists()];report=render('R10','R10 / separate task acceptance and residuals',summary,images);files=list(edition('R10').rglob('*'))+[report,out/'case_summary.json',out/'tests.log',out/'config_reproducibility.json',out/'user_timebox.json',out/'r10_overview.png',out/'acceptance.html',APP/'results/R09/latest.json',APP/'results/R08/latest.json']
 for p in paths.values():files+=list(p.rglob('*'))
 files+=images+[out/'repair_figures.json',out/'time_breakdown.json',out/'global_hold_diagnosis.json']+list((out/'global_filter_probe').glob('*'))
 files+=list((out/'ledger_clock_probe').rglob('*'))
 files.append(out/'docking_diagnosis.json')
 files.append(out/'mowing_start_diagnosis.json')
 files.append(out/'repair_attempts.json')
 files.append(out/'endpoint_tracking_diagnosis.json')
 files.append(out/'startup_recovery_diagnosis.json')
 sources=list((PKG/'r10_backend').glob('*'))+[APP/'build/r10_global_source/global_manager.cpp',APP/'build/r10_global_source/CMakeLists.txt',APP/'build/r10_global/r10_global_manager']+list((PKG/'scripts').glob('*_r10.py'))+list((PKG/'tests').glob('*_r10.py'))+list((ROOT/'docs/three_tasks/R10').glob('*.md'));d=seal('R10',summary,[p for p in files if '__pycache__' not in p.parts],sources,'R11');print(json.dumps(dict(status=d['automated_status'],pending=pending)));return 0 if not pending else 2
