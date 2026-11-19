"""Separate task acceptance; protected component receipts and full-job evidence differ."""
from common import *
from recheck_evidence_r10 import current_analysis
from prepare_r10 import build,build_mowing,build_spray
from runtime_spray_r10 import run as spray_run
from analyze_spray_r10 import analyze as spray_analyze
from runtime_mowing_r10 import run as mow_run
from analyze_mowing_r10 import analyze as mow_analyze
from analyze_r09 import analyze as component_mow_analyze
from analyze_r08 import analyze as inspect_analyze

def matches_case(name, spec):
    """Allow only the specified mission, workload and simulation budget into a stage row."""
    scopes = {
        'spray_full_A1': ('spray', 'A1', 'nominal', 0, (1800,)),
        'spray_tool_jam': ('spray', 'A1', 'tool_jam', 3, (240, 300)),
        'mowing_full_B1': ('mowing', 'B1', 'nominal', 0, (1800,)),
        'mowing_startup_perception_loss': ('mowing', 'B1', 'startup_perception_loss', 5, (420,)),
    }
    task, cluster, case, limit, budgets = scopes[name]
    limit_field = 'segment_limit' if task == 'mowing' else 'point_limit'
    return (not spec.get('diagnostic_only') and not spec.get('candidate_profile')
            and spec.get('task') == task and spec.get('cluster') == cluster
            and spec.get('case') == case and spec.get(limit_field) == limit
            and spec.get('segment_start', 0) == 0 and spec.get('sim_seconds') in budgets)


def experiment(args):
    """Run a scoped vehicle experiment without replacing an official case receipt."""
    entries = {
        'spray_precision': PKG/'scripts/qualify_spray_session_r10.py',
        'mowing_four_gear': PKG/'scripts/qualify_b1_context.py',
        'spray_time_order': APP/'spray_route_time.py',
        'vehicle_measurements': APP/'r10_checkpoint.py',
    }
    if not args or args[0] not in entries:
        raise ValueError('Choose an R10 experiment: '+', '.join(entries))
    return subprocess.run([str(BASE/'venvs/scene/bin/python'),str(entries[args[0]]),*args[1:]],cwd=ROOT).returncode


def main(command,args):
 if command=='experiment':return experiment(args)
 require('R09');out=APP/'results/R10';out.mkdir(parents=True,exist_ok=True)
 if command=='build':build();return 0
 if command=='build-mowing':build_mowing();return 0
 if command=='build-spray':build_spray();return 0
 if command=='import-recording':
  aliases={'spray_tool_jam':'spray','spray_full_A1':'spray_full_A1','mowing_full_B1':'mowing_full_B1'}
  name=args[0];record=read(APP/'results/R12'/(aliases[name]+'_latest.json'));p=ROOT/record['path']
  if not matches_case(name,read(p/'spec.json')):raise ValueError('RECORDED_CASE_SCOPE_MISMATCH')
  d=current_analysis(p,mow_analyze if name.startswith('mowing') else spray_analyze)
  dump(out/(name+'_latest.json'),dict(path=str(p.relative_to(ROOT)),passed=d['passed'],source='same-version real recorded case; reanalyzed, not inherited video credit'));print(json.dumps(dict(case=name,passed=d['passed'],path=str(p))));return 0 if d['passed'] else 2
 if command=='test':
  case=args[0] if args else 'spray_full_A1'
  if case=='stationary_gyro':
   probe=PKG/'scripts/probe_stationary_gyro_r10.py'
   return subprocess.run(['/usr/bin/python3',str(probe),'--output',str(ROOT/'validation/R10/stationary_yaw_bias_native.json')],cwd=ROOT).returncode
  if case=='precision_progress':
   from probe_precision_bt_r10 import main as probe_precision
   return probe_precision()
  if case=='global_filter_guards':
   env=os.environ.copy();env['ROS_DOMAIN_ID']='133';return subprocess.run(['/usr/bin/python3',str(PKG/'scripts/probe_spray_native_r10.py'),'global'],cwd=ROOT,env=env).returncode
  if case in ['mowing_global_filter_guards','mowing_stopped_revalidation']:
   name,domain=('probe_mowing_global_r10.py','138') if case=='mowing_global_filter_guards' else ('probe_mowing_revalidation_r10.py','139')
   env=os.environ.copy();env['ROS_DOMAIN_ID']=domain;return subprocess.run([str(BASE/'venvs/scene/bin/python'),str(PKG/'scripts'/name)],cwd=ROOT,env=env).returncode
  if case=='stopped_revalidation':
   env=os.environ.copy();env['ROS_DOMAIN_ID']='135';return subprocess.run([str(BASE/'venvs/scene/bin/python'),str(PKG/'scripts/probe_spray_native_r10.py'),'revalidation'],cwd=ROOT,env=env).returncode
  if case=='candidate_gnss':
   env=os.environ.copy();env['ROS_DOMAIN_ID']='140';return subprocess.run([str(BASE/'venvs/scene/bin/python'),str(PKG/'scripts/probe_spray_native_r10.py'),'candidate'],cwd=ROOT,env=env).returncode
  if case=='bridge_shutdown':
   env=os.environ.copy();env.update(ROS_DOMAIN_ID='137',GZ_PARTITION='r10_bridge_shutdown_probe');return subprocess.run(['/usr/bin/python3',str(PKG/'scripts/probe_bridge_r10.py')],cwd=ROOT,env=env).returncode
  if case=='ledger_clock':
   env=os.environ.copy();env['ROS_DOMAIN_ID']='134';return subprocess.run([str(BASE/'venvs/scene/bin/python'),str(PKG/'scripts/probe_ledger_clock_r10.py')],cwd=ROOT,env=env).returncode
  if case=='spray_ledger_clock':
   env=os.environ.copy();env['ROS_DOMAIN_ID']='136';return subprocess.run([str(BASE/'venvs/scene/bin/python'),str(PKG/'scripts/probe_spray_ledger_r10.py')],cwd=ROOT,env=env).returncode
  if case in ['record_spray_full_A1','record_mowing_full_B1','record_mowing_startup_perception_loss']:
   from stage_r12 import record
   case=case.removeprefix('record_');p,d,video=record(case)
   # Same original task and quantitative gates, with separately bound material/camera assets.
   dump(p/'r10_recorded_test.json',dict(case=case,scope=('first five approved execution segments with real START_WORK perception fault; original work polygons retained' if case=='mowing_startup_perception_loss' else 'full current task; mowing uses explicitly approved safe execution edition')+'; actual R12 observer camera simultaneously recorded',video_acceptance=video))
  elif case=='spray_full_A1':p=spray_run(cluster='A1',case='nominal',point_limit=0,seconds=1800);d=spray_analyze(p)
  elif case=='spray_tool_jam':p=spray_run(cluster='A1',case='tool_jam',point_limit=3,seconds=240);d=spray_analyze(p)
  elif case=='mowing_startup_perception_loss':p=mow_run(cluster='B1',case='startup_perception_loss',segment_limit=5,seconds=420);d=mow_analyze(p)
  elif case=='mowing_terrain_pilot':p=mow_run(cluster='B1',case='terrain_rejoin_pilot',segment_start=12,segment_limit=1,seconds=480);d=mow_analyze(p)
  elif case=='mowing_headland':p=mow_run(cluster='B1',case='nominal',segment_limit=5,seconds=420);d=mow_analyze(p)
  elif case=='mowing_endpoint_pilot':p=mow_run(cluster='B1',case='nominal',segment_limit=7,seconds=600);d=mow_analyze(p)
  elif case=='mowing_full_B1':p=mow_run(cluster='B1',case='nominal',segment_limit=0,seconds=1800);d=mow_analyze(p)
  else:raise ValueError('test spray_full_A1 | spray_tool_jam | mowing_full_B1; inspection and unchanged blocked cases use sealed R08/R09 evidence')
  dump(out/(case+'_latest.json'),dict(path=str(p.relative_to(ROOT)),passed=d['passed']));print(json.dumps(dict(case=case,passed=d['passed'],path=str(p),failed=[k for k,v in d['checks'].items() if not v])));return 0 if d['passed'] else 2
 if command!='finalize':raise ValueError('build / test / finalize / check / open')
 paths={};cases={};pending=[]
 with (out/'tests.log').open('w') as f:component=subprocess.run([str(BASE/'venvs/scene/bin/python'),'-m','unittest','discover','-s',str(PKG/'tests'),'-p','*_r10.py','-v'],stdout=f,stderr=subprocess.STDOUT,timeout=30)
 if component.returncode:pending.append('R10 grouping component checks failed')
 gyro_path=ROOT/'validation/R10/stationary_yaw_bias_native.json'
 gyro=read(gyro_path) if gyro_path.is_file() else {}
 bindings=gyro.get('bindings',{})
 required_gyro={str((PKG/'scripts'/filename).relative_to(ROOT)) for filename in
               ('stationary_gyro_r10.py','spray_inputs_r10.py','probe_stationary_gyro_r10.py')}
 if not (gyro.get('passed') and required_gyro<=bindings.keys() and
         all((ROOT/path).is_file() and sha(ROOT/path)==digest for path,digest in bindings.items())):
  pending.append('Current spray stationary gyro calibration probe missing or failed')
 precision_path=out/'precision_progress_probe/result.json';precision_probe=read(precision_path) if precision_path.exists() else {}
 if not (precision_probe.get('passed') and precision_probe.get('binary_sha256')==sha(APP/'build/r10_spray_bt/r10_spray_supervisor') and precision_probe.get('policy_sha256')==sha(edition('R10')/'spray/mission.json') and precision_probe.get('header_sha256')==sha(PKG/'r10_backend/precision_progress.hpp') and precision_probe.get('driver_sha256')==sha(PKG/'scripts/probe_precision_bt_r10.py') and precision_probe.get('generator_sha256')==sha(PKG/'scripts/precision_supervisor_r10.py')):pending.append('Current spray precision-progress BT probe missing or failed')
 probe_path=out/'global_filter_probe/result.json';probe=read(probe_path) if probe_path.exists() else {}
 if not (probe.get('passed') and probe.get('probe_sha256')==sha(PKG/'scripts/probe_spray_native_r10.py') and probe.get('template_sha256')==sha(PKG/'scripts/probe_global_r10.py') and probe.get('binary_sha256')==sha(APP/'build/r10_spray_global/r10_global_manager') and probe.get('policy_sha256')==sha(edition('R10')/'spray/manager.json')):pending.append('R10 current global correction native guard probe missing or failed')
 recovery_path=out/'revalidation_probe/result.json';recovery_probe=read(recovery_path) if recovery_path.exists() else {}
 if not (recovery_probe.get('passed') and recovery_probe.get('binary_sha256')==sha(APP/'build/r10_spray_global/r10_global_manager') and recovery_probe.get('policy_sha256')==sha(edition('R10')/'spray/manager.json') and recovery_probe.get('source_sha256')==sha(PKG/'scripts/probe_spray_native_r10.py') and recovery_probe.get('template_sha256')==sha(PKG/'scripts/probe_revalidation_r10.py')):pending.append('R10 production stopped revalidation probe missing or failed')
 candidate_path=out/'candidate_gnss_probe/result.json';candidate_probe=read(candidate_path) if candidate_path.exists() else {}
 if not (candidate_probe.get('passed') and candidate_probe.get('binary_sha256')==sha(APP/'build/r10_spray_global/r10_global_manager') and candidate_probe.get('policy_sha256')==sha(edition('R10')/'spray/manager.json') and candidate_probe.get('source_sha256')==sha(PKG/'scripts/probe_spray_native_r10.py') and candidate_probe.get('template_sha256')==sha(PKG/'scripts/probe_revalidation_r10.py')):pending.append('Independent candidate GNSS native recovery probe missing or failed')
 mowing_probes={}
 for folder,driver,template,key in [('mowing_global_filter_probe','probe_mowing_global_r10.py','probe_global_r10.py','probe_sha256'),('mowing_revalidation_probe','probe_mowing_revalidation_r10.py','probe_revalidation_r10.py','source_sha256')]:
  path=out/folder/'result.json';d=read(path) if path.exists() else {};mowing_probes[folder]=d
  if folder=='mowing_revalidation_probe' and d.get('worker_sha256')!=sha(PKG/'scripts/mission_mowing_r10.py'):
   pending.append('Mowing revalidation probe must exercise the current mission heartbeat')
  if not (d.get('passed') and d.get('binary_sha256')==sha(APP/'build/r10_global/r10_global_manager') and d.get('policy_sha256')==sha(edition('R10')/'mowing/manager.json') and d.get(key)==sha(PKG/'scripts'/driver) and d.get('template_sha256')==sha(PKG/'scripts'/template)):pending.append('Current '+folder+' missing or failed')
 bridge_path=out/'bridge_shutdown_probe/result.json';bridge_probe=read(bridge_path) if bridge_path.exists() else {}
 if not (bridge_probe.get('passed') and bridge_probe.get('binary_sha256')==sha(APP/'build/r10_bridge/r10_tool_bridge') and bridge_probe.get('source_sha256')==sha(PKG/'r10_backend/tool_bridge.cpp') and bridge_probe.get('probe_sha256')==sha(PKG/'scripts/probe_bridge_r10.py')):pending.append('R10 current bridge lifecycle traffic probe missing or failed')
 clock_path=out/'ledger_clock_probe/result.json';clock_probe=read(clock_path) if clock_path.exists() else {}
 if not (clock_probe.get('passed') and clock_probe.get('policy_sha256')==sha(edition('R10')/'mowing/tool.json') and all(c['source_sha256']==sha(PKG/'scripts'/('ledger_r09.py' if c['variant']=='original' else 'ledger_mowing_r10.py')) for c in clock_probe.get('cases',[]))):pending.append('R10 current ledger heartbeat clock probe missing or failed')
 spray_clock_path=out/'spray_ledger_clock_probe/result.json';spray_clock_probe=read(spray_clock_path) if spray_clock_path.exists() else {}
 if not (spray_clock_probe.get('passed') and spray_clock_probe.get('policy_sha256')==sha(edition('R10')/'spray/tool.json') and all(c['source_sha256']==sha(PKG/'scripts'/('ledger_r07.py' if c['variant']=='original' else 'ledger_spray_r10.py')) for c in spray_clock_probe.get('cases',[]))):pending.append('R10 current spray ledger heartbeat clock probe missing or failed')
 for name in ['spray_full_A1','spray_tool_jam','mowing_full_B1','mowing_startup_perception_loss']:
  f=out/(name+'_latest.json')
  if f.exists():
   p=ROOT/read(f)['path']
   if not matches_case(name,read(p/'spec.json')):
    pending.append(name+' run does not match its declared task, cluster, case, range or budget')
    continue
   paths[name]=p;cases[name]=current_analysis(p,mow_analyze if name.startswith('mowing') else spray_analyze)
  else:pending.append(name+' not run')
 for name,stage,case,fn in [('inspection_full_connector','R08','nominal',inspect_analyze),('inspection_follow_stop_resume','R08','leader',inspect_analyze),('inspection_detour','R08','detour',inspect_analyze),('inspection_persistent_block','R08','blocked',inspect_analyze),('mowing_persistent_block','R09','blocked',component_mow_analyze)]:
  sealed=read(APP/'results'/stage/'latest.json');record=sealed['summary']['cases'][case];task=record['task_result'];# Locate the exact sealed artifact, never an arbitrary latest successful sample.
  analysis_name='analysis_r08.json' if stage=='R08' else 'analysis_r09.json'
  candidates=[(ROOT/x['path']).parent for x in sealed['artifacts'] if x['path'].endswith('/'+analysis_name)]
  p=next((p for p in candidates if read(p/'spec.json')['case']==case),None)
  if p is None:pending.append(name+' sealed evidence missing');continue
  paths[name]=p;cases[name]=current_analysis(p,fn)
 for name,d in cases.items():
  if not d['passed']:pending.append(name+' failed current evidence checks')
 d=cases.get('spray_full_A1',{}).get('task_result',{})
 if not(d.get('passed') and d.get('selected_points')==d.get('original_points') and d.get('selected_points',0)>=56):pending.append('Full original A1 not completed; partial point demo does not replace it')
 if not cases.get('inspection_full_connector',{}).get('full_connector_passed'):pending.append('Full original connector missing')
 d=cases.get('mowing_full_B1',{}).get('task_result',{})
 if not(d.get('passed') and d.get('selected_segments')==d.get('original_segments') and d.get('selected_segments',0)>5):pending.append('Full approved B1 execution not completed; partial swaths do not replace it')
 from safe_plan_r10 import load_checked,validate
 try:
  safe=load_checked();safe_validation=validate(safe,read(edition('R09')/'mission.json'))
  if d.get('safe_plan_revision')!=safe['revision'] or d.get('execution_segments')!=len(safe['cluster']['segments']):pending.append('Current approved B1 safe execution revision not fully retested')
 except (ValueError,OSError,KeyError) as ex:safe_validation=dict(passed=False,error=repr(ex));pending.append('Safe execution path or original area preservation failed')
 dump(out/'safe_plan_validation.json',safe_validation)
 table=[]
 for name,d in cases.items():
  task=d['task_result'];table.append(dict(case=name,passed=d['passed'],simulation_seconds=task.get('t'),wall_seconds=task.get('wall_seconds'),terminal=task.get('terminal','COMPLETED' if task.get('passed') else 'INCOMPLETE'),reason=task.get('manual_reason') or task.get('error'),selected_points=task.get('selected_points'),original_points=task.get('original_points'),confirmed_points=len(task.get('completed',[])) if 'completed' in task else None,selected_segments=task.get('selected_segments'),original_segments=task.get('original_segments'),execution_segments=task.get('execution_segments'),reference_original_segments=task.get('reference_original_segments'),original_reference_status=task.get('original_reference_status'),safe_plan_revision=task.get('safe_plan_revision'),confirmed_segments=len(task.get('completed_segments',[])) if 'completed_segments' in task else None,confirmed_area_m2=d.get('selected_confirmed_union_m2'),residual_area_m2=d.get('selected_residual_m2'),remaining_original_route_m=task.get('remaining_original_m'),scope=d['scope'],path=str(paths[name].relative_to(ROOT))))
 for row in table:
  notes={name:read(paths[row['case']]/name) for name in ['repair_stop_note.json','timebox_reserve_stop.json'] if (paths[row['case']]/name).exists()}
  if notes:row['termination_context']=notes
  live_path=paths[row['case']]/'live.json'
  if live_path.exists():
   live=read(live_path);row['terminal_snapshot']=dict(bt_reason=(live.get('bt') or {}).get('reason_code'),localization_reason=(live.get('loc') or {}).get('reason'),gate_reasons=(live.get('gate') or {}).get('reasons'),scope='Last snapshot for context, not an independent root-cause conclusion')
  if row['case']=='mowing_full_B1':
   events=[json.loads(line) for line in (paths[row['case']]/'mission_events.jsonl').read_text().splitlines()];row['endpoint_recovery_successes']=[e for e in events if e['event']=='ENDPOINT_RECOVERY_SUCCESS'];row['observation_resume_count']=sum(e['event']=='ENDPOINT_OBSERVATION_RESUME' for e in events)
 summary=dict(passed=not pending,global_policy_probe=probe,stopped_revalidation_probe=recovery_probe,ledger_clock_probe=clock_probe,cases=table,pending=pending,original_obligations=read(edition('R10')/'catalog.json')['original_denominators'],formal_credit=False,original_S17='PARTIAL',original_S18='PARTIAL',scope='Three separately run task acceptance cases plus declared bounded faults; no sixty-run matrix, no A2/A3/B2/B3 or cutting-effectiveness claim')
 summary['stationary_gyro_probe']=gyro
 summary['precision_progress_probe']=precision_probe
 summary['bridge_shutdown_probe']=bridge_probe
 summary['spray_ledger_clock_probe']=spray_clock_probe
 summary['mowing_native_probes']=mowing_probes
 summary['safe_execution_validation']=safe_validation
 summary['independent_candidate_gnss_probe']=candidate_probe
 summary['original_B1_reference_status']='PARTIAL'
 # Keep the measured four-stop experiments visible beside the original stage rows. They
 # have no weight in the nine-case score, and retain the exact physical-feedback reports.
 experiment_dir=ROOT/'validation/r10_continuation_20261119'
 comparison_files=[]
 if (experiment_dir/'checkpoint.json').is_file():
  comparison_files=[experiment_dir/'a1/comparison.json',experiment_dir/'b1/physical_connector_report.json',experiment_dir/'checkpoint.json']
  if not all(f.is_file() for f in comparison_files):raise ValueError('Incomplete four-stop experiment reports')
  spray_evidence,mow_evidence,checkpoint=(read(f) for f in comparison_files)
  if checkpoint.get('full_cluster_credit') is not False:raise ValueError('Subsets cannot replace full task evidence')
  summary['measured_vehicle_experiments']=dict(
      formal_credit=False,source_reports=[dict(path=str(f.relative_to(ROOT)),sha256=sha(f)) for f in comparison_files],
      spray=[dict(session=r['session'],selected=r['selected_count'],completed=r['completed_count'],
                  passed=r['measured_subset_pass'],elapsed_sim_s=r['elapsed_sim_s']) for r in spray_evidence['runs']],
      four_gear=dict(session=mow_evidence['session'],verified=mow_evidence['verified_parking_stops'],
                     selected=mow_evidence['selected_sections'],passed=mow_evidence['passed']))
  full_selection=experiment_dir/'a1/full_selection.json'
  if full_selection.is_file():
   full=read(full_selection)
   if full['selected_count']!=full['total_original'] or full['total_original']!=56 or full['formal_credit'] is not False:
    raise ValueError('Expected the supplementary original 56-point selection')
   comparison_files.append(full_selection)
   summary['measured_vehicle_experiments']['full_spray_selection']=dict(
       session=full['session'],selected=full['selected_count'],completed=full['completed_count'],
       passed=full['measured_subset_pass'],sim_s=full['elapsed_sim_s'],error=full['error'],
       path=str(full_selection.relative_to(ROOT)),sha256=sha(full_selection))
 from overview_r10 import build as build_overview
 dump(out/'case_summary.json',summary)
 build_overview()
 images=[p for p in [out/'r10_overview.png',out/'repair_docking.png',out/'repair_start_work.png'] if p.exists()];report=render('R10','R10 / separate task acceptance and residuals',summary,images);files=list(edition('R10').rglob('*'))+[report,out/'case_summary.json',out/'tests.log',out/'config_reproducibility.json',out/'user_timebox.json',out/'r10_overview.png',out/'acceptance.html',APP/'results/R09/latest.json',APP/'results/R08/latest.json']
 for p in paths.values():files+=list(p.rglob('*'))
 files+=images+[out/'repair_figures.json',out/'time_breakdown.json',out/'global_hold_diagnosis.json']+list((out/'global_filter_probe').glob('*'))
 files+=list((out/'ledger_clock_probe').rglob('*'))
 files+=list((out/'spray_ledger_clock_probe').rglob('*'))
 files+=list((out/'revalidation_probe').rglob('*'))
 for folder in mowing_probes:files+=list((out/folder).rglob('*'))
 files+=list((out/'bridge_shutdown_probe').rglob('*'))+[APP/'build/r10_bridge/r10_tool_bridge',APP/'build/r10_bridge_source/CMakeLists.txt']
 files+=list((out/'candidate_gnss_probe').glob('*'))+[APP/'build/r10_spray_global/r10_global_manager',APP/'build/r10_spray_global_source/global_manager.cpp',APP/'build/r10_spray_global_source/stopped_revalidation.inc']
 files+=list((out/'precision_progress_probe').glob('*'))+[APP/'build/r10_spray_bt/r10_spray_supervisor',APP/'build/r10_spray_bt_source/policy.hpp',APP/'build/r10_spray_bt_source/supervisor.cpp']
 files.extend([out/'safe_plan_validation.json',gyro_path])
 files.extend(comparison_files)
 files.append(out/'docking_diagnosis.json')
 files.append(out/'mowing_start_diagnosis.json')
 files.append(out/'repair_attempts.json')
 files.append(out/'endpoint_tracking_diagnosis.json')
 files.append(out/'startup_recovery_diagnosis.json')
 sources=list((PKG/'r10_backend').glob('*'))+[APP/'build/r10_global_source/global_manager.cpp',APP/'build/r10_global_source/CMakeLists.txt',APP/'build/r10_global/r10_global_manager']+list((PKG/'scripts').glob('*_r10.py'))+list((PKG/'tests').glob('*_r10.py'))+list((ROOT/'docs/three_tasks/R10').glob('*.md'));d=seal('R10',summary,[p for p in files if '__pycache__' not in p.parts],sources,'R11');print(json.dumps(dict(status=d['automated_status'],pending=pending)));return 0 if not pending else 2
