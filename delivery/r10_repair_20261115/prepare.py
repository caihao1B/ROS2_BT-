"""Publish source/configuration and compact evidence, not local runtime datasets."""
from pathlib import Path
import json,hashlib,base64
ROOT=Path(__file__).resolve().parents[2];OUT=Path(__file__).resolve().parent;APP=ROOT/'optimization_v2/three_tasks';PKG=ROOT/'optimization_v2/ws/src/task_platform';paths={}
def add(p,dest=None):
 if p.is_file() and '__pycache__' not in p.parts and p.suffix not in ['.pyc','.pyo']:paths[dest or str(p.relative_to(ROOT))]=p
for p in PKG.rglob('*'):
 if p.is_file() and '__pycache__' not in p.parts and (any('_'+s.lower() in p.name for s in ['R08','R09','R10','R11','R12']) or any(x in p.parts for x in ['r08_nav','r08_backend','r09_backend','r10_backend'])):add(p)
statuses={}
for stage in ['R08','R09','R10','R11','R12']:
 receipt=APP/'results'/stage/'latest.json';r=json.loads(receipt.read_text()) if receipt.exists() else {};statuses[stage]=r.get('automated_status','NOT_RUN')
 for p in (ROOT/'docs/three_tasks'/stage).glob('*.md'):add(p)
 for p in (APP/'editions'/stage).rglob('*'):add(p)
 for name in ['latest.json','tests.log','native_tests.json','case_summary.json','user_timebox.json','config_reproducibility.json','docking_diagnosis.json','mowing_start_diagnosis.json','time_breakdown.json','repair_figures.json','global_hold_diagnosis.json','repair_attempts.json','endpoint_tracking_diagnosis.json','startup_recovery_diagnosis.json']:
  add(APP/'results'/stage/name,'validation/'+stage+'/'+name)
 for p in (APP/'results'/stage).glob('*.png'):add(p,'media/'+stage.lower()+'_'+p.name)
 cases={pointer.stem.removesuffix('_latest'):ROOT/json.loads(pointer.read_text())['path'] for pointer in (APP/'results'/stage).glob('*_latest.json')}
 if stage in ['R08','R09'] and r.get('automated_status')=='PASS':
  cases={}
  for artifact in r.get('artifacts',[]):
   if artifact['path'].endswith('/analysis_'+stage.lower()+'.json'):
    casepath=(ROOT/artifact['path']).parent;spec=json.loads((casepath/'spec.json').read_text());cases[spec['task']+'_'+spec['case']]=casepath
 for case,p in cases.items():
  for name in ['analysis_'+stage.lower()+'.json','analysis_r09.json','analysis_r10.json','analysis_spray_r10.json','analysis_mowing_r10.json','recording_acceptance.json','r10_recorded_test.json','repair_stop_note.json','timebox_reserve_stop.json','runtime.json','ros_shutdown.json','nav_shutdown.json','performance_summary.json','barrier_injection.json','fault_injection.json','fault_physical_check.json','startup_fault.json','mission_result.json','mission_events.jsonl','spec.json']:
   add(p/name,'validation/'+stage+'/'+case+'/'+name)
add(APP/'results/R10/global_filter_probe/result.json','validation/R10/global_filter_probe.json')
add(APP/'results/R10/ledger_clock_probe/result.json','validation/R10/ledger_clock_probe.json')
# Failed attempts remain inspectable after latest pointers advance.
for trial in json.loads((APP/'results/R10/repair_attempts.json').read_text()):
 run=ROOT/trial['path']
 for name in ['recording_acceptance.json','mission_result.json','runtime.json','startup_fault.json','spec.json']:
  add(run/name,'validation/R10/repair_attempts/'+run.name+'/'+name)
add(ROOT/'docs/three_tasks/R10修复与完整任务复验_20261115.md')
for p in (OUT/'intermediate_sources').rglob('*'):add(p)
add(APP/'results/R09/tool_timing/result.json','validation/R09/tool_timing.json')
for task in ['spray','inspection','mowing']:
 add(APP/'results/R11'/task/'comparison.json','validation/R11/'+task+'/comparison.json')
 for variant in ['original','material']:
  add(APP/'results/R11'/task/variant/'preview.png','media/r11_'+task+'_'+variant+'.png')
  add(APP/'results/R11'/task/variant/'result.json','validation/R11/'+task+'/'+variant+'.json')
media=APP/'results/R12/media';add(media/'manifest.json','validation/R12/video_manifest.json');add(media/'three_tasks_short.mp4','media/r12_three_tasks_short.mp4');add(media/'behavior_highlights.mp4','media/r12_behavior_highlights.mp4')
if (media/'manifest.json').exists():
 for e in json.loads((media/'manifest.json').read_text())['episodes']:
  run=ROOT/e['run'];add(run/'video/preview.png','media/r12_'+e['case']+'.png');add(run/'video/recording.json','validation/R12/'+e['case']+'/recording.json');video=ROOT/e['full_path']
  if video.stat().st_size<24_000_000:add(video,'media/r12_'+e['case']+'_full.mp4')
for name in ['r10_overview.png','acceptance.html']:add(APP/'results/R10'/name,('media/' if name.endswith('.png') else 'validation/R10/')+name)
for name in ['prepare.py','readme.py','report.py','overview.py','qa_videos.py']:add(OUT/name)
add(OUT/'qa/video_checks.json','validation/R12/repair_video_checks.json')
add(OUT/'preservation_check.json','validation/R12/preservation_check.json')
add(ROOT/'docs/three_tasks/本轮推进结果_R08-R12_20261114.md')
add(OUT/'README.md','README.md');add(ROOT/'docs/three_tasks/本轮推进结果_R08-R10_20261114.md')
manifest=[];payload=[];binary_payload=[]
for dest,p in sorted(paths.items()):
 data=p.read_bytes();binary=p.suffix.lower() in ['.png','.pgm','.jpg','.npz','.mp4'] or len(data)>60000;manifest.append(dict(source=str(p),path=dest,bytes=len(data),binary=binary,sha256=hashlib.sha256(data).hexdigest(),git_blob=hashlib.sha1(b'blob '+str(len(data)).encode()+b'\0'+data).hexdigest()))
 if binary:binary_payload.append(dict(path=dest,content=base64.b64encode(data).decode()))
 else:payload.append(dict(path=dest,mode='100644',type='blob',content=data.decode()))
(OUT/'manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2));(OUT/'text_payload.json').write_text(json.dumps(payload,ensure_ascii=False));(OUT/'binary_payload.json').write_text(json.dumps(binary_payload));print(json.dumps(dict(files=len(manifest),statuses=statuses,total_bytes=sum(x['bytes'] for x in manifest))))
chunks=[];chunk=[];size=0
for entry in payload:
 n=len(json.dumps(entry,ensure_ascii=False))
 if size+n>160000 and chunk:chunks.append(chunk);chunk=[];size=0
 chunk.append(entry);size+=n
if chunk:chunks.append(chunk)
for i,chunk in enumerate(chunks):(OUT/f'chunk_{i}.json').write_text(json.dumps(chunk,ensure_ascii=False))
(OUT/'chunks.json').write_text(json.dumps(dict(count=len(chunks))))
