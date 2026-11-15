"""Retain actual trial outcomes while latest pointers advance. No regrading."""
from pathlib import Path
import json
ROOT=Path(__file__).resolve().parents[2];APP=ROOT/'optimization_v2/three_tasks'
target=APP/'results/R10/repair_attempts.json'
old=json.loads(target.read_text()) if target.exists() else []
records={r['path']:r for r in old}
for p in sorted([p for stage in ['R10','R12'] for p in (APP/'results'/stage).glob('*_20*')]):
    if p.name.split('_',1)[1]<'20261114_215952' or not (p/'runtime.json').exists():continue
    if not (p/'mission_result.json').exists():continue
    spec=json.loads((p/'spec.json').read_text());mission=json.loads((p/'mission_result.json').read_text())
    receipt=json.loads((p/'recording_acceptance.json').read_text()) if (p/'recording_acceptance.json').exists() else {}
    path=str(p.relative_to(ROOT))
    analysis_path=p/('analysis_mowing_r10.json' if spec['task']=='mowing' else 'analysis_spray_r10.json')
    analysis=json.loads(analysis_path.read_text()) if analysis_path.exists() else {}
    row=dict(path=path,role=spec['task']+' '+spec['case']+' actual repair trial',
             task_passed_at_execution=receipt.get('task_passed',analysis.get('passed',False)),
             recording_passed_at_execution=receipt.get('passed',False),
             source_sha256=spec.get('online_probe_sha256'),terminal=mission.get('terminal'),
             reason=mission.get('error'),simulation_t=mission.get('t'),
             scope='Historical result for exact recorded bindings; no approval inherited by changed sources',
             selected_offset=spec.get('segment_start',0),recording_present=bool(receipt))
    if 'completed' in mission:row['completed_points']=len(mission['completed'])
    if 'completed_segments' in mission:row['completed_segments']=len(mission['completed_segments'])
    for name in ['repair_stop_note.json']:
        if (p/name).exists():row['termination_context']=json.loads((p/name).read_text())
    records[path]=row
target.write_text(json.dumps(list(records.values()),ensure_ascii=False,indent=2)+'\n')
print('Preserved actual trial outcomes:',len(records))
