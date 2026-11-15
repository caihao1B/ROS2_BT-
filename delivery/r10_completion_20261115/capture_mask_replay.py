"""Read recorded rays only: measure the unchanged capture-frame mask effect."""
from pathlib import Path
import sys,json
import numpy as np
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'optimization_v2/ws/src/task_platform/scripts'))
from self_capture_mowing_r10 import keep_environment
run=ROOT/'optimization_v2/three_tasks/results/R12/mowing_20261115_023724_657473'
cfg=json.loads((run/'mission.json').read_text())
terrain=json.loads((ROOT/'optimization_v2/three_tasks/editions/R10/mowing/terrain.json').read_text())
records=[]
for filename in sorted((run/'diagnostics').glob('capture_*.npz')):
    data=np.load(filename);metadata=json.loads(filename.with_suffix('.json').read_text());units=[]
    for unit in cfg['near_sensor']['units']:
        raw=data[unit['name']];raw=raw[np.isfinite(raw).all(axis=1)]
        body=raw@Rotation.from_euler('xyz',[0,unit.get('pitch',0),unit['yaw']]).as_matrix().T+unit['xyz']
        keep=keep_environment(body,terrain)
        units.append(dict(unit=unit['name'],finite_points=len(body),removed_self_returns=int((~keep).sum()),remaining_points=int(keep.sum()),removed_body_xyz_min=body[~keep].min(axis=0).tolist() if (~keep).any() else None,removed_body_xyz_max=body[~keep].max(axis=0).tolist() if (~keep).any() else None))
    records.append(dict(capture=filename.stem,t=metadata['combined_t'],units=units))
report=dict(run=str(run.relative_to(ROOT)),captures=records,scope='Recorded raw rays filtered in their own sensor/body capture frames; no online control, no resimulation, no task PASS; main raw source not in these snapshots, so this does not reconstruct the complete combined cloud.')
target=Path(__file__).with_suffix('.json');target.write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(dict(captures=len(records),removed_by_unit={u['name']:sum(next(x['removed_self_returns'] for x in c['units'] if x['unit']==u['name']) for c in records) for u in cfg['near_sensor']['units']})))
