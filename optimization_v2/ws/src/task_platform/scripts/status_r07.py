"""Read-only progress view; does not join ROS or start a simulation."""
from pathlib import Path
import json
from common import APP
runs=sorted((APP/'results/R07').glob('spray_20*'))
if not runs:raise SystemExit('R07 has no run yet')
p=runs[-1]
def load(name):
    try:return json.loads((p/name).read_text())
    except (ValueError,OSError):return {}
live=load('live.json');bt=live.get('bt') or {};loc=live.get('loc') or {};gate=live.get('gate') or {};result=load('analysis_r07.json')
print(json.dumps(dict(run=str(p),sim_seconds=live.get('t'),phase=live.get('phase'),point_index=live.get('index'),decision=bt.get('action'),reason=bt.get('reason_code'),evidence=bt.get('evidence'),localization=loc.get('reason'),pose=loc.get('pose'),motion=gate,finished=(p/'runtime.json').exists(),passed=result.get('passed'),failed_checks=[k for k,v in result.get('checks',{}).items() if not v]),ensure_ascii=False,indent=2))
