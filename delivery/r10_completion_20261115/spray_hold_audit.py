import json,sys
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[2];run=ROOT/'optimization_v2/three_tasks/results/R12/spray_20261115_020757_802998'
previous=None;items=[]
def point(T,p):return Rotation.from_quat(T[3:]).apply(p)+T[:3]
def angle(a,b):return (Rotation.from_quat(a[3:]).inv()*Rotation.from_quat(b[3:])).magnitude()
for line in (run/'reference/reference.jsonl').open():
 row=json.loads(line);n=row.get('native_supervisor')
 if not n:continue
 if n.get('plan_revalidation_required') and previous and not previous.get('plan_revalidation_required'):
  reject=n.get('last_rejected_map',{});p=np.array(reject['paired_odom'][:3]);next=np.array(reject['candidate']);last=np.array(previous['raw_map_correction']);filtered=np.array(previous['correction']);items.append(dict(t=row['t'],raw_to_last_accepted_m=float(np.linalg.norm(point(next,p)-point(last,p))),raw_to_last_accepted_rad=float(angle(next,last)),raw_to_filtered_m=float(np.linalg.norm(point(next,p)-point(filtered,p))),raw_to_filtered_rad=float(angle(next,filtered)),limit_m=.15,limit_rad=.035,raw_capture_t=reject['capture_t']))
 previous=n
out=Path(__file__).with_suffix('.json');out.write_text(json.dumps(dict(run=str(run.relative_to(ROOT)),first_latches=items,scope='Offline diagnostic using recorded map corrections and measured paired odometry only; no truth; not a rerun or a new PASS'),indent=2));print(out.read_text())
