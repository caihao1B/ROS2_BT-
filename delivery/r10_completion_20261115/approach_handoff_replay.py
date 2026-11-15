"""Apply the proposed stop decision to recorded observations; no simulated credit."""
from pathlib import Path
import sys,json
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT/'optimization_v2/ws/src/task_platform/scripts'))
from approach_handoff_r10 import ApproachHandoff
run=ROOT/'optimization_v2/three_tasks/results/R12/spray_20261115_024715_380236'
cfg=json.loads((ROOT/'optimization_v2/three_tasks/editions/R10/spray/mission.json').read_text())
policy=cfg['global_approach_handoff'];events=[json.loads(l) for l in (run/'mission_events.jsonl').read_text().splitlines()]
gates=[]
for line in (run/'control/gate.jsonl').open():
    g=json.loads(line);gates.append((g['t'],g.get('v',0.),g['mode']=='DRIVE'))
index=0;event_index=0;active=None;decision=None;results=[];seen=False
for line in (run/'reference/reference.jsonl').open():
    r=json.loads(line);t=r['t']
    while event_index<len(events) and events[event_index]['t']<=t:
        e=events[event_index];event_index+=1
        if e['event']=='NAV_GOAL':active=e;decision=ApproachHandoff(policy);seen=False
        elif e['event'] in ['NAV_SUCCESS','NAV_CANCEL_ACK','BT_INTERRUPTS_NAV','TASK_STOP']:active=None
    if not active or seen or 'pose' not in r:continue
    while index+1<len(gates) and gates[index+1][0]<=t:index+=1
    g=gates[index];proposal=decision.update(t,r['pose'],active['target'],r.get('valid',False) and g[2] and 0<=t-g[0]<.15,g[1])
    if proposal:
        seen=True;results.append(dict(point=active['point'],attempt=active['attempt'],t=t,proposal=proposal))
out=Path(__file__).with_suffix('.json');out.write_text(json.dumps(dict(run=str(run.relative_to(ROOT)),proposals=results,scope='Offline recorded qualified observations and gated commands. A stop proposal is not work admission; saved runtime and time savings cannot be inferred without a new actual run.'),indent=2)+'\n')
print(json.dumps(dict(proposals=[dict(point=r['point'],t=r['t'],distance=r['proposal']['distance_m']) for r in results])))
