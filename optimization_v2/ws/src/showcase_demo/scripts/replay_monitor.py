"""Regression replay of recorded sensor estimates; no new live-run claim."""
from common_showcase import *
from motion_monitor import ProgressMonitor
import math

def main():
    case=BASE/'results/S17/showcase_repair_ackermann_20261113_181900';source=case/'online/observations.jsonl';p=ProgressMonitor();first=None;samples=0
    for line in source.read_text().splitlines():
        row=json.loads(line)
        if row.get('record_type')!='pose' or row['t']<9.5:continue
        q=row['pose'][3:];h=math.atan2(2*(q[3]*q[2]+q[0]*q[1]),1-2*(q[1]**2+q[2]**2));state=p.update(row['t'],row['pose'][:2],'connector_0',True,False,h);samples+=1
        if state['manual'] and first is None:first=row['t']
    result=dict(scope='OFFLINE_REPLAY_OF_RECORDED_SENSOR_ESTIMATES',not_a_new_physical_run=True,source=str(source.relative_to(ROOT)),source_sha256=sha(source),algorithm_sha256=sha(PKG/'scripts/motion_monitor.py'),samples=samples,first_manual_sim_t=first,passed=first is not None,meaning='current position-and-heading monitor still detects the recorded genuinely stationary Ackermann case')
    dump(OUT/'monitor_replay.json',result);print(json.dumps(result,ensure_ascii=False));return 0 if result['passed'] else 1
if __name__=='__main__':raise SystemExit(main())
