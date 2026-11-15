"""Explain the actual final-observation failure; no extrapolated work credit."""
from pathlib import Path
import json,math,sys
ROOT=Path(__file__).resolve().parents[2];OUT=Path(__file__).resolve().parent
run=ROOT/'optimization_v2/three_tasks/results/R12/spray_20261115_040235_910575'
events=[json.loads(s) for s in (run/'mission_events.jsonl').read_text().splitlines()]
refs=[json.loads(s) for s in (run/'reference/reference.jsonl').read_text().splitlines()]
target=[.5,38.5];point='point_9_17';rows=[]
for r in refs:
    if not 458.<=r['t']<=483. or 'odom' not in r or 'pose' not in r:continue
    raw=r['native_supervisor']['raw_map_correction'];o=r['odom'];qx,qy,qz,qw=raw[3:]
    yaw=math.atan2(2*(qw*qz+qx*qy),1-2*(qy*qy+qz*qz))
    candidate=[raw[0]+math.cos(yaw)*o[0]-math.sin(yaw)*o[1],raw[1]+math.sin(yaw)*o[0]+math.cos(yaw)*o[1]]
    rows.append(dict(t=r['t'],filtered_distance_m=math.dist(r['pose'][:2],target),raw_planar_distance_m=math.dist(candidate,target),correction_tau_s=r['native_supervisor']['correction_filter_active_tau_s']))
checks=[e for e in events if e['event']=='DOCKING_SETTLED_CHECK' and e.get('point')==point]
stops=[e['t'] for e in events if e['event']=='LOCAL_DOCK_STOP_FOR_GLOBAL_OBSERVATION' and e.get('point')==point]
report=dict(run=str(run.relative_to(ROOT)),point=point,task_result='11/56; DOCKING_REALIGNMENT_EXHAUSTED',observations=checks,rows=rows,scope='Recorded localization only. Raw curve is a planar map-to-odom diagnostic, not a new localization source. The run ended before the remaining observation window; no counterfactual PASS or future convergence is claimed.')
(OUT/'final_observation_audit.json').write_text(json.dumps(report,indent=2)+'\n')
import matplotlib;matplotlib.use('Agg')
import matplotlib.pyplot as plt
fig,ax=plt.subplots(figsize=(10,4.5),constrained_layout=True)
ax.plot([r['t'] for r in rows],[r['filtered_distance_m'] for r in rows],label='Observed global distance',color='#146c94')
ax.plot([r['t'] for r in rows],[r['raw_planar_distance_m'] for r in rows],label='Raw correction + odom (planar diagnostic)',color='#d18a38',alpha=.65)
for value,label in [(.035,'Original median gate 0.035 m'),(.045,'Original latest-pose gate 0.045 m')]:ax.axhline(value,ls='--',lw=1,label=label)
for i,t in enumerate(stops):ax.axvline(t,color='#697879',ls=':',label='Local stop for global observation' if i==0 else None)
ax.scatter([e['t'] for e in checks],[e['stability']['median_distance_m'] for e in checks],marker='x',color='#b23939',label='Actual stopped window median')
ax.set(title='A1 point 12: final observation ended before its original 8 s bound',xlabel='Simulation time [s]',ylabel='Distance to unchanged target [m]');ax.grid(alpha=.2);ax.legend(fontsize=8)
fig.savefig(OUT/'final_observation_audit.png',dpi=150);plt.close(fig)
print(json.dumps(dict(point=point,last=rows[-1],original_last_window=checks[-1],scope=report['scope'])))
