"""Offline measured repair figures; never imported by navigation or tool control."""
from common import *
import numpy as np
from scipy.spatial.transform import Rotation
from analyze_r02 import rows

def render_repairs():
 import matplotlib;matplotlib.use('Agg')
 import matplotlib.pyplot as plt
 out=APP/'results/R10';paths=[]
 pointer=out/'spray_full_A1_latest.json'
 if pointer.exists():
  run=ROOT/read(pointer)['path'];events=rows(run/'mission_events.jsonl');checks=[e for e in events if e['event']=='DOCKING_SETTLED_CHECK' and e.get('stable')]
  event=next((e for e in checks if e['point']=='point_9_16'),checks[-1] if checks else None)
  if event:
   records=[r for r in rows(run/'reference/reference.jsonl') if r.get('valid') and 'pose' in r and event['t']-3.<=r['t']<=event['t']+.3];t=[];filtered=[];raw=[]
   for r in records:
    c=r['native_supervisor'].get('raw_map_correction')
    if not c:continue
    t.append(r['t']);filtered.append(r['pose'][:2]);raw.append((Rotation.from_quat(c[3:]).apply([*r['odom'][:2],0.])+c[:3])[:2])
   if t:
    filtered=np.asarray(filtered);raw=np.asarray(raw);center=np.median(filtered,axis=0);fig,ax=plt.subplots(2,1,figsize=(11,7),sharex=True)
    for j,name in enumerate(['X','Y']):
     ax[j].plot(t,100*(raw[:,j]-center[j]),label='Raw map correction + local odometry',alpha=.65);ax[j].plot(t,100*(filtered[:,j]-center[j]),label='Published filtered global estimate');ax[j].axvline(event['t'],color='green',linestyle='--',label='Original docking gates passed');ax[j].set_ylabel(name+' relative to window median [cm]');ax[j].grid(alpha=.3);ax[j].legend(fontsize=8)
    ax[-1].set_xlabel('Simulation time [s]');fig.suptitle('Recorded docking window: '+event['point']+' (offline diagnostic; raw map may be older)');fig.tight_layout();p=out/'repair_docking.png';fig.savefig(p,dpi=140);plt.close(fig);paths.append(str(p.relative_to(ROOT)))
 pointer=out/'mowing_startup_perception_loss_latest.json'
 if pointer.exists():
  run=ROOT/read(pointer)['path'];fault=read(run/'startup_fault.json') if (run/'startup_fault.json').exists() else {};events=rows(run/'mission_events.jsonl');interrupt=next((e for e in events if e['event']=='BT_INTERRUPTS_START_WORK' and e['t']>=fault.get('t',1e9)),None)
  if interrupt:
   start=fault['t']-1;end=fault['until']+6;physical=[r for r in rows(run/'tool_evaluation/physical.jsonl') if start<=r['t']<=end];fig,ax=plt.subplots(figsize=(11,4));ax.plot([r['t'] for r in physical],[r['rpm_rad_s'] for r in physical],label='Actual simulated tool angular speed');ax.axhline(.1,color='black',linestyle=':',label='Original stopped threshold');ax.axvspan(fault['t'],fault['until'],color='orange',alpha=.15,label='Risk producer paused');ax.axvline(interrupt['t'],color='red',linestyle='--',label='BT interruption handled')
   stop=next((e for e in events if e['event']=='BOTH_PHYSICAL_STOPS_CONFIRMED' and e['t']>=interrupt['t']),None)
   if stop:ax.axvline(stop['t'],color='green',linestyle='--',label='Body + tool stop confirmed')
   ax.set(xlabel='Simulation time [s]',ylabel='Tool speed [rad/s]',title='Actual startup cancellation and recovery');ax.grid(alpha=.3);ax.legend(fontsize=8);fig.tight_layout();p=out/'repair_start_work.png';fig.savefig(p,dpi=140);plt.close(fig);paths.append(str(p.relative_to(ROOT)))
 dump(out/'repair_figures.json',dict(paths=paths,scope='Offline observed diagnostics; acceptance remains in each run analysis'));print(json.dumps(paths));return paths
if __name__=='__main__':render_repairs()
