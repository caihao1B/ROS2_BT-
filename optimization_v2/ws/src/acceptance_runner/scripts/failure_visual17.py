"""Offline recorded failure figure; never reads or publishes live ROS data."""
from common17 import *
import argparse,numpy as np

def render(case,out):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    case=Path(case).resolve();out=Path(out).resolve();out.mkdir(parents=True,exist_ok=True)
    if not case.is_relative_to(RESULTS) or not out.is_relative_to(RESULTS):raise ValueError('EVIDENCE_PATH_SCOPE')
    metric=read(case/'metrics17.json');observed=rows(case/'online/observations.jsonl');truth=rows(case/'evaluation/truth.jsonl');contacts=rows(case/'evaluation/contact_events17.jsonl')
    poses=[r for r in observed if r['record_type']=='pose'];gates=[r for r in observed if r['record_type']=='gate'];t=np.array([r['t'] for r in poses]);xy=np.array([r['pose'][:2] for r in poses]);tt=np.array([r['t'] for r in truth]);actual=np.array([r['xyz'][:2] for r in truth])-[28,14]
    valid=(t>=tt[0])&(t<=tt[-1]);t=t[valid];xy=xy[valid];aligned=np.c_[np.interp(t,tt,actual[:,0]),np.interp(t,tt,actual[:,1])];error=np.linalg.norm(xy-aligned,axis=1);ct=np.array(sorted({r['t'] for r in contacts}))
    first_contact_context=None
    if len(ct):
        # Strictly before first contact: the first contact itself may push the
        # robot and therefore must not be included in pre-impact velocity.
        pre=np.flatnonzero((tt>=ct[0]-.5)&(tt<ct[0]));last_gate=next((g for g in reversed(gates) if g['t']<ct[0]),None)
        speeds=np.linalg.norm(np.diff(actual[pre],axis=0),axis=1)/np.diff(tt[pre]) if len(pre)>1 else np.array([])
        first_contact_context=dict(window_s=.5,truth_samples=len(pre),sample_time_span_s=[float(tt[pre[0]]),float(tt[pre[-1]])] if len(pre) else [],maximum_precontact_speed_mps=float(speeds.max()) if len(speeds) else None,last_gate=last_gate,interpretation='Offline pre-contact state; zero commanded velocity does not exempt any contact from FAIL')
    fig,axes=plt.subplots(1,3,figsize=(15,4.5));a,b,c=axes
    a.plot(*actual[::10].T,color='#45505c',lw=1,label='Gazebo truth / evaluation domain only');a.plot(*xy[::5].T,color='#2383b4',lw=1,label='Sensor-based estimate')
    if len(ct):
        positions=np.c_[np.interp(ct,tt,actual[:,0]),np.interp(ct,tt,actual[:,1])];a.scatter(*positions.T,color='#d24435',marker='x',s=15,label='Recorded non-ground contacts')
    a.set_aspect('equal');a.set_xlabel('map x (m)');a.set_ylabel('map y (m)');a.set_title('Recorded trajectory and contact locations');a.legend(fontsize=7)
    b.plot(t,error,color='#d1842a',lw=1);b.axhline(.15,color='#c84b40',ls='--',lw=1,label='Position P95 acceptance target');b.set_xlabel('simulation time (s)');b.set_ylabel('position error (m)');b.set_title('Contact and localization disagreement');b.legend(fontsize=7)
    c.plot([r['t'] for r in gates],[r['v'] for r in gates],color='#2383b4',lw=1,label='Commanded linear velocity');c.set_xlabel('simulation time (s)');c.set_ylabel('m/s');c.set_title('A stopped robot can still be contacted');c.legend(fontsize=7)
    for ax in [b,c]:
        if len(ct):
            for ts in ct[::max(1,len(ct)//80)]:ax.axvline(ts,color='#d24435',alpha=.12,lw=.6)
        ax.grid(alpha=.2)
    fig.suptitle('S17 development candidate: '+metric['status']+' — not formal six-cluster evidence\n'+case.name,fontsize=11)
    fig.tight_layout();fig.savefig(out/'failure.png',dpi=145);plt.close(fig)
    dump(out/'failure_evidence17.json',dict(case=str(case.relative_to(ROOT)),status=metric['status'],first_contact_sim_s=float(ct[0]) if len(ct) else None,last_contact_sim_s=float(ct[-1]) if len(ct) else None,recorded_contact_rows=len(ct),first_contact_context=first_contact_context,position_p95_m=metric['position_error_m']['p95'],sources={str((case/p).relative_to(ROOT)):sha(case/p) for p in ['metrics17.json','evaluation/truth.jsonl','evaluation/contact_events17.jsonl','online/observations.jsonl']},truth_used_offline_only=True,formal_credit=False))
    print(out/'failure.png')
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--case',type=Path,required=True);p.add_argument('--out',type=Path,required=True);a=p.parse_args();render(a.case,a.out)
