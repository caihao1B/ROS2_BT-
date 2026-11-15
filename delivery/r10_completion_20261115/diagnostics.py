"""Describe current completed cases from recorded evidence; never assign stage PASS."""
from pathlib import Path
import json,shutil,math
ROOT=Path(__file__).resolve().parents[2];DELIVERY=Path(__file__).resolve().parent
APP=ROOT/'optimization_v2/three_tasks';OUT=APP/'results/R10'
def read(p):return json.loads(p.read_text())
def rows(p):return [json.loads(l) for l in p.read_text().splitlines()]
def write(name,value):
    p=OUT/name;old=DELIVERY/'prior_diagnostics'/name
    if p.exists() and not old.exists():old.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(p,old)
    p.write_text(json.dumps(value,ensure_ascii=False,indent=2)+'\n')
def case(name):
    p=ROOT/read(OUT/(name+'_latest.json'))['path']
    assert (p/'runtime.json').exists(),p
    return p,read(p/'mission_result.json'),rows(p/'mission_events.jsonl')

spray,sm,se=case('spray_full_A1');mowing,mm,me=case('mowing_full_B1');fault,fm,fe=case('mowing_startup_perception_loss')
sa=read(spray/'analysis_spray_r10.json');ma=read(mowing/'analysis_mowing_r10.json');fa=read(fault/'analysis_mowing_r10.json')
docks=[e for e in se if e['event']=='DOCKING_SETTLED_CHECK'];admitted=[e for e in docks if e.get('stable')]
source=str(spray.relative_to(ROOT))
write('docking_diagnosis.json',dict(source=source,task_passed=sa['passed'],confirmed_points=len(sm['completed']),original_points=sm['original_points'],local_docking_actions=sum(e['event']=='LOCAL_DOCK_GOAL' for e in se),admitted_stopped_windows=admitted,map_xy_rmse_m=sa['map_xy_rmse_m'],original_work_limits=dict(median_distance_m=.035,latest_distance_m=.045,p95_scatter_m=.015,minimum_span_s=1.8),scope='Actual completed recording; docking windows and independent physical work checks remain separate'))
recovery_names={'LATE_LOCALIZATION_HOLD_DISPATCHED','STOPPED_RELOCALIZATION_REQUEST','LOCALIZATION_PREVIEW_PREPARED','STOPPED_PLAN_REVALIDATION','LOCALIZATION_PLAN_COMMITTED','LOCALIZATION_REVALIDATED_RESUME'}
global_events=[{k:v for k,v in e.items() if k!='path'} for e in se if e['event'] in recovery_names]
write('global_hold_diagnosis.json',dict(source=source,task_passed=sa['passed'],events=global_events,scope='Actual task events only; synthetic native recovery probe is separately labeled',native_probe='global_filter_probe/result.json',transaction_probe='revalidation_probe/result.json'))
recovery_names={'ENDPOINT_RECOVERY_SUCCESS','RECOVERY_CUSP_STOP_CONFIRMED','STOPPED_ORIGINAL_ARRIVAL_CONFIRMED','ENDPOINT_RECOVERY_INTERRUPTED','RECOVERY_TRACKING_STOP','TOOL_OFF_ALTERNATE_ADMITTED_FOR_VALIDATION','TOOL_OFF_RETREAT_CANDIDATE','COVERAGE_REJOIN_CONFIRMED','COVERAGE_REJOIN_WINDOW','ORIGINAL_WORK_WINDOW_RECHECK'}
write('endpoint_tracking_diagnosis.json',dict(source=str(mowing.relative_to(ROOT)),task_passed=ma['passed'],completed_segments=len(mm['completed_segments']),original_segments=mm['original_segments'],terminal=mm['terminal'],error=mm['error'],events=[e for e in me if e['event'] in recovery_names],confirmed_area_m2=ma['selected_confirmed_union_m2'],residual_area_m2=ma['selected_residual_m2'],physical_stop=ma['checks']['measured_final_stop'],reference_original_segments=mm.get('reference_original_segments'),original_reference_status=mm.get('original_reference_status'),safe_plan_revision=mm.get('safe_plan_revision'),scope='Actual same-version approved B1 safe execution; original reference remains PARTIAL; original feedback denominator retained'))
interrupts=[e for e in fe if e['event']=='BT_INTERRUPTS_START_WORK'];stops=[e for e in fe if e['event']=='BOTH_PHYSICAL_STOPS_CONFIRMED'];delays=[min(s['t']-e['t'] for s in stops if s['t']>=e['t']) for e in interrupts]
startup=dict(run=str(fault.relative_to(ROOT)),task_passed=fa['passed'],startup_stop_delays_sim_s=delays,original_limit_sim_s=2.,uncertainty_pending=any(j['uncertain'] for j in fa['jobs']),unique_jobs=len(fa['jobs']),completed_segments=len(fm['completed_segments']),confirmed_area_m2=fa['selected_confirmed_union_m2'],residual_area_m2=fa['selected_residual_m2'],work_results=fm['work_results'],fault=read(fault/'startup_fault.json'),checks={k:v for k,v in fa['checks'].items() if 'startup' in k or 'perception' in k or 'cancel' in k})
write('startup_recovery_diagnosis.json',startup);write('mowing_start_diagnosis.json',startup)
summary=read(OUT/'case_summary.json')
write('time_breakdown.json',dict(cases=[{k:v for k,v in c.items() if k in ['case','path','passed','simulation_seconds','wall_seconds']} for c in summary['cases']],scope='Measured independent cases. No sixty-run matrix or extrapolated runtime'))

for name in ['repair_docking.png','repair_start_work.png']:
    old=DELIVERY/'prior_diagnostics'/name
    if (OUT/name).exists() and not old.exists():old.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(OUT/name,old)
import matplotlib;matplotlib.use('Agg')
import matplotlib.pyplot as plt
fig,axes=plt.subplots(1,2,figsize=(12,4.5))
for axis,key,limit,title in [(axes[0],'median_distance_m',.035,'Stopped median position error'),(axes[1],'position_scatter_p95_m',.015,'Stopped P95 scatter')]:
    for stable,color,label in [(False,'#c07828','Correction/observation needed'),(True,'#178268','Admitted')]:
        pts=[(e['t'],e['stability'].get(key)) for e in docks if e.get('stable')==stable and isinstance(e.get('stability',{}).get(key),(int,float))]
        axis.scatter([x[0] for x in pts],[x[1] for x in pts],s=13,color=color,label=label)
    axis.axhline(limit,color='#b64a4a',ls='--',label='Original admission limit');axis.set(title=title,xlabel='Simulation seconds',ylabel='m');axis.grid(alpha=.2);axis.legend(fontsize=8)
fig.suptitle('Current A1 recording / actual stopped observations');fig.tight_layout();fig.savefig(OUT/'repair_docking.png',dpi=150);plt.close(fig)
inj=startup['fault'];lo=inj['t']-2;hi=inj['until']+10;physical=[r for r in rows(fault/'tool_evaluation/physical.jsonl') if lo<=r['t']<=hi];gates=[r for r in rows(fault/'control/gate.jsonl') if lo<=r['t']<=hi]
fig,axes=plt.subplots(2,1,figsize=(10,5.5),sharex=True)
axes[0].plot([r['t'] for r in physical],[r['rpm_rad_s'] for r in physical],label='Independent physical tool speed');axes[0].set_ylabel('rad/s')
axes[1].plot([r['t'] for r in gates],[r['v'] for r in gates],label='Gated vehicle request');axes[1].set(ylabel='m/s',xlabel='Simulation seconds')
for ax in axes:
    ax.axvspan(inj['t'],inj['until'],alpha=.15,color='red',label='Actual perception process outage')
    for i,e in enumerate(interrupts):ax.axvline(e['t'],color='#b25328',ls='--',label='BT startup cancellation' if i==0 else None)
    ax.grid(alpha=.2);ax.legend(fontsize=8)
fig.suptitle('Current START_WORK fault / actual cancellation and recovery');fig.tight_layout();fig.savefig(OUT/'repair_start_work.png',dpi=150);plt.close(fig)
write('repair_figures.json',dict(spray_source=source,startup_source=str(fault.relative_to(ROOT)),images=['repair_docking.png','repair_start_work.png'],scope='Current recorded evidence; plots do not assign task credit'))
print(json.dumps(dict(spray=sa['passed'],mowing=ma['passed'],startup=fa['passed'],startup_stop_delays=delays)))
