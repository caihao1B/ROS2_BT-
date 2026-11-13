"""Showcase inspection checkpoints: measured progress bounds each recovery window.

No failed action, commanded motion or elapsed time resets recovery. Original
whole-road endpoints and offline physical trajectory auditing remain required.
"""
from common_showcase import *
from common17 import rows
import math,copy,shutil

def completed_part(m,part):
    known=getattr(m,'showcase_recovery_boundaries',set())
    if part['id'] in known:return
    # Called only after NavigateToPose returns SUCCEEDED and the original
    # executor persists that part as complete.
    if part['id'] not in getattr(m,'inspection_completed_parts',[]):raise ValueError('UNCONFIRMED_RECOVERY_BOUNDARY')
    r=m.admission_recovery
    if r.since is not None or r.exhausted:raise ValueError('ACTIVE_RECOVERY_AT_PROGRESS_BOUNDARY')
    m.showcase_total_replans=getattr(m,'showcase_total_replans',0)+m.replans
    m.showcase_total_resumes=getattr(m,'showcase_total_resumes',0)+r.resumes
    m.log('inspection_recovery_window_closed',id=part['id'],replans=m.replans,observation_resumes=r.resumes,total_replans=m.showcase_total_replans,total_observation_resumes=m.showcase_total_resumes,reason='NAVIGATION_SUCCESS_AT_NEW_REFERENCE_PART',no_work_credit=True)
    m.replans=0;r.resumes=0;m.last_dispatch_token=None
    known.add(part['id']);m.showcase_recovery_boundaries=known

def inspection_state(previous):
    progress=read(previous/'client/inspection_progress17.json')
    jobs=read(previous/'client/obligations.json')['jobs'];parts={p['id']:p for j in jobs for p in j['reference_parts']}
    events=rows(previous/'client/events.jsonl');completed=progress['completed_parts']
    if len(completed)!=len(set(completed)) or not set(completed)<=parts.keys():raise ValueError('RESUME_UNKNOWN_INSPECTION_PART')
    prior_manifest=read(previous/'resume_source.json') if (previous/'resume_source.json').exists() else {}
    inherited=set(prior_manifest.get('inspection_state',{}).get('completed_parts',[]))
    verified={e['id'] for e in events if e['event']=='inspection_part_complete'}|inherited
    if set(completed)!=verified:raise ValueError('RESUME_INSPECTION_EVENT_MISMATCH')
    executed=copy.deepcopy(prior_manifest.get('inspection_state',{}).get('executed_parts',{}))
    for e in events:
        if e['event']=='inspection_temporary_goal':
            if e['id'] not in parts or e['reference']!=parts[e['id']]['poses'][-1]:raise ValueError('RESUME_CHANGED_TEMPORARY_REFERENCE')
            p=copy.deepcopy(parts[e['id']]['poses']);p[-1]=e['target'];executed[e['id']]=p
    checkpoint=read(previous/'client/progress.json') if (previous/'client/progress.json').exists() else None
    if checkpoint:
        label=checkpoint['id'].split('_resume_')[0]
        if label not in parts:raise ValueError('RESUME_UNKNOWN_INSPECTION_SUFFIX')
        poses=executed.get(label,parts[label]['poses']);tail=checkpoint['remaining_poses']
        if len(tail)<2 or tail!=poses[-len(tail):]:raise ValueError('RESUME_NOT_ORIGINAL_INSPECTION_SUFFIX')
    return dict(completed_parts=completed,executed_parts=executed,checkpoint=checkpoint,jobs=jobs,no_work_credit=True)

def restore(m):
    manifest=os.environ.get('S17_RESUME_MANIFEST')
    if not manifest or os.environ.get('S17_OPERATION')!='inspection':return
    d=read(Path(manifest));s=d['inspection_state']
    if m.jobs!=s['jobs']:raise ValueError('INSPECTION_OBLIGATIONS_CHANGED')
    m.inspection_completed_parts=s['completed_parts'];m.inspection_executed_parts=s['executed_parts'];m.showcase_recovery_boundaries=set(s['completed_parts'])
    if s['checkpoint']:dump(m.a.out/'progress.json',s['checkpoint'])
    dump(m.a.out/'inspection_progress17.json',dict(plan_revision=m.c['plan_revision'],completed_parts=m.inspection_completed_parts,no_work_credit=True))
    m.log('inspection_checkpoint_restored',source=d['source'],completed_parts=m.inspection_completed_parts,no_work_credit=True)

def remaining_nominal(previous):
    s=inspection_state(previous);completed=set(s['completed_parts']);seconds=0.
    for j in s['jobs']:
        for p in j['reference_parts']:
            if p['id'] in completed:continue
            poses=s['executed_parts'].get(p['id'],p['poses'])
            if s['checkpoint'] and s['checkpoint']['id'].split('_resume_')[0]==p['id']:poses=s['checkpoint']['remaining_poses']
            seconds+=sum(math.dist(a[:2],b[:2]) for a,b in zip(poses,poses[1:]))/.12+5.
    return seconds+30.  # startup allowance; hard wall and simulator deadlines remain.

def audit_chain(contract,truth,admitted,vehicle,previous):
    from inspection_audit17 import audit
    prior=read(previous/'showcase_result.json');trajectory=[];offset=0.;sources={}
    for rel in prior.get('execution_episodes',[str(previous.relative_to(ROOT))]):
        p=ROOT/rel;events=rows(p/'client/events.jsonl');start=next(e['t'] for e in events if e['event']=='mission_admitted')
        if read(p/'client/inspection_contract17.json')!=contract:raise ValueError('INSPECTION_CHAIN_ROUTE_CHANGED')
        recorded=[copy.deepcopy(r) for r in rows(p/'evaluation/truth.jsonl') if r['t']>=start]
        for r in recorded:r['t']+=offset
        trajectory+=recorded;offset=trajectory[-1]['t']+1.
        for f in ['evaluation/truth.jsonl','client/inspection_contract17.json','client/events.jsonl']:sources[str((p/f).relative_to(ROOT))]=sha(p/f)
    current=[copy.deepcopy(r) for r in truth if admitted is not None and r['t']>=admitted]
    for r in current:r['t']+=offset
    result=audit(contract,trajectory+current,0.,vehicle)
    result.update(explicit_restart_chain=True,source_hashes=sources,time_axis='ordered episodes with one second separator; not a continuous simulation; no interpolated positions')
    return result
