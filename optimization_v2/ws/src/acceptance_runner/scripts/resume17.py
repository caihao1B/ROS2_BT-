"""Restore only confirmed ledger state; no command acceptance earns work credit."""
from common17 import *
import sqlite3,math
from collections import deque
from shapely.geometry import shape

def prepare(previous,out,cluster):
    previous=Path(previous).resolve();out=Path(out).resolve()
    if not previous.is_relative_to(RESULTS) or not out.is_relative_to(RESULTS):raise ValueError('RESUME_PATH_SCOPE')
    from inputs17 import active
    m=read(previous/'client/mission.json');runtime=read(previous/'runtime.json');cleanup=read(previous/'cleanup14.json');plan=read(active()['plan'])
    if runtime.get('tool_shutdown',{}).get('state')!='STOPPED' or cleanup.get('survivors') or not runtime.get('database_unchanged'):raise ValueError('PREVIOUS_STOP_NOT_VERIFIED')
    if m.get('uncertainty_pending') or any(x.get('uncertain') for x in m.get('tool_results',[])):raise ValueError('UNCERTAIN_WORK_REQUIRES_RECONCILIATION')
    if m['plan_revision']!=plan['plan_revision'] or m['cluster']!=cluster:raise ValueError('RESUME_VERSION_OR_CLUSTER')
    obs=[r for r in rows(previous/'online/observations.jsonl') if r.get('record_type')=='pose']
    if not obs:raise ValueError('NO_VERIFIED_RESTART_POSE')
    pose=obs[-1]['pose'];q=pose[3:];heading=math.atan2(2*(q[3]*q[2]+q[0]*q[1]),1-2*(q[1]**2+q[2]**2))
    dest=out/'ledger/ledger.sqlite';dest.parent.mkdir(parents=True,exist_ok=True)
    with sqlite3.connect('file:'+str(previous/'ledger/ledger.sqlite')+'?mode=ro',uri=True) as src,sqlite3.connect(dest) as target:src.backup(target)
    paths=[previous/f for f in ['client/mission.json','runtime.json','cleanup14.json','ledger/ledger.sqlite','online/observations.jsonl']]
    manifest=dict(source=str(previous.relative_to(ROOT)),cluster=cluster,plan_revision=plan['plan_revision'],mission=m,restart_pose_prior=[pose[0]+28,pose[1]+14,heading],restart_pose_source='last sensor-based estimate; not truth',source_hashes={str(p.relative_to(ROOT)):sha(p) for p in paths},copied_ledger_sha256=sha(dest))
    path=out/'resume_source.json';dump(path,manifest);return path

def restore_mission(m):
    path=os.environ.get('S17_RESUME_MANIFEST')
    if not path:return
    p=Path(path).resolve()
    if not p.is_relative_to(RESULTS):raise ValueError('RESUME_MANIFEST_SCOPE')
    d=read(p);old=d['mission'];database=m.a.out.parent/'ledger/ledger.sqlite'
    for f,h in d['source_hashes'].items():
        if sha(ROOT/f)!=h:raise ValueError('RESUME_SOURCE_CHANGED '+f)
    if d['plan_revision']!=m.c['plan_revision'] or d['cluster']!=os.environ['S17_CLUSTER']:raise ValueError('RESUME_IDENTITY_CHANGED')
    known={j['id'] for j in m.jobs}
    if not set(old['done']).issubset(known):raise ValueError('RESUME_UNKNOWN_JOB')
    with sqlite3.connect('file:'+str(database)+'?mode=ro',uri=True) as db:
        statuses={r[0]:(r[1],r[2],r[3]) for r in db.execute('SELECT key,status,quantity,uncertain FROM jobs')}
    for j in m.jobs:
        if j['id'] in old['done'] and j['kind']=='POINT_SERVICE':
            value=statuses.get(m.c['plan_revision']+':'+j['id'])
            if not value or value[0]!='SUCCEEDED' or value[1]<2.-1e-9 or value[2]:raise ValueError('RESUME_UNCONFIRMED_POINT '+j['id'])
    m.done=list(old['done']);m.tool_results=old['tool_results'];m.credit=shape(old['credit']);m.pending=deque(j for j in m.jobs if j['id'] not in m.done)
    if m.credit.difference(m.area).area>1e-8:raise ValueError('RESUME_CREDIT_OUTSIDE_ORIGINAL_AREA')
    dump(m.a.out/'restored17.json',dict(done=m.done,pending=[j['id'] for j in m.pending],confirmed_points=old['confirmed_points'],plan_revision=d['plan_revision'],source=d['source'],full_task_complete_claim=False))
