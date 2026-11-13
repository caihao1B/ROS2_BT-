"""Explicit operator restart from verified feedback, with the original task fixed."""
from common_showcase import *
import sqlite3,types
from shapely import union_all,wkb
from shapely.geometry import shape

def prepare(previous,out,cluster):
    previous=Path(previous).resolve();out=Path(out).resolve()
    prior=read(previous/'showcase_result.json');spec=read(previous/'spec.json')
    plan=read(Path(os.environ['SHOWCASE_PLAN']));mission=read(previous/'client/mission.json')
    if not (prior['safety'] and prior['localization_pass'] and prior['demo_version_valid']):raise ValueError('RESUME_SOURCE_NOT_SAFE_AND_LOCALIZED')
    if spec['cluster']!=cluster or spec['operation']!=os.environ.get('S17_OPERATION','work') or spec['plan_revision']!=plan['plan_revision']:raise ValueError('RESUME_TASK_IDENTITY')
    if spec['vehicle']=='ackermann' and spec['operation']=='work':
        with sqlite3.connect('file:'+str(previous/'ledger/ledger.sqlite')+'?mode=ro',uri=True) as db:
            records=list(db.execute('SELECT key,credit,uncertain FROM jobs'))
        if any(uncertain for _,_,uncertain in records):raise ValueError('RESUME_UNCERTAIN_DATABASE')
        credited=union_all([wkb.loads(blob) for key,blob,_ in records if blob and key.startswith(plan['plan_revision']+':')])
        area=shape(mission['credit']).union(shape(mission['residual']))
        if credited.intersection(area).symmetric_difference(shape(mission['credit'])).area>1e-7:raise ValueError('RESUME_CREDIT_DOES_NOT_MATCH_LEDGER')
    source=(BASE/'ws/src/acceptance_runner/scripts/resume17.py').read_text()
    before="plan=read(active()['plan'])"
    if source.count(before)!=1:raise ValueError('RESUME_ADAPTER_SOURCE_CHANGED')
    source=source.replace(before,"plan=read(Path(os.environ['SHOWCASE_PLAN']))")
    module=types.ModuleType('showcase_verified_restart');exec(compile(source,str(__file__),'exec'),module.__dict__)
    manifest=module.prepare(previous,out,cluster)
    if spec['operation']=='inspection':
        from inspection_recovery import inspection_state
        d=read(manifest);d['inspection_state']=inspection_state(previous)
        for f in ['client/inspection_progress17.json','client/obligations.json','client/events.jsonl','client/inspection_contract17.json','evaluation/truth.jsonl','client/progress.json']:
            if (previous/f).exists():d['source_hashes'][str((previous/f).relative_to(ROOT))]=sha(previous/f)
        dump(manifest,d)
    proof=dict(source=str(previous.relative_to(ROOT)),source_result_sha256=sha(previous/'showcase_result.json'),source_spec_sha256=sha(previous/'spec.json'),manifest_sha256=sha(manifest),operator_restart=True,automatic_retry=False,ledger_union_verified=True,original_task_unchanged=True)
    dump(out/'showcase_resume_proof.json',proof);return manifest
