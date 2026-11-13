"""Verify evidence integrity even for PARTIAL receipts; never promote status."""
from common17 import *

def audit(receipt=None):
    receipt=Path(receipt or RESULTS/'latest.json').resolve()
    plan=read(BASE/'plan/steps.json');steps={s['id']:s for s in plan['steps']}
    digests={};visited={};active=set()
    def contained(value):
        p=(ROOT/value).resolve()
        if not p.is_relative_to(ROOT) or not p.is_file():raise ValueError('MISSING_OR_EXTERNAL_EVIDENCE '+str(value))
        return p
    def digest(p):
        if p not in digests:digests[p]=sha(p)
        return digests[p]
    def inspect(path,required=None):
        data=read(path);sid=data['step']
        if sid in active:raise ValueError('RECEIPT_DEPENDENCY_CYCLE')
        if data['schema_version']!=1 or data['plan_revision']!=plan['revision'] or sid not in steps:raise ValueError('INVALID_RECEIPT_SCHEMA')
        status=data['automated_status']
        if status not in ['PASS','PARTIAL','FAIL','NOT_RUN'] or (required and status!=required):raise ValueError('INVALID_DEPENDENCY_STATUS '+sid)
        if path in visited:return
        active.add(sid)
        if not data.get('source_files') or not data.get('artifacts'):raise ValueError('EMPTY_EVIDENCE_LIST')
        declared={r['path'] for r in data['artifacts']}
        expected={f'docs/optimization_v2/{sid}/{n}.md' for n in ['TECHNICAL','CODE_WALKTHROUGH','MANUAL_CHECK','RESULTS']}
        expected|={f'optimization_v2/lessons/{sid}/{n}' for n in ['lesson.cpp','lesson.py','README.md']}
        expected.add(data['report_path'])
        if not expected.issubset(declared):raise ValueError('REQUIRED_DELIVERY_MISSING '+sid)
        check_ids={c['id'] for c in steps[sid]['checks']}
        if not check_ids.issubset(data.get('checks',{})):raise ValueError('CHECK_IDS_MISSING '+sid)
        if status=='PASS' and (data['check_exit_code']!=0 or any(data['checks'][k]!='PASS' for k in check_ids)):raise ValueError('UNSUPPORTED_PASS '+sid)
        for item in data['source_files']+data['artifacts']:
            if digest(contained(item['path']))!=item['sha256']:raise ValueError('EVIDENCE_CHANGED '+item['path'])
        for dep in steps[sid]['depends_on']:
            p=BASE/'results'/dep/'latest.json'
            if digest(p)!=data.get('dependency_receipts',{}).get(dep):raise ValueError('DEPENDENCY_RECEIPT_CHANGED '+dep)
            inspect(p,'PASS')
        active.remove(sid);visited[path]=status
    inspect(receipt)
    data=read(receipt)
    return dict(integrity_pass=True,acceptance_status=data['automated_status'],verified_files=len(digests),verified_receipts={p.parent.name:s for p,s in visited.items()},scope='Evidence hashes and dependency contracts only; no new task completion credit')

if __name__=='__main__':
    print(json.dumps(audit(),ensure_ascii=False,indent=2))
