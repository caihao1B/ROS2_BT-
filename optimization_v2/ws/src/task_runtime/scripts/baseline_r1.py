"""Snapshot and verify all frozen receipt payloads without rewriting history."""
from common_r1 import *
import time
INDEX=APP/'baseline/frozen_s01_s18.json'

def inventory():
    files={};states={};receipts={}
    for i in range(1,19):
        sid=f'S{i:02}';p=BASE/f'results/{sid}/latest.json';r=read(p)
        states[sid]=r['automated_status'];receipts[sid]=sha(p)
        items=r['source_files']+r['artifacts']+[dict(path=str(p.relative_to(ROOT)),sha256=sha(p))]
        for item in items:
            key=item['path'];q=(ROOT/key).resolve()
            if not q.is_relative_to(ROOT):raise ValueError('HISTORICAL_PATH_OUTSIDE_ROOT '+key)
            if key in files and files[key]!=item['sha256']:raise ValueError('CONFLICTING_HISTORICAL_HASH '+key)
            files[key]=item['sha256']
    return dict(schema='R01-FROZEN-HISTORY/1',stage_states=states,receipts=receipts,files=files)

def verify(data):
    start=time.monotonic();issues=[];total=0
    for rel,digest in data['files'].items():
        p=ROOT/rel
        if not p.is_file():issues.append(dict(path=rel,reason='MISSING'));continue
        total+=p.stat().st_size
        if sha(p)!=digest:issues.append(dict(path=rel,reason='HASH_CHANGED'))
    return dict(passed=not issues,files=len(data['files']),bytes=total,issues=issues,
        wall_seconds=time.monotonic()-start,stage_states=data['stage_states'])

def freeze():
    if INDEX.exists():return verify(read(INDEX))
    data=inventory();result=verify(data)
    if not result['passed']:raise ValueError('EXISTING_HISTORY_INVALID '+json.dumps(result['issues'][:10]))
    dump(INDEX,data);dump(APP/'baseline/before.json',result);return result

if __name__=='__main__':print(json.dumps(freeze(),ensure_ascii=False))
