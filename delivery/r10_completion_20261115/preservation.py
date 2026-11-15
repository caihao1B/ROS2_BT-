"""Verify protected receipts/raw videos and a byte-exact prior media backup.

Run after simulation and exports, not as a broad scan during sensor workloads.
"""
from pathlib import Path
import hashlib,json
ROOT=Path(__file__).resolve().parents[2];OUT=Path(__file__).resolve().parent;BASE=OUT/'baseline'
def sha(path):
    h=hashlib.sha256()
    with path.open('rb') as f:
        for data in iter(lambda:f.read(1024*1024),b''):h.update(data)
    return h.hexdigest()
def check(path,expected,kind):
    file=ROOT/path
    return dict(path=path,kind=kind,unchanged=file.is_file() and sha(file)==expected)
rows=[];baseline=json.loads((BASE/'manifest.json').read_text())
for r in baseline:
    if r['path'] in ['optimization_v2/three_tasks/results/R08/latest.json','optimization_v2/three_tasks/results/R09/latest.json']:
        rows.append(check(r['path'],r['sha256'],'protected receipt'))
prior=BASE/'optimization_v2/three_tasks/results/R12/latest.json'
if prior.exists():
    d=json.loads(prior.read_text())
    for r in d['artifacts']:
        if r['path'].endswith('/video/full.mp4'):rows.append(check(r['path'],r['sha256'],'prior raw recording'))
        elif '/results/R12/media/' in r['path']:
            relative=r['path'].split('/results/R12/media/',1)[1]
            backup=OUT/'previous_assembled_media'/relative
            rows.append(dict(path=str(backup.relative_to(ROOT)),kind='prior assembled media backup',unchanged=backup.is_file() and sha(backup)==r['sha256']))
result=dict(passed=bool(rows) and all(r['unchanged'] for r in rows),checks=rows,scope='Protected R08/R09 receipts, previous raw video files and prior assembled media; current stage seals checked separately')
(OUT/'preservation_check.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n');print(json.dumps(dict(passed=result['passed'],checked=len(rows))))
if not result['passed']:raise SystemExit(2)
