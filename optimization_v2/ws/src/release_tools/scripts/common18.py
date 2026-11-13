"""Separate S18 delivery evidence; no changes to frozen S01--S17 receipts."""
from pathlib import Path
import os, sys, json, hashlib, datetime
ROOT=Path(__file__).resolve().parents[5]
BASE=ROOT/'optimization_v2'
PKG=BASE/'ws/src/release_tools'
RESULTS=BASE/'results/S18'
def read(p):return json.loads(Path(p).read_text())
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def dump(p,d):
    p=Path(p);p.parent.mkdir(parents=True,exist_ok=True);tmp=p.with_name(p.name+'.tmp.'+str(os.getpid()))
    tmp.write_text(json.dumps(d,indent=2,ensure_ascii=False,allow_nan=False)+'\n');os.replace(tmp,p)
def stamp():return datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%d_%H%M%S_%f')
def add17():sys.path.insert(0,str(BASE/'ws/src/acceptance_runner/scripts'))
def session_module():
    sys.path.insert(0,str(BASE/'ws/src/project_baseline/scripts'))
    import session_owner
    return session_owner
