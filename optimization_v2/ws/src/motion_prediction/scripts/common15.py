"""Paths and narrow helpers; no simulator script or future labels imported."""
from pathlib import Path
import json,hashlib,sys
PKG=Path(__file__).resolve().parents[1]
BASE=PKG.parents[2]
ROOT=BASE.parent
GUARD=PKG.parent/'prediction_risk_guard'
S14=PKG.parent/'mission_bt'

def read(path):return json.loads(Path(path).read_text())
def dump(path,value):
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    tmp=path.with_suffix(path.suffix+'.tmp');tmp.write_text(json.dumps(value,ensure_ascii=False,indent=2,allow_nan=False)+'\n');tmp.replace(path)
def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def add14():
    sys.path.insert(0,str(S14/'scripts'))
    from common14 import replace_once
    return replace_once
