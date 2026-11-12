"""S11 local paths and content identities. No ROS, simulator or truth subscription."""
import hashlib,json,os,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[5]
BASE=ROOT/'optimization_v2';PKG=BASE/'ws/src/coverage_partition'
BIN=BASE/'ws/install/coverage_partition/lib/coverage_partition/partition_metis'
def read(p):return json.loads(Path(p).read_text())
def dump(p,d):
    p=Path(p);p.parent.mkdir(parents=True,exist_ok=True)
    tmp=p.with_suffix(p.suffix+'.tmp');tmp.write_text(json.dumps(d,ensure_ascii=False,indent=2,allow_nan=False)+'\n');tmp.replace(p)
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def identity(d):return hashlib.sha256(json.dumps(d,sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest()
def files(p):return sorted(x for x in Path(p).rglob('*') if x.is_file() and '__pycache__' not in x.parts)
def row(p):return {'path':str(Path(p).relative_to(ROOT)),'sha256':sha(p)}
def inputs():
    return [BASE/'config'/s for s in ['partition.yaml','tool_profiles.yaml','tool_contract.yaml','localization_supervisor.yaml','vehicles/qualified_profiles.json','vehicles/diff.yaml','vehicles/ackermann.yaml']]+files(BASE/'maps/prior')
def feature(g,**properties):
    from shapely.geometry import mapping
    return {'type':'Feature','geometry':mapping(g),'properties':properties}
def collection(fs):return {'type':'FeatureCollection','features':fs}
