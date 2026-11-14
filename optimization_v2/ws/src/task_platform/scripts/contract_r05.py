"""A new rig/map binding validates the old bundle and the new calibration separately."""
import os,sys,json,hashlib
from pathlib import Path
ROOT=Path(__file__).resolve().parents[5];BASE=ROOT/'optimization_v2'
def read(p):return json.loads(Path(p).read_text())
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def canonical(x):return hashlib.sha256(json.dumps(x,sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest()
def verify_runtime_contract(request):
 binding_path=Path(os.environ['R05_BINDING']);b=read(binding_path);source=BASE/'maps/frozen/manifest.json';m=read(source)
 if sha(source)!=b['source_manifest_sha256'] or canonical({k:v for k,v in b.items() if k!='revision'})!=b['revision']:raise ValueError('R05_BINDING_CHANGED')
 if canonical({k:v for k,v in m.items() if k!='revision'})!=m['revision']:raise ValueError('SOURCE_MANIFEST_INVALID')
 for item in m['files']:
  p=(BASE/'maps/frozen'/item['path']).resolve()
  if not p.is_relative_to((BASE/'maps/frozen').resolve()) or sha(p)!=item['sha256']:raise ValueError('SOURCE_MAP_CHANGED')
 for pathkey,hashkey in [('rig_path','rig_sha256'),('vehicle_path','vehicle_sha256'),('slam_config_path','slam_config_sha256'),('atlas_path','atlas_sha256'),('reference_cloud_path','reference_cloud_sha256')]:
  p=(ROOT/b[pathkey]).resolve()
  if not p.is_relative_to(ROOT) or sha(p)!=b[hashkey]:raise ValueError('RIG_VEHICLE_CHANGED')
 if request.get('rig_binding_revision')!=b['revision'] or request.get('map_id')!=m['map_id'] or request.get('rig_revision')!=b['rig_revision']:raise ValueError('REQUEST_BINDING_MISMATCH')
 return dict(m,calibration_revision=b['rig_revision'],revision=b['revision'],source_manifest_revision=m['revision'])
