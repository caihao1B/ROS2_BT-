"""Shared S07 paths and observation-only math; keep S05/S06 source immutable."""
import sys,json,math,hashlib
from pathlib import Path
import numpy as np
import yaml
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[5];BASE=ROOT/'optimization_v2';PKG=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(BASE/'ws/src/map_manager/scripts'))
from s06_common import read,dump,digest,stamp,pose,wrap,transform
def canonical(x):return hashlib.sha256(json.dumps(x,sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest()
def safe(root,path):
    root=Path(root).resolve();p=Path(path)
    if p.is_absolute() or '..' in p.parts:raise ValueError('UNSAFE_PATH')
    p=(root/p).resolve()
    if not p.is_relative_to(root) or not p.is_file():raise ValueError('MISSING_OR_ESCAPED_FILE')
    return p
def verify_bundle(folder=None,request=None):
    folder=Path(folder or BASE/'maps/frozen');m=read(folder/'manifest.json')
    if m['schema']!='S07-FROZEN/1' or m['frame']!='map' or m['mode']!='LOCALIZATION':raise ValueError('MAP_SCHEMA_MISMATCH')
    if canonical({k:v for k,v in m.items() if k!='revision'})!=m['revision']:raise ValueError('MANIFEST_REVISION_MISMATCH')
    for item in m['files']:
        if digest(safe(folder,item['path']))!=item['sha256']:raise ValueError('MAP_CONTENT_CHANGED: '+item['path'])
    if digest(BASE/'config/sensors.yaml')!=m['calibration_revision']:raise ValueError('CALIBRATION_MISMATCH')
    if digest(BASE/'results/S06/latest.json')!=m['source_receipt_sha256']:raise ValueError('SOURCE_RECEIPT_CHANGED')
    if request is not None:
        for key in ['map_id','frame_epoch','revision','calibration_revision']:
            if request.get(key)!=m[key]:raise ValueError('REQUEST_BINDING_MISMATCH: '+key)
        if request.get('purpose')!='DIAGNOSTIC_MOTION_FIXTURE':raise ValueError('REAL_WORK_PLAN_NOT_IMPLEMENTED_UNTIL_S12')
    return m
def capture_inputs():
    files=[PKG/'scripts'/x for x in ['s07_common.py','localization_runtime.py','localization_preprocessor.py','localization_quality.py','localization_observer.py','localization_motion.py','map_refinement.py']]+list((PKG/'src').glob('*'))
    files+=[BASE/'config'/x for x in ['slam_localization.yaml','map_contract.yaml','relocalization_cases.yaml','s07_dependency_lock.json']]
    files+=[BASE/'maps/frozen/manifest.json',BASE/'results/S06/latest.json',BASE/'ws/install/global_pose_manager/lib/global_pose_manager/global_pose_manager']
    return [dict(path=str(p.relative_to(ROOT)),sha256=digest(p)) for p in files if p.is_file()]
