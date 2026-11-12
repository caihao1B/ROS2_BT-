"""S06 paths and numerical helpers; no simulator truth access in online modules."""
import json, math, hashlib
from pathlib import Path
import numpy as np
import yaml
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[5]
BASE=ROOT/'optimization_v2'
PKG=Path(__file__).resolve().parents[1]
def read(p): return json.loads(Path(p).read_text())
def dump(p,d):
    p=Path(p);p.parent.mkdir(parents=True,exist_ok=True)
    q=p.with_name(p.name+'.tmp');q.write_text(json.dumps(d,ensure_ascii=False,indent=2)+'\n');q.replace(p)
def stamp(m): return m.header.stamp.sec+m.header.stamp.nanosec*1e-9
def wrap(x): return (x+math.pi)%(2*math.pi)-math.pi
def euler(q): return Rotation.from_quat(q).as_euler('xyz')
def pose(m):
    p=m.pose.pose;return np.array([p.position.x,p.position.y,p.position.z]),np.array([p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w])
def digest(p): return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def transform(xyz,p,q): return Rotation.from_quat(q).apply(xyz)+p
def capture_inputs():
    names=['s06_common.py','mapping_scene.py','mapping_inputs.py','mapping_observer.py','mapping_motion.py','mapping_runtime.py']
    files=[PKG/'scripts'/n for n in names]+list((PKG/'src').glob('*.cpp'))
    files+=[BASE/'config'/n for n in ['slam_mapping.yaml','routes/mapping_routes.yaml','sensors.yaml','local_estimator.yaml','icp_odometry.yaml','localization_policy.yaml','s06_dependency_lock.json']]
    files+=[BASE/'ws/install/map_manager/lib/map_manager'/n for n in ['mapping_manager','graph_probe']]
    files+=[BASE/'ws/src/sensor_sim_adapter/scripts/fanout.py',BASE/'results/S05/latest.json']
    return [dict(path=str(p.relative_to(ROOT)),sha256=digest(p)) for p in files]
