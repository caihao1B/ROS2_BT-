"""Stage-local mathematics and paths. Online callers never read world/evaluation files."""
import json,math
from pathlib import Path
import numpy as np
import yaml
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[5];BASE=ROOT/'optimization_v2';PKG=Path(__file__).resolve().parents[1]
def read(p):return json.loads(Path(p).read_text())
def dump(p,d):
    p=Path(p);p.parent.mkdir(parents=True,exist_ok=True);q=p.with_name(p.name+'.tmp');q.write_text(json.dumps(d,ensure_ascii=False,indent=2)+'\n');q.replace(p)
def policy():return yaml.safe_load((BASE/'config/localization_policy.yaml').read_text())
def stamp(m):return m.header.stamp.sec+m.header.stamp.nanosec*1e-9
def set_stamp(m,t):m.header.stamp.sec,m.header.stamp.nanosec=divmod(round(t*1e9),10**9)
def quat(m):return np.array([m.x,m.y,m.z,m.w])
def euler(q):return Rotation.from_quat(q).as_euler('xyz')
def wrap(x):return (x+math.pi)%(2*math.pi)-math.pi
def pose(m):return np.array([m.pose.pose.position.x,m.pose.pose.position.y,m.pose.pose.position.z]),quat(m.pose.pose.orientation)
def validate_config(c,icp,tf):
    p=c['s05_fused']['ros__parameters'];errors=[]
    if p['two_d_mode'] or p['world_frame']!='odom':errors.append('local estimator must retain 3D and use odom')
    if p['publish_tf'] or icp['s05_icp']['ros__parameters']['publish_tf']:errors.append('TF ownership conflict')
    for k in ['guess_frame_id','ground_truth_frame_id']:
        if icp['s05_icp']['ros__parameters'][k]:errors.append('ICP must not use external pose guess/truth')
    if icp['s05_icp']['ros__parameters']['wait_imu_to_init']:errors.append('ICP must be pure lidar')
    if any(p['twist0_config'][:6]) or any(p['twist1_config'][:6]) or any(p['twist1_config'][12:]):errors.append('duplicate pose/twist or absolute ICP feedback')
    if {p.get(k) for k in ['twist0','twist1','imu0']}!={'/s05/inputs/wheel','/s05/inputs/icp_increment','/s05/inputs/imu'}:errors.append('unexpected/duplicate estimator source')
    if sum(k.startswith(('odom','twist','pose','imu')) and k[-1:].isdigit() for k in p)!=3:errors.append('extra or duplicate source')
    if p['imu0_config'][5] or any(p['imu0_config'][12:]):errors.append('unobserved yaw or gravity-uncorrected acceleration')
    if tf['dynamic']!={'odom_to_base_link':'s05_local_odometry'}:errors.append('wrong TF owner')
    for name in ['process_noise_covariance','initial_estimate_covariance']:
        values=np.array(p.get(name,[]))
        if values.size!=225 or not np.isfinite(values).all() or (values.size==225 and np.linalg.eigvalsh(values.reshape(15,15)).min()<=0):errors.append('explicit positive covariance required for reset: '+name)
    if errors:raise ValueError('; '.join(errors))
    return True
