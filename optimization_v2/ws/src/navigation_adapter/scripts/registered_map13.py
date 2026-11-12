"""Version-bound local frozen-map registration, fitted on separate observations.
Does NOT change GNSS values, S08 innovation thresholds, or the original database.
"""
from common13 import *
import argparse,signal,copy
import rclpy
from std_msgs.msg import String
from scipy.spatial.transform import Rotation

def validate_registration():
 p=BASE/'config/connector_registration.json';c=read(p)
 if not c['passed'] or c['source_map_revision']!=read(BASE/'maps/frozen/manifest.json')['revision'] or c['gnss_alignment_sha256']!=sha(BASE/'config/gnss_alignment.yaml'):raise ValueError('REGISTRATION_VERSION_REJECTED')
 for sample in c['train']+c['heldout']:
  if sha(ROOT/sample['source']/'sources/events.jsonl')!=sample['source_sha256']:raise ValueError('REGISTRATION_OBSERVATIONS_CHANGED')
 if max(c['heldout_residuals_m'])>=.08:raise ValueError('REGISTRATION_HELD_OUT_ERROR')
 return c

def main():
 ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path,required=True);a=ap.parse_args();a.out.mkdir(parents=True,exist_ok=True);c=validate_registration();fingerprint=sha(BASE/'config/connector_registration.json');h=c['rotation_yaw_rad'];rot=Rotation.from_euler('z',h);R=rot.as_matrix();tr=np.r_[c['translation_xy_m'],0.];A=np.zeros((6,6));A[:3,:3]=R;A[3:,3:]=R
 rclpy.init();n=rclpy.create_node('s13_registered_map');pub=n.create_publisher(String,'/s08/map',20);stop=[False];f=(a.out/'events.jsonl').open('w')
 def receive(m):
  d=json.loads(m.data);d['registration_revision']=fingerprint;d['raw_frozen_map_pose']=d.get('pose');valid=sha(BASE/'config/connector_registration.json')==fingerprint
  if d.get('pose'):
   p=d['pose'];xy=R@np.array(p[:3])+tr;lo,hi=np.array(c['site_extent_xy']);valid=valid and np.all(xy[:2]>=lo-1.5) and np.all(xy[:2]<=hi+1.5)
   d['pose']=[*xy.tolist(),*(rot*Rotation.from_quat(p[3:])).as_quat().tolist()];cov=A@np.array(d['covariance']).reshape(6,6)@A.T
   cov[0,0]+=c['additional_position_sigma_m']**2;cov[1,1]+=c['additional_position_sigma_m']**2;cov[5,5]+=c['additional_yaw_sigma_rad']**2;d['covariance']=cov.reshape(-1).tolist()
  if not valid:d.update(valid=False,reason='REGISTRATION_STALE_OR_OUTSIDE_CALIBRATED_TILE')
  message=String(data=json.dumps(d));pub.publish(message);f.write(message.data+'\n');f.flush()
 n.create_subscription(String,'/s13/unregistered_map',receive,20)
 def end(s,f):stop[0]=True
 for sig in [signal.SIGINT,signal.SIGTERM]:signal.signal(sig,end)
 (a.out/'ready').write_text(fingerprint)
 try:
  while not stop[0]:rclpy.spin_once(n,timeout_sec=.02)
 finally:f.close();n.destroy_node();rclpy.try_shutdown()
if __name__=='__main__':main()
