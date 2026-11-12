"""Bounded, independent stationary GNSS / frozen-SLAM correspondences.
Fit uses observed messages only, never fixture coordinates or simulator truth.
"""
from common13 import *
import argparse
from scipy.spatial.transform import Rotation

def sample(out):
 r=rows(out/'sources/events.jsonl');maps=[x for x in r if x.get('kind')=='map' and x.get('valid') and 15<=x['t']<=39];gps=[x for x in r if x.get('kind')=='gnss' and x.get('valid') and x.get('mode')=='fixed' and 15<=x['t']<=39]
 if len(maps)<15 or len(gps)<70:raise ValueError('INSUFFICIENT_INDEPENDENT_OBSERVATIONS '+str(out))
 a=[];b=[]
 for d in maps:
  g=min(gps,key=lambda x:abs(x['t']-d['t']))
  if abs(g['t']-d['t'])>.15:continue
  p=d['pose'];antenna=Rotation.from_quat(p[3:]).apply(g['antenna_base_m']);a.append(p[:2]);b.append((np.array(g['antenna'])-antenna)[:2])
 if len(a)<15:raise ValueError('TIMESTAMP_MATCH_INSUFFICIENT')
 a=np.array(a);b=np.array(b);return dict(source=str(out.relative_to(ROOT)),source_sha256=sha(out/'sources/events.jsonl'),samples=len(a),map_xy=np.median(a,axis=0).tolist(),site_xy=np.median(b,axis=0).tolist(),map_scatter_p95_m=float(np.percentile(np.linalg.norm(a-np.median(a,axis=0),axis=1),95)),gnss_scatter_p95_m=float(np.percentile(np.linalg.norm(b-np.median(b,axis=0),axis=1),95)))

def fit(train,held):
 a=np.array([s['map_xy'] for s in train]);b=np.array([s['site_xy'] for s in train]);ac=a-a.mean(0);bc=b-b.mean(0);U,S,Vt=np.linalg.svd(ac.T@bc);R=Vt.T@U.T
 if np.linalg.det(R)<0:Vt[-1]*=-1;R=Vt.T@U.T
 t=b.mean(0)-R@a.mean(0);h=math.atan2(R[1,0],R[0,0]);res=lambda xs:[float(np.linalg.norm(R@np.array(s['map_xy'])+t-s['site_xy'])) for s in xs];tr=res(train);ho=res(held)
 passed=max(tr+ho)<.08 and abs(h)<.035 and max(s['map_scatter_p95_m'] for s in train+held)<.12
 c=dict(schema='S13-REGIONAL-REGISTRATION/1',source_map_revision=read(BASE/'maps/frozen/manifest.json')['revision'],gnss_alignment_sha256=sha(BASE/'config/gnss_alignment.yaml'),scope='connector first 6 m, local 8 m corridor only; not a full-map deformation correction',train=train,heldout=held,rotation_yaw_rad=h,translation_xy_m=t.tolist(),train_residuals_m=tr,heldout_residuals_m=ho,passed=passed,additional_position_sigma_m=.04,additional_yaw_sigma_rad=.005,source='observed fixed GNSS antenna positions minus lever arm, paired with frozen SLAM matches; no evaluation truth',site_extent_xy=[b.min(0).tolist(),b.max(0).tolist()]);return c

if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--collect',action='store_true');a=p.parse_args();base=BASE/'results/S13/registration_observations';base.mkdir(parents=True,exist_ok=True)
 if a.collect:
  import fcntl
  from runtime13 import run_case
  lock=(ROOT/'native_20261108/runs/session.lock').open('a');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
  for arc in [3,6,1.5,4.5]:
   out=base/str(arc)
   if (out/'runtime.json').exists() and read(out/'runtime.json')['success']:continue
   r=run_case(out,'connector','calibration_'+str(arc),48,160,41,False,'diff','DWB')
   if not r['success']:raise ValueError('CALIBRATION_COLLECTION_FAILED')
 c=fit([sample(BASE/'results/S13/dev_inspection_diff1'),sample(base/'3'),sample(base/'6')],[sample(base/'1.5'),sample(base/'4.5')]);dump(base/'registration_candidate.json',c);print(json.dumps(c,indent=2))
 if not c['passed']:raise SystemExit('HELD_OUT_REGISTRATION_REJECTED')
 dump(BASE/'config/connector_registration.json',c)
