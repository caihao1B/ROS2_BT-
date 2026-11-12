"""WGS84/ECEF/ENU; reuse S04 geodesy, never evaluate robot truth here."""
from common08 import *
import importlib.util
spec=importlib.util.spec_from_file_location('s04_geodesy',BASE/'ws/src/sensor_sim_adapter/scripts/common.py');geo=importlib.util.module_from_spec(spec);spec.loader.exec_module(geo)
def solve(a):
    src=np.array([c['enu'] for c in a['control_points']]);dst=np.array([c['map'] for c in a['control_points']]);x=src[:,:2]-src[:,:2].mean(0);y=dst[:,:2]-dst[:,:2].mean(0)
    if len(src)<3 or np.linalg.svd(x,compute_uv=False)[1]<1.:raise ValueError('survey controls are insufficient or collinear')
    u,s,vt=np.linalg.svd(x.T@y);r=vt.T@u.T
    if np.linalg.det(r)<0:vt[-1]*=-1;r=vt.T@u.T
    R=np.eye(3);R[:2,:2]=r;translation=dst.mean(0)-R@src.mean(0)
    if np.max(np.linalg.norm(src@R.T+translation-dst,axis=1))>.05:raise ValueError('survey residual > 5 cm')
    return R,translation

def convert(llh,cov,a):
    if not np.isfinite(llh).all() or not (-90<llh[0]<90 and -180<=llh[1]<=180):raise ValueError('invalid geodetic coordinates')
    C=np.asarray(cov).reshape(3,3)
    if not np.isfinite(C).all() or not np.allclose(C,C.T,atol=1e-9) or np.linalg.eigvalsh(C).min()<=0:raise ValueError('invalid covariance')
    R,tr=solve(a);enu=geo.to_enu(llh,a['datum_llh']);p=R@enu+tr
    # NavSatFix covariance is in tangent ENU at FIX, not at datum.
    J=R@geo.enu_rotation(a['datum_llh'])@geo.enu_rotation(llh).T
    C=J@C@J.T+np.eye(3)*a['translation_sigma_m']**2
    d=np.array([-p[1],p[0],0.]);C+=np.outer(d,d)*a['yaw_sigma_rad']**2
    return p,C
