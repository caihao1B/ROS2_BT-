"""Known WGS84 samples, independent PROJ oracle, covariance and survey failures."""
import sys,copy,json
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from alignment import *
a=validate();checks={};p,C=convert(a['datum_llh'],np.diag([.01,.04,.09]),a);checks['datum_origin']=np.allclose(p,[-28,-14,0],atol=1e-8)
# Published WGS84 semi-major/minor axes at equator and pole.
checks['ecef_equator']=np.allclose(geo.ecef([0,0,0]),[6378137,0,0],atol=1e-6)
checks['ecef_pole']=np.allclose(geo.ecef([90,0,0]),[0,0,6356752.314245],atol=1e-6)
for point in [[30,15,1],[122,58,3],[-20,-10,0]]:
 checks['roundtrip_'+str(point)]=np.allclose(geo.to_enu(geo.from_enu(point,a['datum_llh']),a['datum_llh']),point,atol=1e-6)
b=copy.deepcopy(a);R=Rotation.from_euler('z',np.pi/2).as_matrix()
for row in b['control_points']:row['map']=(R@np.array(row['enu'])).tolist()
b['translation_sigma_m']=0.;b['yaw_sigma_rad']=0.;_,rot=convert(b['datum_llh'],np.diag([.01,.04,.09]),b);checks['covariance_90deg']=np.allclose(rot,np.diag([.04,.01,.09]),atol=1e-10)
for name,mutate in [('collinear',lambda z:z.update(control_points=[dict(enu=[i,0,0],map=[i,0,0]) for i in range(3)])),('bad_survey',lambda z:z['control_points'][0].update(map=[9,9,0]))]:
 b=copy.deepcopy(a);mutate(b)
 try:solve(b);checks[name]=False
 except ValueError:checks[name]=True
for name,cov in [('negative',np.diag([-1,1,1])),('zero',np.zeros((3,3))),('nan',np.full((3,3),np.nan))]:
 try:convert(a['datum_llh'],cov,a);checks[name]=False
 except ValueError:checks[name]=True
import subprocess
r=subprocess.run(['cs2cs','+proj=latlong','+datum=WGS84','+to','+proj=geocent','+datum=WGS84','-f','%.9f'],input='121.4737 31.2304 10\n',capture_output=True,text=True,timeout=5)
checks['independent_proj_ecef']=r.returncode==0 and np.allclose(geo.ecef(a['datum_llh']),[float(x) for x in r.stdout.split()[:3]],rtol=0,atol=1e-6)
print(json.dumps(dict(passed=all(checks.values()),checks=checks),indent=2));assert all(checks.values())
