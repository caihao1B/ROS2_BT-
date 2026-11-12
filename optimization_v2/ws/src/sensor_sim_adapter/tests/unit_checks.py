"""Numerical units, seeded noise boundaries, optical geometry and bounded contracts."""
import json,math,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[5];sys.path.insert(0,str(ROOT/'optimization_v2/ws/src/sensor_sim_adapter/scripts'))
from common import *
def run():
    c=settings();origin=c['gnss']['origin']
    for xyz in [[0,0,0],[100,50,2],[-50,80,-3]]:assert np.linalg.norm(to_enu(from_enu(np.array(xyz),origin),origin)-xyz)<1e-6
    a=np.random.default_rng(41).normal(0,.02,10000);b=np.random.default_rng(41).normal(0,.02,10000);assert np.array_equal(a,b);assert .019<float(a.std())<.021
    profile=yaml.safe_load((BASE/'config/fault_profiles.yaml').read_text())['sensor_faults'];assert phase(12,profile)['name']=='transport_delay' and phase(20,profile)['name']=='nominal'
    assert phase(63,profile)['gnss_mode']=='false_fixed' and np.linalg.norm(phase(63,profile)['gnss_bias_enu_m'])>3
    fx=640/(2*math.tan(c['stereo']['hfov_rad']/2));p03=-fx*c['stereo']['baseline_m'];assert abs(-p03/fx-.12)<1e-12
    # Optical +z is chassis +x; optical +x is chassis -y.
    rot=np.array([[0,0,1],[-1,0,0],[0,-1,0]]);assert np.linalg.det(rot)==1 and np.array_equal(rot@np.array([0,0,1]),[1,0,0])
    assert c['limits']['record_bytes']<c['limits']['case_disk_bytes'] and c['limits']['queue_bytes']<=33554432
    for name in ['observe.py','fanout.py','motion.py']:
        s=(PKG/'scripts'/name).read_text();assert '/s04/raw/' not in s and 'evaluation/truth' not in s and 'world.sdf' not in s
    # Original numerical self-filter/pose-shift tests are reused separately in stage runner.
    return dict(passed=True,checks=['WGS84 ENU roundtrip','seed repeatability and variance','fault boundaries','camera baseline and optical axes','bounded memory/disk','online code input boundary'])
if __name__=='__main__':print(json.dumps(run(),indent=2))
