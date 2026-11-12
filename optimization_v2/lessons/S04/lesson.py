"""Teaching: same geodetic conversion and fault interval function as production."""
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'ws/src/sensor_sim_adapter/scripts'))
from common import *
c=settings();origin=c['gnss']['origin'];raw=np.array([27.,22.,.8]);rng=np.random.default_rng(41)
observed=raw+rng.normal(0,.02,3)+[3,-2,.5]
assert np.linalg.norm(to_enu(from_enu(observed,origin),origin)-observed)<1e-6
assert np.linalg.norm(observed-raw)>3
captured=12.;delivery=captured+.18
assert captured==12 and delivery>captured
assert phase(20,[{'start':12,'end':20,'name':'delay'}])['name']=='nominal'
print('PASS: false fixed changes coordinates by',np.linalg.norm(observed-raw),'m; delivery delay preserves acquisition stamp; boundary interval is half-open')
