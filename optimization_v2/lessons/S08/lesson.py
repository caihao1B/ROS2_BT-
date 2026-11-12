"""Teaching example calls actual ENU implementation, not a second estimator."""
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'ws/src/gnss_supervisor/scripts'))
from alignment import *
a=validate();llh=geo.from_enu([30,15,1],a['datum_llh']);p,C=convert(llh,np.diag([.0004,.0004,.0016]),a)
print('map antenna / m:',np.round(p,6),'cov / m²:',C.diagonal());assert np.allclose(p,[2,1,1],atol=1e-6)
try:convert(llh,np.diag([-1,1,1]),a)
except ValueError as e:print('invalid covariance rejected:',e)
else:raise AssertionError('negative covariance accepted')
