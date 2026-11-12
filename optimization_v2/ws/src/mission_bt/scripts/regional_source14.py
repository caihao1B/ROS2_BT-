"""GNSS in declared ENU coordinates; uncertainty of map fit stays on map side.
Reuses original S08 quality/fault/contract handling and the unchanged supervisor.
"""
from common14 import *
from binding14 import region,ATLAS
sys.path.insert(0,str(BASE/'ws/src/gnss_supervisor/scripts'))
import source_adapter
from alignment import geo

def main():
    atlas,_=region();fingerprint=sha(ATLAS)
    alignment=dict(datum_llh=atlas['datum_llh'],antenna_base_m=atlas['antenna_base_m'])
    def convert(llh,cov,a):
        if sha(ATLAS)!=fingerprint:raise ValueError('REGIONAL_ATLAS_CHANGED_RESTART_REQUIRED')
        llh=np.asarray(llh);C=np.asarray(cov).reshape(3,3)
        if not np.isfinite(llh).all() or not (-90<llh[0]<90 and -180<=llh[1]<=180):raise ValueError('invalid GNSS coordinates')
        if not np.isfinite(C).all() or not np.allclose(C,C.T,atol=1e-9) or np.linalg.eigvalsh(C).min()<=0:raise ValueError('invalid GNSS covariance')
        point=geo.to_enu(llh,atlas['datum_llh'])-np.array(atlas['site_origin'])
        J=geo.enu_rotation(atlas['datum_llh'])@geo.enu_rotation(llh).T
        return point,J@C@J.T+np.eye(3)*atlas['datum_translation_sigma_m']**2
    source_adapter.validate=lambda:alignment;source_adapter.convert=convert;source_adapter.main()
if __name__=='__main__':main()
