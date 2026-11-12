"""Observable alignment, degenerate geometry, and immutable obligation guards."""
import sys,copy,tempfile
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from common14 import *
from regional_math14 import SurfaceMap,rotation,rigid_fit
from binding14 import verify_derived,canonical,atlas,PLAN

def reject(fn):
    try:fn()
    except ValueError:return
    raise AssertionError('Expected closed rejection')

def main():
    rng=np.random.default_rng(41);z=rng.uniform(.4,3,1000);along=rng.uniform(-3,3,1000)
    points=np.vstack([np.c_[np.full(1000,4.),along,z],np.c_[along,np.full(1000,5.),z],np.c_[np.full(1000,-4.),along,z]])
    r=rotation(.12);t=np.array([.3,-.2]);raw=points.copy();raw[:,:2]=(raw[:,:2]-t)@r;raw+=rng.normal(0,.006,raw.shape)
    reference=SurfaceMap(points);reference.prepare_normals();fit=reference.align(raw,[0.,0.,0.])
    assert fit['valid'] and math.dist(fit['pose'][:2],t)<.02 and abs(fit['pose'][2]-.12)<.005,fit
    plane=SurfaceMap(points[:1000]);plane.prepare_normals();assert not plane.align(raw[:1000],[0.,0.,0.])['valid']
    reject(lambda:rigid_fit([[0,0],[1,0],[2,0],[3,0]],[[0,0],[1,0],[2,0],[3,0]]))
    import binding14
    with tempfile.TemporaryDirectory() as temporary:
        f=Path(temporary)/'bound_input';f.write_text('verified');old_region=os.environ.get('S14_REGISTERED_REGION');os.environ['S14_REGISTERED_REGION']='signature_test'
        try:
            binding14._PLAN_CACHE['signature_test']=({'sentinel':True},binding14.signatures([f]))
            assert binding14.active_plan()['sentinel']
            f.write_text('modified');reject(binding14.active_plan)
            f.unlink();reject(binding14.active_plan)
        finally:
            binding14._PLAN_CACHE.pop('signature_test',None);os.environ.pop('S14_REGISTERED_REGION',None)
            if old_region is not None:os.environ['S14_REGISTERED_REGION']=old_region
    original=plan();a=atlas();d=read(PLAN);verify_derived(d,original,a,'A')
    from contracts12 import verify_selected
    from common12 import config
    geometry=verify_selected(d['zones'],config())
    assert geometry['passed'],geometry
    for field in ['epoch','area','point','atlas','map_revision','calibration','scope']:
        bad=copy.deepcopy(d)
        if field=='epoch':bad['frame']['frame_epoch']='old'
        if field=='area':bad['zones']['B']['plans'][0]['coverage_check']['domain_area_m2']-=1
        if field=='point':bad['zones']['A']['plans'][0]['points']=[]
        if field=='atlas':bad['atlas_revision']='old'
        if field=='map_revision':bad['frame']['map_revision']='old'
        if field=='calibration':bad['frame']['calibration_revision']='old'
        if field=='scope':bad['runtime_eligible_clusters'].append('A2')
        bad.pop('plan_revision');bad['plan_revision']=canonical(bad);reject(lambda:verify_derived(bad,original,a,'A'))
    print(json.dumps(dict(surface_recovery=fit,degenerate_rejected=True,old_epoch_rejected=True,original_denominators_preserved=True,recomputed_geometry_kinematics_time=geometry,atlas_heldout={k:v['heldout_residuals_m'] for k,v in a['regions'].items()})))
if __name__=='__main__':main()
