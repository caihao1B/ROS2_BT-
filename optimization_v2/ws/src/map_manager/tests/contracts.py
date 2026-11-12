"""Boundary tests for actual map admission, temporal filtering and installed ABI."""
import sys,tempfile,subprocess
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from s06_common import *
from mapping_export import safe,verify_manifest
from mapping_inputs import temporal_filter
from mapping_analysis import ghosting
from collections import deque
def main():
    for package,version in read(BASE/'config/s06_dependency_lock.json')['packages'].items():
        actual=subprocess.check_output(['dpkg-query','-W','-f=${Version}',package],text=True);assert version and actual==version,(package,actual,version)
    with tempfile.TemporaryDirectory(dir=BASE/'runtime',prefix='s06_contract_') as tmp:
        root=Path(tmp);f=root/'map.db';f.write_bytes(b'admission-test-only; not a mapping result')
        relative=str(f.relative_to(ROOT));manifest=root/'manifest.json';dump(manifest,dict(schema='S06-MAPPING/1',bindings=[dict(path=relative,sha256=digest(f))]));assert verify_manifest(manifest)
        f.write_bytes(b'changed')
        try:verify_manifest(manifest)
        except ValueError:pass
        else:raise AssertionError('changed map accepted')
        for path in ['../map.db','/tmp/map.db']:
            try:safe(root,path)
            except ValueError:pass
            else:raise AssertionError('unsafe path accepted')
    stable=np.array([[2.,0.,1.],[2.,.3,1.],[2.,.6,1.]])
    history=deque(maxlen=5)
    for t in [0.,.5]:temporal_filter(stable,[0,0,0],[0,0,0,1],history,t)
    points=np.vstack([stable,[[6.,0.,1.],[0.,2.,0.]]]);filtered,_,mask=temporal_filter(points,[0,0,0],[0,0,0,1],history,1.)
    assert mask.tolist()==[True,True,True,False,True]
    p=yaml.safe_load((BASE/'config/slam_mapping.yaml').read_text())['/**']['ros__parameters'];assert p['subscribe_rgbd'] and p['subscribe_scan_cloud'] and not p['subscribe_stereo'];assert p['Reg/Force3DoF']=='false' and p['Mem/IncrementalMemory']=='true'
    assert p['Mem/UseOdomGravity']=='true' and float(p['Optimizer/GravitySigma'])>0
    # Metric boundary fixture only, not a claimed sensor experiment. Zero overlap
    # must be unavailable rather than a zero error or quantile(empty) exception.
    points=np.array([[i*.05,j*.05,1.] for i in range(10) for j in range(10)])
    graph=dict(loop_scans={'1':points.tolist(),'2':points.tolist()},nodes=[dict(id=1,raw=[0,0,0,0,0,0,1],optimized=[0,0,0,0,0,0,1]),dict(id=2,raw=[100,0,0,0,0,0,1],optimized=[0,0,0,0,0,0,1])])
    assert ghosting(graph,'raw')['available'] is False and ghosting(graph,'raw')['overlap_fraction']==0
    assert ghosting(graph,'optimized')['available'] and ghosting(graph,'optimized')['p95_m']<1e-9
    print('PASS: installed versions, map mutation, traversal, observed temporal filtering, stereo+scan interface, empty-overlap metric boundary')
if __name__=='__main__':main()
