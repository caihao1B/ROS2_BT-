"""Offline integration: actual adapted import and generated Gazebo spawn."""
import sys,os,tempfile,math,xml.etree.ElementTree as E
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from common_showcase import *
from showcase_resume import prepare
from showcase_overlay import compose
previous=BASE/'results/S17/showcase_A1_inspection_20261113_204730'
if not previous.exists():
    print('SKIP local captured checkpoint not bundled');raise SystemExit(0)
os.environ.update(SHOWCASE_PLAN=str(OUT/'v1/mission_plan.json'),S17_OPERATION='inspection',S17_CLUSTER='A1',S14_REGISTERED_REGION='C',S17_CONTROLLER='DWB',S17_PREDICTION='disabled')
with tempfile.TemporaryDirectory(dir=BASE/'results/S17') as directory:
    out=Path(directory);manifest=prepare(previous,out,'A1');os.environ['S17_RESUME_MANIFEST']=str(manifest)
    overlay=compose(out/'overlay');sys.path.insert(0,str(overlay))
    from mission14 import inspection_execute
    expected_file=out/'overlay/showcase_source/scripts/showcase_inspection17.py'
    assert Path(inspection_execute.__code__.co_filename)==expected_file
    from scene14 import compose as scene
    scene(out/'scene','diff','nominal_spray_cluster','showcase_restore_contract')
    actual=list(map(float,E.parse(out/'scene/world.sdf').find("./world/model[@name='vehicle']/pose").text.split()))
    expected=read(manifest)['restart_pose_prior'];assert math.dist([actual[0],actual[1],actual[5]],expected)<1e-10
    dump(OUT/'inspection_spawn_contract.json',dict(passed=True,actual=[actual[0],actual[1],actual[5]],expected=expected,isolated_inspection_executor=True,source_sha256=sha(PKG/'scripts/showcase_overlay.py')))
    print('PASS actual isolated executor import and Gazebo checkpoint spawn')
