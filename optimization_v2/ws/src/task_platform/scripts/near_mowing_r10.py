"""R09 capture-frame self filter before unchanged near/main ego compensation."""
from pathlib import Path

def derive(source):
    replacements=[
        ('from scipy.spatial.transform import Rotation,Slerp','from scipy.spatial.transform import Rotation,Slerp\nfrom self_capture_mowing_r10 import keep_environment'),
        ("units=cfg['near_sensor']['units'];", "terrain=json.loads((Path(__file__).resolve().parents[4]/'three_tasks/editions/R10/mowing/terrain.json').read_text());units=cfg['near_sensor']['units'];"),
        ("near.popleft();parts=[];sources=[];", "near.popleft();parts=[];sources=[];self_removed=0;"),
        ("rs=ts[0]@Rotation.from_euler('xyz',[0.,unit.get('pitch',0.),unit['yaw']]).as_matrix();", "mount=Rotation.from_euler('xyz',[0.,unit.get('pitch',0.),unit['yaw']]).as_matrix();keep=keep_environment(x@mount.T+np.array(unit['xyz']),terrain);self_removed+=int((~keep).sum());x=x[keep];rs=ts[0]@mount;"),
        ("main=main[np.isfinite(main).all(axis=1)];main=", "main=main[np.isfinite(main).all(axis=1)];keep=keep_environment(main+np.array(f.c['lidar']['xyz']),terrain);self_removed+=int((~keep).sum());main=main[keep];main="),
        ("odom_motion_compensated=True,missing_far_area=", "odom_motion_compensated=True,capture_self_returns_removed=self_removed,self_filter_basis='original R09 geometry at each sensor capture before motion compensation',missing_far_area=")]
    for before,after in replacements:
        if source.count(before)!=1:raise ValueError('NEAR_DERIVATION_SOURCE_CHANGED '+before)
        source=source.replace(before,after)
    return source

if __name__=='__main__':
    template=Path(__file__).with_name('near_r08.py');exec(compile(derive(template.read_text()),str(template),'exec'),globals())
