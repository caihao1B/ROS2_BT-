"""Reject duplicate/correlated source configurations and test geometric observability."""
import copy,sys,subprocess
from pathlib import Path
ROOT=Path(__file__).resolve().parents[5];sys.path.insert(0,str(ROOT/'optimization_v2/ws/src/localization_guard/scripts'))
from support import *
from cloud_filter import geometry
def run():
    locked=read(BASE/'config/s05_dependency_lock.json')['packages']
    installed={name:subprocess.check_output(['dpkg-query','-W','-f=${Version}',name],text=True) for name in locked}
    assert installed==locked,'installed dependency versions differ from S05 lock'
    local=yaml.safe_load((BASE/'config/local_estimator.yaml').read_text());icp=yaml.safe_load((BASE/'config/icp_odometry.yaml').read_text());tf=yaml.safe_load((BASE/'config/tf_ownership.yaml').read_text());assert validate_config(local,icp,tf);rejections=[]
    for case in ['duplicate_source','pose_and_twist','external_guess','imu_icp','two_tf','flatten_slope']:
        a,b=copy.deepcopy(local),copy.deepcopy(icp);p=a['s05_fused']['ros__parameters'];q=b['s05_icp']['ros__parameters']
        if case=='duplicate_source':p['odom1']=p['twist0']
        elif case=='pose_and_twist':p['twist1_config'][0]=True
        elif case=='external_guess':q['guess_frame_id']='odom'
        elif case=='imu_icp':q['wait_imu_to_init']=True
        elif case=='two_tf':q['publish_tf']=True
        else:p['two_d_mode']=True
        try:validate_config(a,b,tf)
        except ValueError:rejections.append(case)
        else:raise AssertionError(case+' was not rejected')
    rng=np.random.default_rng(41);p=rng.uniform(-5,5,(1500,3));plane=p.copy();plane[:,2]=rng.normal(0,.001,1500);three=p.copy();three[:500,0]=3;three[500:1000,1]=3;three[1000:,2]=0
    low=geometry(plane)[0];high=geometry(three)[0];assert low<policy()['cloud']['normal_eigen_min']<high
    # A public odom must not inherit arbitrary unanchored EKF position corrections.
    from odom_output import Integrator
    from nav_msgs.msg import Odometry
    integrator=Integrator();msg=Odometry();msg.pose.pose.orientation.w=1.;msg.twist.twist.linear.x=.4
    for i in range(101):
        set_stamp(msg,i*.01);msg.pose.pose.position.x=1000. if i%2 else -1000.
        result=integrator.update(msg)
    assert abs(result.pose.pose.position.x-.4)<1e-9
    set_stamp(msg,0.);result=integrator.update(msg);assert abs(result.pose.pose.position.x)<1e-12
    return dict(passed=True,installed_versions=installed,continuous_velocity_integration_and_epoch_reset=True,rejected=rejections,plane_normal_eigen_min=low,three_planes_normal_eigen_min=high)
if __name__=='__main__':print(json.dumps(run(),indent=2))
