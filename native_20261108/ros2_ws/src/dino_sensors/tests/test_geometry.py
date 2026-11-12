#!/usr/bin/env python3
"""Independent geometry and covariance checks, fixed seed, no ROS/Gazebo."""
import math,pathlib,sys,unittest
import numpy as np
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parents[1]/'scripts'))
from sensor_math import rotation,inside,rear_to_base,shift_pose_covariance,shift_twist_covariance

class GeometryTests(unittest.TestCase):
    def test_rotation_and_invalid_quaternion(self):
        q=[0,0,math.sqrt(.5),math.sqrt(.5)]
        np.testing.assert_allclose(rotation(q)@np.array([1,0,0]),[0,1,0],atol=1e-12)
        for q in ([0,0,0,0],[0,0,0,float('nan')]):
            with self.assertRaises(ValueError):rotation(q)

    def test_convex_padding_does_not_erase_distant_obstacle(self):
        s={'type':'convex','planes':[[1,0,0,-1],[-1,0,0,-1],[0,1,0,-1],[0,-1,0,-1],[0,0,1,-1],[0,0,-1,-1]]}
        np.testing.assert_array_equal(inside(np.array([[0,0,0],[1.02,0,0],[1.05,0,0],[0,0,2]]),s,.03),[True,True,False,False])

    def test_tyre_axis_and_steered_coordinates(self):
        s={'type':'wheel_y','radius':.3315,'width':.168}
        points=np.array([[.3,0,0],[0,.2,0],[0,0,.4],[0,.08,0]])
        np.testing.assert_array_equal(inside(points,s,0),[True,False,False,True])
        rot=rotation([0,0,math.sin(.2),math.cos(.2)])
        np.testing.assert_allclose((points@rot.T)@rot,points,atol=1e-12)

    def test_turning_base_velocity_is_not_rear_lateral_zero(self):
        a=.819;w=.125;dt=1e-5
        first=np.array(rear_to_base(0,0,0,a)[:2]);second=np.array(rear_to_base(.5*dt,0,w*dt,a)[:2])
        np.testing.assert_allclose((second-first)/dt,[.5,a*w],atol=1e-6)

    def test_pose_covariance_matches_seeded_rigid_body_motion(self):
        rng=np.random.default_rng(20261023);a=.819;yaw=.7
        cov=np.diag([1e-5,2e-5,0,0,0,3e-5]);samples=rng.multivariate_normal(np.zeros(6),cov,120000)
        xy=np.array([rear_to_base(x,y,yaw+h,a)[:2] for x,y,_,__,___,h in samples])
        expected=shift_pose_covariance(cov,yaw,a)[:2,:2]
        np.testing.assert_allclose(np.cov(xy.T),expected,rtol=.025,atol=2e-7)

    def test_twist_covariance_contains_lever_arm_correlation(self):
        cov=np.diag([.01,.02,0,0,0,.03]);result=shift_twist_covariance(cov,.819)
        self.assertAlmostEqual(result[1,1],.02+.819**2*.03)
        self.assertAlmostEqual(result[1,5],.819*.03)
        self.assertGreaterEqual(np.linalg.eigvalsh(result).min(),-1e-12)

if __name__=='__main__':unittest.main(verbosity=2)
