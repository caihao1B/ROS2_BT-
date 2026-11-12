"""Pure geometry used by the self filter; no ROS or evaluation truth."""
import math
import numpy as np

def rotation(q):
    x,y,z,w=q;n=x*x+y*y+z*z+w*w
    if not math.isfinite(n) or abs(n-1)>1e-3:raise ValueError('invalid quaternion')
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])

def inside(points,shape,padding):
    if shape['type']=='convex':
        eq=np.asarray(shape['planes']);return np.all(points@eq[:,:3].T+eq[:,3]<=padding,axis=1)
    if shape['type']=='wheel_y':
        return ((points[:,0]**2+points[:,2]**2)<=(shape['radius']+padding)**2)&(np.abs(points[:,1])<=shape['width']/2+padding)
    raise ValueError('unknown self-filter geometry')

def rear_to_base(x,y,yaw,a):return x+a*math.cos(yaw),y+a*math.sin(yaw),yaw

def shift_pose_covariance(cov,yaw,a):
    j=np.eye(6);j[0,5]=-a*math.sin(yaw);j[1,5]=a*math.cos(yaw)
    return j@np.asarray(cov).reshape(6,6)@j.T

def shift_twist_covariance(cov,a):
    j=np.eye(6);j[1,5]=a
    return j@np.asarray(cov).reshape(6,6)@j.T
