"""Teaching counterpart: rigid correction and explicit version binding, no ROS."""
import hashlib,json,math
import numpy as np
from scipy.spatial.transform import Rotation

def se2(x,y,yaw):
    t=np.eye(4);t[:3,:3]=Rotation.from_euler('z',yaw).as_matrix();t[:2,3]=[x,y];return t

def revision(value):return hashlib.sha256(json.dumps(value,sort_keys=True,separators=(',',':')).encode()).hexdigest()

def main():
    odom=se2(1,0,.1);observed=se2(4,2,.2);correction=observed@np.linalg.inv(odom)
    assert np.allclose(correction@odom,observed);print('T_map_odom translation:',correction[:3,3])
    manifest={'map_id':'example','frame_epoch':'fixed-1'};old=revision(manifest);manifest['frame_epoch']='fixed-2'
    assert old!=revision(manifest);print('old plan rejected after frame revision')
    count=0
    for valid in [True,False,True,True,True]:
        count=count+1 if valid else 0;print('valid/count/allowed:',valid,count,count>=3)
    assert count==3;print('PASS; production additionally checks time span, covariance, stationary state, freshness and ownership')
if __name__=='__main__':main()
