"""Teaching of measured encoder mapping, calibration and Ackermann curvature."""
import math,sys
from pathlib import Path
base=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(base/'ws/src/chassis_bridge/scripts'))
sys.path.insert(0,str(base/'ws/src/robot_description/scripts'))
from kinematics import limit,steering,curvature,twist
from generate import config
def main():
    d=config('diff');r=d['wheel']['radius'];T=d['calibration']['effective_track_m'];v,w=.6,.2
    velocities=[(v-w*T/2)/r,(v+w*T/2)/r];vv,ww=twist(velocities,[0,0],d)
    assert abs(vv-v)<1e-12 and abs(ww-w)<1e-12
    a=config('ackermann');angles=steering(.5,a);assert abs(curvature(*angles,a)-.5)<1e-12
    assert limit(0,1,a)[:2]==(0.,0.) and limit(float('nan'),1,a)[:2]==(0.,0.)
    print('PASS differential inverse:',vv,ww,'Ackermann steering:',angles,'invalid / stationary yaw rejected')
if __name__=='__main__':main()
