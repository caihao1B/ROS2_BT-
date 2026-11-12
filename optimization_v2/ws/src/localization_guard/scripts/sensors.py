"""Trusted simulation-side extension of frozen S04 errors; no oracle to estimators."""
import argparse,signal,sys
from support import BASE,Path
sys.path.insert(0,str(BASE/'ws/src/sensor_sim_adapter/scripts'))
from adapter import Adapter,stamp,np
class FaultAdapter(Adapter):
    def __init__(self,*args,faults=False,imu_only=False,**kwargs):self.faults=faults;self.imu_only=imu_only;super().__init__(*args,**kwargs)
    def enqueue(self,k,m,t,size=500):
        if k=='imu' and ((self.faults and 86<=t<94) or (self.imu_only and 20<=t<28)):self.drops[k]+=1;self.audit(k,t,event='injected_dropout');return
        if self.faults and k=='points' and 65<=t<79:
            data=np.frombuffer(m.data,dtype=[('x','<f4'),('y','<f4'),('z','<f4'),('intensity','<f4'),('ring','<u2')]).copy();mask=data['z']>-.60
            for axis in ['x','y','z']:data[axis][mask]=np.nan
            m.data=data.tobytes();self.audit(k,t,event='ground_only_injection',removed=int(mask.sum()))
        super().enqueue(k,m,t,size)
def main():
    p=argparse.ArgumentParser();p.add_argument('--vehicle',type=Path,required=True);p.add_argument('--out',type=Path,required=True);p.add_argument('--seed',type=int,default=41);p.add_argument('--streams',required=True);p.add_argument('--faults',action='store_true');p.add_argument('--imu-only',action='store_true');a=p.parse_args()
    profile=[dict(name='wheel_slip',start=20,end=50,wheel_scale_error=.30),dict(name='gyro_bias',start=52,end=61,gyro_z_bias_rad_s=.04)] if a.faults else []
    node=FaultAdapter(a.vehicle,a.out,a.seed,profile,a.streams,faults=a.faults,imu_only=a.imu_only);stop=[False]
    def end(s,f):stop[0]=True
    signal.signal(signal.SIGINT,end);signal.signal(signal.SIGTERM,end)
    try:
        while not stop[0]:node.executor.spin_once(timeout_sec=.01)
    finally:node.finish()
if __name__=='__main__':main()
