"""Candidate-only soft future occupancy; never clears observed/unknown cells."""
import math,numpy as np

def apply(cost,x,y,packet,now):
    if not packet or packet.get('frame')!='odom' or packet.get('status')!='OK' or not 0<=now-packet.get('t',-100)<.6:return 0
    marked=np.zeros(cost.shape,bool)
    for obj in packet.get('forecasts',[]):
        age=now-obj.get('t',-100);dt=obj.get('dt',0)
        if not 0<=age<=.6 or not 0<dt<=1:continue
        mass=np.zeros(cost.shape,float)
        for mode in obj.get('modes',[]):
            probability=mode.get('probability',0)
            if not math.isfinite(probability) or not 0<=probability<=1:continue
            swept=np.zeros(cost.shape,bool)
            for i,(mean,raw_cov) in enumerate(zip(mode['mean'],mode['covariance'])):
                if not 0<=dt*(i+1)-age<=2.:continue
                covariance=np.asarray(raw_cov,float)
                if covariance.shape!=(2,2) or not np.isfinite(covariance).all() or not np.allclose(covariance,covariance.T,atol=1e-6):continue
                eigen=np.linalg.eigvalsh(covariance)
                if eigen[0]<-1e-6 or eigen[-1]>4:continue
                radius=.5*math.hypot(*obj['size'][:2])+.15+2.4477*math.sqrt(max(0,eigen[-1]))+dt
                center=np.asarray(mean,float)
                if center.shape!=(2,) or not np.isfinite(center).all():continue
                ix=np.flatnonzero(abs(x-center[0])<=radius);iy=np.flatnonzero(abs(y-center[1])<=radius)
                if len(ix) and len(iy):swept[np.ix_(iy,ix)]|=(x[ix][None,:]-center[0])**2+(y[iy][:,None]-center[1])**2<=radius**2
            mass+=probability*swept
        selected=(mass>=.15)&(cost>=0)&(cost<90)
        cost[selected]=np.maximum(cost[selected],np.rint(60+30*np.minimum(1,mass[selected])).astype(cost.dtype));marked|=selected
    return int(marked.sum())
