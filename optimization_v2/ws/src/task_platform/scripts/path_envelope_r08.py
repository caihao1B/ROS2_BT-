"""Unknown outside the current physical footprint is never admitted."""
import numpy as np
def envelope_free(cost,points,pose,footprint,margin=0.):
 cost=np.asarray(cost);p=np.asarray(points)-np.asarray(pose[:2]);h=pose[2];C=np.array([[np.cos(h),np.sin(h)],[-np.sin(h),np.cos(h)]]);b=p@C.T;f=np.asarray(footprint);lo=f.min(axis=0)-margin;hi=f.max(axis=0)+margin;current=((b>=lo)&(b<=hi)).all(axis=1)
 return bool((((cost>=0)&(cost<100))|((cost<0)&current)).all())
