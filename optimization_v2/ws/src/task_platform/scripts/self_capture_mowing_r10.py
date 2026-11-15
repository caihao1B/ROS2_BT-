"""Remove known robot returns in their own capture frame, before ego compensation.

Exactly the original R09 mask and low-ground exception; no new free-space cells
and no enlargement of the masked volume.
"""
import numpy as np

def keep_environment(body_points,terrain):
    p=np.asarray(body_points);x,y,z=p[:,0],p[:,1],p[:,2]
    lo,hi,left,right=terrain['self_mask'];m=terrain['self_uncertainty_margin_m']
    padded=(x>lo)&(x<hi)&(y>left)&(y<right)
    physical=(x>lo+m)&(x<hi-m)&(y>left+m)&(y<right-m)
    own=(z<terrain['self_z_max'])&(physical|(padded&(z>.08)))
    return ~own
