"""Bounded point-to-point ICP on elevated surfaces, held-out geometry validation.
Reference KD-tree is built once. This refines an RTAB candidate; never searches
arbitrary global positions and never reads simulator truth.
"""
import numpy as np
from scipy.spatial.transform import Rotation

def refine(raw,position,quaternion,tree,max_translation=.15,max_rotation=.035):
    rotation=Rotation.from_quat(quaternion).as_matrix();position=np.asarray(position,dtype=float);world=raw@rotation.T+position;fit=world[::2].copy();total_r=np.eye(3);total_t=np.zeros(3)
    if len(fit)<100 or len(world[1::2])<100:return None,dict(reason='INSUFFICIENT_HELD_OUT_SURFACES')
    for iteration in range(15):
        distance,index=tree.query(fit);mask=distance<.60
        if mask.sum()<100:return None,dict(reason='INSUFFICIENT_REFINEMENT_CORRESPONDENCES')
        a=fit[mask];b=tree.data[index[mask]];ca=a.mean(axis=0);cb=b.mean(axis=0);u,s,vh=np.linalg.svd((a-ca).T@(b-cb))
        if s[1]/max(s[0],1e-12)<1e-3:return None,dict(reason='DEGENERATE_REFINEMENT_GEOMETRY')
        update=vh.T@u.T
        if np.linalg.det(update)<0:vh[-1]*=-1;update=vh.T@u.T
        shift=cb-update@ca;fit=fit@update.T+shift;total_r=update@total_r;total_t=update@total_t+shift
        if np.linalg.norm(update@ca+shift-ca)<1e-5 and np.linalg.norm(Rotation.from_matrix(update).as_rotvec())<1e-5:break
    new_position=total_r@position+total_t;new_rotation=total_r@rotation;translation=float(np.linalg.norm(new_position-position));angle=float(np.linalg.norm(Rotation.from_matrix(total_r).as_rotvec()));details=dict(translation_m=translation,rotation_rad=angle,iterations=iteration+1,fit_points=len(fit),validation_points=len(world[1::2]))
    if translation>max_translation or angle>max_rotation:return None,dict(**details,reason='REFINEMENT_EXCEEDS_SMALL_CORRECTION_BOUND')
    validation=world[1::2]@total_r.T+total_t;distance=tree.query(validation)[0];inlier=distance[distance<.35]
    residual=float(np.mean(inlier**2)) if len(inlier) else 9999.;radius2=max(1.,float(np.mean(np.sum(raw[1::2]**2,axis=1))))
    return dict(position=new_position,quaternion=Rotation.from_matrix(new_rotation).as_quat(),distance=distance,rotation=total_r,variance_floor=residual,angular_variance_floor=residual/radius2),dict(**details,reason='BOUNDED_REFINEMENT')
