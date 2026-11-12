"""Sensor-only SE(2) map registration. No deployment or simulator geometry input.
Coarse point correspondences, then robust point-to-plane refinement; holdout
points never enter the optimizer. Global registration uses separate stations.
"""
import numpy as np
from scipy.spatial import cKDTree

def rotation(yaw):
    c,s=np.cos(yaw),np.sin(yaw);return np.array([[c,-s],[s,c]])

class SurfaceMap:
    def __init__(self,points):
        p=np.asarray(points,dtype=float);self.points=p[np.isfinite(p).all(1)&(p[:,2]>.2)]
        self.tree=cKDTree(self.points);self.normals=None
    def prepare_normals(self):
        # Stable planar neighbourhoods; cylinder surfaces remain locally planar.
        _,ids=self.tree.query(self.points,k=12,workers=2);near=self.points[ids];d=near-near.mean(1)[:,None,:]
        vals,vec=np.linalg.eigh(np.einsum('nki,nkj->nij',d,d)/12)
        self.normals=vec[:,:,0];self.normal_valid=(vals[:,0]<.25*np.maximum(vals[:,1],1e-8))&(np.linalg.norm(self.normals[:,:2],axis=1)>.35)
    def align(self,raw,seed,coarse=True):
        raw=np.asarray(raw,dtype=float);raw=raw[np.isfinite(raw).all(1)&(raw[:,2]>.2)&(np.linalg.norm(raw,axis=1)<30)]
        if len(raw)<200:return dict(valid=False,reason='INSUFFICIENT_SURFACES')
        r=rotation(seed[2]);p=np.array(seed[:2],dtype=float);fit=raw[::2].copy();fit[:,:2]=fit[:,:2]@r.T+p
        if coarse:
            for _ in range(25):
                ds,ix=self.tree.query(fit);mask=ds<1.8
                if mask.sum()<100:return dict(valid=False,reason='NO_COARSE_MATCH')
                cut=np.quantile(ds[mask],.85);mask&=ds<=max(.15,cut)
                a,b=fit[mask,:2],self.points[ix[mask],:2];ca,cb=a.mean(0),b.mean(0);u,_,vt=np.linalg.svd((a-ca).T@(b-cb));dr=vt.T@u.T
                if np.linalg.det(dr)<0:vt[-1]*=-1;dr=vt.T@u.T
                shift=cb-dr@ca;fit[:,:2]=fit[:,:2]@dr.T+shift;p=dr@p+shift;r=dr@r
                if np.linalg.norm(shift+dr@ca-ca)<1e-5 and abs(np.arctan2(dr[1,0],dr[0,0]))<1e-5:break
        if self.normals is not None:
            for _ in range(18):
                ds,ix=self.tree.query(fit);mask=(ds<.6)&self.normal_valid[ix]
                if mask.sum()<100:return dict(valid=False,reason='INSUFFICIENT_PLANAR_MATCH')
                a,b,n=fit[mask],self.points[ix[mask]],self.normals[ix[mask]];center=a[:,:2].mean(0);delta=a[:,:2]-center
                residual=np.einsum('ij,ij->i',a-b,n);J=np.c_[n[:,:2],-n[:,0]*delta[:,1]+n[:,1]*delta[:,0]]
                weight=np.minimum(1.,.08/np.maximum(np.abs(residual),1e-9));H=J.T@(J*weight[:,None])
                if np.linalg.eigvalsh(H).min()<1e-3:return dict(valid=False,reason='DEGENERATE_SURFACE_GEOMETRY')
                step=np.linalg.solve(H,-J.T@(weight*residual));step[:2]=np.clip(step[:2],-.2,.2);step[2]=np.clip(step[2],-.03,.03)
                dr=rotation(step[2]);shift=center-dr@center+step[:2];fit[:,:2]=fit[:,:2]@dr.T+shift;p=dr@p+shift;r=dr@r
                if np.linalg.norm(step[:2])<1e-5 and abs(step[2])<1e-5:break
        test=raw[1::2].copy();test[:,:2]=test[:,:2]@r.T+p;ds=self.tree.query(test)[0];inside=ds[ds<.35]
        overlap=len(inside)/len(ds);p95=float(np.quantile(inside,.95)) if len(inside) else 100.
        yaw=float(np.arctan2(r[1,0],r[0,0]));shift=float(np.linalg.norm(p-np.asarray(seed[:2])));angle=float(abs(np.arctan2(np.sin(yaw-seed[2]),np.cos(yaw-seed[2]))))
        valid=overlap>=.55 and p95<=.3 and shift<3 and angle<.35
        return dict(valid=bool(valid),reason='HELD_OUT_SURFACE_MATCH' if valid else 'GEOMETRY_OR_CAPTURE_BOUND',pose=[*p.tolist(),yaw],overlap=overlap,p95_m=p95,heldout_points=len(ds),translation_from_seed_m=shift,yaw_from_seed_rad=angle)

def rigid_fit(source,target):
    source,target=np.asarray(source),np.asarray(target);c=source.mean(0);d=target.mean(0);x,y=source-c,target-d
    if len(x)<4 or np.linalg.svd(x,compute_uv=False)[1]<1:raise ValueError('CALIBRATION_BASELINE_DEGENERATE')
    u,_,vt=np.linalg.svd(x.T@y);r=vt.T@u.T
    if np.linalg.det(r)<0:vt[-1]*=-1;r=vt.T@u.T
    return r,d-r@c
