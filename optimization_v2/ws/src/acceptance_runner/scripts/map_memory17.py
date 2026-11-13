"""Confirmed terrain memory in supervised map coordinates, not drifting odom.

Only three-frame upstream confirmed lethal evidence enters memory. Clearing
requires fresh observed nonlethal values throughout its projected bounding box; unknown cannot
clear. Reprojection is temporary and never fed back into persistent memory.
"""
import math
import numpy as np

class MapMemory:
    def __init__(self):self.cells=set();self.resolution=None;self.last=-1.
    def update(self,live,confirmed,origin,resolution,rotation,translation,t):
        if not np.isfinite(np.r_[origin,resolution,rotation.ravel(),translation,t]).all() or resolution<=0:raise ValueError('INVALID_MEMORY_TRANSFORM')
        if self.resolution is not None and abs(self.resolution-resolution)>1e-6:raise ValueError('MEMORY_RESOLUTION_CHANGED')
        if t<self.last:self.cells.clear()
        self.last=t;self.resolution=resolution
        height,width=live.shape;offsets=np.array([[-.5,-.5],[-.5,.5],[.5,-.5],[.5,.5]])*(resolution-1e-7)
        def indices(points):return np.floor((points-origin)/resolution).astype(int)
        # Keep a fixed map lattice. Transforming the previous raster into the
        # next one repeatedly would accumulate rounding/inflation at every scan.
        if self.cells:
            keys=np.array(sorted(self.cells));centers=(keys+.5)*resolution
            samples=(centers[:,None,:]+offsets-translation)@rotation
            ij=indices(samples);lo=ij.min(axis=1);hi=ij.max(axis=1)
            clear=(lo[:,0]>=0)&(lo[:,1]>=0)&(hi[:,0]<width)&(hi[:,1]<height)&np.all(hi-lo<=2,axis=1)
            # Every source cell in the rotated square's bounding box must be
            # freshly nonlethal, including its centre, edges and corner slivers.
            for dx in range(3):
                for dy in range(3):
                    q=lo+[dx,dy];needed=np.all(q<=hi,axis=1);valid=needed&(q[:,0]>=0)&(q[:,0]<width)&(q[:,1]>=0)&(q[:,1]<height)
                    values=np.full(len(q),-1,dtype=np.int16);values[valid]=live[q[valid,1],q[valid,0]]
                    clear &= ~needed | (valid&(values>=0)&(values<100))
            self.cells.difference_update(map(tuple,keys[clear].tolist()))
        ys,xs=np.where(confirmed&(live>=100))
        if len(xs):
            centers=origin+(np.c_[xs,ys]+.5)*resolution
            corners=(centers[:,None,:]+offsets)@rotation.T+translation
            self.cells.update(map(tuple,np.floor(corners.reshape(-1,2)/resolution).astype(int).tolist()))
        if len(self.cells)>200000:raise ValueError('CONFIRMED_MAP_MEMORY_BUDGET')
        remembered=np.full_like(live,-1)
        if self.cells:
            centers=(np.array(sorted(self.cells))+.5)*resolution
            corners=(centers[:,None,:]+offsets-translation)@rotation
            ij=indices(corners.reshape(-1,2));valid=(ij[:,0]>=0)&(ij[:,0]<width)&(ij[:,1]>=0)&(ij[:,1]<height)
            remembered[ij[valid,1],ij[valid,0]]=100
        return remembered
