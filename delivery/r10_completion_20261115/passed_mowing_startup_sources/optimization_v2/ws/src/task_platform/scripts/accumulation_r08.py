"""Bounded local surface window; unique occupied cells, never point-count repeats."""
from collections import deque
import numpy as np
from scipy.spatial.transform import Rotation
class SurfaceWindow:
 def __init__(self,max_age=1.5,voxel=.06):self.age=max_age;self.voxel=voxel;self.frames=deque()
 def add(self,points,position,quaternion,t):
  r=Rotation.from_quat(quaternion);p=np.asarray(position);points=np.asarray(points);upright=points[points[:,2]>.2];self.frames.append((t,r.apply(upright)+p))
  while self.frames and t-self.frames[0][0]>self.age+1e-6:self.frames.popleft()
  # Newest measurement wins; transform only observed static-filtered surfaces.
  surfaces=r.inv().apply(np.vstack([x[1] for x in reversed(self.frames)])-p);surfaces=surfaces[(surfaces[:,2]>.2)&(np.linalg.norm(surfaces,axis=1)<48.)]
  if len(surfaces):_,ids=np.unique(np.floor(surfaces/self.voxel).astype('int32'),axis=0,return_index=True);surfaces=surfaces[np.sort(ids)]
  merged=np.vstack([surfaces,points[points[:,2]<=.2]])
  return merged,dict(frames=len(self.frames),max_age_s=t-self.frames[0][0],surface_cells=len(surfaces),voxel_m=self.voxel,motion_source='local odometry only')
