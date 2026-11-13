"""Keep tracked moving objects out of persistent static-terrain memory.

Current lidar/cost observations and frozen occupancy are never cleared here.
Only the supplementary historical terrain memory is filtered by measured,
confirmed object tracks in the supervised estimated map frame.
"""
import numpy as np
from scipy.spatial import cKDTree
from map_memory17 import MapMemory
from dynamic_envelope17 import Envelope,to_map

class DynamicTerrainMemory(MapMemory):
    def __init__(self):super().__init__();self.moving=Envelope();self.removed_dynamic=0
    def update(self,live,confirmed,origin,resolution,rotation,translation,t,packet=None):
        self.moving.update(to_map(packet,rotation,translation),t)
        if self.moving.cells:
            # A tracked object's measured history can invalidate a historical
            # terrain classification, but never a live lethal observation.
            if self.cells:
                keys=np.asarray(sorted(self.cells));centers=(keys+.5)*resolution;remove=np.zeros(len(keys),bool)
                tracks=list(self.moving.cells.items());queries=(np.array([cell for cell,_ in tracks])+.5)*self.moving.resolution;radii=[v[1]+self.moving.resolution/np.sqrt(2)+resolution/np.sqrt(2) for _,v in tracks]
                for indices in cKDTree(centers).query_ball_point(queries,radii):remove[indices]=True
                self.cells.difference_update(map(tuple,keys[remove].tolist()));self.removed_dynamic+=int(remove.sum())
            h,w=live.shape;x=origin[0]+(np.arange(w)+.5)*resolution;y=origin[1]+(np.arange(h)+.5)*resolution
            mask=np.zeros_like(live);self.moving.apply(mask,x,y,rotation,translation)
            confirmed=confirmed&(mask==0)
        return super().update(live,confirmed,origin,resolution,rotation,translation,t)
