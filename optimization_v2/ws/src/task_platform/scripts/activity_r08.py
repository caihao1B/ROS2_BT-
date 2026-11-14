"""Temporary avoidance envelopes from observed moving-object history only."""
import math
import numpy as np
from shapely.geometry import MultiPoint,LineString,Point
from shapely.ops import unary_union
from shapely import contains_xy

class ActivityMemory:
 def __init__(self,seconds=35.,margin=.25):self.seconds=seconds;self.margin=margin;self.tracks={}
 def update(self,t,tracks,eligible):
  for tr in tracks:
   k=tr['id']
   if (k in eligible or k in self.tracks) and tr['confirmed'] and tr['state']=='OBSERVED' and tr['observed_age_s']<.25:
    item=self.tracks.setdefault(k,dict(samples=[],radius=0.));item['radius']=max(item['radius'],math.hypot(*tr['size'][:2])/2)
    p=list(tr['position'][:2])
    if not item['samples'] or math.dist(p,item['samples'][-1][1])>.05 or t-item['samples'][-1][0]>.5:item['samples'].append((t,p))
  polygons=[]
  for k in list(self.tracks):
   item=self.tracks[k];item['samples']=[s for s in item['samples'] if t-s[0]<=self.seconds]
   if not item['samples']:del self.tracks[k];continue
   polygons.append(MultiPoint([p for _,p in item['samples']]).convex_hull.buffer(item['radius']+self.margin))
  return unary_union(polygons)

def apply_grid(cost,origin,resolution,geometry):
 out=np.asarray(cost).copy()
 if not geometry.is_empty:
  x=origin[0]+(np.arange(out.shape[1])+.5)*resolution;y=origin[1]+(np.arange(out.shape[0])+.5)*resolution;xx,yy=np.meshgrid(x,y);out[contains_xy(geometry,xx,yy)]=100
 return out

def route_conflict(geometry,path,ego,footprint):
 if geometry.is_empty:return dict(conflict=False,clear_ahead_m=0.)
 line=LineString(np.asarray(path)[:,:2]);s=line.project(Point(ego));radius=max(math.hypot(x,y) for x,y in footprint)+.1
 points=[line.interpolate(min(line.length,s+d)).coords[0] for d in np.linspace(0,1.5,16)]
 conflict=LineString(points).buffer(radius).intersects(geometry)
 # Select a rejoin beyond the observed activity, rather than inside its envelope.
 parts=list(geometry.geoms) if hasattr(geometry,'geoms') else [geometry];ahead=0.
 for g in parts:
  for p in g.exterior.coords:ahead=max(ahead,line.project(Point(p))-s+2.)
 return dict(conflict=conflict,clear_ahead_m=ahead,observed_envelope_m2=geometry.area,history_s=35.,source='temporary convex observed activity envelope; not static ground or future truth')
