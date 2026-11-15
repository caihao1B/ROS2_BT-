"""Bounded local A* over CURRENT observed free cells; unknown is occupied.

A circumscribed disc plus cell diagonal protects every vehicle heading. The
result is only a local candidate and expires with its observation timestamp.
"""
import heapq,math,time
import numpy as np
from scipy.ndimage import distance_transform_edt

def segment_free(a,b,free,origin,resolution):
 count=max(1,int(math.ceil(math.dist(a,b)/(resolution*.4))));p=np.linspace(a,b,count+1);ij=np.floor((p-np.asarray(origin))/resolution).astype(int)
 if ((ij<0).any() or (ij[:,0]>=free.shape[1]).any() or (ij[:,1]>=free.shape[0]).any()):return False
 return bool(free[ij[:,1],ij[:,0]].all())

def plan(cost,origin,resolution,start,goals,footprint,margin=.1,max_expansions=20000,max_wall_s=.25):
 begin=time.monotonic();cost=np.asarray(cost);known=(cost>=0)&(cost<100);padded=np.pad(known,1,constant_values=False);distance=distance_transform_edt(padded)[1:-1,1:-1]*resolution
 radius=max(math.hypot(x,y) for x,y in footprint)+math.sqrt(2)*margin
 free=distance>radius+resolution*math.sqrt(2)/2
 def cell(p):q=np.floor((np.array(p[:2])-origin)/resolution).astype(int);return int(q[0]),int(q[1])
 def inside(q):return 0<=q[0]<free.shape[1] and 0<=q[1]<free.shape[0] and bool(free[q[1],q[0]])
 def center(q):return np.array(origin)+(np.array(q)+.5)*resolution
 source=cell(start);metadata=dict(truth_used=False,unknown_is_occupied=True,radius_m=radius,expansions=0)
 if not inside(source):return dict(metadata,valid=False,reason='START_FOOTPRINT_NOT_OBSERVED')
 candidates=[(int(i),np.asarray(p[:2]),cell(p)) for i,p in goals if inside(cell(p))]
 if not candidates:return dict(metadata,valid=False,reason='NO_OBSERVED_REJOIN_TARGET')
 # Farthest observed route target within the caller's bounded local horizon.
 for index,goal,target in reversed(candidates):
  queue=[(math.dist(source,target),0.,source)];best={source:0.};parent={};found=False
  while queue:
   _,g,u=heapq.heappop(queue)
   if g>best[u]+1e-9:continue
   if u==target:found=True;break
   metadata['expansions']+=1
   if metadata['expansions']>max_expansions or time.monotonic()-begin>max_wall_s:return dict(metadata,valid=False,reason='BOUNDED_LOCAL_SEARCH_EXHAUSTED')
   for dx,dy in [(1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)]:
    v=(u[0]+dx,u[1]+dy)
    if not inside(v) or (dx and dy and (not inside((u[0]+dx,u[1])) or not inside((u[0],u[1]+dy)))):continue
    w=math.hypot(dx,dy)*(1.+.01*cost[v[1],v[0]]);ng=g+w
    if ng<best.get(v,float('inf')):best[v]=ng;parent[v]=u;heapq.heappush(queue,(ng+math.dist(v,target),ng,v))
  if not found:continue
  cells=[target]
  while cells[-1]!=source:cells.append(parent[cells[-1]])
  raw=[np.array(start[:2])]+[center(c) for c in reversed(cells)]+[goal];simple=[raw[0]];i=0
  while i<len(raw)-1:
   j=len(raw)-1
   while j>i+1 and not segment_free(raw[i],raw[j],free,origin,resolution):j-=1
   if not segment_free(raw[i],raw[j],free,origin,resolution):return dict(metadata,valid=False,reason='CONTINUOUS_PATCH_REJECTED')
   simple.append(raw[j]);i=j
  points=[]
  for a,b in zip(simple,simple[1:]):
   n=max(1,int(math.ceil(math.dist(a,b)/.05)));h=math.atan2(b[1]-a[1],b[0]-a[0]);points.extend([[float(x),float(y),h] for x,y in np.linspace(a,b,n+1)[:-1]])
  points.append([float(goal[0]),float(goal[1]),points[-1][2]]);length=sum(math.dist(a[:2],b[:2]) for a,b in zip(points,points[1:]));return dict(metadata,valid=True,reason='OBSERVED_FULL_FOOTPRINT_LOCAL_ASTAR',poses=points,rejoin_index=index,length_m=length,compute_wall_s=time.monotonic()-begin)
 return dict(metadata,valid=False,reason='NO_ROUTE_IN_CURRENT_OBSERVED_WINDOW',global_no_path_proof=False)
