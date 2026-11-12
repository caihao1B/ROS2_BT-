"""Obstacle-aware visibility graph, NN+2-opt and actual OR-Tools TSP comparison."""
import heapq
import shapely
from common12 import *
from motion12 import straight,rotate_at,segment,summarize

def visibility(nodes,motion,c):
    local,_,_=body_local('diff');radius=float(np.max(np.linalg.norm(local,axis=1)))+.20
    safe=motion.buffer(-radius,quad_segs=12)
    allowed=[safe.covers(Point(p)) for p in nodes]
    if not all(allowed):raise ValueError('UNREACHABLE_DOCKING_POINTS '+str([i for i,x in enumerate(allowed) if not x]))
    corners=[]
    for poly in polygons(safe):
        for ring in [poly.exterior,*poly.interiors]:corners.extend(list(ring.coords)[:-1])
    xy=np.asarray([*nodes,*corners]);n=len(xy);adj=[[] for _ in xy]
    for i in range(n):
        js=np.arange(i+1,n)
        if len(js)==0:continue
        lines=shapely.linestrings(np.stack([np.tile(xy[i],(len(js),1)),xy[js]],axis=1));mask=shapely.covers(safe,lines)
        for j in js[mask]:
            distance=float(np.linalg.norm(xy[j]-xy[i]));adj[i].append((int(j),distance));adj[j].append((i,distance))
    costs=[];paths={}
    for start in range(len(nodes)):
        dist=[math.inf]*n;prev=[-1]*n;dist[start]=0;queue=[(0.,start)]
        while queue:
            d,i=heapq.heappop(queue)
            if d>dist[i]+1e-12:continue
            for j,w in adj[i]:
                if d+w<dist[j]-1e-12:dist[j]=d+w;prev[j]=i;heapq.heappush(queue,(dist[j],j))
        if any(not math.isfinite(x) for x in dist[:len(nodes)]):raise ValueError('DISCONNECTED_VISIBILITY_GRAPH')
        costs.append(dist[:len(nodes)])
        for end in range(len(nodes)):
            seq=[end]
            while seq[-1]!=start:seq.append(prev[seq[-1]])
            paths[start,end]=xy[list(reversed(seq))].tolist()
    return np.asarray(costs),paths,dict(radius_m=radius,vertices=n,edges=sum(map(len,adj))//2)

def nearest_2opt(cost):
    left=set(range(1,len(cost)));order=[0]
    while left:
        v=min(left,key=lambda i:(cost[order[-1],i],i));order.append(v);left.remove(v)
    def value(o):return sum(cost[o[i],o[(i+1)%len(o)]] for i in range(len(o)))
    for _ in range(30):
        changed=False
        for i in range(1,len(order)-1):
            for j in range(i+1,len(order)):
                alt=order[:i]+list(reversed(order[i:j+1]))+order[j+1:]
                if value(alt)<value(order)-1e-9:order=alt;changed=True
        if not changed:break
    return order,float(value(order))

def build_route(order,paths,points,motion,c,depot):
    segs=[];current=[*depot,0.];visited=[]
    for a,b in zip(order,order[1:]+[0]):
        polyline=paths[a,b]
        for p,q in zip(polyline,polyline[1:]):
            if math.dist(p,q)<1e-8:continue
            theta=math.atan2(q[1]-p[1],q[0]-p[0]);turn=rotate_at(current,theta,motion,c)
            if turn['timing']['motion_seconds']>1e-8:segs.append(turn)
            s=segment(straight(p,q,c['path_step_m']),'TRANSIT','diff',motion,c);segs.append(s);current=s['poses'][-1]
        turn=rotate_at(current,0.,motion,c)
        if turn['timing']['motion_seconds']>1e-8:segs.append(turn)
        current=[*polyline[-1],0.]
        if b:
            pt=points[b-1];visited.append(pt['id']);service=segment([current],'POINT_SERVICE','diff',motion,c,tool=True,target_xy=pt['xy'],task_id=pt['id'],requested_quantity=2.0)
            service['action_seconds']=c['startup_seconds']+c['shutdown_seconds']+c['spray_dwell_seconds']+c['spray_docking_seconds'];segs.append(service)
    time=summarize(segs,c,len(points))
    return dict(segments=segs,visited=visited,timing=time,start=[*depot,0.],end=current)

def plan_spray(meta,g,labels,c,out):
    points=g['points'];centre=meta['domain'].representative_point();depot=[centre.x-.42,centre.y];motion=meta['motion'];plans=[]
    for cluster in range(3):
        pts=[p for p in points if int(labels[p['unit']])==cluster]
        if not pts:raise ValueError('EMPTY_SPRAY_CLUSTER')
        nodes=[depot]+[[p['xy'][0]-.42,p['xy'][1]] for p in pts]
        cost,paths,graph=visibility(nodes,motion,c);base_order,base_cost=nearest_2opt(cost)
        ort=backend(dict(mode='tsp',cost=np.rint(cost*1000).astype(int).tolist()),out,f'tsp_cluster_{cluster}')
        if sorted(ort['order'])!=list(range(len(nodes))) or ort['order'][0]!=0:raise ValueError('INVALID_TSP_ORDER')
        alternatives={}
        for name,order in [('nn_2opt',base_order),('ortools',ort['order'])]:alternatives[name]=build_route(order,paths,pts,motion,c,depot)
        method=min(alternatives,key=lambda x:alternatives[x]['timing']['total_s']);p=alternatives[method]
        p.update(cluster=cluster,zone='A',vehicle='diff',method=method,points=pts,graph=graph,
            comparison={name:dict(timing=r['timing'],order=r['visited']) for name,r in alternatives.items()})
        p['coverage_check']=dict(all_serviceable_once=sorted(p['visited'])==sorted(x['id'] for x in pts),unreachable=[],planned_count=len(pts))
        plans.append(p)
    return plans
