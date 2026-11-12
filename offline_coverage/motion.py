"""Forward Dubins candidates and bounded SE(2) hybrid search, no XY A* fallback."""
import heapq
import itertools
import math
import time

TAU=2*math.pi
def wrap(x):return math.atan2(math.sin(x),math.cos(x))
def mod(x):return x%TAU


def step(p,k,s):
    x,y,a=p
    if abs(k)<1e-12:return [x+s*math.cos(a),y+s*math.sin(a),a]
    b=a+k*s
    return [x+(math.sin(b)-math.sin(a))/k,y+(math.cos(a)-math.cos(b))/k,wrap(b)]


def sample(start,primitives,ds=.10):
    poses=[list(start)];controls=[]
    for k,length in primitives:
        if length<1e-9:continue
        n=max(1,math.ceil(length/ds));s=length/n
        for _ in range(n):
            controls.append([k,s]);poses.append(step(poses[-1],k,s))
    return {'poses':poses,'controls':controls,'length_m':sum(x[1] for x in controls)}


def join(a,b):
    if math.dist(a['poses'][-1][:2],b['poses'][0][:2])>1e-6 or abs(wrap(a['poses'][-1][2]-b['poses'][0][2]))>1e-6:
        raise ValueError('Cannot join disconnected vehicle states')
    return {'poses':a['poses']+b['poses'][1:],'controls':a['controls']+b['controls'],'length_m':a['length_m']+b['length_m']}


def dubins(start,goal,radius,ds=.10):
    d=math.dist(start[:2],goal[:2])/radius;theta=math.atan2(goal[1]-start[1],goal[0]-start[0])
    a,b=mod(start[2]-theta),mod(goal[2]-theta)
    sa,sb,ca,cb=math.sin(a),math.sin(b),math.cos(a),math.cos(b);cab=math.cos(a-b)
    options=[]
    p2=2+d*d-2*cab+2*d*(sa-sb)
    if p2>=-1e-10:
        t=math.atan2(cb-ca,d+sa-sb);options.append(('LSL',[mod(-a+t),math.sqrt(max(0,p2)),mod(b-t)]))
    p2=2+d*d-2*cab+2*d*(-sa+sb)
    if p2>=-1e-10:
        t=math.atan2(ca-cb,d-sa+sb);options.append(('RSR',[mod(a-t),math.sqrt(max(0,p2)),mod(-b+t)]))
    p2=-2+d*d+2*cab+2*d*(sa+sb)
    if p2>=-1e-10:
        p=math.sqrt(max(0,p2));t=math.atan2(-ca-cb,d+sa+sb)-math.atan2(-2,p)
        options.append(('LSR',[mod(-a+t),p,mod(-b+t)]))
    p2=d*d-2+2*cab-2*d*(sa+sb)
    if p2>=-1e-10:
        p=math.sqrt(max(0,p2));t=math.atan2(ca+cb,d-sa-sb)-math.atan2(2,p)
        options.append(('RSL',[mod(a-t),p,mod(b-t)]))
    t=(6-d*d+2*cab+2*d*(sa-sb))/8
    if abs(t)<=1:
        p=mod(TAU-math.acos(t));q=mod(a-math.atan2(ca-cb,d-sa+sb)+p/2)
        options.append(('RLR',[q,p,mod(a-b-q+p)]))
    t=(6-d*d+2*cab+2*d*(-sa+sb))/8
    if abs(t)<=1:
        p=mod(TAU-math.acos(t));q=mod(-a-math.atan2(ca-cb,d+sa-sb)+p/2)
        options.append(('LRL',[q,p,mod(b-a-q+p)]))
    results=[]
    for mode,lengths in options:
        primitives=[({'L':1/radius,'R':-1/radius,'S':0}[m],x*radius) for m,x in zip(mode,lengths)]
        path=sample(start,primitives,ds)
        error=math.dist(path['poses'][-1][:2],goal[:2])+abs(wrap(path['poses'][-1][2]-goal[2]))
        if error>1e-6:raise ValueError('Dubins analytic endpoint error: '+mode+' '+str(error))
        path['poses'][-1]=list(goal);path['mode']=mode;results.append(path)
    return sorted(results,key=lambda p:(p['length_m'],p['mode']))


def connect(start,goal,radius,collision_free,max_nodes=2500,max_seconds=10.,ds=.10):
    begin=time.monotonic();checks=0
    if math.dist(start[:2],goal[:2])<1e-8 and abs(wrap(start[2]-goal[2]))<1e-8:
        return {'ok':True,'path':sample(start,[]),'reason':'IDENTICAL_POSE','expanded':0}
    for p in dubins(start,goal,radius,ds):
        checks+=1
        if collision_free(p):return {'ok':True,'path':p,'reason':'DUBINS_'+p['mode'],'expanded':0}
    if max_nodes==0:return {'ok':False,'path':None,'reason':'NO_COLLISION_FREE_DUBINS','expanded':0}
    # Resolution is a search policy, never a clearance or turning-radius relaxation.
    xy=.55;bins=72;length=1.0;serial=itertools.count()
    def key(p):return round(p[0]/xy),round(p[1]/xy),round(mod(p[2])/TAU*bins)%bins
    def heuristic(p):return max(math.dist(p[:2],goal[:2]),radius*abs(wrap(p[2]-goal[2])))
    root=key(start);best={root:0.};nodes=[(list(start),None,None)]
    queue=[(heuristic(start),next(serial),0.,0)];expanded=0
    while queue and expanded<max_nodes:
        if time.monotonic()-begin>max_seconds:
            return {'ok':False,'path':None,'reason':'CONNECTOR_TIME_BUDGET_EXHAUSTED','expanded':expanded}
        _,_,cost,node_id=heapq.heappop(queue)
        pose,parent,edge=nodes[node_id]
        if cost!=best.get(key(pose)):continue
        expanded+=1
        if expanded%12==0 and math.dist(pose[:2],goal[:2])<radius*5:
            for tail in dubins(pose,goal,radius,ds):
                if collision_free(tail):
                    chunks=[];k=node_id
                    while nodes[k][1] is not None:
                        chunks.append(nodes[k][2]);k=nodes[k][1]
                    path=sample(start,[])
                    for chunk in reversed(chunks):path=join(path,chunk)
                    path=join(path,tail)
                    return {'ok':True,'path':path,'reason':'SE2_HYBRID_DUBINS','expanded':expanded}
        for curvature in (0.,1/radius,-1/radius,.5/radius,-.5/radius):
            candidate=sample(pose,[(curvature,length)],ds);end=candidate['poses'][-1];k=key(end)
            g=cost+length*(1+.03*abs(curvature)*radius)
            if g>=best.get(k,math.inf) or not collision_free(candidate):continue
            best[k]=g;nodes.append((end,node_id,candidate))
            heapq.heappush(queue,(g+heuristic(end),next(serial),g,len(nodes)-1))
    return {'ok':False,'path':None,'reason':'CONNECTOR_NODE_BUDGET_EXHAUSTED' if queue else 'DISCRETE_SEARCH_EXHAUSTED',
            'expanded':expanded,'not_a_global_infeasibility_proof':True}
