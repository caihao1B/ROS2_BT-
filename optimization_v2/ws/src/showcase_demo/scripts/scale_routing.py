"""Offline scale-study routing, derived from frozen routing14.py.

Fields2Cover supplies whole-zone swaths. Dynamic programming chooses a closed
route through swath directions. This is optimal only on this finite graph.
"""
from common14 import *
from common12 import config,body_local,backend,dubins
from motion12 import straight,segment,effect_polygon,summarize
from mowing12 import lines
from shapely.geometry import shape,box,LineString,mapping
from shapely.affinity import translate
import shapely,copy

class CollisionMap:
    def __init__(self,observation,extent):
        g=np.load(observation);a=g['data'];res=float(g['resolution']);origin=g['origin']
        if str(g['frame'])!='map' or res<=0 or not np.isin(a,np.arange(-1,101)).all():raise ValueError('ROUTE_OBSERVATION_INVALID')
        j,i=np.where((a==100)|(a<0));blocked=shapely.union_all(shapely.box(origin[0]+i*res,origin[1]+j*res,origin[0]+(i+1)*res,origin[1]+(j+1)*res))
        self.allowed=box(*origin,*(origin+np.array(a.shape)[::-1]*res)).difference(blocked).intersection(box(*extent[0],*extent[1]))
        shapely.prepare(self.allowed);self.footprint=body_local('ackermann')[0];self.c=config()
    def valid(self,poses):
        p=np.asarray(poses,float)
        if len(p)<2 or p.shape[1]!=3 or not np.isfinite(p).all():return False
        ds=np.linalg.norm(np.diff(p[:,:2],axis=0),axis=1);dh=np.abs((np.diff(p[:,2])+math.pi)%(2*math.pi)-math.pi);radius=np.linalg.norm(self.footprint,axis=1).max()
        pad=self.c['safety_margin_m']+self.c['sweep_numerical_margin_m']+np.max(ds+radius*dh,initial=0)/2
        co=np.cos(p[:,2,None]);si=np.sin(p[:,2,None]);fp=self.footprint
        xy=np.stack([p[:,0,None]+co*fp[:,0]-si*fp[:,1],p[:,1,None]+si*fp[:,0]+co*fp[:,1]],axis=-1)
        swept=shapely.buffer(shapely.polygons(xy),pad/math.cos(math.pi/16),quad_segs=4)
        return bool(np.all(shapely.covers(self.allowed,swept)))

def work_options(zone,raw,phase,collision,c):
    region=translate(shape(zone['plans'][0]['coverage_check']['covered']),xoff=-28,yoff=-14);options=[];angle=raw['angle'];offset=phase*np.array([-math.sin(angle),math.cos(angle)])
    for row in raw['swaths']:
        a=np.array(row['a'])+offset-[28,14];b=np.array(row['b'])+offset-[28,14]
        for part in lines(LineString([a,b]).intersection(region.buffer(c['work_spacing_m']/2,join_style=2))):
            if part.length<c['minimum_work_segment_m']:continue
            a=np.array(part.coords[0]);b=np.array(part.coords[-1]);u=(b-a)/np.linalg.norm(b-a)
            poses=np.array(straight(a-u*(.5+c['swath_endpoint_overrun_m']),b+u*(c['swath_endpoint_overrun_m']-.5),c['path_step_m']))
            contribution=effect_polygon(poses[1:-1],c).intersection(region)
            if contribution.area<1e-10:continue
            # Remove only unproductive endpoint travel; keep two extra samples
            # (about 16 cm) for docking, tool start and measured-speed admission.
            lo=0;hi=len(poses)
            while lo+4<hi and contribution.difference(effect_polygon(poses[lo+2:hi-1],c)).area<1e-9:lo+=1
            while lo+4<hi and contribution.difference(effect_polygon(poses[lo+1:hi-2],c)).area<1e-9:hi-=1
            poses=poses[max(0,lo-2):min(len(poses),hi+2)];work_id=len(options)//2
            for reverse in [False,True]:
                p=poses.copy()
                if reverse:
                    # Mower rectangle is centered 0.5m ahead of the rear axle.
                    # Reversing a STRAIGHT swath requires a 1m axle translation
                    # to keep every tool-center position exactly unchanged.
                    p[:,:2]+=np.c_[np.cos(p[:,2]),np.sin(p[:,2])];p[:,2]+=math.pi;p=p[::-1].copy()
                if effect_polygon(p,c).symmetric_difference(effect_polygon(poses,c)).area>1e-8:raise ValueError('TOOL_CENTER_INVARIANCE')
                options.append(dict(work_id=work_id,row_id=row['id'],reverse=reverse,poses=p.tolist(),start=p[0].tolist(),end=p[-1].tolist(),valid=collision.valid(p)))
    return options

def candidate(zone,raw,phase,collision,c):
    options=work_options(zone,raw,phase,collision,c);count=len(options)//2
    if not 1<=count<=10:return None,dict(reason='WORK_COUNT_OUTSIDE_BOUNDED_SEARCH',count=count)
    depot=np.array(zone['plans'][0]['start']);depot[:2]-=[28,14];edges={};start=time.monotonic()
    for i in range(-1,len(options)):
        if i>=0 and not options[i]['valid']:continue
        for j in range(-1,len(options)):
            if i==j or (i>=0 and j>=0 and options[i]['work_id']==options[j]['work_id']) or (j>=0 and not options[j]['valid']):continue
            a=depot.tolist() if i<0 else options[i]['end'];b=depot.tolist() if j<0 else options[j]['start']
            for path in dubins.dubins(a,b,2.11,c['path_step_m']/2):
                if collision.valid(path['poses']):edges[(i,j)]=path;break
            if time.monotonic()-start>30:raise TimeoutError('ROUTE_CANDIDATE_BUDGET')
    # A missing single Dubins edge is not proof of physical disconnection.
    # Search a small, declared set of headland waypoint poses. Cached legs keep
    # this bounded; every combined path still passes the full padded footprint.
    via_poses=zone.get('turn_waypoints',[]);cache={}
    def leg(a,b):
        key=tuple(a)+tuple(b)
        if key not in cache:
            cache[key]=next((p for p in dubins.dubins(a,b,2.11,c['path_step_m']/2) if collision.valid(p['poses'])),None)
        return cache[key]
    for i in range(-1,len(options)):
        if i>=0 and not options[i]['valid']:continue
        for j in range(-1,len(options)):
            if i==j or (i,j) in edges or (i>=0 and j>=0 and options[i]['work_id']==options[j]['work_id']) or (j>=0 and not options[j]['valid']):continue
            a=depot.tolist() if i<0 else options[i]['end'];b=depot.tolist() if j<0 else options[j]['start']
            alternatives=[]
            for via in via_poses:
                if time.monotonic()-start>120:break
                mid=[via[0]-28,via[1]-14,via[2]]
                left=leg(a,mid)
                if not left:continue
                right=leg(mid,b)
                if right:
                    poses=left['poses']+right['poses'][1:]
                    if collision.valid(poses):alternatives.append(dict(poses=poses,length_m=left['length_m']+right['length_m']))
            if alternatives:edges[(i,j)]=min(alternatives,key=lambda p:p['length_m'])
    # Exact Held-Karp on the finite directed orientation graph. The obstacle
    # checks are on the entire padded footprint, not the centerline.
    work_lengths=[float(np.linalg.norm(np.diff(np.array(o['poses'])[:,:2],axis=0),axis=1).sum()) for o in options];dp={}
    for j in range(len(options)):
        if (-1,j) in edges:dp[(1<<options[j]['work_id'],j)]=(edges[(-1,j)]['length_m']+work_lengths[j],[j])
    for mask in range(1,1<<count):
        for i in range(len(options)):
            if (mask,i) not in dp:continue
            cost,sequence=dp[(mask,i)]
            for j in range(len(options)):
                bit=1<<options[j]['work_id'];key=(mask|bit,j)
                if mask&bit or (i,j) not in edges:continue
                value=cost+edges[(i,j)]['length_m']+work_lengths[j]
                if key not in dp or value<dp[key][0]:dp[key]=(value,sequence+[j])
    solutions=[(cost+edges[(i,-1)]['length_m'],seq) for (mask,i),(cost,seq) in dp.items() if mask==(1<<count)-1 and (i,-1) in edges]
    evidence=dict(angle_rad=raw['angle'],phase_m=phase,work_count=count,feasible_directed_edges=len(edges),search_wall_s=time.monotonic()-start)
    if not solutions:return None,evidence|dict(reason='NO_CLOSED_COLLISION_FREE_ROUTE')
    cost,sequence=min(solutions);segments=[];previous=-1;motion=shape(zone['motion_domain']);headland=shape(zone['plans'][0]['whole_zone_headland'])
    for j in sequence+[-1]:
        p=np.array(edges[(previous,j)]['poses']);p[:,:2]+=[28,14]
        turn=segment(p.tolist(),'TURN','ackermann',motion,c,stop_curvature=True,connector_source='six forward analytic Dubins alternatives checked on measured map');segments.append(turn)
        if j<0:break
        opt=options[j];p=np.array(opt['poses']);p[:,:2]+=[28,14];kind='HEADLAND_WORK' if headland.covers(LineString(p[:,:2]).interpolate(.5,normalized=True)) else 'COVERAGE'
        segments.append(segment(p.tolist(),kind,'ackermann',motion,c,tool=True,row_id=opt['row_id'],cluster=0,reverse_swath_direction=opt['reverse']))
        previous=j
    timing=summarize(segments,c);target=shape(zone['plans'][0]['coverage_check']['covered']);credit=shapely.union_all([effect_polygon([p for p,v in zip(s['poses'],s['timing']['velocities']) if v>=.02],c) for s in segments if s['tool_on']]);coverage=target.intersection(credit).area/target.area
    good=timing['all_segments_valid'] and coverage>=1-1e-8 and timing['total_s']<=1440
    evidence.update(reason='VALID' if good else 'COVERAGE_GEOMETRY_OR_NOMINAL_TIME_REJECTED',coverage_fraction=coverage,timing=timing,order=sequence)
    return (dict(segments=segments,timing=timing,coverage_fraction=coverage,required_area_m2=target.area,angle_rad=raw['angle'],phase_m=phase,work_order=sequence) if good else None),evidence

def verify_route(route,original,atlas):
    from binding14 import canonical
    body=copy.deepcopy(route);revision=body.pop('revision')
    if canonical(body)!=revision:raise ValueError('EXECUTION_ROUTE_HASH')
    if route['base_plan_revision']!=original['plan_revision'] or route['atlas_revision']!=atlas['revision'] or route['cluster']!='B1':raise ValueError('EXECUTION_ROUTE_LINEAGE')
    for rel,digest in route['bindings'].items():
        path=(ROOT/rel).resolve()
        if not path.is_relative_to(ROOT) or not path.is_file() or sha(path)!=digest:raise ValueError('EXECUTION_ROUTE_INPUT_CHANGED '+rel)
    base=original['zones']['B'];target=shape(base['plans'][0]['coverage_check']['covered']);c=config();collision=CollisionMap(ROOT/route['observation'],atlas['regions']['B']['site_extent_xy']);segments=route['segments'];verified=[];credit=[]
    if not segments or math.dist(segments[0]['poses'][0],base['plans'][0]['start'])>1e-5 or math.dist(segments[-1]['poses'][-1],base['plans'][0]['end'])>1e-5:raise ValueError('ORIGINAL_DEPOT_OBLIGATION_CHANGED')
    previous=None
    for s in segments:
        p=np.array(s['poses'],float)
        if previous is not None and (math.dist(previous[:2],p[0,:2])>1e-5 or abs(math.atan2(math.sin(previous[2]-p[0,2]),math.cos(previous[2]-p[0,2])))>1e-5):raise ValueError('ROUTE_DISCONTINUITY')
        previous=p[-1].copy();p[:,:2]-=[28,14]
        if not collision.valid(p):raise ValueError('EXECUTION_ROUTE_COLLISION_OR_UNVERIFIED_REGION')
        work=s['kind'] in ['COVERAGE','HEADLAND_WORK']
        if s['tool_on']!=work:raise ValueError('EXECUTION_TOOL_SEGMENT_MISMATCH')
        q=segment(s['poses'],s['kind'],'ackermann',shape(base['motion_domain']),c,tool=work,stop_curvature=not work)
        if not q['validation']['passed']:raise ValueError('EXECUTION_SWEEP_OR_KINEMATICS')
        verified.append(q)
        if work:credit.append(effect_polygon([p for p,v in zip(q['poses'],q['timing']['velocities']) if v>=.02],c))
    fraction=target.intersection(shapely.union_all(credit)).area/target.area;timing=summarize(verified,c)
    if abs(route['required_area_m2']-target.area)>1e-9 or fraction<1-1e-8 or timing['total_s']>1440:raise ValueError('EXECUTION_COVERAGE_OR_TIME_CONTRACT')
    return dict(passed=True,coverage_fraction=fraction,required_area_m2=target.area,timing=timing)
