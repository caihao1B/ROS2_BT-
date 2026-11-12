"""Whole-zone F2C line phase, logical cluster clipping, safe forward CC turns."""
from common12 import *
from motion12 import *

def lines(g):
    if g.is_empty:return []
    if g.geom_type=='LineString':return [g]
    if hasattr(g,'geoms'):return [l for x in g.geoms for l in lines(x)]
    return []

def connection(start,options,motion,c,out,name):
    if not options:return []
    queries=[dict(id=i,start=start,goal=o['start']) for i,o in enumerate(options)]
    response=backend(dict(mode='turns',radius=2.11,ds=c['path_step_m']/2,queries=queries),out,name);choices=[]
    for raw in response['turns']:
        poses=raw['poses'];i=raw['id']
        if len(poses)<2:continue
        if math.dist(poses[0][:2],start[:2])>1e-3 or abs(wrap(poses[0][2]-start[2]))>1e-3:continue
        goal=options[i]['start']
        if math.dist(poses[-1][:2],goal[:2])>1e-3 or abs(wrap(poses[-1][2]-goal[2]))>1e-3:continue
        s=segment(poses,'TURN','ackermann',motion,c,connector_source='Fields2Cover DubinsCurvesCC' if raw['continuous_curvature'] else 'Fields2Cover Dubins + measured-stop steering events',stop_curvature=not raw['continuous_curvature'])
        if s['validation']['passed']:choices.append((s['timing']['motion_seconds'],i,s))
    return choices

def plan_mowing(meta,g,labels,c,out):
    domain=meta['domain'];motion=meta['motion'];regions=[union_all([p for i,p in enumerate(g['cells']) if labels[i]==k]) for k in range(3)]
    expanded=domain.buffer(.40,join_style=2)
    raw=backend(dict(mode='swaths',work=mapping(expanded),motion=mapping(motion),spacing=c['work_spacing_m'],headland=c['whole_zone_headland_m']),out,'f2c_whole_zone')
    depot_point=domain.representative_point();depot=[depot_point.x-.5,depot_point.y,raw['angle']];plans=[]
    # The reserve belongs to the whole zone. It is not regenerated around three clusters.
    inner=shape(raw['inner_work']);headland=domain.difference(inner);global_footprints=[]
    for cluster,region in enumerate(regions):
        work=[];rejected=[]
        for row in raw['swaths']:
            cut=LineString([row['a'],row['b']]).intersection(region.buffer(c['work_spacing_m']/2,join_style=2))
            for part in lines(cut):
                if part.length<c['minimum_work_segment_m']:
                    rejected.append(dict(reason='BELOW_MIN_LENGTH',line=mapping(part)));continue
                a=np.array(part.coords[0]);b=np.array(part.coords[-1]);u=(b-a)/np.linalg.norm(b-a);a-=u*c['swath_endpoint_overrun_m'];b+=u*c['swath_endpoint_overrun_m'];options=[]
                for start,end in [(a,b),(b,a)]:
                    theta=math.atan2(end[1]-start[1],end[0]-start[0]);offset=.5*np.array([math.cos(theta),math.sin(theta)]);poses=straight(start-offset,end-offset,c['path_step_m'])
                    kind='HEADLAND_WORK' if headland.covers(part.interpolate(.5,normalized=True)) else 'COVERAGE'
                    s=segment(poses,kind,'ackermann',motion,c,tool=True,row_id=row['id'],cluster=cluster)
                    if s['validation']['passed']:options.append(dict(start=poses[0],segment=s))
                if options:work.append(dict(id=len(work),options=options))
                else:rejected.append(dict(reason='WORK_ENVELOPE_REJECTED',line=mapping(part)))
        segs=[];remaining={w['id']:w for w in work};current=list(depot);unconnected=[];sequence=[]
        while remaining:
            ranked=[]
            for item in remaining.values():
                for opt in item['options']:
                    cost=math.dist(current[:2],opt['start'][:2])+2.11*abs(wrap(current[2]-opt['start'][2]));ranked.append((cost,item['id'],opt))
            candidates=[opt|dict(work_id=wid) for _,wid,opt in sorted(ranked,key=lambda v:(v[0],v[1],v[2]['start'][2]))[:c['turn_candidate_count']]]
            choices=connection(current,candidates,motion,c,out,f'cluster_{cluster}_connect_{len(sequence)}')
            if not choices:
                unconnected=list(remaining);break
            _,i,turn=min(choices,key=lambda x:(x[0],candidates[x[1]]['work_id']));opt=candidates[i];segs.extend([turn,opt['segment']]);current=opt['segment']['poses'][-1];sequence.append(opt['work_id']);del remaining[opt['work_id']]
        home=connection(current,[dict(start=depot)],motion,c,out,f'cluster_{cluster}_return')
        if home:segs.append(home[0][2]);current=home[0][2]['poses'][-1]
        elif math.dist(current[:2],depot[:2])>1e-4:unconnected.append('RETURN_TO_DEPOT')
        covered=[]
        for s in segs:
            if s['tool_on']:
                accepted=[p for p,v in zip(s['poses'],s['timing']['velocities']) if v>=.02]
                covered.append(effect_polygon(accepted,c))
        union=union_all(covered);confirmed=union.intersection(region);residual=region.difference(confirmed);global_footprints.append(union)
        plans.append(dict(cluster=cluster,zone='B',vehicle='ackermann',method='Fields2Cover global swaths + CC or stopped-Dubins turns',segments=segs,work_order=sequence,start=depot,end=current,timing=summarize(segs,c),
          coverage_check=dict(fraction=confirmed.area/region.area,domain_area_m2=region.area,covered_m2=confirmed.area,residual_m2=residual.area,residual=mapping(residual),covered=mapping(confirmed),unconnected=unconnected,rejected=rejected,all_work_segments=len(sequence)==len(work)),
          whole_zone_headland=mapping(headland),f2c_angle=raw['angle'],f2c_decomposition_count=len(polygons(shape(raw['decomposition'])))))
    return plans,dict(covered=mapping(union_all(global_footprints).intersection(domain)),residual=mapping(domain.difference(union_all(global_footprints))),headland=mapping(headland),f2c_raw=raw)
