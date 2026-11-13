"""Connector inspection keeps the approved curve and tool disabled throughout."""
from common17 import *
import math,copy

def enabled():return os.environ.get('S17_OPERATION')=='inspection'

def route(kind):
    from inputs17 import active
    direction='A_to_B' if kind=='diff' else 'B_to_A'
    r=next(r for r in read(active()['plan'])['connector_routes'] if r['vehicle']==kind and r['direction']==direction)
    if os.environ.get('S17_BENCHMARK')=='dynamic_prefix':
        r=copy.deepcopy(r);s=next(s for s in r['segments'] if s['kind']=='CONNECTOR');poses=s['poses']
        begin,finish=([71.,38.],[88.,56.]) if kind=='diff' else ([88.,56.],[71.,38.])
        start=min(range(len(poses)),key=lambda i:math.dist(poses[i][:2],begin));end=min(range(start+1,len(poses)),key=lambda i:math.dist(poses[i][:2],finish))
        s['poses']=poses[start:end+1];s['timing']['steering_events']=[];length=sum(math.dist(a[:2],b[:2]) for a,b in zip(s['poses'],s['poses'][1:]));s['timing']['motion_seconds']=length/.12+20
        r.update(segments=[s],start=s['poses'][0],end=s['poses'][-1],benchmark='dynamic_prefix',benchmark_length_m=length,timing=dict(total_s=length/.12+20,scope='development segment near observed failure; no full-road credit'))
        return r
    if os.environ.get('S17_BENCHMARK')!='controller_prefix':return r
    # Identical, bounded prefix for each controller of the same chassis.
    # It is explicitly a different benchmark, never full-road credit.
    r=copy.deepcopy(r);segments=[];length=0.
    for original in r['segments']:
        if original['kind']=='ROTATE':continue
        s=copy.deepcopy(original);poses=s['poses'];end=1;part_length=0.
        for i in range(1,len(poses)):
            ds=math.dist(poses[i-1][:2],poses[i][:2])
            if length+ds>8.:break
            length+=ds;part_length+=ds;end=i+1
        if end>=2:
            s['poses']=poses[:end];s['timing']['steering_events']=[e for e in s['timing'].get('steering_events',[]) if e['index']<end]
            # A scheduling hint only; measured time decides the comparison.
            s['timing']['motion_seconds']=part_length/.12+10.;segments.append(s)
        if end<len(poses):break
    if not segments:raise ValueError('EMPTY_CONTROLLER_BENCHMARK')
    r.update(segments=segments,end=segments[-1]['poses'][-1],benchmark='controller_prefix',benchmark_length_m=length,timing=dict(total_s=length/.12+20.,scope='benchmark scheduling hint only'))
    return r

def jobs(m):
    r=route(m.a.vehicle);result=[]
    for i,s in enumerate(r['segments']):
        # The next path carries the desired entry heading; Nav2 handles diff
        # heading alignment. A zero-length ROTATE is not a followable path.
        if s['kind']=='ROTATE':continue
        if len(s['poses'])<2:raise ValueError('EMPTY_INSPECTION_SEGMENT')
        j=m.transit_job(s,'connector_'+str(i),reference=True);j['kind']='INSPECTION'
        if m.a.vehicle=='diff':
            # Bound each navigation action without changing the whole road.
            # The upstream action has a 120s deadline: 8m at 0.12m/s leaves
            # time for startup, stopping and short perception holds.
            poses=j['poses'];cuts=[0];arc=0.
            for k in range(1,len(poses)):
                arc+=math.dist(poses[k-1][:2],poses[k][:2])
                if arc>=8.:cuts.append(k);arc=0.
            if cuts[-1]!=len(poses)-1:cuts.append(len(poses)-1)
            j['reference_parts']=[dict(id=j['id']+'_reference_'+str(n),poses=poses[a:b+1]) for n,(a,b) in enumerate(zip(cuts,cuts[1:]))]
        result.append(j)
    dump(m.a.out/'inspection_contract17.json',dict(direction=r['direction'],vehicle=m.a.vehicle,original_route=r,benchmark=r.get('benchmark'),tool_permitted=False,required_jobs=[j['id'] for j in result],nominal_s=r['timing']['total_s']))
    return result

def execute(m,j,replan):
    if m.working or m.work_permission():raise ValueError('INSPECTION_TOOL_MUST_BE_OFF')
    m.api['revalidate']()
    completed=set(getattr(m,'inspection_completed_parts',[]))
    executed=getattr(m,'inspection_executed_parts',{})
    for part in j['reference_parts']:
        if part['id'] in completed:continue
        p=executed.get(part['id'],part['poses']);checkpoint=m.a.out/'progress.json'
        if checkpoint.exists():
            progress=read(checkpoint);label=progress.get('id','')
            if label==part['id'] or label.startswith(part['id']+'_resume_'):
                tail=progress.get('remaining_poses',[])
                if len(tail)<2 or len(tail)>len(p) or tail!=p[-len(tail):]:raise ValueError('INSPECTION_PROGRESS_NOT_ORIGINAL_ROUTE_SUFFIX')
                p=tail;m.log('inspection_resume_suffix',id=part['id'],remaining=len(p),no_work_credit=True)
        if len(p)<2:raise ValueError('INSPECTION_REFERENCE_EMPTY')
        if m.a.vehicle=='ackermann':
            dh=math.atan2(math.sin(p[1][2]-p[0][2]),math.cos(p[1][2]-p[0][2]));k=max(-1/2.11,min(1/2.11,dh/max(.001,math.dist(p[0][:2],p[1][:2]))));m.api['align'](k)
        from inspection_obstacles17 import conflicts
        moving=conflicts(m.st,p) if hasattr(m,'st') else []
        candidate=os.environ.get('S17_PREDICTION','disabled')!='disabled'
        # Inspection permits a spatial detour inside the authorized corridor.
        # Intermediate chunk endpoints are scheduling aids, not service points.
        # Keep each real segment end exact and record every temporary offset.
        from dynamic_envelope17 import inspection_target
        if part['id'] not in executed:
            target,offset=inspection_target(m.st,p,terminal=part is j['reference_parts'][-1])
            if offset:
                m.log('inspection_temporary_goal',id=part['id'],reference=p[-1],target=target,lateral_offset_m=offset,reason='OBSERVED_DYNAMIC_OCCUPANCY_AT_CHUNK_BOUNDARY',no_work_credit=True)
                p=copy.deepcopy(p);p[-1]=target
            executed[part['id']]=p;m.inspection_executed_parts=executed
        navigate=True
        m.log('inspection_reference',id=part['id'],action='NavigateToPose' if navigate else 'FollowPath',controller=m.a.controller,no_work_credit=True,reason='CANDIDATE_PREDICTED_COST_ONLINE_PLAN' if candidate else 'OBSERVED_MOVING_OBJECT_NEAR_REFERENCE' if moving else 'BOUNDED_REPLAN' if replan else 'QUALIFIED_REFERENCE',observed_conflicts=moving)
        r=m.api['follow'](part['id'],p,nav=navigate)
        if r['status']!=4:raise RuntimeError('INSPECTION_NAV_FAILED '+str(r.get('error_code')))
        completed.add(part['id']);m.inspection_completed_parts=sorted(completed)
        dump(m.a.out/'inspection_progress17.json',dict(plan_revision=m.c['plan_revision'],completed_parts=m.inspection_completed_parts,no_work_credit=True))
        m.log('inspection_part_complete',id=part['id'])
