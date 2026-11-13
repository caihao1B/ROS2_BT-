"""Real fixtures and observation faults, scored separately from task success."""
from common17 import *
from overlay17 import build,replace
from xml.etree import ElementTree as E
import argparse,math,numpy as np

KINDS=['dynamic_crossing','persistent_block','low_obstacle','slope','gnss_false_fixed','gnss_occlusion','localization_loss','sensor_delay','tool_loss']
PHYSICAL=KINDS[:4]

def fixture(world,out):
    fault=os.environ['S17_FAULT']
    if fault not in PHYSICAL:return
    from inspection17 import route
    r=route('diff');poses=next(s['poses'] for s in r['segments'] if s['kind']!='ROTATE')
    arc=0.;p=poses[0]
    for before,p in zip(poses,poses[1:]):
        arc+=math.dist(before[:2],p[:2])
        if arc>=(6. if fault=='slope' else 4.):break
    x,y,yaw=p;size={'persistent_block':[.5,12.,1.8],'low_obstacle':[.35,.55,.06],'dynamic_crossing':[.45,.45,1.5]}.get(fault)
    name='s17_fixture_'+fault;dynamic=fault=='dynamic_crossing';center=[x,y,.75 if dynamic else (size[2]/2 if size else 0.)]
    phase=math.pi/4;amplitude=[-2*math.sin(yaw),2*math.cos(yaw),0]
    spawn=[x+amplitude[0]*math.sin(phase),y+amplitude[1]*math.sin(2*phase),.75] if dynamic else center
    model=E.SubElement(world,'model',name=name);E.SubElement(model,'static').text='false' if dynamic else 'true';E.SubElement(model,'pose').text=' '.join(map(str,[*spawn,0,0,yaw]));link=E.SubElement(model,'link',name='body')
    if dynamic:
        inertial=E.SubElement(link,'inertial');E.SubElement(inertial,'mass').text='20';I=E.SubElement(inertial,'inertia')
        for k,v in [('ixx',4.0875),('iyy',4.0875),('izz',.675),('ixy',0),('ixz',0),('iyz',0)]:E.SubElement(I,k).text=str(v)
        motion=world.find("plugin[@name='s02::ScriptedMotion']")
        if motion is None:raise ValueError('DYNAMIC_PLUGIN_MISSING')
        actor=E.SubElement(motion,'actor')
        for k,v in dict(name=name,center=' '.join(map(str,center)),amplitude=' '.join(map(str,amplitude)),period='24',phase=str(phase)).items():E.SubElement(actor,k).text=v
    mesh=None
    if fault=='slope':
        # A physical triangular ramp: 18 degree ascending and descending faces.
        length=4.;width=6.;height=2*math.tan(math.radians(18));vertices=[(-2,-3,0),(2,-3,0),(0,-3,height),(-2,3,0),(2,3,0),(0,3,height)]
        faces=[(2,3,1),(6,5,4),(6,4,1),(3,6,1),(5,6,3),(2,5,3),(5,2,1),(4,5,1)]
        mesh=Path(out)/'fault_ramp.obj';mesh.write_text('\n'.join(['v '+' '.join(map(str,v)) for v in vertices]+['f '+' '.join(map(str,f)) for f in faces])+'\n')
    for tag in ['collision','visual']:
        part=E.SubElement(link,tag,name='body');geometry=E.SubElement(part,'geometry')
        if mesh:E.SubElement(E.SubElement(geometry,'mesh'),'uri').text=str(mesh.resolve())
        else:E.SubElement(E.SubElement(geometry,'box'),'size').text=' '.join(map(str,size))
        if tag=='visual':E.SubElement(E.SubElement(part,'material'),'diffuse').text='.9 .25 .12 1'
    dump(Path(out)/'fault_fixture17.json',dict(fault=fault,name=name,center_site=center,yaw=yaw,size=size,slope_deg=18 if mesh else None,physical_collision=True,ground_truth_online=False,prior_and_plan_unchanged=True,motion_amplitude=amplitude if dynamic else None))

def source_fault(s,fault):
    ph={'gnss_false_fixed':{'gnss':'false_fixed','bias_enu':[1.2,-.8,0.]},'gnss_occlusion':{'gnss':'dropout'},'localization_loss':{'gnss':'dropout','map':'dropout'}}[fault]
    block="""    def navigation(m):
        d=json.loads(m.data)
        if 'test_start' in state:return
        moving=d.get('navigation_allowed') and abs(d.get('v',0))>.03
        if not moving:state.pop('moving_since',None)
        elif 'moving_since' not in state:state['moving_since']=d['t']
        elif d['t']-state['moving_since']>=1.5:state['test_start']=d['t']
    n.create_subscription(String,'/s08/status',navigation,20)
    def fault(t):
        start=state.get('test_start')
        active=start is not None and start<=t<start+20.
        if active and not state.get('injected'):
            state['injected']=True;record(dict(kind='injection',fault=FAULT_NAME,t=t,end=start+20.,source='declared observation fault after admitted movement'))
        return PHASE if active else {}
""".replace('FAULT_NAME',repr(fault)).replace('PHASE',repr(ph))
    return replace(s,"    def fault(t):return next((x for x in faults if x['start']<=t<x['end']),{})",block.rstrip())

def adapt(scripts,fault):
    p=scripts/'scene14.py';s=p.read_text();s=replace(s,"    root.write(out / 'world.sdf', encoding='unicode')","    from faults17 import fixture\n    fixture(world,out)\n    root.write(out / 'world.sdf', encoding='unicode')");p.write_text(s)
    if fault.startswith('gnss_') or fault=='localization_loss':
        (scripts/'fault_source17.py').write_text(source_fault((BASE/'ws/src/gnss_supervisor/scripts/source_adapter.py').read_text(),fault))
        p=scripts/'regional_source14.py';p.write_text(replace(p.read_text(),'import source_adapter','import fault_source17 as source_adapter'))
    if fault=='tool_loss':
        p=scripts/'mission14.py';p.write_text(replace(p.read_text(),"self.a.case=='tool_loss' and (self.st.get('ledger')", "self.a.case=='nominal_spray_cluster' and (self.st.get('ledger')"))
    extra="        launch('fault_observer',['/usr/bin/python3',str(S14PKG/'s17_source/scripts/fault_io17.py'),'--out',str(out/'fault_inputs')]+(['--delay'] if os.environ['S17_FAULT']=='sensor_delay' else []))\n"
    extra+="        launch('fault_dynamic_bridge',[bridge,'/s02/evaluation/dynamic_poses@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'],evaluation)\n"
    extra+="        launch('fault_dynamic_evaluator',['/usr/bin/python3',str(S14PKG/'scripts/fault_dynamic17.py'),'--out',str(out/'fault_dynamic')],evaluation)\n"
    original=BASE/'ws/src/terrain_perception/scripts/dynamic_evaluator.py';(scripts/'fault_dynamic17.py').write_text(replace(original.read_text(),"'/s09/evaluation/dynamic_poses'","'/s02/evaluation/dynamic_poses'"))
    p=scripts/'runtime14.py';s=p.read_text();patch='    source=replace_once(source,"        launch(\'terrain\',",'+repr(extra+"        launch('terrain',")+')\n'
    if fault=='sensor_delay':patch+='    source=replace_once(source,"\'-p\',\'config_file:=\'+str(out/\'terrain.json\')])","\'-p\',\'config_file:=\'+str(out/\'terrain.json\'),\'-r\',\'/s04/perception/points:=/s17/delayed/perception\'])")\n'
    s=replace(s,"    module = types.ModuleType('s14_pipeline')",patch+"    module = types.ModuleType('s14_pipeline')");p.write_text(s)

def assess(out,fault,m):
    obs=rows(out/'online/observations.jsonl');sources=rows(out/'sources/events.jsonl');events=rows(out/'client/events.jsonl');gates=[r for r in obs if r['record_type']=='gate'];injections=[r for r in sources if r.get('kind')=='injection']
    exposure=False;response=False;detail={}
    if fault in ['gnss_false_fixed','gnss_occlusion','localization_loss'] and injections:
        start=injections[0]['t'];end=injections[0]['end'];statuses=[r for r in obs if r['record_type']=='status' and start+1<=r['t']<=end]
        exposure=len(statuses)>=10;detail['injection']=injections[0];detail['states']=sorted({r['state'] for r in statuses});detail['reasons']=sorted({r['reason'] for r in statuses});detail['gnss_reasons']=sorted({r.get('gnss_reason','') for r in statuses})
        if fault=='gnss_false_fixed':response=any('INNOVATION' in r.get('gnss_reason','') and r.get('gnss_reason')!='FIXED_AND_INNOVATION_VALID' for r in statuses)
        elif fault=='gnss_occlusion':response=any(r.get('navigation_allowed') and r.get('source')=='MAP' and r.get('gnss_reason')!='FIXED_AND_INNOVATION_VALID' for r in statuses)
        else:response=any(not r.get('navigation_allowed') for r in statuses) and any(g['reason']=='LOCALIZATION_STOP' and g['v']==0 for g in gates if start<=g['t']<=end)
    elif fault=='sensor_delay':
        delayed=[r for r in rows(out/'fault_inputs/delivery.jsonl') if r['requested_delay']>1]
        injection=read(out/'fault_inputs/injection17.json') if (out/'fault_inputs/injection17.json').exists() else {}
        moving=bool(injection) and abs(injection.get('observed_v',0))>.03 and any(abs(g.get('measured_v',0))>.03 for g in gates if injection['t']-.3<=g['t']<=injection['t'])
        exposure=moving and len(delayed)>=10 and all(r['original_header_preserved'] and r['delivered']-r['acquired']>=1.2-1e-6 for r in delayed)
        if delayed:
            lo=delayed[0].get('queued_at',delayed[0]['delivered']-delayed[0]['requested_delay']);hi=delayed[-1]['delivered'];response=any(g['v']==0 and g['reason'].startswith(('PERCEPTION','GEOMETRY')) for g in gates if lo<=g['t']<=hi)
        detail['injection']=injection;detail['motion_at_injection_verified']=moving;detail['delayed_frames']=len(delayed);detail['acquisition_age_s']=[min([r['delivered']-r['acquired'] for r in delayed],default=0),max([r['delivered']-r['acquired'] for r in delayed],default=0)]
    elif fault=='tool_loss':
        injected=[r for r in events if r['event']=='fault_injected' and r.get('mode')=='DROP_FEEDBACK'];mission=read(out/'client/mission.json');uncertain=[r for r in mission['tool_results'] if r.get('uncertain')]
        exposure=bool(injected);response=bool(uncertain) and not m['complete'] and m['monitor']['tool_stopped'];detail.update(injections=injected,uncertain_jobs=uncertain)
    elif fault in PHYSICAL:
        f=read(out/'scene/fault_fixture17.json');center=np.array(f['center_site'][:2])-[28,14];statuses=[r for r in obs if r['record_type']=='status'];seen=[]
        # This is offline fixture association. Runtime perception never reads
        # simulator geometry, actor position or the known fixture center.
        from scipy.spatial.transform import Rotation
        for frame in rows(out/'fault_inputs/terrain.jsonl'):
            if not statuses:break
            st=min(statuses,key=lambda r:abs(r['t']-frame['t']));transform=st.get('correction')
            if not transform or abs(st['t']-frame['t'])>.3:continue
            cols=frame['columns'];C={k:i for i,k in enumerate(cols)}
            cells=np.array(frame['cells'])
            if not len(cells):continue
            xyz=np.c_[cells[:,:2],np.zeros(len(cells))];world=Rotation.from_quat(transform[3:]).apply(xyz)+transform[:3]
            near=np.linalg.norm(world[:,:2]-center,axis=1)<(3.5 if fault in ['slope','dynamic_crossing'] else 1.)
            matched=cells[near]
            for row in matched:
                if row[C['diff_cost']]==100:seen.append(dict(t=frame['t'],slope=row[C['slope_deg']],height=row[C['top_relative_m']],step=row[C['step_m']]))
        admission=next((r['t'] for r in events if r['event']=='mission_admitted'),float('inf'))
        exposure=len(seen)>=3
        # Startup/stale-data stops are not evidence of reacting to this fixture.
        protective=[r for r in obs if r['record_type']=='perception' and r['t']>admission and r.get('reason')=='GEOMETRIC_PROTECTIVE_STOP']
        response=any(g['v']==0 and g['reason'].startswith(('PERCEPTION','GEOMETRY')) and any(abs(g['t']-r['t'])<=.5 for r in protective) for g in gates if g['t']>admission)
        detour=False
        if m['complete'] and exposure and m['safety']:
            from shapely.geometry import LineString,Point
            contract=read(out/'client/inspection_contract17.json')
            reference=LineString([p[:2] for s in contract['original_route']['segments'] for p in s['poses']])
            # Qualified reference is site ENU; measured estimator poses are map.
            estimated=[r for r in obs if r['record_type']=='pose' and r['t']>admission]
            detour=any(reference.distance(Point(r['pose'][0]+28,r['pose'][1]+14))>.3 for r in estimated)
            response=response or detour
        detail.update(protective_observations=len(protective),completed_observed_detour=detour,response_scope='post-admission geometric protective stop, or completed measured detour; startup and stale-data stops excluded')
        detail.update(observed_lethal_cells_near_fixture=len(seen),peak_slope_deg=max([x['slope'] for x in seen],default=0),physical_fixture=f)
        if fault=='slope':exposure=exposure and any(x['slope']>12 for x in seen)
        if fault=='dynamic_crossing':
            actor=[r for r in rows(out/'fault_dynamic/dynamic.jsonl') if r['name']==f['name']];lateral=[(np.array(r['xyz'][:2])-np.array(f['center_site'][:2]))@np.array([-math.sin(f['yaw']),math.cos(f['yaw'])]) for r in actor]
            crossing=bool(lateral) and min(lateral)<-.5 and max(lateral)>.5;exposure=exposure and crossing;detail['independent_crossing_verified']=crossing;detail['actor_samples']=len(actor)
    detail['gate_reasons']=sorted({g['reason'] for g in gates})
    safe=m['safety'];passed=safe and exposure and response
    return dict(status='PASS' if passed else 'FAIL',fault=fault,exposure_verified=bool(exposure),expected_response_verified=bool(response),safety=bool(safe),task_completed=m['complete'],task_status=m['status'],measurement=m,detail=detail,formal_credit=False,scope='bounded fault response; incomplete work remains residual, never normal completion')

def run(a):
    if not a.out.resolve().is_relative_to(RESULTS) or not (a.seed in read(EVAL/'seeds.json')['paired'] or (a.seed==17310 and a.out.name.startswith('study_probe_'))):raise ValueError('FAULT_SCOPE')
    inspection=a.fault!='tool_loss';os.environ.update(S17_CLUSTER='A2' if not inspection else 'A1',S17_CONTROLLER='DWB',S17_OPERATION='inspection' if inspection else 'cluster',S17_BENCHMARK='controller_prefix' if inspection else '',S17_FAULT=a.fault,S17_PREDICTION='disabled')
    os.environ.pop('S17_SURVEY_STATIC',None);os.environ.pop('S17_RESUME_MANIFEST',None)
    from inputs17 import snapshot
    snapshot(a.out);scripts=build(a.out/'overlay',RESULTS/'qualification/stations.json',a.rtf,True);adapt(scripts,a.fault);sys.path.insert(0,str(scripts));from runtime14 import run_case
    run_case(a.out,'zone_a','nominal_spray_cluster',200,a.wall,seed=a.seed,gui=False,kind='diff',controller='DWB')
    from evaluate17 import evaluate
    spec=dict(id=a.out.name,seed=a.seed,vehicle='diff',controller='DWB',cluster='A1' if inspection else 'A2',operation='inspection' if inspection else 'cluster',scope='FAULT_RESPONSE_NOT_FORMAL')
    m=evaluate(a.out,spec,BASE/'build_s17/monitor');dump(a.out/'navigation_measurement17.json',m);result=assess(a.out,a.fault,m);dump(a.out/'metrics17.json',result);return result

if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path,required=True);ap.add_argument('--seed',type=int,required=True);ap.add_argument('--fault',choices=KINDS,required=True);ap.add_argument('--wall',type=float,default=650);ap.add_argument('--rtf',type=float,default=1.);a=ap.parse_args()
    if not .4<=a.rtf<=1 or not 200<=a.wall<=800:ap.error('BOUNDED_FAULT_REQUIRED')
    raise SystemExit(0 if run(a)['status']=='PASS' else 1)
