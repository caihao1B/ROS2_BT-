"""Local-input ablation under one frozen lidar map and common initialization.

The wheel/IMU group removes lidar *increments* from the local EKF; the global
lidar map remains common to every group. This is not a lidar-free robot claim.
GNSS is withheld only after a verified initial global reference is established.
"""
from common17 import *
from overlay17 import build,replace
import argparse,yaml

def configure_scene(out,method):
    path=Path(out)/'fused.yaml';cfg=yaml.safe_load(path.read_text());p=cfg['/**']['ros__parameters']
    if method=='wheel_imu':
        for k in list(p):
            if k.startswith('twist1'):del p[k]
        p['twist0']='/s05/inputs/wheel_baseline'
    if method in ['visual','gnss']:
        p.update(twist2='/s17/inputs/visual_increment',twist2_config=[False]*6+[True]*6+[False]*3,twist2_queue_size=5,twist2_rejection_threshold=4.)
    path.write_text(yaml.safe_dump(cfg,sort_keys=False))
    vo=dict(use_sim_time=True,frame_id='base_link',odom_frame_id='s17_visual_odom',publish_tf=False,guess_frame_id='',ground_truth_frame_id='',subscribe_rgbd=True,approx_sync=False,qos=1,sync_queue_size=5,wait_for_transform=.1,**{'Reg/Strategy':'0','Odom/Strategy':'0','Odom/FilteringStrategy':'0','Odom/ResetCountdown':'0','Odom/GuessMotion':'true','OdomF2M/MaxSize':'1000','Vis/FeatureType':'8','Vis/MinInliers':'20','Vis/MaxFeatures':'400'})
    (Path(out)/'visual17.yaml').write_text(yaml.safe_dump({'/**':{'ros__parameters':vo}},sort_keys=False))

def run(a):
    if not a.out.resolve().is_relative_to(RESULTS) or not (a.seed in read(EVAL/'seeds.json')['paired'] or (a.seed==17310 and a.out.name.startswith('study_probe_'))):raise ValueError('EXPERIMENT_SCOPE')
    os.environ.update(S17_CLUSTER='A1',S17_CONTROLLER='DWB',S17_OPERATION='inspection',S17_BENCHMARK='controller_prefix',S17_LOCAL_INPUT=a.method,S17_PREDICTION='disabled')
    os.environ.pop('S17_RESUME_MANIFEST',None);os.environ.pop('S17_SURVEY_STATIC',None)
    from inputs17 import snapshot
    snapshot(a.out);scripts=build(a.out/'overlay',RESULTS/'qualification/stations.json',a.rtf,True)
    p=scripts/'scene14.py';s=p.read_text();s=replace(s,"    compose13(out, kind, 'navigation_only', session)","    compose13(out, kind, 'navigation_only', session)\n    from localization_compare17 import configure_scene\n    configure_scene(out,os.environ['S17_LOCAL_INPUT'])");p.write_text(s)
    original=BASE/'ws/src/gnss_supervisor/scripts/source_adapter.py';s=original.read_text()
    s=replace(s,"    def fault(t):return next((x for x in faults if x['start']<=t<x['end']),{})", """    def navigation(m):
        d=json.loads(m.data)
        if d.get('navigation_allowed') and 'study_start' not in state:state['study_start']=d['t']
    n.create_subscription(String,'/s08/status',navigation,10)
    def fault(t):
        if os.environ['S17_LOCAL_INPUT']!='gnss' and 'study_start' in state and t>=state['study_start']+2.:return {'gnss':'dropout'}
        return {}""")
    s='import os\n'+s;(scripts/'localization_source17.py').write_text(s)
    p=scripts/'regional_source14.py';p.write_text(replace(p.read_text(),'import source_adapter','import localization_source17 as source_adapter'))
    if a.method in ['visual','gnss']:
        injection="""        launch('visual_odometry',['/opt/ros/jazzy/lib/rtabmap_odom/stereo_odometry','--ros-args','-r','__node:=s17_visual_odometry','-r','__ns:=/s17/visual','--params-file',str(out/'scene/visual17.yaml'),'-r','rgbd_image:=/s07/inputs/stereo','-r','odom:=/s17/visual/raw','-r','odom_info:=/s17/visual/info','-r','/tf:=/s17/internal/visual_tf','--log-level','warn'])
        launch('visual_gate',['/usr/bin/python3',str(S14PKG/'s17_source/scripts/visual_gate17.py'),'--out',str(out/'visual_gate')])
        launch('mapping_inputs',"""
        p=scripts/'runtime14.py';s=p.read_text();s=replace(s,"    module = types.ModuleType('s14_pipeline')",'    source=replace_once(source,"        launch(\'mapping_inputs\',",'+repr(injection)+')\n    module = types.ModuleType(\'s14_pipeline\')');p.write_text(s)
    sys.path.insert(0,str(scripts));from runtime14 import run_case
    runtime=run_case(a.out,'zone_a','nominal_spray_cluster',200,a.wall,seed=a.seed,gui=False,kind='diff',controller='DWB')
    from evaluate17 import evaluate
    spec=dict(id=a.out.name,seed=a.seed,cluster='A1',vehicle='diff',controller='DWB',operation='inspection',scope='PAIRED_PREFIX')
    m=evaluate(a.out,spec,BASE/'build_s17/monitor');info=rows(a.out/'visual_gate/frames.jsonl');quality=rows(a.out/'visual_gate/quality.jsonl');gnss=[r for r in rows(a.out/'sources/events.jsonl') if r.get('kind')=='gnss']
    m.update(scenario='localization',method=a.method,task_completed=m['complete'],task_hash=sha(a.out/'client/inspection_contract17.json'),limits_hash=canonical(dict(vehicle=sha(BASE/'config/vehicles/diff.yaml'),gate=sha(BASE/'config/controller_profiles.yaml'),seconds=200,rtf=a.rtf)),formal_credit=False,visual_frames=len(info),visual_increment_frames=sum(x['increment_valid'] for x in quality),visual_lost_frames=sum(x['lost'] for x in info),gnss_dropout_frames=sum(x['mode']=='no_fix' for x in gnss),input_scope='wheel+IMU; add lidar increments; add gated stereo VO increments; add continued GNSS supervision. Frozen lidar map and trusted initialization common to all. Raw diagnostic producers remain running in all groups.')
    dump(a.out/'metrics17.json',m);return m

if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path,required=True);ap.add_argument('--seed',type=int,required=True);ap.add_argument('--method',choices=['wheel_imu','lidar','visual','gnss'],required=True);ap.add_argument('--rtf',type=float,default=1.);ap.add_argument('--wall',type=float,default=600);a=ap.parse_args()
    if not .4<=a.rtf<=1 or not 150<=a.wall<=750:ap.error('BOUNDED_STUDY_REQUIRED')
    m=run(a);raise SystemExit(0 if m['safety'] else 1)
