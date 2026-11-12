"""Serial actual Gazebo pipeline; keep S14 sources and receipt immutable."""
from common15 import *
import types,os,signal,inspect
once=add14()
import runtime14
from owned14 import cleanup

def adapt(source):
    source=once(source,'from scene14 import compose','from scene15 import compose')
    source=once(source,"'tool_sim','navigation_adapter']","'tool_sim','navigation_adapter','motion_prediction']")
    source=once(source,"str(BASE/'ws/install/mission_bt/lib/mission_bt/command_gate')","str(BASE/'ws/install/prediction_risk_guard/lib/prediction_risk_guard/command_gate15')")
    source=once(source,"str(BASE/'ws/install/mission_bt/lib/mission_bt/mission_bt')","str(BASE/'ws/install/prediction_risk_guard/lib/prediction_risk_guard/mission_bt15')")
    source=once(source,"str(S14PKG/'scripts/worker14.py')","str(BASE/'ws/src/prediction_risk_guard/scripts/worker15.py')")
    source=once(source,"        launch('fusion',",'''        prediction_cfg=yaml.safe_load((BASE/'config/prediction.yaml').read_text());dump(out/'prediction_config.json',prediction_cfg)
        launch('prediction',['/usr/bin/python3',str(BASE/'ws/src/motion_prediction/scripts/prediction_node15.py'),'--out',str(out/'prediction'),'--method',os.environ['S15_METHOD'],'--fault',os.environ.get('S15_FAULT','none')])
        launch('prediction_guard',[str(BASE/'ws/install/prediction_risk_guard/lib/prediction_risk_guard/guard15'),'--ros-args','-p','use_sim_time:=true','-p','config_file:='+str(out/'prediction_config.json'),'-p','log_file:='+str(out/'risk.jsonl')])
        launch('dynamic_evaluation_bridge',[bridge,'/s09/evaluation/dynamic_poses@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'],evaluation)
        launch('dynamic_evaluator',['/usr/bin/python3',str(S09/'dynamic_evaluator.py'),'--out',str(out/'dynamic_evaluation')],evaluation)
        launch('fusion',''')
    source=once(source,"ready=['drive/bridge_ready'","ready=['prediction/ready','drive/bridge_ready'")
    source=once(source,"(BASE/'config/rviz/navigation.rviz').read_text()","(BASE/'config/rviz/prediction.rviz').read_text()")
    return source

def pipeline():
    text=inspect.getsource(runtime14.runtime)
    text=once(text,"    exec(compile(source, module.__file__, 'exec'), module.__dict__)","    source=S15_ADAPT(source)\n    exec(compile(source, module.__file__, 'exec'), module.__dict__)")
    scope=dict(runtime14.__dict__);scope['S15_ADAPT']=adapt
    exec(compile(text,str(Path(__file__)),'exec'),scope)
    return scope['runtime'](False,False,True)

def run_case(out,scenario,method,split='development',seconds=75,wall=300,gui=False,collect=False,fault='none'):
    if scenario not in ['turning','stop_go','yield','occlusion']:raise ValueError('Unknown motion scenario')
    keys=dict(S14_REGISTERED_REGION='A',S15_SCENARIO=scenario,S15_METHOD=method,S15_SPLIT=split,S15_FAULT=fault)
    old={key:os.environ.get(key) for key in keys};os.environ.update(keys);out=Path(out)
    try:
        result=pipeline().run_case(out,'zone_a',('collect_' if collect else 'benchmark_')+scenario,seconds,wall,41,gui,'diff')
        result.update(scope='OBSERVATION_COLLECTION' if collect else 'TWO_POINT_DYNAMIC_BENCHMARK',method=method,scenario=scenario,split=split,fault=fault)
        dump(out/'runtime.json',result);return result
    finally:
        handlers={s:signal.getsignal(s) for s in [signal.SIGINT,signal.SIGTERM,signal.SIGALRM]}
        for s in handlers:signal.signal(s,lambda s,f:None)
        try:
            c=cleanup(out)
            if c.get('survivors'):raise RuntimeError('OWNED_CLEANUP_FAILED')
        finally:
            for s,h in handlers.items():signal.signal(s,h)
            for key,value in old.items():
                if value is None:os.environ.pop(key,None)
                else:os.environ[key]=value
