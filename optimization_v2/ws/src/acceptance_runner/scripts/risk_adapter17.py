"""Development-only S15 IMM integration on S17 full tasks, no benchmark swap.

Reuse only the reviewed risk recovery changes. The S15 two-point benchmark
make_jobs/run replacements are explicitly excluded; S17 obligations survive.
"""
from common17 import *
from overlay17 import replace
import ast,math,yaml

def mission(source):
    upstream=BASE/'ws/src/prediction_risk_guard/scripts/mission15.py';tree=ast.parse(upstream.read_text());applied=[]
    for node in tree.body:
        if not isinstance(node,ast.Assign) or not any(isinstance(t,ast.Name) and t.id=='source' for t in node.targets):continue
        call=node.value
        if not isinstance(call,ast.Call) or not isinstance(call.func,ast.Name) or call.func.id!='once':continue
        old,new=map(ast.literal_eval,call.args[1:])
        if old in ['    def make_jobs(self):','    def run(self):\n        self.save()']:continue
        source=replace(source,old,new);applied.append(old)
    if len(applied)!=7:raise ValueError('S15_RECOVERY_ADAPTER_CHANGED '+str(len(applied)))
    return source

def pipeline(source):
    source=replace(source,"str(BASE/'ws/install/mission_bt/lib/mission_bt/command_gate')","str(BASE/'ws/install/prediction_risk_guard/lib/prediction_risk_guard/command_gate15')")
    source=replace(source,"str(BASE/'ws/install/mission_bt/lib/mission_bt/mission_bt')","str(BASE/'ws/install/prediction_risk_guard/lib/prediction_risk_guard/mission_bt15')")
    extra="""        from risk_adapter17 import config
        config(out,kind)
        launch('prediction',['/usr/bin/python3',str(BASE/'ws/src/motion_prediction/scripts/prediction_node15.py'),'--out',str(out/'prediction'),'--method',os.environ['S17_PREDICTION'],'--fault','none'])
        launch('prediction_guard',[str(BASE/'ws/install/prediction_risk_guard/lib/prediction_risk_guard/guard15'),'--ros-args','-p','use_sim_time:=true','-p','config_file:='+str(out/'prediction_config.json'),'-p','log_file:='+str(out/'risk.jsonl')])
        launch('fusion',"""
    return replace(source,"        launch('fusion',",extra)

def config(out,kind):
    cfg=yaml.safe_load((BASE/'config/prediction.yaml').read_text());terrain=yaml.safe_load((BASE/'config/traversability.yaml').read_text())['profiles'][kind]
    cfg['risk']['robot_radius_m']=max(cfg['risk']['robot_radius_m'],max(math.hypot(*p) for p in terrain['footprint'])+terrain['margin_m'])
    dump(out/'prediction_config.json',cfg)
