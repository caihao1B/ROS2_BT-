"""Run the original task stack in a new task-owned result directory."""
from common_r1 import *
import argparse,shutil,math
from registry_r1 import validate_task

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--spec',type=Path,required=True);ap.add_argument('--out',type=Path,required=True)
    ap.add_argument('--seconds',type=int,required=True);ap.add_argument('--wall',type=float,required=True);ap.add_argument('--rtf',type=float,required=True);ap.add_argument('--gui',action='store_true');a=ap.parse_args()
    if not 180<=a.seconds<=1800 or not 1<=a.wall<=5000 or not math.isfinite(a.rtf) or not .4<=a.rtf<=1:raise ValueError('CHILD_BUDGET_OUT_OF_RANGE')
    spec=read(a.spec);profile,expected=validate_task(spec['task_id'],spec['cluster'])
    for key in ['task_version','vehicle','operation','controller','seed','scope','formal_credit']:
        if expected[key]!=spec[key]:raise ValueError('CHILD_SPEC_MISMATCH '+key)
    if a.out.resolve().parent!=(APP/'results'/spec['task_id']).resolve() or a.spec.resolve()!=a.out.resolve()/'spec.json':raise ValueError('CHILD_RESULT_SCOPE')
    for key in list(os.environ):
        if key.startswith('S17_'):os.environ.pop(key)
    os.environ.update(S17_CLUSTER=spec['cluster'],S17_OPERATION=spec['operation'],S17_CONTROLLER=spec['controller'],S17_PREDICTION='disabled')
    from inputs17 import snapshot
    from overlay17 import build,replace
    snapshot(a.out)
    overlay=build(a.out/'overlay',BASE/'results/S17/qualification/stations.json',a.rtf,full=True)
    # Exact-match adaptation, separately archived. No old source file is edited.
    source=overlay/'runtime14.py';before=sha(source);text=source.read_text()
    original="    def launch(name,args,e=env,aux=False):\n"
    injected=original+"        from lifecycle_r1 import environment as r01_environment\n        e=r01_environment(name,e,out)\n"
    patch='    source=replace_once(source,'+repr(original)+','+repr(injected)+')\n'
    stop_original="        baseline.s04.s03.stop_all([p for p in children if p is not server])"
    stop_injected="        from lifecycle_r1 import close_owned_views\n        close_owned_views(commands,children,out)\n"+stop_original
    patch+='    source=replace_once(source,'+repr(stop_original)+','+repr(stop_injected)+')\n'
    text=replace(text,"    module = types.ModuleType('s14_pipeline')",patch+"    module = types.ModuleType('s14_pipeline')")
    source.write_text(text);shutil.copy2(PKG/'scripts/lifecycle_r1.py',overlay/'lifecycle_r1.py')
    dump(a.out/'adapter_r1.json',dict(upstream_generated_sha256=before,adapted_sha256=sha(source),
        helper_sha256=sha(overlay/'lifecycle_r1.py'),changes=['server-only process loader environment','generated RViz camera frame','owned RViz window close before ROS teardown'],
        source_world_sha256=profile['bindings']['world']['sha256'],task_version=profile['version']))
    sys.path.insert(0,str(overlay))
    from runtime14 import run_case
    r=run_case(a.out,'zone_a','nominal_spray_cluster' if spec['vehicle']=='diff' else 'nominal_mow_cluster',
        a.seconds,a.wall,seed=spec['seed'],gui=a.gui,kind=spec['vehicle'],controller=spec['controller'])
    return 0 if r.get('success') else 1

if __name__=='__main__':raise SystemExit(main())
