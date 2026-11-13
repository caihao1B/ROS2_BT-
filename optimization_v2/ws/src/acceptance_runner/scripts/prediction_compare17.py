"""New-seed actual two-point comparisons using frozen S15/S16 implementations.

This deliberately declares the S14 benchmark map and task. It cannot earn
six-cluster completion or continuous-map qualification credit.
"""
from common17 import *
import argparse,inspect,subprocess,os

def run(a):
    out=a.out.resolve()
    if not out.is_relative_to(RESULTS) or not (a.seed in read(EVAL/'seeds.json')['paired'] or (a.seed==17310 and out.name.startswith('study_probe_'))):raise ValueError('PAIRED_TEST_IDENTITY')
    sys.path.insert(0,str(BASE/'ws/src/experience_memory/scripts'))
    import runtime16,runtime15,scene15
    from analysis15 import evaluate_case
    once=runtime15.once
    source=inspect.getsource(runtime15.run_case)
    source=once(source,"seconds,wall,41,gui,'diff'", "seconds,wall,"+str(a.seed)+",gui,'diff'")
    scope=dict(runtime15.__dict__);exec(compile(source,str(Path(runtime15.__file__)),'exec'),scope)
    runtime15.run_case=scope['run_case']
    original_compose=scene15.compose
    def compose(*args):
        hint=original_compose(*args)
        from xml.etree import ElementTree as E
        p=Path(args[0])/'world.sdf';tree=E.parse(p);tree.getroot().find('world/physics/real_time_factor').text=str(a.rtf);tree.write(p,encoding='unicode');return hint
    scene15.compose=compose
    # The copied run function must use the currently selected memory adapter.
    def pipeline():
        scope['adapt']=runtime15.adapt
        return runtime15.pipeline()
    scope['pipeline']=pipeline
    if a.family=='prediction':
        if a.method not in ['no_prediction','imm','learned']:raise ValueError('METHOD')
        result=runtime15.run_case(out,a.scenario,a.method,'test',100,a.wall,False)
    else:
        if a.method not in ['full','event','cache']:raise ValueError('METHOD')
        snapshot=ROOT/read(BASE/'results/S16/latest.json')['report_path'];snapshot=snapshot.parent/'offline/initial.sqlite'
        if not snapshot.is_file():raise ValueError('TRAINING_ONLY_CACHE_SNAPSHOT_MISSING')
        result=runtime16.run_case(out,a.scenario,a.method,snapshot,100,a.wall,False)
        dump(out/'cache_source17.json',dict(path=str(snapshot.relative_to(ROOT)),sha256=sha(snapshot),training_only=True))
    m=evaluate_case(out)
    contacts=read(out/'evaluation/contacts.json');truth=rows(out/'evaluation/truth.jsonl');physical=rows(out/'tool_evaluation/physical.jsonl')
    clocks=[x['t'] for x in truth];last=max(clocks+[result.get('sim_seconds_observed',0)]+[x['t'] for x in physical]);dump(out/'monitor_input.json',dict(clock=clocks+[last],contacts=contacts,tool_stopped=m['stop_confirmed'],limit_s=100))
    subprocess.run([str(BASE/'build_s17/monitor'),str(out/'monitor_input.json'),str(out/'monitor.json')],check=False,timeout=20);independent=read(out/'monitor.json')
    resource=read(out/'resources.json');rtf=(resource[-1]['t']-resource[0]['t'])/(resource[-1]['wall']-resource[0]['wall']) if len(resource)>1 else None
    data=dict(status='PASS' if m['safe'] and independent['passed'] else 'FAIL',experiment_executed=True,task_completed=m['completed'],scope='NEW_SEED_TWO_POINT_BENCHMARK_ON_FROZEN_S14_MAP; not six clusters',seed=a.seed,scenario=a.scenario,family=a.family,method=a.method,task_hash=m['benchmark_task_sha'],fixture_hash=m['fixture_sha'],limits_hash=canonical(dict(seconds=100,rtf=a.rtf,gate=sha(BASE/'config/controller_profiles.yaml'),vehicle=sha(BASE/'config/vehicles/diff.yaml'))),sim_seconds_including_stop=last,rtf=rtf,original_measurement=m,independent_monitor=independent,formal_credit=False)
    if a.family=='memory':
        forecasts=rows(out/'prediction/forecasts.jsonl');good=[x for x in forecasts if 'memory' in x]
        data['memory']=dict(cpu_ms=sum(x['cpu_ms'] for x in good),wall_ms=sum(x['wall_ms'] for x in good),hits=sum(d['lookup']['hit'] for x in good for d in x['memory']['decisions']),learned_calls=sum(x['memory']['learned_calls'] for x in good),frames=len(good))
    dump(out/'metrics17.json',data);return data

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True);p.add_argument('--seed',type=int,required=True);p.add_argument('--family',choices=['prediction','memory'],required=True);p.add_argument('--method',required=True);p.add_argument('--scenario',choices=['turning','occlusion'],required=True);p.add_argument('--rtf',type=float,default=1.);p.add_argument('--wall',type=float,default=360);a=p.parse_args()
    if not .4<=a.rtf<=1 or not 120<=a.wall<=600:p.error('BOUNDED_PAIR_REQUIRED')
    raise SystemExit(0 if run(a)['status']=='PASS' else 1)
