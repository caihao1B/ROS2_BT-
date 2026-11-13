"""One bounded recorded episode. Separate journal and no formal S17 PASS credit."""
from common_showcase import *
import argparse,subprocess,signal,fcntl,datetime,math

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--cluster',choices=['A1','A2','A3','B1','B2','B3'],required=True);ap.add_argument('--operation',choices=['work','inspection'],default='work');ap.add_argument('--plan',type=Path,default=OUT/'v1/mission_plan.json');ap.add_argument('--probe',action='store_true',help='predeclared repair, excluded from final recording budget');ap.add_argument('--seconds',type=int);ap.add_argument('--child',type=Path);ap.add_argument('--bounded-attempt',action='store_true',help='inspection only: attempt unchanged full route under remaining hard wall budget; completion not promised');ap.add_argument('--resume-from',type=Path,help='explicit restart after checking a safely stopped episode');a=ap.parse_args()
    plan=read(a.plan);kind='diff' if a.cluster[0]=='A' else 'ackermann';controller='DWB' if kind=='diff' else 'RPP'
    if a.child:
        out=a.child;spec=read(out/'spec.json');os.environ.update(S17_CLUSTER=a.cluster,S17_OPERATION=a.operation,S17_CONTROLLER=controller,S17_PREDICTION='disabled',SHOWCASE_PLAN=str(a.plan.resolve()),SHOWCASE_RECORD='1');os.environ.pop('S17_BENCHMARK',None)
        os.environ.pop('S17_RESUME_MANIFEST',None)
        if spec.get('resume_from'):
            from showcase_resume import prepare
            os.environ['S17_RESUME_MANIFEST']=str(prepare(ROOT/spec['resume_from'],out,a.cluster))
        from inputs17 import snapshot
        snapshot(out)
        from showcase_overlay import compose
        overlay=compose(out/'overlay');sys.path.insert(0,str(overlay))
        from runtime14 import run_case
        result=run_case(out,'zone_a','nominal_spray_cluster' if kind=='diff' else 'nominal_mow_cluster',spec['seconds'],spec['wall_limit_s']-30,seed=spec['seed'],gui=False,kind=kind,controller=controller)
        return 0 if result.get('success') else 1
    lock=(ROOT/'native_20261108/runs/session.lock').open('a');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    if os.getpgrp()!=os.getpid():os.setsid()
    sys.path.insert(0,str(BASE/'ws/src/project_baseline/scripts'));from session_owner import save_session
    from binding_check17 import validate
    validate()
    p=plan['zones'][a.cluster[0]]['plans'][int(a.cluster[1])-1]
    nominal=p['timing']['total_s'] if a.operation=='work' else next(r['timing']['total_s'] for r in plan['connector_routes'] if r['vehicle']==kind and r['direction']==('A_to_B' if kind=='diff' else 'B_to_A'))
    if a.resume_from and a.operation=='inspection':
        from inspection_recovery import remaining_nominal
        nominal=remaining_nominal(a.resume_from.resolve())
    seconds=a.seconds or min(1800,math.ceil(nominal*1.45+120))
    if not 100<=seconds<=1800:raise ValueError('SHOWCASE_SIM_BOUND_100_TO_1800_REQUIRED')
    journal=OUT/'recording_budget.json';budget=read(journal) if journal.exists() else dict(limit_wall_s=10800,consumed_wall_s=0.,episodes=[])
    remaining=budget['limit_wall_s']-budget['consumed_wall_s'];wall=min(3600,math.ceil(seconds/.5+90))
    if not a.probe:
        if remaining<nominal/.55+90 and not (a.bounded_attempt and a.operation=='inspection' and remaining>=300):raise ValueError('FINAL_RECORDING_BUDGET_INSUFFICIENT')
        wall=min(wall,int(remaining)-45)  # Reserve bounded cancellation/cleanup.
    out=BASE/'results/S17'/('showcase_'+('probe_' if a.probe else '')+a.cluster+'_'+a.operation+'_'+datetime.datetime.now().strftime('%Y%m%d_%H%M%S'));out.mkdir()
    spec=dict(id=out.name,vehicle=kind,cluster=a.cluster,controller=controller,operation=a.operation,scope='INDEPENDENT_SHOWCASE_ONLY',formal_credit=False,showcase_plan=str(a.plan.resolve()),plan_revision=plan['plan_revision'],seed=17400+int(a.cluster[1])+(10 if kind=='ackermann' else 0),seconds=seconds,wall_limit_s=wall,nominal_s=nominal,repair_probe=a.probe,bounded_attempt=a.bounded_attempt,conservative_nominal_wall_s=nominal/.55+90)
    if a.resume_from:
        previous=a.resume_from.resolve()
        if not previous.is_relative_to(BASE/'results/S17'):raise ValueError('RESUME_LOCAL_SCOPE_REQUIRED')
        spec['resume_from']=str(previous.relative_to(ROOT))
    dump(out/'spec.json',spec);dump(OUT/'current.json',dict(run=str(out.relative_to(ROOT)),spec=spec));save_session(BASE/'runtime/S17.json',str(Path(__file__).resolve()),out.name);print(str(out),flush=True)
    args=[str(BASE/'venvs/scene/bin/python'),str(Path(__file__).resolve()),'--cluster',a.cluster,'--operation',a.operation,'--plan',str(a.plan.resolve()),'--child',str(out)]
    child=None;started=time.monotonic()
    def stop(s,f):raise KeyboardInterrupt
    for s in [signal.SIGINT,signal.SIGTERM,signal.SIGALRM]:signal.signal(s,stop)
    signal.setitimer(signal.ITIMER_REAL,wall)
    try:
        with (out/'launch.log').open('w') as f:child=subprocess.Popen(args,stdout=f,stderr=subprocess.STDOUT);code=child.wait();child=None
        # The owned runtime has exited and performed cleanup. Do not interrupt
        # offline scoring with the live-simulation watchdog. Accounting remains
        # conservative: elapsed postprocessing is still recorded in finally.
        signal.setitimer(signal.ITIMER_REAL,0)
        dump(out/'command.json',dict(argv=args,exit_code=code,formal_credit=False))
        from evaluate_showcase import evaluate
        result=evaluate(out,spec);print(json.dumps({k:result.get(k) for k in ['case','status','reason','required_points','confirmed_points','coverage_fraction','safety','localization_pass','wall_seconds','manual_handoff']}),flush=True)
        return 0 if result['status']=='PASS' else 1
    finally:
        signal.setitimer(signal.ITIMER_REAL,0)
        if child and child.poll() is None:
            child.send_signal(signal.SIGINT)
            try:child.wait(timeout=30)
            except subprocess.TimeoutExpired:child.terminate();child.wait(timeout=10)
        elapsed=time.monotonic()-started;dump(out/'wall_accounting.json',dict(elapsed_s=elapsed,repair_probe=a.probe,final_budget_charged=not a.probe))
        if not a.probe:
            budget['consumed_wall_s']+=elapsed;budget['episodes'].append(dict(run=str(out.relative_to(ROOT)),wall_seconds=elapsed));dump(journal,budget)
        session=BASE/'runtime/S17.json'
        if session.exists() and read(session).get('run_id')==out.name:session.unlink()
if __name__=='__main__':raise SystemExit(main())
