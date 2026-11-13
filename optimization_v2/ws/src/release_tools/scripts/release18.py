"""Explicit manual-validation launcher; historical acceptance is never rewritten."""
from common18 import *
import argparse, subprocess, signal, time, fcntl, math

def current():
    return {f'S{i:02}':read(BASE/f'results/S{i:02}/latest.json').get('automated_status')
            if (BASE/f'results/S{i:02}/latest.json').exists() else 'NOT_RUN' for i in range(1,19)}

def doctor():
    add17()
    from binding_check17 import validate
    atlas,plan=validate()
    dependencies={k:str(v) for k,v in dict(ROS_DISTRO=os.environ.get('ROS_DISTRO'),
        ROS_DOMAIN_ID=os.environ.get('ROS_DOMAIN_ID'),GZ_PARTITION=os.environ.get('GZ_PARTITION')).items()}
    return dict(status='RUNTIME_INPUT_BINDINGS_VALID',dependencies=dependencies,
        map_revision=atlas['revision'],plan_revision=plan['plan_revision'],
        work_clusters=[z+str(i) for z in ['A','B'] for i in range(1,4)],
        full_map_tasks_accepted=False,stage_receipts=current(),
        notice='Version and input readiness only; S17 remains PARTIAL; no runtime was launched')

def spec_for(command,cluster,vehicle):
    if command=='spray':kind='diff';cluster=cluster or 'A1';operation='cluster'
    elif command=='mow':kind='ackermann';cluster=cluster or 'B1';operation='cluster'
    elif command=='inspect':kind=vehicle;cluster='A1' if kind=='diff' else 'B1';operation='inspection'
    else:raise ValueError('UNKNOWN_OPERATION')
    if cluster not in ['A1','A2','A3','B1','B2','B3'] or (cluster[0]=='A')!=(kind=='diff'):
        raise ValueError('CLUSTER_CHASSIS_MISMATCH')
    return dict(cluster=cluster,vehicle=kind,operation=operation,
        controller='DWB' if kind=='diff' else 'RPP',seed=17023,
        scope='S18_MANUAL_VALIDATION_NOT_FORMAL',formal_credit=False)

def verified_restart(path):
    path=path.resolve()
    if not path.is_relative_to(BASE/'results/S17') or not path.name.startswith('release18_'):
        raise ValueError('USE_S18_OWNED_RUN_FOR_RESTART; older runs use their original verified entrypoint')
    metric=read(path/'metrics17.json');runtime=read(path/'runtime.json');clean=read(path/'cleanup14.json')
    if not metric.get('safety') or not metric.get('localization_pass') or runtime.get('tool_shutdown',{}).get('state')!='STOPPED' or clean.get('survivors'):
        raise ValueError('RESTART_REQUIRES_SAFE_LOCALIZED_STOPPED_SOURCE')
    spec=read(path/'spec.json')
    if spec['operation']=='inspection':raise ValueError('INSPECTION_SUFFIX_RESTART_NOT_QUALIFIED_IN_RELEASE_ENTRY; work restart only')
    return spec

def termination_label(result,error):
    result=dict(result,release_termination=error or 'NATURAL_TASK_EXIT')
    if result.get('status')=='PARTIAL' and error:
        result['scoring_reason']=result.get('reason')
        result['reason']='REQUESTED_STOP' if error=='RELEASE_STOP_REQUEST' else 'RELEASE_WALL_TIMEOUT'
    return result

def launch(a):
    if not 60<=a.sim_seconds<=1800 or not 90<=a.wall_timeout<=5000:raise ValueError('BOUNDED_SIM_60_1800_AND_WALL_90_5000_REQUIRED')
    if not math.isfinite(a.rtf) or not .4<=a.rtf<=1.:raise ValueError('RTF_0_4_TO_1_REQUIRED')
    spec=verified_restart(a.from_run) if a.command=='resume' else spec_for(a.command,a.cluster,a.vehicle)
    doctor()
    lock=(ROOT/'native_20261108/runs/session.lock').open('a');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    if os.getpgrp()!=os.getpid():os.setsid()
    run_id='release18_'+stamp();out=BASE/'results/S17'/run_id;out.mkdir(parents=True)
    spec=dict(spec,id=run_id);dump(out/'spec.json',spec)
    session=BASE/'runtime/S18.json';session_module().save_session(session,str(Path(__file__).resolve()),run_id)
    args=[str(BASE/'venvs/scene/bin/python'),str(BASE/'ws/src/acceptance_runner/scripts/one17.py'),
          '--spec',str(out/'spec.json'),'--out',str(out),'--wall',str(a.wall_timeout-25),
          '--seconds',str(a.sim_seconds),'--rtf',str(a.rtf)]
    if a.gui:args.append('--gui')
    if a.command=='resume':args+=['--resume-from',str(a.from_run.resolve())]
    child=None;started=time.monotonic();code=None;error=None;launcher_hash=sha(Path(__file__))
    def interrupted(sig,frame):raise KeyboardInterrupt('RELEASE_WALL_TIMEOUT' if sig==signal.SIGALRM else 'RELEASE_STOP_REQUEST')
    previous={s:signal.signal(s,interrupted) for s in [signal.SIGINT,signal.SIGTERM,signal.SIGALRM]}
    signal.setitimer(signal.ITIMER_REAL,a.wall_timeout)
    dump(RESULTS/'last_run.json',dict(run=str(out.relative_to(ROOT)),spec=spec))
    print(str(out),flush=True)
    try:
        with (out/'launch.log').open('w') as f:
            child=subprocess.Popen(args,stdout=f,stderr=subprocess.STDOUT)
            code=child.wait()
        signal.setitimer(signal.ITIMER_REAL,0)
    except KeyboardInterrupt as exc:error=str(exc)
    finally:
        signal.setitimer(signal.ITIMER_REAL,0)
        for s in previous:signal.signal(s,lambda *_:None)
        if child and child.poll() is None:
            child.send_signal(signal.SIGINT)
            try:child.wait(timeout=25)
            except subprocess.TimeoutExpired:
                child.terminate()
                try:child.wait(timeout=10)
                except subprocess.TimeoutExpired:child.kill();child.wait(timeout=5)
        # Verify run-specific ownership during cleanup, never signal other ROS sessions.
        add17();sys.path.insert(0,str(BASE/'ws/src/mission_bt/scripts'))
        from owned14 import cleanup
        cleaned=cleanup(out)
        dump(out/'release_command.json',dict(argv=args,exit_code=code,error=error,launcher_sha256=launcher_hash,
            wall_seconds=time.monotonic()-started,formal_credit=False,cleanup=cleaned))
        if session.exists() and read(session).get('run_id')==run_id:session.unlink()
        for s,h in previous.items():signal.signal(s,h)
    result=dict(status='FAIL',reason=error or 'EVALUATION_PENDING')
    if (out/'runtime.json').exists():
        try:
            from evaluate17 import evaluate
            result=evaluate(out,spec,BASE/'build_s17/monitor')
        except Exception as exc:result=dict(status='FAIL',reason='SCORING_ERROR '+str(exc))
    result=termination_label(result,error)
    dump(out/'release_result.json',result)
    print(json.dumps({k:result.get(k) for k in ['status','reason','safety','localization_pass','confirmed_points','coverage_fraction']},ensure_ascii=False))
    return 0 if result.get('status')=='PASS' else 2

def main():
    ap=argparse.ArgumentParser(description='S18 partial delivery: explicit bounded validation, not full-map acceptance')
    sub=ap.add_subparsers(dest='command',required=True)
    for cmd in ['status','doctor','audit','stop','learning']:
        p=sub.add_parser(cmd)
        if cmd=='audit':p.add_argument('--out',type=Path)
    for cmd in ['spray','mow','inspect','resume']:
        p=sub.add_parser(cmd);p.add_argument('--cluster',choices=['A1','A2','A3','B1','B2','B3'])
        p.add_argument('--vehicle',choices=['diff','ackermann'],default='diff');p.add_argument('--gui',action='store_true')
        p.add_argument('--sim-seconds',type=int,default=180);p.add_argument('--wall-timeout',type=int,default=540);p.add_argument('--rtf',type=float,default=.55)
        if cmd=='resume':p.add_argument('--from-run',type=Path,required=True)
    a=ap.parse_args()
    if a.command=='status':print(json.dumps(dict(receipt_status_not_hash_rechecked=current(),last_run=read(RESULTS/'last_run.json') if (RESULTS/'last_run.json').exists() else None),indent=2,ensure_ascii=False));return 0
    if a.command=='doctor':print(json.dumps(doctor(),indent=2,ensure_ascii=False));return 0
    if a.command=='stop':r=session_module().stop_session(BASE/'runtime/S18.json',25);print(json.dumps(r));return 0 if r['status'] in ['STOPPED','NOT_RUNNING'] else 2
    if a.command=='learning':print(ROOT/'docs/optimization_v2/学习导航.md');return 0
    if a.command=='audit':
        from audit18 import audit
        audit(a.out or RESULTS/('audit_'+stamp()));return 0
    return launch(a)

if __name__=='__main__':
    try:raise SystemExit(main())
    except (ValueError,OSError,KeyError) as exc:print(str(exc),file=sys.stderr);raise SystemExit(2)
