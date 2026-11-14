"""Task-specific configuration/results with a single owned simulator session."""
from common_r1 import *
import argparse,fcntl,signal,subprocess,time,math,shutil
from registry_r1 import TASKS,load_task,validate_task
from lifecycle_r1 import validate as validate_compat
from ownership_r1 import identity,matches,request_stop

def budgets(seconds,wall,rtf,stop_at):
    if not 180<=seconds<=1800 or not 180<=wall<=5000 or not math.isfinite(rtf) or not .4<=rtf<=1.:
        raise ValueError('BOUNDED_TASK_REQUIRES_SIM_180_1800_WALL_180_5000_RTF_0_4_1')
    if stop_at is not None and not 20<=stop_at<seconds-10:raise ValueError('SMOKE_STOP_OUTSIDE_TASK_BUDGET')

def try_read(path):
    try:return read(path)
    except (OSError,ValueError):return None

def run(a):
    d,spec=validate_task(a.task,a.cluster);validate_compat()
    seconds=a.sim_seconds if a.sim_seconds is not None else d['defaults']['sim_seconds']
    wall=a.wall_timeout if a.wall_timeout is not None else d['defaults']['wall_timeout']
    rtf=a.rtf if a.rtf is not None else d['defaults']['rtf'];stop_at=a.stop_after_sim
    if a.command=='smoke' and stop_at is None:stop_at=60
    budgets(seconds,wall,rtf,stop_at)
    if os.environ.get('LD_PRELOAD'):raise ValueError('EXTERNAL_PRELOAD_NOT_QUALIFIED')
    lock=(ROOT/'native_20261108/runs/session.lock').open('a')
    try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    except BlockingIOError:raise ValueError('ANOTHER_PROJECT_SESSION_IS_ACTIVE')
    session=APP/'runtime/session.json'
    if session.exists():raise ValueError('EXISTING_SESSION_REQUIRES_INSPECTION '+str(session))
    run_id='r01_'+a.task+'_'+stamp();out=APP/'results'/a.task/run_id;out.mkdir(parents=True)
    spec.update(id=run_id);dump(out/'spec.json',spec);dump(out/'task_config.json',d)
    sources={str(p.relative_to(ROOT)):sha(p) for p in PKG.rglob('*') if p.is_file() and '__pycache__' not in p.parts}
    dump(out/'runtime_source_binding.json',sources)
    state=identity(os.getpid());state.update(task_id=a.task,run_id=run_id,output=str(out),marker=str(Path(__file__).resolve()))
    dump(session,state);dump(out/'owner.json',state)
    dump(APP/'results'/a.task/'last_run.json',dict(run=str(out.relative_to(ROOT)),task_version=d['version'],status='RUNNING'))
    argv=[str(BASE/'venvs/scene/bin/python'),str(PKG/'scripts/one_r1.py'),'--spec',str(out/'spec.json'),
        '--out',str(out),'--seconds',str(seconds),'--wall',str(wall-65),'--rtf',str(rtf)]
    if a.gui:argv.append('--gui')
    reason=None;child=None;start=time.monotonic();deadline=start+wall;error=None;escalations=[];seen=[];loaded=None
    def interrupted(sig,frame):
        nonlocal reason
        if reason is None:reason='OPERATOR_STOP'
    previous={s:signal.signal(s,interrupted) for s in [signal.SIGINT,signal.SIGTERM]}
    print(str(out),flush=True)
    result={}
    try:
        with (out/'launcher.log').open('w') as log:
            child=subprocess.Popen(argv,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            while child.poll() is None:
                now=time.monotonic();live=try_read(out/'online/live.json')
                if live:
                    status=live.get('status') or {}
                    if not seen or live.get('t')!=seen[-1]['t']:
                        seen.append(dict(t=live.get('t'),navigation_allowed=status.get('navigation_allowed',False),
                            navigation_active=live.get('navigation_active',False),tool_stopped=(live.get('tool') or {}).get('stopped_confirmed'),
                            has_pose=live.get('pose') is not None))
                    if stop_at is not None and live.get('t',0)>=stop_at and reason is None:reason='SMOKE_DURATION_REACHED'
                if loaded is None:
                    processes=try_read(out/'processes.json')
                    for process in (processes or {}).get('commands',[]):
                        if process['role']!='server':continue
                        try:
                            maps=Path('/proc',str(process['pid']),'maps').read_text()
                            shim=validate_compat()['preload'];boost='/usr/lib/x86_64-linux-gnu/libboost_thread.so.1.83.0'
                            if shim in maps and boost in maps:loaded=dict(pid=process['pid'],shim=shim,boost=boost,verified=True)
                        except OSError:pass
                if now>=deadline-60 and reason is None:reason='WALL_BUDGET_STOP'
                if reason:
                    dump(out/'stop_request.json',dict(reason=reason,last_sim=live.get('t') if live else None,wall_elapsed=now-start))
                    child.send_signal(signal.SIGINT);break
                time.sleep(.1)
            if child.poll() is None:
                try:child.wait(timeout=min(30,max(1,deadline-time.monotonic()-25)))
                except subprocess.TimeoutExpired:
                    escalations.append('CHILD_TERM');child.terminate()
                    try:child.wait(timeout=5)
                    except subprocess.TimeoutExpired:escalations.append('CHILD_KILL');child.kill();child.wait(timeout=5)
    except BaseException as exc:error=repr(exc)
    finally:
        # Preserve both the child cleanup and the independent parent cleanup.
        if child and child.poll() is None:
            child.send_signal(signal.SIGINT)
            try:child.wait(timeout=15)
            except subprocess.TimeoutExpired:escalations.append('EXCEPTION_CHILD_KILL');child.kill();child.wait(timeout=5)
        add17();sys.path.insert(0,str(BASE/'ws/src/mission_bt/scripts'))
        from owned14 import cleanup
        if (out/'cleanup14.json').exists():shutil.copy2(out/'cleanup14.json',out/'child_cleanup14.json')
        try:cleaned=cleanup(out)
        except Exception as exc:
            error=(error or '')+' CLEANUP_FAILED '+repr(exc);cleaned=dict(survivors=['UNKNOWN'])
        dump(out/'cleanup_r1.json',cleaned)
        dump(out/'startup_samples.json',seen);dump(out/'loaded_compat.json',loaded or dict(verified=False))
        runtime=try_read(out/'runtime.json') or {};metric={}
        if runtime:
            try:
                with (out/'scoring.log').open('w') as log:
                    scored=subprocess.run([str(BASE/'venvs/scene/bin/python'),str(PKG/'scripts/score_r1.py'),str(out)],stdout=log,stderr=subprocess.STDOUT,timeout=20)
                metric=try_read(out/'metrics17.json') or {}
                if scored.returncode:error=(error or '')+' SCORING_FAILED'
            except (subprocess.TimeoutExpired,OSError) as exc:error=(error or '')+' SCORING_FAILED '+repr(exc)
        bad=[dict(role=c['role'],exit=c.get('exit_code')) for c in runtime.get('commands',[]) if c.get('exit_code')!=0 and not (c['role']=='client' and c.get('exit_code')==1)]
        admitted=any(x['navigation_allowed'] and x['has_pose'] for x in seen)
        expected_child_exit=bool(child and (child.returncode in (0,1) or (child.returncode==-signal.SIGINT and reason is not None)))
        runtime_ok=bool(runtime and expected_child_exit and admitted and loaded and runtime.get('tool_shutdown',{}).get('state')=='STOPPED'
            and runtime.get('database_unchanged') and not bad and not cleaned.get('survivors') and not escalations and not error)
        result=dict(task_id=a.task,task_version=d['version'],run_id=run_id,
            lifecycle_status='PASS' if runtime_ok else 'FAIL',mission_status=metric.get('status','NOT_SCORED'),
            mission_complete=metric.get('complete',False),safety=metric.get('safety',False),localization_pass=metric.get('localization_pass',False),
            termination=reason or 'TASK_RETURNED',error=error,admitted=admitted,compat_loaded=bool(loaded),
            server_exit=next((c.get('exit_code') for c in runtime.get('commands',[]) if c['role']=='server'),None),
            abnormal_process_exits=bad,expected_nonzero_exits=[dict(role=c['role'],exit=c['exit_code']) for c in runtime.get('commands',[]) if c.get('exit_code') and c['role']=='client'],
            escalations=escalations,tool_shutdown=runtime.get('tool_shutdown'),survivors=cleaned.get('survivors'),
            sim_seconds=metric.get('sim_seconds_including_stop'),wall_seconds=time.monotonic()-start,
            child_exit=child.returncode if child else None,formal_credit=False,
            scope='R01 lifecycle probe; task completion scored separately using unchanged original standards')
        dump(out/'command.json',dict(argv=argv,exit_code=result['child_exit'],reason=reason,source_hashes=sources))
        dump(out/'result.json',result)
        dump(APP/'results'/a.task/'last_run.json',dict(run=str(out.relative_to(ROOT)),task_version=d['version'],result=result))
        if session.exists() and read(session).get('run_id')==run_id:session.unlink()
        for sig,handler in previous.items():signal.signal(sig,handler)
    print(json.dumps(result,ensure_ascii=False),flush=True)
    if a.command=='smoke':return 0 if runtime_ok and result['safety'] and result['localization_pass'] else 2
    return 0 if runtime_ok and result['mission_status']=='PASS' else 2

def main():
    ap=argparse.ArgumentParser(description='Three independent task profiles; R01 retains original lightweight vehicles')
    sub=ap.add_subparsers(dest='command',required=True)
    for name in ['list','status','doctor','check','open']:
        p=sub.add_parser(name)
        if name=='open':p.add_argument('--print-path',action='store_true')
    for name in ['run','smoke']:
        p=sub.add_parser(name);p.add_argument('task',choices=TASKS);p.add_argument('--cluster');p.add_argument('--gui',action='store_true')
        p.add_argument('--sim-seconds',type=int);p.add_argument('--wall-timeout',type=int);p.add_argument('--rtf',type=float);p.add_argument('--stop-after-sim',type=float)
    p=sub.add_parser('stop');p.add_argument('task',choices=TASKS)
    a=ap.parse_args()
    if a.command in ['run','smoke']:return run(a)
    if a.command=='stop':
        r=request_stop(a.task);print(json.dumps(r,ensure_ascii=False));return 0 if r['status'] in ['STOPPED','NOT_RUNNING'] else 2
    if a.command=='doctor':
        profiles=[validate_task(task)[0] for task in TASKS];validate_compat()
        print(json.dumps(dict(status='INPUTS_VALID',profiles=[{k:d[k] for k in ['task_id','version','active_vehicle','operation','controller','map_id','plan_revision','results_root']} for d in profiles]),indent=2,ensure_ascii=False));return 0
    if a.command=='list':print(json.dumps([load_task(task) for task in TASKS],indent=2,ensure_ascii=False));return 0
    if a.command=='status':
        stage=try_read(APP/'results/R01/latest.json') or {}
        print(json.dumps(dict(stage={k:stage.get(k) for k in ['stage','automated_status','original_stage_states','next_step']},tasks={task:try_read(APP/'results'/task/'last_run.json') for task in TASKS},session=try_read(APP/'runtime/session.json')),indent=2,ensure_ascii=False));return 0
    if a.command=='check':
        from report_r1 import verify_receipt
        r=verify_receipt();print(json.dumps(r,ensure_ascii=False));return 0 if r['passed'] else 2
    if a.command=='open':
        r=read(APP/'results/R01/latest.json');path=ROOT/r['report_path']
        if a.print_path:print(path)
        else:subprocess.Popen(['xdg-open',str(path)],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
        return 0

if __name__=='__main__':
    try:raise SystemExit(main())
    except (ValueError,OSError,KeyError) as exc:print(str(exc),file=sys.stderr);raise SystemExit(2)
