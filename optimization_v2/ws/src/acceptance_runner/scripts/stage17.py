"""Serial, bounded acceptance shards with explicit missing-case accounting."""
from common17 import *
from checkpoint17 import Journal
from report17 import aggregate,render
from evaluate17 import evaluate
from matrix17 import freeze
import argparse,datetime,fcntl,signal,subprocess,traceback,platform,math

def require_completed_checkpoint(record):
    # Resume skips completed work only. A failed or partial recorded run must
    # not silently allow the batch to continue collecting easier seeds.
    if record['metrics'].get('status')!='PASS':
        raise ValueError('RECORDED_CASE_NOT_COMPLETE '+record['spec']['id']+'; diagnose before collecting more seeds')

def publish(out,data,binding):
    from inputs17 import receipt_assets
    try:asset_evidence=receipt_assets()
    except Exception as exc:
        asset_evidence={};data['status']='FAIL';data['error']='INPUT_ASSET_VALIDATION_FAILED: '+str(exc);data['checks']['S17-C01']='FAIL'
    data['qualification']=read(RESULTS/'qualification/preflight.json') if (RESULTS/'qualification/preflight.json').exists() else None
    if (RESULTS/'qualification/surface_qualification.json').exists():data['qualification']['new_station_validation']=read(RESULTS/'qualification/surface_qualification.json')
    dump(out/'summary.json',data);dump(EVAL/'acceptance_summary.json',data);render(out,data)
    docs=ROOT/'docs/optimization_v2/S17';artifacts=files(docs)+files(BASE/'lessons/S17')+files(out/'component_contracts')+files(RESULTS/'component_contracts/current')+[out/'index.html',out/'summary.json']
    for name in ['qualification.png','browser_check.json','report_preview.png','failure.png','failure_evidence17.json']:
        if (out/name).exists():artifacts.append(out/name)
    evidence={str(p.relative_to(ROOT)):sha(p) for p in artifacts};evidence.update(asset_evidence)
    journal=Journal(EVAL/'checkpoints')
    for c in journal.collect(binding):
        p=journal.directory/(c['identity']+'.json');evidence[str(p.relative_to(ROOT))]=sha(p);evidence.update(c['evidence'])
    for p in [out/'binding.json',out/'hardware.json',out/'resume_proof.json']:
        if p.exists():evidence[str(p.relative_to(ROOT))]=sha(p)
    for row in data.get('demo',[]):
        for p in files(ROOT/row['case_path']):evidence[str(p.relative_to(ROOT))]=sha(p)
    r=dict(schema_version=1,step='S17',plan_revision='20261109.1',run_id=out.name,checked_at_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),automated_status=data['status'],manual_status='NOT_CHECKED',reason=data.get('error') or f"{60-len(data['missing_formal'])}/60 actual full-cluster trials; missing items remain NOT_RUN",check_exit_code=0 if data['status']=='PASS' else 1,checks=data['checks'],report_path=str((out/'index.html').relative_to(ROOT)),source_files=[dict(path=k,sha256=v) for k,v in source_binding().items()],artifacts=[dict(path=k,sha256=v) for k,v in sorted(evidence.items())],dependency_receipts=binding['dependencies'])
    dump(RESULTS/'latest.json',r)

def main():
    ap=argparse.ArgumentParser();ap.add_argument('mode',choices=['test','demo','stop']);ap.add_argument('--suite',choices=['formal','ablations']);ap.add_argument('--case',choices=['acceptance_preview','resume_partial']);ap.add_argument('--seed',type=int,default=41);ap.add_argument('--sim-seconds',type=int,default=1800);ap.add_argument('--wall-timeout',type=int,default=21600);ap.add_argument('--shard',default='1/10');ap.add_argument('--resume',action='store_true');ap.add_argument('--headless',action='store_true');ap.add_argument('--rtf',type=float,default=.4);a=ap.parse_args()
    sys.path.insert(0,str(BASE/'ws/src/project_baseline/scripts'));from session_owner import save_session,stop_session
    session=BASE/'runtime/S17.json'
    if a.mode=='stop':print(json.dumps(stop_session(session,25)));return 0
    if a.seed!=41 or not 1<=a.sim_seconds<=1800 or not 1<=a.wall_timeout<=21600:ap.error('Frozen root seed 41 and bounded wall/simulation required')
    if not math.isfinite(a.rtf) or not .4<=a.rtf<=1.:ap.error('RTF must be 0.4 to 1.0, recorded in experiment identity')
    if a.mode=='test' and (not a.suite or a.case):ap.error('test requires --suite')
    if a.mode=='demo' and (not a.case or a.suite):ap.error('demo requires --case')
    try:n,total=map(int,a.shard.split('/'));assert total==10 and 1<=n<=10
    except (ValueError,AssertionError):ap.error('--shard must be 1/10 to 10/10')
    if a.suite=='formal' and a.sim_seconds!=1800:ap.error('formal cluster budget must be 1800 seconds')
    freeze();lock=(ROOT/'native_20261108/runs/session.lock').open('a')
    try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    except BlockingIOError:print('SIMULATOR_BUSY: only one owned session allowed');return 2
    if os.getpgrp()!=os.getpid():os.setsid()
    run_id=a.mode+'_'+datetime.datetime.now().strftime('%Y%m%d_%H%M%S')+'_'+str(os.getpid());out=RESULTS/run_id;out.mkdir(parents=True);save_session(session,str(Path(__file__).resolve()),run_id)
    source_scope='formal' if a.suite=='formal' else 'all'
    binding=dict(sources=source_binding(source_scope),dependencies=dependencies(),assets={},execution=dict(requested_rtf=a.rtf))
    from inputs17 import snapshot
    for p in snapshot(out).values():
        if p.exists():binding['assets'][str(p.relative_to(ROOT))]=sha(p)
    from hardware17 import collect
    dump(out/'binding.json',binding);dump(out/'hardware.json',dict(collect(),command=sys.argv))
    journal=Journal(EVAL/'checkpoints');start=time.monotonic();child=None;exitcode=1
    def interrupted(s,f):raise KeyboardInterrupt('S17 interrupted; committed cases remain resumable')
    for s in [signal.SIGINT,signal.SIGTERM,signal.SIGALRM]:signal.signal(s,interrupted)
    signal.setitimer(signal.ITIMER_REAL,max(1,a.wall_timeout-10));data=aggregate(journal.collect(binding))
    try:
        publish(out,data,binding)
        for dep in ['S14','S15','S16']:
            r=subprocess.run(['bash',str(BASE/'stepctl.sh'),'check',dep],stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,timeout=300);(out/(dep+'.log')).write_text(r.stdout)
            if r.returncode:raise ValueError('PREREQUISITE '+dep)
        from verify17 import verify
        verify(out/'component_contracts')
        if a.mode=='demo':
            from binding_check17 import validate
            validate();count=2 if a.case=='resume_partial' else 1;seconds=min(a.sim_seconds/count-8,(100 if a.rtf<.6 else 120) if count==2 else 160);previous=None;measured=[]
            if seconds<60:raise ValueError('DEMO_REQUIRES_AT_LEAST_136_SIM_SECONDS')
            for phase in range(count):
                dest=out/f'{a.case}_{phase+1}';dest.mkdir();spec=dict(id=dest.name,cluster='A2',vehicle='diff',controller='DWB',seed=17023,scope='PREVIEW_NOT_FORMAL');dump(dest/'spec.json',spec)
                remain=a.wall_timeout-(time.monotonic()-start)-30
                if remain<seconds/a.rtf+25:raise TimeoutError('DEMO_WALL_BUDGET')
                args=[str(BASE/'venvs/scene/bin/python'),str(PKG/'scripts/one17.py'),'--spec',str(dest/'spec.json'),'--out',str(dest),'--wall',str(min(remain,seconds/a.rtf+90)),'--seconds',str(seconds),'--rtf',str(a.rtf)]
                if previous:args+=['--resume-from',str(previous)]
                if not a.headless:args+=['--gui']
                with (dest/'launch.log').open('w') as log:
                    child=subprocess.Popen(args,stdout=log,stderr=subprocess.STDOUT);code=child.wait();child=None
                dump(dest/'command.json',dict(argv=args,exit_code=code));m=evaluate(dest,spec,BASE/'build_s17/monitor');measured.append(dict(spec=spec,metrics=m,case_path=str(dest.relative_to(ROOT))));previous=dest
                if not m['safety'] or not m['original_denominator']:raise ValueError('DEMO_SAFETY_OR_DENOMINATOR_FAILED')
            if count==2:
                old=read(out/f'{a.case}_1/client/mission.json');new=read(previous/'client/mission.json');restored=read(previous/'client/restored17.json');old_keys={j['key'] for j in old['tool_results'] if j['state']=='SUCCEEDED'}
                new_events=rows(previous/'client/events.jsonl');goals=[r for r in new_events if r['event']=='tool_goal'];duplicates=[r for r in goals if r.get('key') in old_keys]
                proof=dict(old_confirmed=old['confirmed_points'],restored_confirmed=restored['confirmed_points'],new_confirmed=new['confirmed_points'],already_completed_keys=sorted(old_keys),duplicate_goals=duplicates,sim_total_s=sum(x['metrics']['sim_seconds_including_stop'] for x in measured),phase2_tool_goals=len(goals))
                proof['passed']=old['confirmed_points']>0 and restored['confirmed_points']==old['confirmed_points'] and new['confirmed_points']>old['confirmed_points'] and bool(goals) and not duplicates and proof['sim_total_s']<=a.sim_seconds
                dump(out/'resume_proof.json',proof)
                if not proof['passed']:raise ValueError('RESUME_NOT_DEMONSTRATED')
            data['demo']=measured;data['demo_acceptance']='PASS';exitcode=0
        elif a.suite=='formal':
            from binding_check17 import validate
            validate()
            matrix=freeze()[1];selected=matrix['formal'][(n-1)*6:n*6]
            monitor=BASE/'build_s17/monitor'
            if not monitor.exists():raise ValueError('BUILD_S17_REQUIRED')
            for spec in selected:
                old=journal.load(spec,binding)
                if old:
                    if not a.resume:raise ValueError('ALREADY_EXECUTED: use --resume; no duplicate score')
                    require_completed_checkpoint(old)
                    print('RESUME VERIFIED '+spec['id'],flush=True);continue
                remain=a.wall_timeout-(time.monotonic()-start)-45
                if remain<1800/a.rtf+90:raise TimeoutError('INSUFFICIENT_CASE_WALL_BUDGET; retained completed checkpoints')
                dest=out/spec['id'];dest.mkdir();dump(dest/'spec.json',spec)
                args=[str(BASE/'venvs/scene/bin/python'),str(PKG/'scripts/one17.py'),'--spec',str(dest/'spec.json'),'--out',str(dest),'--wall',str(min(4900,remain)),'--seconds',str(a.sim_seconds),'--rtf',str(a.rtf)]
                print('RUN '+spec['id'],flush=True)
                with (dest/'launch.log').open('w') as log:
                    child=subprocess.Popen(args,stdout=log,stderr=subprocess.STDOUT);code=child.wait();child=None
                dump(dest/'command.json',dict(argv=args,exit_code=code))
                m=evaluate(dest,spec,monitor)
                if source_binding(source_scope)!=binding['sources'] or dependencies()!=binding['dependencies']:raise ValueError('INPUTS_CHANGED_DURING_RUN')
                journal.commit(spec,binding,dest,m);data=aggregate(journal.collect(binding));publish(out,data,binding)
                if m['status']!='PASS':raise ValueError('FORMAL_CASE_NOT_COMPLETE '+spec['id']+'; diagnose before collecting more seeds')
            exitcode=0
        else:
            from ablation17 import execute as ablations
            def launch(args,logpath,timeout):
                nonlocal child
                if timeout<1:raise TimeoutError('ABLATION_BATCH_WALL_BUDGET')
                with logpath.open('w') as log:
                    child=subprocess.Popen(args,stdout=log,stderr=subprocess.STDOUT);code=child.wait(timeout=timeout);child=None
                return code
            ablations(out,binding,lambda:a.wall_timeout-(time.monotonic()-start),launch,a.resume,a.rtf)
            data=aggregate(journal.collect(binding));exitcode=0
    except (Exception,KeyboardInterrupt) as e:
        demos=locals().get('measured',[]);data=aggregate(journal.collect(binding));data['demo']=demos;data['error']=str(e);(out/'failure.txt').write_text(traceback.format_exc());print(str(e),flush=True)
    finally:
        signal.setitimer(signal.ITIMER_REAL,0)
        if child and child.poll() is None:
            child.send_signal(signal.SIGINT)
            try:child.wait(timeout=25)
            except subprocess.TimeoutExpired:child.terminate();child.wait(timeout=10)
        publish(out,data,binding)
        if session.exists() and read(session).get('run_id')==run_id:session.unlink()
    return exitcode
if __name__=='__main__':raise SystemExit(main())
