"""Bounded development probe; never contributes to frozen formal seed counts."""
from common17 import *
import argparse,subprocess,fcntl,signal,datetime

def main():
    ap=argparse.ArgumentParser();ap.add_argument('case',choices=['connector_diff','connector_ackermann','A1','A2','A3','B1','B2','B3']);ap.add_argument('--seconds',type=int,default=1800);ap.add_argument('--wall',type=int,default=3600);ap.add_argument('--rtf',type=float,default=1.);ap.add_argument('--dynamic-prefix',action='store_true');ap.add_argument('--prediction',choices=['disabled','imm'],default='disabled');a=ap.parse_args()
    if not 100<=a.seconds<=1800 or not 120<=a.wall<=5000 or not .4<=a.rtf<=1:ap.error('BOUNDED_PILOT_REQUIRED')
    lock=(ROOT/'native_20261108/runs/session.lock').open('a');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    if os.getpgrp()!=os.getpid():os.setsid()
    sys.path.insert(0,str(BASE/'ws/src/project_baseline/scripts'));from session_owner import save_session
    from binding_check17 import validate
    validate()
    name='pilot_'+a.case+'_'+datetime.datetime.now().strftime('%Y%m%d_%H%M%S');out=RESULTS/name;out.mkdir();session=BASE/'runtime/S17.json';save_session(session,str(Path(__file__).resolve()),name)
    connector=a.case.startswith('connector_');kind=a.case.removeprefix('connector_') if connector else ('diff' if a.case[0]=='A' else 'ackermann');cluster=('A1' if kind=='diff' else 'B1') if connector else a.case
    spec=dict(id=name,vehicle=kind,cluster=cluster,controller='DWB' if kind=='diff' else 'RPP',operation='inspection' if connector else 'cluster',seed=17301 if kind=='diff' else 17302,scope='DEVELOPMENT_NOT_FORMAL')
    spec['prediction']=a.prediction
    if a.dynamic_prefix:
        if not connector:ap.error('dynamic prefix is a connector development case')
        spec['benchmark']='dynamic_prefix'
    dump(out/'spec.json',spec);dump(out/'source_start.json',source_binding())
    from hardware17 import collect
    dump(out/'hardware.json',collect());child=None
    def stop(s,f):raise KeyboardInterrupt
    for s in [signal.SIGTERM,signal.SIGINT,signal.SIGALRM]:signal.signal(s,stop)
    signal.setitimer(signal.ITIMER_REAL,a.wall)
    try:
        args=[str(BASE/'venvs/scene/bin/python'),str(PKG/'scripts/one17.py'),'--spec',str(out/'spec.json'),'--out',str(out),'--seconds',str(a.seconds),'--wall',str(a.wall-30),'--rtf',str(a.rtf)]
        print(str(out),flush=True)
        with (out/'launch.log').open('w') as log:child=subprocess.Popen(args,stdout=log,stderr=subprocess.STDOUT);code=child.wait();child=None
        dump(out/'command.json',dict(argv=args,exit_code=code,formal_credit=False))
        from evaluate17 import evaluate
        m=evaluate(out,spec,BASE/'build_s17/monitor');print(json.dumps(m,ensure_ascii=False),flush=True)
        return 0 if m['status']=='PASS' else 1
    finally:
        signal.setitimer(signal.ITIMER_REAL,0)
        if child and child.poll() is None:
            child.send_signal(signal.SIGINT)
            try:child.wait(timeout=30)
            except subprocess.TimeoutExpired:child.terminate();child.wait(timeout=10)
        if session.exists() and read(session).get('run_id')==name:session.unlink()
if __name__=='__main__':raise SystemExit(main())
