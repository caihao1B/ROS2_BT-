"""Bounded actual connector repair probe; isolated from recording budget and formal scores."""
from common_showcase import *
import argparse,subprocess,fcntl,signal,datetime

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--kind',choices=['diff','ackermann'],default='ackermann');ap.add_argument('--seconds',type=int,default=600);ap.add_argument('--wall',type=int,default=1400);ap.add_argument('--child',type=Path);ap.add_argument('--gui',action='store_true');a=ap.parse_args()
    if not 100<=a.seconds<=1800 or not 120<=a.wall<=4000:ap.error('bounded repair probe required')
    if a.child:
        out=a.child;spec=read(out/'spec.json');os.environ.update(S17_CLUSTER=spec['cluster'],S17_OPERATION='inspection',S17_CONTROLLER=spec['controller'],S17_BENCHMARK='dynamic_prefix',S17_PREDICTION='disabled')
        from inputs17 import snapshot
        snapshot(out)
        from showcase_overlay import compose
        overlay=compose(out/'overlay');sys.path.insert(0,str(overlay))
        from runtime14 import run_case
        r=run_case(out,'zone_a','nominal_spray_cluster' if a.kind=='diff' else 'nominal_mow_cluster',a.seconds,a.wall-30,seed=17301 if a.kind=='diff' else 17302,gui=a.gui,kind=a.kind,controller=spec['controller']);return 0 if r.get('success') else 1
    lock=(ROOT/'native_20261108/runs/session.lock').open('a');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    if os.getpgrp()!=os.getpid():os.setsid()
    sys.path.insert(0,str(BASE/'ws/src/project_baseline/scripts'));from session_owner import save_session
    from binding_check17 import validate
    validate();out=BASE/'results/S17'/('showcase_repair_'+a.kind+'_'+datetime.datetime.now().strftime('%Y%m%d_%H%M%S'));out.mkdir()
    spec=dict(id=out.name,vehicle=a.kind,cluster='A1' if a.kind=='diff' else 'B1',controller='DWB' if a.kind=='diff' else 'RPP',operation='inspection',benchmark='dynamic_prefix',seed=17301 if a.kind=='diff' else 17302,scope='SHOWCASE_REPAIR_NOT_FORMAL',formal_credit=False)
    dump(out/'spec.json',spec);save_session(BASE/'runtime/S17.json',str(Path(__file__).resolve()),out.name);print(str(out),flush=True)
    args=[str(BASE/'venvs/scene/bin/python'),str(Path(__file__).resolve()),'--kind',a.kind,'--seconds',str(a.seconds),'--wall',str(a.wall),'--child',str(out)]+(['--gui'] if a.gui else [])
    child=None
    def stop(s,f):raise KeyboardInterrupt
    for s in [signal.SIGINT,signal.SIGTERM,signal.SIGALRM]:signal.signal(s,stop)
    signal.setitimer(signal.ITIMER_REAL,a.wall)
    try:
        with (out/'launch.log').open('w') as f:child=subprocess.Popen(args,stdout=f,stderr=subprocess.STDOUT);code=child.wait();child=None
        dump(out/'command.json',dict(argv=args,exit_code=code,formal_credit=False,recording_budget_credit=False))
        from evaluate17 import evaluate
        result=evaluate(out,spec,BASE/'build_s17/monitor');print(json.dumps(result),flush=True);return 0 if result['status']=='PASS' else 1
    finally:
        signal.setitimer(signal.ITIMER_REAL,0)
        if child and child.poll() is None:
            child.send_signal(signal.SIGINT)
            try:child.wait(timeout=30)
            except subprocess.TimeoutExpired:child.terminate();child.wait(timeout=10)
        session=BASE/'runtime/S17.json'
        if session.exists() and read(session).get('run_id')==out.name:session.unlink()
if __name__=='__main__':raise SystemExit(main())
