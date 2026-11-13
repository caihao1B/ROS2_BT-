"""Owned bounded development runner, never a formal checkpoint."""
from common17 import *
import argparse,fcntl,datetime,signal,subprocess

def main():
    ap=argparse.ArgumentParser();ap.add_argument('family',choices=['localization','coverage','prediction','memory','fault']);ap.add_argument('--method',required=True);ap.add_argument('--scenario',choices=['turning','occlusion'],default='turning');ap.add_argument('--rtf',type=float,default=1.);ap.add_argument('--wall',type=int,default=750);a=ap.parse_args()
    if not .4<=a.rtf<=1 or not 180<=a.wall<=3100:ap.error('BOUNDED_PROBE_REQUIRED')
    lock=(ROOT/'native_20261108/runs/session.lock').open('a');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    if os.getpgrp()!=os.getpid():os.setsid()
    sys.path.insert(0,str(BASE/'ws/src/project_baseline/scripts'));from session_owner import save_session
    name='study_probe_'+a.family+'_'+a.method+'_'+datetime.datetime.now().strftime('%Y%m%d_%H%M%S');out=RESULTS/name;out.mkdir();session=BASE/'runtime/S17.json';save_session(session,str(Path(__file__).resolve()),name)
    dump(out/'spec.json',dict(id=name,family=a.family,method=a.method,scenario=a.scenario,seed=17310,scope='DEVELOPMENT_PROBE_NOT_FORMAL',formal_credit=False))
    from hardware17 import collect
    dump(out/'hardware.json',collect());dump(out/'source_start.json',source_binding());child=None
    args=[str(BASE/'venvs/scene/bin/python'),str(PKG/'scripts'/(('prediction' if a.family in ['prediction','memory'] else a.family)+'_compare17.py')),'--out',str(out),'--seed','17310','--method',a.method,'--rtf',str(a.rtf),'--wall',str(a.wall-40)]
    args[-1]=str(min(a.wall-40,600 if a.family in ['prediction','memory'] else 750 if a.family=='localization' else 800 if a.family=='fault' else 3000))
    if a.family=='fault':args[1]=str(PKG/'scripts/faults17.py');args[args.index('--method')]='--fault'
    if a.family in ['prediction','memory']:args+=['--family',a.family,'--scenario',a.scenario]
    def interrupted(s,f):raise KeyboardInterrupt
    for s in [signal.SIGINT,signal.SIGTERM,signal.SIGALRM]:signal.signal(s,interrupted)
    signal.setitimer(signal.ITIMER_REAL,a.wall)
    try:
        print(out,flush=True)
        with (out/'launch.log').open('w') as log:child=subprocess.Popen(args,stdout=log,stderr=subprocess.STDOUT);code=child.wait();child=None
        dump(out/'command.json',dict(argv=args,exit_code=code,formal_credit=False))
        if (out/'metrics17.json').exists():print(json.dumps(read(out/'metrics17.json'),ensure_ascii=False),flush=True)
        return code
    finally:
        signal.setitimer(signal.ITIMER_REAL,0)
        if child and child.poll() is None:
            child.send_signal(signal.SIGINT)
            try:child.wait(timeout=25)
            except subprocess.TimeoutExpired:child.terminate();child.wait(timeout=10)
        if session.exists() and read(session).get('run_id')==name:session.unlink()

if __name__=='__main__':raise SystemExit(main())
