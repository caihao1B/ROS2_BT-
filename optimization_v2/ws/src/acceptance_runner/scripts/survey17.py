"""Actual stationary sensors at new route bounds; noisy observations only."""
from common17 import *
import argparse,fcntl,signal,atexit
from overlay17 import build

def main():
    ap=argparse.ArgumentParser();ap.add_argument('stations',nargs='*');ap.add_argument('--rtf',type=float,default=.4);ap.add_argument('--vacant-field',action='store_true');ap.add_argument('--output',type=Path);a=ap.parse_args()
    if a.vacant_field:os.environ['S17_SURVEY_STATIC']='1'
    out=(a.output or RESULTS/'qualification').resolve()
    if not out.is_relative_to(RESULTS):raise ValueError('SURVEY_OUTPUT_SCOPE')
    cfg=read(out/'stations.json')
    overlay=build(out/'overlay',out/'stations.json',a.rtf);sys.path.insert(0,str(overlay))
    from runtime14 import run_case
    lock=(ROOT/'native_20261108/runs/session.lock').open('a');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    if os.getpgrp()!=os.getpid():os.setsid()
    sys.path.insert(0,str(BASE/'ws/src/project_baseline/scripts'))
    from session_owner import save_session
    session=BASE/'runtime/S17.json';save_session(session,str(Path(__file__).resolve()),'S17_qualification_survey')
    atexit.register(lambda:session.unlink(missing_ok=True) if session.exists() and read(session).get('pid')==os.getpid() else None)
    for key in a.stations or list(cfg['stations']):
        s=cfg['stations'][key];dest=out/'survey'/key
        if (dest/'runtime.json').exists() and read(dest/'runtime.json').get('success'):print('EXISTING STATION '+key,flush=True);continue
        if dest.exists():raise ValueError('Preserve unsuccessful station; use a new explicit attempt '+key)
        print('S17 SURVEY '+key,flush=True)
        r=run_case(dest,'zone_a','calibration_station_'+key,32,150,seed=s['seed'],kind=s['vehicle'],probe=True,survey=True)
        if not r['success'] or not r['database_unchanged']:raise RuntimeError('SURVEY_FAILED '+key)
if __name__=='__main__':main()
