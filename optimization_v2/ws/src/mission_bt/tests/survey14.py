"""Bounded station observations; station placement is simulator-only prior data."""
import sys,fcntl,signal
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from runtime14 import *

def main():
    lock=(ROOT/'native_20261108/runs/session.lock').open('a');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    if os.getpgrp()!=os.getpid():os.setsid()
    config=read(BASE/'config/s14_survey_stations.json')
    for key in sys.argv[1:]:
        s=config['stations'][key];out=BASE/'results/S14/survey_20261112'/key
        if out.exists():raise ValueError('Preserve existing station evidence: '+str(out))
        print('SURVEY '+key,flush=True)
        r=run_case(out,'zone_a','calibration_station_'+key,32,140,seed=s['seed'],kind=s['vehicle'],probe=True,survey=True)
        if not r['success'] or not r['database_unchanged']:raise RuntimeError('Survey failed: '+key)
if __name__=='__main__':main()
