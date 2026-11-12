"""Owned bounded development run; stage acceptance is produced by stage14.py."""
import sys,fcntl
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from runtime14 import *
if __name__=='__main__':
    lock=(ROOT/'native_20261108/runs/session.lock').open('a');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    if os.getpgrp()!=os.getpid():os.setsid()
    case=sys.argv[1];kind='ackermann' if 'mow' in case or case.startswith('ackermann_') else 'diff';out=BASE/'results/S14'/('dev_'+case+'_'+str(os.getpid()));print(out,flush=True)
    result=run_case(out,'zone_a',case,float(sys.argv[2]) if len(sys.argv)>2 else 180,float(sys.argv[3]) if len(sys.argv)>3 else 500,kind=kind)
    raise SystemExit(0 if result['success'] else 1)
