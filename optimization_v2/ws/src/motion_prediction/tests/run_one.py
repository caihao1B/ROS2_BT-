"""Bounded owned development run; never emits a formal stage PASS."""
import sys,os,fcntl,argparse
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from runtime15 import run_case,BASE,ROOT
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--scenario',default='turning');p.add_argument('--method',default='imm');p.add_argument('--split',default='development');p.add_argument('--collect',action='store_true');p.add_argument('--seconds',type=float,default=75);p.add_argument('--wall',type=float,default=320);p.add_argument('--fault',default='none');p.add_argument('--gui',action='store_true');a=p.parse_args()
    lock=(ROOT/'native_20261108/runs/session.lock').open('a');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    if os.getpgrp()!=os.getpid():os.setsid()
    out=BASE/'results/S15'/f'dev_{a.scenario}_{a.method}_{os.getpid()}';print(out,flush=True)
    result=run_case(out,a.scenario,a.method,a.split,a.seconds,a.wall,a.gui,a.collect,a.fault)
    raise SystemExit(0 if result['success'] else 1)
