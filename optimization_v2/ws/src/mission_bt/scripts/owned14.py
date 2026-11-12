"""Last-resort cleanup bound to this run's recorded unique Gazebo partition.
No global ROS kill and no dependence on a still-live wrapper process.
"""
from common14 import *
import psutil,signal

def cleanup(out):
    manifest=out/'processes.json'
    if not manifest.exists():return dict(owned=0,survivors=[])
    saved=read(manifest);partition=saved['partition'];owners=[]
    for p in psutil.process_iter(['pid','uids','create_time']):
        try:
            if p.pid!=os.getpid() and p.uids().real==os.getuid() and p.create_time()>=saved['created_wall']-1 and p.environ().get('GZ_PARTITION')==partition:owners.append(p)
        except (psutil.Error,ProcessLookupError):pass
    records=[]
    for p in owners:
        try:records.append(dict(pid=p.pid,created=p.create_time(),argv=p.cmdline()))
        except psutil.Error:pass
    # Stop task production first; physical command/watchdogs remain alive briefly.
    producers=[]
    for p in owners:
        try:
            args=' '.join(p.cmdline())
            if 'worker14.py' in args or '/command_gate' in args:producers.append(p);p.send_signal(signal.SIGINT)
        except psutil.Error:pass
    if producers:psutil.wait_procs(producers,timeout=2.5);time.sleep(.8)
    for p in owners:
        try:p.send_signal(signal.SIGCONT);p.send_signal(signal.SIGINT)
        except psutil.Error:pass
    _,alive=psutil.wait_procs(owners,timeout=3)
    for p in alive:
        try:p.terminate()
        except psutil.Error:pass
    _,alive=psutil.wait_procs(alive,timeout=2)
    for p in alive:
        try:p.kill()
        except psutil.Error:pass
    psutil.wait_procs(alive,timeout=1)
    survivors=[p.pid for p in alive if p.is_running() and p.status()!=psutil.STATUS_ZOMBIE]
    result=dict(owned=len(owners),processes=records,survivors=survivors,partition=partition);dump(out/'cleanup14.json',result);return result
