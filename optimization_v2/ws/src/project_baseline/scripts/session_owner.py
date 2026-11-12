"""Only signal a recorded live process with matching UID, start time and command line."""
import json, os, signal, time
from pathlib import Path

def identity(pid):
    p=Path('/proc')/str(pid)
    stat=p.joinpath('stat').read_text().rsplit(')',1)[1].split()
    return {'pid':pid,'start_ticks':stat[19],'uid':p.stat().st_uid,
            'argv':p.joinpath('cmdline').read_bytes().replace(b'\0',b' ').decode()}

def save_session(path, marker, run_id):
    data=identity(os.getpid());data.update(marker=marker,run_id=run_id)
    path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(json.dumps(data))

def matches(data):
    try:
        live=identity(data['pid'])
        return (live['start_ticks']==data['start_ticks'] and live['uid']==os.getuid()==data['uid']
                and data['marker'] in live['argv'])
    except (OSError,KeyError,ValueError):return False

def stop_session(path, timeout=25):
    if not path.exists():return {'status':'NOT_RUNNING'}
    data=json.loads(path.read_text())
    if not matches(data):return {'status':'STALE_SESSION_REFUSED','pid':data.get('pid')}
    os.kill(data['pid'],signal.SIGINT)
    end=time.monotonic()+timeout
    while time.monotonic()<end:
        if not path.exists() or not matches(data):return {'status':'STOPPED','run_id':data['run_id']}
        time.sleep(.1)
    return {'status':'STOP_TIMEOUT','pid':data['pid']}
