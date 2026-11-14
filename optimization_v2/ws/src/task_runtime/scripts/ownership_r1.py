"""PID/start-time/UID checked stop; task identities cannot stop each other."""
from common_r1 import *
import signal,time
def identity(pid):
    p=Path('/proc')/str(pid);parts=(p/'stat').read_text().rsplit(')',1)[1].split()
    return dict(pid=pid,start_ticks=parts[19],uid=p.stat().st_uid,
        argv=(p/'cmdline').read_bytes().replace(b'\0',b' ').decode(),state=parts[0])
def matches(d):
    try:
        live=identity(d['pid'])
        return live['uid']==d['uid']==os.getuid() and live['start_ticks']==d['start_ticks'] and d['marker'] in live['argv'] and live['state']!='Z'
    except (OSError,KeyError,ValueError):return False
def request_stop(task,timeout=55):
    path=APP/'runtime/session.json'
    if not path.exists():return dict(status='NOT_RUNNING',task=task)
    d=read(path)
    if d['task_id']!=task:return dict(status='TASK_SCOPE_MISMATCH',requested=task,active=d['task_id'])
    if not matches(d):return dict(status='STALE_SESSION_REFUSED',pid=d.get('pid'))
    os.kill(d['pid'],signal.SIGINT);deadline=time.monotonic()+timeout
    while time.monotonic()<deadline:
        if not path.exists() or not matches(d):
            result=Path(d['output'])/'result.json'
            return dict(status='STOPPED' if result.exists() else 'EXITED_WITHOUT_RESULT',task=task,
                output=d['output'],result=read(result) if result.exists() else None)
        time.sleep(.1)
    return dict(status='STOP_TIMEOUT',task=task,pid=d['pid'])
