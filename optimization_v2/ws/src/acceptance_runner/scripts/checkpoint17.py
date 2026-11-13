"""Atomic, content-addressed case journal. Failed attempts stay visible."""
from common17 import *
import fcntl

class Journal:
    def __init__(self, directory):
        self.directory=Path(directory);self.directory.mkdir(parents=True,exist_ok=True)
    def identity(self,spec,binding):return canonical(dict(spec=spec,binding=binding))
    def load(self,spec,binding):
        key=self.identity(spec,binding);p=self.directory/(key+'.json')
        if not p.exists():return None
        d=read(p)
        if d.get('identity')!=key or d.get('spec')!=spec or d.get('binding')!=binding:raise ValueError('CHECKPOINT_IDENTITY')
        for path,h in d['evidence'].items():
            q=(ROOT/path).resolve()
            if not q.is_relative_to(RESULTS) or not q.is_file() or sha(q)!=h:raise ValueError('CHECKPOINT_EVIDENCE_CHANGED '+path)
        return d
    def commit(self,spec,binding,out,metrics):
        out=Path(out).resolve()
        if not out.is_relative_to(RESULTS):raise ValueError('CHECKPOINT_OUTSIDE_S17')
        key=self.identity(spec,binding);p=self.directory/(key+'.json')
        with (self.directory/'.lock').open('a') as lock:
            fcntl.flock(lock,fcntl.LOCK_EX)
            if p.exists():raise ValueError('DUPLICATE_SCORE '+spec['id'])
            evidence={str(x.relative_to(ROOT)):sha(x) for x in files(out)}
            if not evidence or not (out/'metrics17.json').exists():raise ValueError('MISSING_CASE_METRICS')
            if read(out/'metrics17.json')!=metrics:raise ValueError('CHECKPOINT_METRICS_DISAGREE')
            dump(p,dict(identity=key,spec=spec,binding=binding,evidence=evidence,metrics=metrics,case_path=str(out.relative_to(ROOT)),finished=True,created_wall=time.time()))
    def collect(self,binding):
        result=[]
        for p in self.directory.glob('*.json'):
            d=read(p)
            recorded=d.get('binding',{})
            same_context={k:v for k,v in recorded.items() if k!='sources'}=={k:v for k,v in binding.items() if k!='sources'}
            if not same_context:continue
            sources=recorded.get('sources',{})
            if any(not (ROOT/k).is_file() or sha(ROOT/k)!=v for k,v in sources.items()):continue
            result.append(self.load(d['spec'],recorded))
        ids=[d['spec']['id'] for d in result]
        if len(ids)!=len(set(ids)):raise ValueError('DUPLICATE_CASE_ID')
        return result
