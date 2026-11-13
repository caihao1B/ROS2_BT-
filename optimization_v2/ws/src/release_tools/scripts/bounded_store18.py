"""Opt-in bounded payload store. Never removes unregistered or protected files."""
from common18 import *
import fcntl

class BoundedStore:
    def __init__(self,root,limit,protected=(),max_entries=4096):
        root=Path(root)
        if root.is_symlink() or not root.parent.resolve().exists():raise ValueError('STORE_ROOT_NOT_OWNED')
        if not isinstance(limit,int) or limit<=0:raise ValueError('POSITIVE_QUOTA_REQUIRED')
        self.root=root.resolve();self.root.mkdir(exist_ok=True)
        self.limit=limit;self.max_entries=max_entries;self.protected={Path(p).resolve() for p in protected}
        self.index=self.root/'index.json';self.lockpath=self.root/'.lock'
        if self.index.is_symlink() or self.lockpath.is_symlink():raise ValueError('STORE_METADATA_SYMLINK')
        if not self.index.exists():
            if any(self.root.iterdir()):raise ValueError('REFUSE_UNENROLLED_DIRECTORY')
            dump(self.index,dict(schema='BOUNDED-PAYLOAD/1',entries=[]))

    def put(self,name,data,pinned=False):
        if not isinstance(data,bytes) or len(data)>self.limit:raise ValueError('PAYLOAD_EXCEEDS_QUOTA')
        if Path(name).name!=name or name in ['.','..','index.json','.lock'] or name.startswith('.'):
            raise ValueError('INVALID_CHUNK_NAME')
        with self.lockpath.open('a') as lock:
            fcntl.flock(lock,fcntl.LOCK_EX)
            state=read(self.index);entries=state['entries'];dest=self.root/name
            if dest.exists() or dest.is_symlink() or dest.resolve() in self.protected:raise ValueError('NO_OVERWRITE_OR_PROTECTED_TARGET')
            known={e['name'] for e in entries}|{'index.json','.lock'}
            if any(p.name not in known for p in self.root.iterdir()):raise ValueError('UNREGISTERED_FILE_PRESENT')
            for e in entries:
                p=self.root/e['name']
                if p.is_symlink() or p.parent!=self.root or not p.is_file() or p.stat().st_size!=e['size'] or sha(p)!=e['sha256']:
                    raise ValueError('CHUNK_CHANGED_OR_SYMLINK')
            total=sum(e['size'] for e in entries);removals=[]
            for e in entries:
                if total+len(data)<=self.limit and len(entries)-len(removals)<self.max_entries:break
                if e.get('pinned') or (self.root/e['name']).resolve() in self.protected:continue
                removals.append(e);total-=e['size']
            if total+len(data)>self.limit or len(entries)-len(removals)>=self.max_entries:
                raise ValueError('PROTECTED_QUOTA_EXHAUSTED')
            # Evict disposable chunks before writing, so even peak payload stays
            # bounded. A failed new write cannot consume the protected budget.
            for e in removals:
                (self.root/e['name']).unlink();entries.remove(e)
            dump(self.index,state)
            try:
                with dest.open('xb') as f:f.write(data);f.flush();os.fsync(f.fileno())
            except BaseException:
                if dest.exists():dest.unlink()
                raise
            entries.append(dict(name=name,size=len(data),sha256=sha(dest),pinned=bool(pinned)))
            dump(self.index,state)
            return dict(payload_bytes=total+len(data),limit_bytes=self.limit,evicted=[e['name'] for e in removals],entries=len(entries))
