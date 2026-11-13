"""Bounded single-writer SQLite; failed lookup always falls back to fresh inference."""
import sqlite3,json,time,math
import numpy as np
from common16 import BASE,config,versions
from features16 import Admission
class Store:
 def __init__(self,path,cfg=None,version=None):
  self.path=path;self.cfg=cfg or config();self.version=version or versions();self.guard=Admission(self.cfg)
  self.db=sqlite3.connect(str(path),timeout=.005);self.db.row_factory=sqlite3.Row
  version=self.db.execute('PRAGMA user_version').fetchone()[0]
  if version not in [0,16]:raise ValueError('SCHEMA_VERSION_MISMATCH')
  self.db.execute('PRAGMA page_size=4096');self.db.execute('PRAGMA journal_mode=DELETE');self.db.execute('PRAGMA synchronous=FULL');self.db.execute('PRAGMA temp_store=MEMORY')
  # Reserve > half of total budget for the worst-case rollback journal and headers.
  self.pages=max(16,(self.cfg['max_store_bytes']-65536)//(2*4096+16))
  count=self.db.execute('PRAGMA page_count').fetchone()[0]
  if count>self.pages:raise ValueError('EXISTING_DATABASE_EXCEEDS_BUDGET')
  self.db.execute('PRAGMA max_page_count='+str(self.pages));self.db.executescript((BASE/'contracts/experience_schema.sql').read_text());self.db.commit();self.queries=0
  if self.db.execute('PRAGMA quick_check').fetchone()[0]!='ok':raise ValueError('DATABASE_INTEGRITY')
  saved=self.db.execute("SELECT value FROM metadata WHERE key='binding'").fetchone()
  self.bound_version=json.loads(saved[0]) if saved else dict(self.version)
  if not saved:
   with self.db:self.db.execute("INSERT INTO metadata VALUES('binding',?)",(json.dumps(self.bound_version,sort_keys=True),))
 def close(self):self.db.close()
 def size(self):
  from pathlib import Path
  return sum(p.stat().st_size for p in [Path(str(self.path)+s) for s in ['', '-journal','-wal','-shm']] if p.exists())
 def audit(self,now,reason,key):
  self.db.execute('INSERT INTO audit(t,reason,key) VALUES(?,?,?)',(now,reason,key))
  self.db.execute('DELETE FROM audit WHERE seq NOT IN (SELECT seq FROM audit ORDER BY seq DESC LIMIT ?)',(self.cfg['max_audit_rows'],))
 def put(self,key,x,now,provenance):
  body=json.dumps(x['vector'],allow_nan=False,separators=(',',':'))
  if len(body.encode())>self.cfg['max_record_bytes'] or len(provenance)>1024:raise ValueError('RECORD_TOO_LARGE')
  if self.guard.check(x)!='ADMITTED':return False
  with self.db:
   # FIFO by last use, then stable key. Audit is bounded, with cumulative counters.
   while self.db.execute('SELECT COUNT(*) FROM prototypes').fetchone()[0]>=self.cfg['max_prototypes']:
    victim=self.db.execute('SELECT key FROM prototypes ORDER BY last_used,key LIMIT 1').fetchone()[0]
    self.db.execute('DELETE FROM prototypes WHERE key=?',(victim,));self.audit(now,'LRU_EVICT',victim)
    self.db.execute("INSERT INTO metadata VALUES('evicted','1') ON CONFLICT(key) DO UPDATE SET value=CAST(value AS INTEGER)+1")
   self.db.execute('INSERT OR REPLACE INTO prototypes VALUES(?,?,?,?,?,?,?,?,?,?,?,?,?,?)',(key,self.version['map'],self.version['model'],self.version['feature'],x['category'],x['mode'],body,'fresh_imm',now,now+self.cfg['ttl_s'],1.,now,0,provenance))
  return True
 def lookup(self,x,now):
  start=time.perf_counter();query_ms=maintenance_ms=0.
  result=dict(hit=False,reason=self.guard.check(x,version=self.version==self.bound_version),key=None,distance=None,query_ms=0.,maintenance_ms=0.)
  if result['reason']!='ADMITTED':return result
  if len(x['vector'])!=17 or not all(math.isfinite(v) for v in x['vector']):return dict(result,reason='NONFINITE')
  try:
   saved=self.db.execute("SELECT value FROM metadata WHERE key='last_clock'").fetchone()
   if saved and now<float(saved[0]):return dict(result,reason='CLOCK_REWIND')
   candidates=self.db.execute('SELECT * FROM prototypes WHERE category=? AND mode=? ORDER BY key LIMIT ?', (x['category'],x['mode'],self.cfg['max_prototypes'])).fetchall()
   rejected=[];best=None
   for row in candidates:
    version=all(row[k+'_version']==self.version[k] for k in ['map','model','feature'])
    expired=now<row['created'] or now>=row['expires']
    f=np.array(json.loads(row['features']));v=f[3:5];current=np.array(x['velocity']);den=np.linalg.norm(v)*np.linalg.norm(current)
    cosine=float(np.clip(v@current/den,-1,1)) if den>.0064 else (1. if max(np.linalg.norm(v),np.linalg.norm(current))<.08 else -1.)
    reason=self.guard.check(x,cosine,version,expired)
    if reason!='ADMITTED':rejected.append(reason);continue
    quality=row['quality']*2**(-(now-row['created'])/self.cfg['half_life_s'])
    if quality<self.cfg['minimum_decayed_quality']:rejected.append('DECAYED');continue
    distance=float(np.sqrt(np.mean(((f-x['vector'])/self.cfg['feature_scales'])**2)))
    if distance>self.cfg['similarity_rms_max']:rejected.append('NOT_SIMILAR');continue
    if best is None or distance<best[0]:best=(distance,row)
   query_ms=(time.perf_counter()-start)*1000
   if query_ms>self.cfg['query_timeout_ms']:result.update(reason='QUERY_TIMEOUT')
   elif best:result.update(hit=True,reason='HIT_FRESH_IMM',key=best[1]['key'],distance=best[0])
   else:result.update(reason=rejected[0] if rejected else 'EMPTY_OR_CATEGORY_MODE_MISS')
   maintenance=time.perf_counter()
   with self.db:
    self.queries+=1
    if self.queries%32==0:
     for row in self.db.execute('SELECT key FROM prototypes WHERE expires<=? ORDER BY key',(now,)).fetchall():
      self.db.execute('DELETE FROM prototypes WHERE key=?',(row[0],));self.audit(now,'TTL_EVICT',row[0])
    self.db.execute("INSERT INTO metadata VALUES('last_clock',?) ON CONFLICT(key) DO UPDATE SET value=excluded.value",(str(now),))
    if result['hit']:self.db.execute('UPDATE prototypes SET last_used=?,uses=uses+1 WHERE key=?',(now,result['key']))
   maintenance_ms=(time.perf_counter()-maintenance)*1000
   if (time.perf_counter()-start)*1000>self.cfg['query_timeout_ms']:result.update(hit=False,reason='QUERY_TIMEOUT')
  except (sqlite3.Error,ValueError,TypeError,OverflowError) as exc:result.update(hit=False,reason='DATABASE_ERROR',error=type(exc).__name__)
  result.update(query_ms=query_ms,maintenance_ms=maintenance_ms)
  return result
 def maintain(self,now):
  with self.db:
   expired=self.db.execute('SELECT key FROM prototypes WHERE expires<=? ORDER BY key',(now,)).fetchall()
   for r in expired:self.db.execute('DELETE FROM prototypes WHERE key=?',(r[0],));self.audit(now,'TTL_EVICT',r[0])
  return len(expired)
