import argparse,json,sys,os,time,shutil
from common16 import *
from store16 import Store
from engine16 import MemoryEngine
p=argparse.ArgumentParser();p.add_argument('--method',required=True);a=p.parse_args()
snapshot=Path(os.environ['S16_SNAPSHOT']);path=Path(os.environ['S16_CASE'])/'memory.sqlite';shutil.copyfile(snapshot,path)
class UnavailableStore:
 def __init__(self):self.version=versions()
 def lookup(self,x,now):return dict(hit=False,reason='DATABASE_UNAVAILABLE',query_ms=0.,maintenance_ms=0.)
 def close(self):pass
try:store=Store(path)
except Exception:store=UnavailableStore()
engine=MemoryEngine(a.method,store)
print(json.dumps(dict(ready=True,method=a.method)),flush=True)
try:
 for line in sys.stdin:
  r=json.loads(line)
  if r.get('stop'):break
  frames=r['observations'];fault=os.environ.get('S16_FAULT','none');t=frames[-1]['t'];old=store.version.copy()
  try:
   if fault=='wrong_match' and 18<=t<28:store.version=dict(old,map='injected-map-mismatch')
   start=time.perf_counter();cpu=time.process_time();results=[engine.predict([f]) for f in frames];result=results[-1]
   memory=dict(result['memory'])
   for key in ['observe_ms','feature_ms','query_ms','maintenance_ms','inference_ms','learned_calls','imm_calls']:memory[key]=sum(x['memory'][key] for x in results)
   memory['decisions']=[d for x in results for d in x['memory']['decisions']]
   result.update(memory=memory,processed_frames=len(frames),wall_ms=(time.perf_counter()-start)*1000,cpu_ms=(time.process_time()-cpu)*1000)
   if fault=='wrong_match' and 18<=t<28:result['injected_fault']='MAP_VERSION_MISMATCH'
  except Exception as exc:result=dict(t=t,frame='odom',method=a.method,status='INFERENCE_ERROR',forecasts=[],error=repr(exc))
  finally:store.version=old
  print(json.dumps(dict(id=r['id'],result=result),allow_nan=False),flush=True)
finally:
 dump(path.with_suffix('.audit.json'),dict(bytes=store.size(),rows=store.db.execute('SELECT COUNT(*) FROM prototypes').fetchone()[0],integrity=store.db.execute('PRAGMA quick_check').fetchone()[0],snapshot_sha=sha(snapshot)) if isinstance(store,Store) else dict(status='DATABASE_UNAVAILABLE_FALLBACK'))
 store.close()
