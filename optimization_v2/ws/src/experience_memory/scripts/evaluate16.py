"""Offline evaluation boundary: future observed labels exist ONLY in this module."""
from common16 import *
import argparse,time,math,shutil,gzip,resource
from collections import defaultdict,Counter
import numpy as np
from evaluate15 import rows,future_labels,score
from history15 import observation
from features16 import context
from store16 import Store
from engine16 import MemoryEngine
from trajectron15 import Trajectron
GROUPS=['full','event','cache']
def load_episode(e):
 frames=rows(ROOT/e['observations']['path']);online=rows(ROOT/e['source_run']/'online/observations.jsonl');i=0;state={'robot':{}};gate={}
 for frame in frames:
  while i<len(online) and online[i]['t']<=frame['t']:
   j=online[i];i+=1
   if j['record_type']=='status':state.update(t=j['t'],navigation_allowed=j['navigation_allowed'])
   elif j['record_type']=='gate':gate=j
   elif j['record_type']=='odom':
    p=j['pose'];qx,qy,qz,qw=p[3:7];state['robot']=dict(t=j['t'],x=p[0],y=p[1],yaw=math.atan2(2*(qw*qz+qx*qy),1-2*(qy*qy+qz*qz)),v=gate.get('measured_v',0.),w=gate.get('measured_w',0.))
  frame['context']=json.loads(json.dumps(state))
 tracks=defaultdict(list)
 for frame in frames:
  for o in frame['objects']:
   r=observation(frame,o)
   if r and (not tracks[r['id']] or r['t']>tracks[r['id']][-1]['t']):tracks[r['id']].append(r)
 return frames,tracks

def prepare(out):
 out=Path(out);out.mkdir(parents=True,exist_ok=True);manifest=read(BASE/'datasets/prediction/manifest.json');cfg=config();anchor=time.time();model=Trajectron();snapshot=out/'initial.sqlite';store=Store(snapshot);training=[];inputs={};frame_counts={}
 # Only train labels can determine a stored model-selection recommendation.
 for e in manifest['episodes']:
  for p in [ROOT/e['observations']['path'],ROOT/e['source_run']/'online/observations.jsonl']:inputs[str(p.relative_to(ROOT))]=sha(p)
  if e['split']!='train':continue
  frames,tracks=load_episode(e);engine=MemoryEngine('full',store,model);last={};frame_counts[e['episode_id']]=len(frames)
  for frame in frames:
   result=engine.predict([frame],anchor+frame['t'])
   for d,pred in zip(result['memory']['decisions'],result['forecasts']):
    k=d['id'];f=next(x for x in engine.base.history.features(frame['t']) if x['id']==k)
    if f['classification']!='person' or f['t']-last.get(k,-100)<.8:continue
    last[k]=f['t'];labels=future_labels(tracks[k],f['t'])
    if labels is None:continue
    fast=engine.base.filters[k].predict(3.2);ms=score(fast,labels);ls=score(pred,labels)
    qualified=ms['weighted_ade_m']<=cfg['train_imm_ade_max_m'] and ms['weighted_ade_m']<=ls['weighted_ade_m']
    key=e['episode_id']+':'+str(k)+':'+str(f['t'])
    added=store.put(key,d['context'],anchor-60,json.dumps(dict(episode=e['episode_id'],split='train',observed_sha=e['observations']['sha256']))) if qualified else False
    training.append(dict(episode=e['episode_id'],t=f['t'],id=k,imm_ade=ms['weighted_ade_m'],learned_ade=ls['weighted_ade_m'],qualified=qualified,stored=added))
 store.close();dump(out/'training.json',training);dump(out/'inputs.json',inputs)
 info=dict(anchor_utc=anchor,sha256=sha(snapshot),bytes=snapshot.stat().st_size,versions=versions(),source='train-only observed labels',test_labels_in_store=False,prototypes=sum(x['stored'] for x in training))
 dump(out/'snapshot.json',info);print('SNAPSHOT '+json.dumps(info),flush=True);return info

def replay(out):
 out=Path(out);info=read(out/'snapshot.json');snapshot=out/'initial.sqlite';assert sha(snapshot)==info['sha256'];model=Trajectron();manifest=read(BASE/'datasets/prediction/manifest.json');measurements=[];samples=[];examples=[];schedules={};cfg=yaml.safe_load((BASE/'evaluation/memory_ablation.yaml').read_text());summaries={}
 # Warm-up separate from measured cases. No input-specific model training occurs.
 warmed=False
 for split in ['val','test']:
  for ei,e in enumerate([x for x in manifest['episodes'] if x['split']==split]):
   frames,tracks=load_episode(e)
   for repeat in range(cfg['replay_repeats']):
    order=GROUPS[(ei+repeat)%3:]+GROUPS[:(ei+repeat)%3]
    for group in order:
     db=out/(e['episode_id']+'_'+str(repeat)+'_'+group+'.sqlite');shutil.copyfile(snapshot,db);store=Store(db);engine=MemoryEngine(group,store,model)
     if not warmed:
      warm=MemoryEngine('full',store,model)
      for f in frames[:40]:warm.predict([f],info['anchor_utc']+f['t'])
      warmed=True
     schedule=[];last={};ts=time.perf_counter();cpu=time.process_time();case_rows=[]
     for frame in frames:
      r=engine.predict([frame],info['anchor_utc']+frame['t']);m=r['memory'];ds=m['decisions']
      for d in ds:schedule.append([frame['t'],d['id'],d['event'],d['event_reasons']])
      row=dict(split=split,episode=e['episode_id'],group=group,repeat=repeat,t=frame['t'],wall_ms=r['wall_ms'],cpu_ms=r['cpu_ms'],**{k:v for k,v in m.items() if k!='decisions'},queries=sum(d['lookup']['reason']!='NOT_QUERIED' for d in ds),hits=sum(d['lookup']['hit'] for d in ds));measurements.append(row);case_rows.append(row)
      if repeat!=0:continue
      for d,pred in zip(ds,r['forecasts']):
       k=d['id'];ft=d['t']
       if pred['classification']!='person' or ft-last.get(k,-100)<.8:continue
       labels=future_labels(tracks[k],ft)
       if labels is None:continue
       last[k]=ft;s=score(pred,labels)
       sample=dict(split=split,episode=e['episode_id'],group=group,t=ft,id=k,hit=d['lookup']['hit'],selected=d['selected'],lookup=d['lookup'],context=d['context'],**{k:s[k] for k in ['weighted_ade_m','weighted_fde_m','moment_ellipse_coverage95']})
       samples.append(sample)
       if group=='cache' and len([x for x in examples if x['episode']==e['episode_id']])<24:
        f=next(x for x in engine.base.history.features(frame['t']) if x['id']==k)
        examples.append(dict(**sample,history=f['state'],prediction=pred,future_observed=labels.tolist()))
     schedule_sha=__import__('hashlib').sha256(json.dumps(schedule).encode()).hexdigest();schedules[e['episode_id']+':'+str(repeat)+':'+group]=schedule_sha
     # Full loop includes bookkeeping/scoring and is reported separately; core prediction timers exclude offline future labels.
     summaries[e['episode_id']+':'+str(repeat)+':'+group]=dict(core_cpu_ms=sum(x['cpu_ms'] for x in case_rows),core_wall_ms=sum(x['wall_ms'] for x in case_rows),evaluator_wall_s=time.perf_counter()-ts,evaluator_cpu_s=time.process_time()-cpu,snapshot_sha=info['sha256'],db_bytes=store.size(),schedule_sha=schedule_sha)
     store.close();db.unlink() # Keep frozen initial DB and bounded measured audits, not 72 duplicate stores.
   print('REPLAY '+e['episode_id'],flush=True)
 metrics={}
 for split in ['val','test']:
  metrics[split]={}
  for group in GROUPS:
   mm=[x for x in measurements if x['split']==split and x['group']==group];ss=[x for x in samples if x['split']==split and x['group']==group]
   metrics[split][group]=dict(frames=len(mm),scored_windows=len(ss),cpu_ms=sum(x['cpu_ms'] for x in mm),wall_ms=sum(x['wall_ms'] for x in mm),latency_p95_ms=float(np.percentile([x['wall_ms'] for x in mm],95)),**{k:sum(x[k] for x in mm) for k in ['observe_ms','feature_ms','query_ms','maintenance_ms','inference_ms','learned_calls','imm_calls','queries','hits']},**{k:float(np.mean([x[k] for x in ss])) if ss else None for k in ['weighted_ade_m','weighted_fde_m','moment_ellipse_coverage95']})
 event={(x['episode'],x['t'],x['id']):x for x in samples if x['split']=='test' and x['group']=='event'}
 hits=[x for x in samples if x['split']=='test' and x['group']=='cache' and x['hit']];wrong=[x for x in hits if x['weighted_ade_m']>event[(x['episode'],x['t'],x['id'])]['weighted_ade_m']+.02]
 fair=all(len({schedules[e['episode_id']+':'+str(r)+':'+g] for g in GROUPS})==1 for e in manifest['episodes'] if e['split'] in ['val','test'] for r in range(3))
 report=dict(metrics=metrics,fair_schedules=fair,schedules=schedules,paired_episodes=summaries,test_scored_hits=len(hits),wrong_hits=len(wrong),wrong_hit_fraction=len(wrong)/len(hits) if hits else None,wrong_hit_definition='cached weighted ADE > same-time event baseline + 0.02 m; evaluator-only observed futures',rss_peak_bytes=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss*1024,gpu_inference='N/A: CPU-only pinned model',initial_snapshot_sha=info['sha256'],schema='S16-PAIRED-REPLAY/1')
 dump(out/'offline.json',report);dump(out/'examples.json',examples)
 for name,data in [('measurements',measurements),('scores',samples)]:
  with gzip.open(out/(name+'.jsonl.gz'),'wt') as f:
   for x in data:f.write(json.dumps(x)+'\n')
 return report
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True);p.add_argument('--prepare-only',action='store_true');a=p.parse_args();prepare(a.out)
 if not a.prepare_only:replay(a.out)
