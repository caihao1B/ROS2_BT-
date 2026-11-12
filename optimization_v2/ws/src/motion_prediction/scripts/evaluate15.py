"""Offline observed-future scoring; this module is never imported online."""
from common15 import *
import gzip,math,time,argparse
from collections import defaultdict
import numpy as np
import yaml
from scipy.special import logsumexp
from history15 import History,prepare,observation
from imm15 import IMM
from trajectron15 import Trajectron

def rows(path):
    op=gzip.open if str(path).endswith('.gz') else open
    with op(path,'rt') as f:return [json.loads(line) for line in f if line.strip()]

def future_labels(records,t,dt=.4,horizon=3.2):
    """Evaluator-only labels from future observed positions, no interpolated long occlusion."""
    ts=np.array([r['t'] for r in records]);xy=np.array([r['position'] for r in records]);out=[]
    for future in t+np.arange(1,round(horizon/dt)+1)*dt:
        hi=int(np.searchsorted(ts,future))
        if hi<len(ts) and abs(ts[hi]-future)<1e-7:out.append(xy[hi]);continue
        if not 0<hi<len(ts) or ts[hi]-ts[hi-1]>.8:return None
        u=(future-ts[hi-1])/(ts[hi]-ts[hi-1]);out.append(xy[hi-1]*(1-u)+xy[hi]*u)
    return np.array(out)

def cv_prediction(f,velocity,horizon=3.2):
    state=np.array(f['state']);p=state[-1,:2];v=np.array(velocity);times=np.arange(1,round(horizon/.4)+1)*.4
    cov=np.array(f['covariance']);covs=[cov+np.eye(2)*(.1*t)**2 for t in times]
    return dict(t=f['t'],id=f['id'],frame='odom',dt=.4,horizon_s=horizon,algorithm='CAUSAL_CV_SCORING_REFERENCE',modes=[dict(mode='CV',probability=1.,mean=(p+times[:,None]*v).tolist(),covariance=np.array(covs).tolist())])

def score(prediction,labels,scale=1.):
    probabilities=np.array([m['probability'] for m in prediction['modes']]);probabilities/=probabilities.sum()
    means=np.array([m['mean'] for m in prediction['modes']]);cov=np.array([m['covariance'] for m in prediction['modes']])*scale
    residual=labels[None]-means;distance=np.linalg.norm(residual,axis=-1)
    inv=np.linalg.inv(cov);mahal=np.einsum('mti,mtij,mtj->mt',residual,inv,residual)
    component_log=-.5*(mahal+np.linalg.slogdet(cov)[1]+2*math.log(2*math.pi))
    nll=-logsumexp(np.log(np.maximum(probabilities,1e-20))[:,None]+component_log,axis=0)
    # Mixture moment ellipse is a declared calibration diagnostic, not all-mode union.
    mean=np.einsum('m,mti->ti',probabilities,means);delta=means-mean
    total=np.einsum('m,mtij->tij',probabilities,cov+delta[:,:,:,None]*delta[:,:,None,:])
    d=labels-mean;radius=np.einsum('ti,tij,tj->t',d,np.linalg.inv(total),d)
    return dict(weighted_ade_m=float(probabilities@distance.mean(axis=1)),weighted_fde_m=float(probabilities@distance[:,-1]),min_ade_m=float(distance.mean(axis=1).min()),min_fde_m=float(distance[:,-1].min()),mean_nll=float(nll.mean()),moment_ellipse_coverage95=float(np.mean(radius<=5.991)),moment_mahalanobis=radius.tolist())

def evaluate(out):
    cfg=yaml.safe_load((BASE/'config/prediction.yaml').read_text());horizon=cfg['horizon_s']
    manifest=read(BASE/'datasets/prediction/manifest.json')
    if manifest['status']!='FROZEN':raise ValueError('Frozen whole-episode dataset required')
    out=Path(out);out.mkdir(parents=True,exist_ok=True);learned=Trajectron();results=[];examples=[];provenance=[]
    for episode in manifest['episodes']:
        path=ROOT/episode['observations']['path']
        if sha(path)!=episode['observations']['sha256']:raise ValueError('Observed dataset hash changed')
        frames=rows(path);all_tracks=defaultdict(list)
        for frame in frames:
            for obj in frame['objects']:
                r=observation(frame,obj)
                if r and (not all_tracks[r['id']] or r['t']>all_tracks[r['id']][-1]['t']):all_tracks[r['id']].append(r)
        history=History();filters={};last={};count=0
        for frame in frames:
            accepted=history.add(frame)
            for r in accepted:
                k=r['id'];f=filters.get(k)
                if f is None or r['t']-f.t>1.2:filters[k]=IMM(r['t'],r['position'],r['covariance'],r['velocity'],**cfg['imm'])
                else:f.update(r['t'],r['position'],r['covariance'])
            features=history.features(frame['t'])
            for f in features:
                k=f['id']
                if f['classification']!='person' or f['t']-last.get(k,-100)<.8:continue
                labels=future_labels(all_tracks[k],f['t'],horizon=horizon)
                if labels is None:continue
                last[k]=f['t'];count+=1
                current=history.tracks[k][-1]
                for method in ['no_prediction','imm','learned']:
                    start=time.perf_counter();cpu=time.process_time()
                    if method=='no_prediction':prediction=cv_prediction(f,current['velocity'],horizon)
                    elif method=='imm':prediction=filters[k].predict(horizon)
                    else:prediction=learned.predict(f,features,horizon)
                    wall_ms=(time.perf_counter()-start)*1000;cpu_ms=(time.process_time()-cpu)*1000
                    s=score(prediction,labels)
                    row=dict(episode_id=episode['episode_id'],split=episode['split'],scenario=episode['scenario'],track_id=k,t=f['t'],method=method,**s,wall_ms=wall_ms,cpu_ms=cpu_ms)
                    results.append(row)
                    if count%3==1:
                        examples.append(dict(**{k:row[k] for k in ['episode_id','split','scenario','track_id','t','method']},history=f['state'],future_observed_labels=labels.tolist(),prediction=prediction,metrics=s))
        provenance.append(dict(episode_id=episode['episode_id'],split=episode['split'],windows=count,source_sha=sha(path)))
        print('SCORE '+episode['episode_id']+' windows='+str(count),flush=True)
    summary={}
    for split in ['train','val','test']:
        summary[split]={}
        for method in ['no_prediction','imm','learned']:
            r=[x for x in results if x['split']==split and x['method']==method]
            metrics=['weighted_ade_m','weighted_fde_m','min_ade_m','min_fde_m','mean_nll','moment_ellipse_coverage95','wall_ms','cpu_ms']
            summary[split][method]=dict(windows=len(r),**{k:float(np.mean([x[k] for x in r])) if r else None for k in metrics},latency_p95_ms=float(np.percentile([x['wall_ms'] for x in r],95)) if r else None)
    # Train-only diagnostic scale, frozen before looking at test calibration; never
    # silently applied to safety or checkpoint outputs. Report raw results above.
    calibration={}
    for method in ['no_prediction','imm','learned']:
        train=[v for x in results if x['split']=='train' and x['method']==method for v in x['moment_mahalanobis']]
        scale=max(.01,float(np.quantile(train,.95)/5.991)) if train else None
        calibration[method]=dict(train_only_moment_scale=scale,applied_online=False)
        for split in ['val','test']:
            values=[v for x in results if x['split']==split and x['method']==method for v in x['moment_mahalanobis']]
            calibration[method][split+'_coverage95']=float(np.mean(np.array(values)<=5.991*scale)) if values and scale else None
    report=dict(schema='S15-OFFLINE-SCORES/1',scope='actual simulated observations; observed future labels only offline; no ground-truth features',dataset_manifest_sha=sha(BASE/'datasets/prediction/manifest.json'),no_prediction_metric='causal CV scoring reference only; no future forecast sent to controller',summary=summary,calibration_diagnostic=calibration,episodes=provenance,network_finetuned=False)
    dump(out/'offline_metrics.json',report)
    with (out/'window_metrics.jsonl').open('w') as f:
        for r in results:f.write(json.dumps(r)+'\n')
    dump(out/'prediction_examples.json',examples)
    return report

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True);a=p.parse_args();print(json.dumps(evaluate(a.out)['summary']))
