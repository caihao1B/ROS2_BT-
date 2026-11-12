"""Whole-episode observation dataset; future labels are generated only offline.

Collection scene scripts stay in the simulator. Online predictors get only
objects on domain119. Independent domain120 truth is a separate audit artifact.
"""
from common15 import *
import argparse,os,fcntl,gzip,time,datetime,shutil
from runtime15 import run_case
SCENARIOS=['turning','stop_go','yield','occlusion']
DATA=BASE/'datasets/prediction'

def simulator_binding():
    paths=[PKG/'scripts/scene15.py',PKG/'scripts/runtime15.py',PKG/'scripts/dataset15.py',PKG/'src/Actor15.cc',BASE/'ws/install/motion_prediction/lib/libs15_actor.so',BASE/'results/S14/latest.json']
    return {str(p.relative_to(ROOT)):sha(p) for p in paths}

def compress(src,dest):
    with Path(src).open('rb') as inp,Path(dest).open('wb') as raw:
        with gzip.GzipFile(fileobj=raw,mode='wb',mtime=0) as out:shutil.copyfileobj(inp,out)

def validate(manifest):
    seen=set()
    assert manifest['simulator_binding']==simulator_binding(),'DATA_SIMULATOR_BINDING_STALE'
    for e in manifest['episodes']:
        assert e['episode_id'] not in seen;seen.add(e['episode_id'])
        assert e['split'] in ['train','val','test']
        for key in ['observations','truth_audit']:
            p=ROOT/e[key]['path'];assert p.is_file() and sha(p)==e[key]['sha256']
    assert len(seen)==12,'12 whole scene episodes required'
    return True

def collect(wall_limit=2400):
    DATA.mkdir(parents=True,exist_ok=True);manifest_path=DATA/'manifest.json'
    binding=simulator_binding();manifest=read(manifest_path) if manifest_path.exists() else dict(schema='S15-OBSERVED-DATASET/1',created_at=datetime.datetime.now(datetime.timezone.utc).isoformat(),status='PARTIAL',simulator_binding=binding,episodes=[],split_rule='entire independent scenario/trajectory episode; no sliding-window split',network_training=False,train_usage='domain statistics and uncertainty calibration only; pretrained network frozen',coordinate_frame='odom',future_labels_online=False)
    if manifest['simulator_binding']!=binding:raise ValueError('Dataset exists with another simulator binding; explicit new collection version required')
    start=time.monotonic()
    for split in ['train','val','test']:
        for scenario in SCENARIOS:
            name=split+'_'+scenario
            if any(e['episode_id']==name for e in manifest['episodes']):continue
            remaining=wall_limit-(time.monotonic()-start)
            if remaining<180:raise TimeoutError('Collection wall budget; checkpoint retained')
            out=BASE/'results/S15'/('collection_'+name+'_'+str(os.getpid()))
            print('COLLECT '+name+' '+str(out),flush=True)
            result=run_case(out,scenario,'no_prediction',split,55,min(210,remaining),collect=True)
            if not result.get('success') or not result['database_unchanged'] or result.get('tool_shutdown',{}).get('state')!='STOPPED':raise RuntimeError('Collection safety/runtime failed '+name)
            directory=DATA/name;directory.mkdir(exist_ok=True)
            compress(out/'fusion/tracks.jsonl',directory/'observations.jsonl.gz');compress(out/'dynamic_evaluation/dynamic.jsonl',directory/'truth_audit.jsonl.gz')
            e=dict(episode_id=name,scenario=scenario,split=split,seed=41,source_run=str(out.relative_to(ROOT)),sim_seconds=result['sim_seconds_observed'],wall_seconds=result['wall_seconds'],fixture_hash=sha(out/'scene/fixture_evaluation_only.json'),runtime_sha=sha(out/'runtime.json'),online_inputs=['/s09/objects'],truth_access='offline evaluator only')
            for key,file in [('observations','observations.jsonl.gz'),('truth_audit','truth_audit.jsonl.gz')]:
                p=directory/file;e[key]=dict(path=str(p.relative_to(ROOT)),sha256=sha(p),bytes=p.stat().st_size)
            manifest['episodes'].append(e);dump(manifest_path,manifest)
    validate(manifest);manifest['status']='FROZEN';dump(manifest_path,manifest)
    print(json.dumps(dict(status=manifest['status'],episodes=len(manifest['episodes']),manifest=str(manifest_path))))

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--wall-timeout',type=float,default=2400);a=p.parse_args()
    lock=(ROOT/'native_20261108/runs/session.lock').open('a');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    if os.getpgrp()!=os.getpid():os.setsid()
    collect(a.wall_timeout)
