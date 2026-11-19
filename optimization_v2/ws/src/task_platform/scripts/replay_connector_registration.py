"""Compare raw-to-frozen surface registration on captured scans. Truth is consulted only AFTER both fits."""
import argparse
from bisect import bisect_left
import json
import math
from pathlib import Path

import numpy as np

from common import APP, ROOT, dump, edition, read, sha
from connector_registration import derive


def records(path):
    with path.open() as f:
        return [json.loads(line) for line in f if line.strip()]


def nearest_time(stream,times,t):
    i=bisect_left(times,t)
    return min(stream[max(0,i-1):min(len(stream),i+1)],key=lambda x:abs(t-x['t']))


def models(source, reference):
    output={}
    for name,source_text in [('original',source),('trimmed',derive(source))]:
        local={}
        exec(compile(source_text,'regional_math.py','exec'),local)
        model=local['SurfaceMap'](reference)
        model.prepare_normals()
        output[name]=model
    return output


def evaluate(path,cold):
    binding=read(edition('R05')/'mowing/binding.json')
    source=(path/'overlay/regional_math.py').read_text()
    m=models(source,np.loadtxt(ROOT/binding['reference_cloud_path'],skiprows=7))
    cfg=read(path/'mission.json')
    ref=records(path/'quality/evidence.jsonl')
    raw=[r for r in ref if 'pose' in r]
    ts=[r['t'] for r in raw]
    fixes=[r for r in records(path/'global_sources/sources.jsonl') if r['kind']=='gnss' and r['valid']]
    fix_t=[r['t'] for r in fixes]
    truth=records(path/'evaluation/truth.jsonl')
    truth_t=[r['t'] for r in truth]
    files=sorted((float(p.stem.split('_')[1]),p) for p in (path/'mapping_inputs').glob('cloud_*.npz'))
    if cold:
        files=[item for item in files if item[0]>=8]
    else:
        files=[item for i,item in enumerate(files) if 890<=item[0]<=1130 or (i%9==0 and 32<item[0]<890)]
    previous={}
    angles=np.linspace(-.15,.15,7)
    report=[]
    for t,p in files:
        data=np.load(p)
        item=dict(t=t)
        for name,model in m.items():
            if cold and name not in previous:
                initial=read(path/'scene/fixture.json')['coarse_operator_map_hint']
                test=[model.align(data['assembled'],[*initial[:2],initial[2]+d],coarse=True) for d in angles]
                good=[g for g in test if g['valid']]
                if not good or max(math.dist(g['pose'][:2],good[0]['pose'][:2]) for g in good)>.08 or max(abs(math.remainder(g['pose'][2]-good[0]['pose'][2],math.tau)) for g in good)>.015:continue
                best=min(good,key=lambda x:x['p95_m'])
            else:
                if cold:
                    seed=previous[name]
                else:
                    # Previous raw SENSOR solution provides the same tracking seed to both fitters.
                    i=bisect_left(ts,t)-1
                    if i<0 or t-ts[i]>.6:continue
                    pp=raw[i].get('raw_pose',raw[i]['pose'])
                    seed=[*pp[:2],2*math.atan2(pp[5],pp[6])]
                best=model.align(data['assembled'],seed,coarse=False)
                if not best['valid']:continue
            pose=best['pose']
            previous[name]=pose
            gnss=nearest_time(fixes,fix_t,t)
            if abs(t-gnss['t'])>.3:continue
            lever=cfg['rig']['gnss']['xyz'][0]
            antenna=[pose[0]+math.cos(pose[2])*lever,pose[1]+math.sin(pose[2])*lever]
            tr=nearest_time(truth,truth_t,t)
            true_xy=np.array(tr['xyz'][:2])-cfg['map_offset']
            item[name]=dict(heldout_overlap=best['overlap'],heldout_p95_m=best['p95_m'],gnss_residual_m=math.dist(antenna,gnss['antenna'][:2]),
                            physical_xy_m=float(np.linalg.norm(np.array(pose[:2])-true_xy)),physical_yaw_rad=abs(math.remainder(pose[2]-tr['rpy'][2],math.tau)))
        report.append(item)
    summary={}
    for name in m:
        accepted=[r[name] for r in report if name in r]
        summary[name]=dict(evaluable=len(accepted),gnss_under_15cm=sum(r['gnss_residual_m']<=.15 for r in accepted),
            median_physical_xy_m=float(np.median([r['physical_xy_m'] for r in accepted])),
            p95_physical_xy_m=float(np.quantile([r['physical_xy_m'] for r in accepted],.95)),
            median_physical_yaw_rad=float(np.median([r['physical_yaw_rad'] for r in accepted])),
            median_raw_gnss_residual_m=float(np.median([r['gnss_residual_m'] for r in accepted])))
    return dict(session=path.name,coarse_hint_map=read(path/'scene/fixture.json')['coarse_operator_map_hint'],
                seven_heading_offsets_rad=angles.tolist(),winner_rule='Minimum held-out surface p95 after all seven LiDAR starts mutually converge within 8 cm / 0.015 rad; no truth or GNSS winner selection',
                training_choice='Archived stationary startup only. Trim to nearest 60% of fitting-half correspondences and 4 cm Huber; 7 yaw guesses are a single capture.',
                fixture='cold coarse hint; same stationary vehicle' if cold else 'moving retained full B1 observation; same previous raw observed pose for both fits',
                frame_count=len(files),surface_map_sha256=sha(ROOT/binding['reference_cloud_path']),comparison=summary,frames=report)


if __name__=='__main__':
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('output',type=Path)
    args=ap.parse_args()
    cases=[(APP/'results/R10/candidates/mowing_20261119_084537_532941',True),(APP/'results/R12/mowing_20261119_072044_764057',False)]
    results=[evaluate(run,cold) for run,cold in cases]
    dump(args.output,dict(scope='Offline paired technical replay only. Every held-out and GNSS gate stays in force in the real runtime; no parking or route credit.',cases=results))
    print(json.dumps([r['comparison'] for r in results],indent=2))
