"""Offline inspection of a stopped B1 run with time-bound raw map/GNSS evidence."""
import argparse
from collections import Counter
import importlib.util
import json
import math
from pathlib import Path

import numpy as np

from common import ROOT, edition, read, sha
from replay_mowing_yaw_r10 import rows


def fine_seed_sensitivity(run, surface, regional, last, fixes, input_hashes):
    # Include original calibrated GNSS measurement covariance; the later global
    # supervisor additionally inflates its covariance for a different frame.
    bound = read(edition('R05')/'mowing/binding.json')
    atlas = read(ROOT/bound['atlas_path'])
    raw_gnss = {r['t']:np.array(r['covariance']).reshape(3,3)[:2,:2] for r in rows(run/'generation_motion/generation.jsonl')
                if r.get('sensor')=='gnss' and r.get('event')=='generated' and r['t']>=1118.}
    odometry = [r for r in rows(run/'reference/reference.jsonl') if r['t']>1118. and r.get('odom')]
    def odom(t):
        return np.array(min(odometry,key=lambda r:abs(r['odom_t']-t))['odom'])
    q=last['raw_pose'][3:]
    yaw=math.atan2(2*(q[3]*q[2]+q[0]*q[1]),1-2*(q[1]**2+q[2]**2))
    start_pose=np.array([*last['raw_pose'][:2],yaw]);start_odom=odom(last['t'])
    input_hashes['generation_motion/generation.jsonl']=sha(run/'generation_motion/generation.jsonl')
    frames={}
    for t in np.arange(1121.,1172.,5.):
        path=run/'mapping_inputs'/f'cloud_{t:.3f}.npz'
        with np.load(path) as captured:frames[float(t)]=captured['assembled']
        input_hashes[str(path.relative_to(run))]=sha(path)
    trials=[]
    for rate in (0.,-.004,-.005,-.006):
        previous=start_pose;previous_local=start_odom
        matches=[]
        for t,raw in frames.items():
            local=odom(t);local[2]-=rate*(t-last['t'])
            h=previous[2]-previous_local[2]
            seed=[*(previous[:2]+regional.rotation(h)@(local[:2]-previous_local[:2])),local[2]+h]
            fitted=surface.align(raw,seed,coarse=False)
            item=dict(t=t,held_out_geometry_valid=fitted['valid'],passes_individual_tracking_gates=False)
            if fitted['valid']:
                pose=np.asarray(fitted['pose']);fix=min(fixes,key=lambda r:abs(r['t']-t))
                key=min(raw_gnss,key=lambda f:abs(f-fix['t']))
                if abs(key-fix['t'])>.10:raise ValueError('Fixed GNSS metadata missing for registration probe')
                C=raw_gnss[key]+np.eye(2)*(.05**2+atlas['datum_translation_sigma_m']**2)
                delta=np.asarray(fix['antenna'][:2])-pose[:2]-regional.rotation(pose[2])@fix['antenna_base_m'][:2]
                residual=float(np.linalg.norm(delta));nis=float(delta@np.linalg.solve(C,delta))
                ok=residual<=.15 and nis<=9.21 and fitted['translation_from_seed_m']<=.15 and fitted['yaw_from_seed_rad']<=.035
                item.update(raw_gnss_residual_m=residual,nis=nis,translation_from_seed_m=fitted['translation_from_seed_m'],
                            yaw_from_seed_rad=fitted['yaw_from_seed_rad'],passes_individual_tracking_gates=bool(ok))
                if ok:previous=pose;previous_local=local
            matches.append(item)
        trials.append(dict(assumed_removed_rate_rad_s=rate,individually_accepted_s=[r['t'] for r in matches if r['passes_individual_tracking_gates']],
                           frames=matches))
    return dict(scope='Fine-tracking seed sensitivity on every available five-second archived cloud after the stop. '
                      'Constant rates change only seed heading. Position from archived odometry. No re-run of EKF, intermediate unseen lidar frames, or supervisor.',
                trials=trials)


def inspect(run):
    fixture = run / 'overlay/regional_math.py'
    spec = importlib.util.spec_from_file_location('recorded_regional_math',fixture)
    regional = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(regional)
    binding = read(edition('R05') / 'mowing/binding.json')
    cloud = ROOT / binding['reference_cloud_path']
    surface = regional.SurfaceMap(np.loadtxt(cloud, skiprows=7))
    surface.prepare_normals()
    sources = (r for r in rows(run/'global_sources/sources.jsonl') if r['t']>=1090)
    maps, fixes = [], []
    for r in sources:
        if r['kind']=='map' and r.get('valid'):
            maps.append(r)
        if r['kind']=='gnss' and r.get('valid') and r.get('mode')=='fixed':
            fixes.append(r)
    last=maps[-1]
    q=last['raw_pose'][3:]
    yaw=math.atan2(2*(q[3]*q[2]+q[0]*q[1]), 1-2*(q[1]**2+q[2]**2))
    cloud_probe=[]
    input_hashes={str(cloud.relative_to(ROOT)):sha(cloud)}
    # Stored frames are sampled ~5 s apart. A separate GNSS-seeded coarse
    # solve isolates seed sensitivity without changing a single live decision.
    for t in [1121.,1126.,1131.,1141.,1151.,1166.]:
        src=run/'mapping_inputs'/f'cloud_{t:.3f}.npz'
        fix=min(fixes,key=lambda r:abs(t-r['t']))
        if abs(fix['t']-t)>.10:
            raise ValueError('No contemporaneous fixed GNSS measurement')
        antenna=np.asarray(fix['antenna'][:2]);lever=fix['antenna_base_m'][:2]
        with np.load(src) as captured:
            raw=captured['assembled']
        results=[]
        for delta in (0.,.035,-.035):
            h=yaw+delta
            seed=[*(antenna-regional.rotation(h)@lever),h]
            match=surface.align(raw,seed,coarse=True)
            target=np.asarray(match['pose'])
            residual=np.linalg.norm(antenna-target[:2]-regional.rotation(target[2])@lever)
            results.append(dict(probe_heading_offset_rad=delta,registered_pose=target.tolist(),
                                held_out_geometry_ok=match['valid'],surface_p95_distance_m=match['p95_m'],
                                overlap=match['overlap'],raw_map_fixed_gnss_separation_m=float(residual)))
        cloud_probe.append(dict(t=t,fix_t=fix['t'],matches=results))
        input_hashes[str(src.relative_to(run))]=sha(src)
    ref_path=run/'reference/reference.jsonl'
    held=[r for r in rows(ref_path) if 1122.<=r['t']<=1162.]
    events_path=run/'mission_events.jsonl'
    events=[r for r in rows(events_path) if 1117.<r['t']]
    gear_path=run/'tool_evaluation/physical.jsonl'
    physical=list(rows(gear_path))
    before=max((r for r in physical if r['t']<975.216),key=lambda r:r['t'])
    after=min((r for r in physical if r['t']>975.22),key=lambda r:r['t'])
    quality_path=run/'quality/evidence.jsonl'
    raw_rejections=Counter(r['reason'] for r in rows(quality_path) if 1119.<r['t']<=1173.7)
    truth_path=run/'evaluation/truth.jsonl'
    truth=[r for r in rows(truth_path) if 1122.<=r['t']<=1162.]
    path_keys=[fixture, run/'global_sources/sources.jsonl',ref_path,events_path,gear_path,truth_path,quality_path]
    input_hashes.update({str(path.relative_to(run)):sha(path) for path in path_keys})
    return dict(scope='Offline diagnosis for original full B1. Simulator pose and tool RPM used only for independent scoring. '
                      'GNSS-seeded re-registration is diagnostic and did not run on the vehicle. Original 0.15 m gate, '
                      'NIS and separate recovery proofs remain mandatory.', session=run.name,
                rejected_map_observations=dict(raw_rejections), last_valid_map_s=last['t'],
                original_late_events=events,
                hold_1122_1162=dict(samples=len(held), reference_reasons=dict(Counter(r['reason'] for r in held)),
                                    any_plan_revalidation_required=any(r['native_supervisor']['plan_revalidation_required'] for r in held),
                                    heading_elapsed_last=held[-1]['native_supervisor']['heading_age'],
                                    local_heading_change_rad=math.remainder(held[-1]['odom'][2]-held[0]['odom'][2],math.tau),
                                    physical_heading_span_rad=max(r['rpy'][2] for r in truth)-min(r['rpy'][2] for r in truth)),
                recovery_gear_975=dict(armed=975.216, stopped=975.22, span_s=.004,
                                       physical_samples_inside=sum(975.216<=r['t']<=975.22 for r in physical),
                                       bracketing_physical_samples=[dict(t=r['t'],rpm_rad_s=r['rpm_rad_s']) for r in (before, after)]),
                raw_cloud_probe=cloud_probe, fine_tracking_seed_sensitivity=fine_seed_sensitivity(run,surface,regional,last,fixes,input_hashes),
                source_sha256=input_hashes, analysis_sha256=sha(Path(__file__)))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run',type=Path)
    parser.add_argument('--output',type=Path)
    args=parser.parse_args();res=inspect(args.run)
    data=json.dumps(res,indent=2,allow_nan=False)+'\n'
    if args.output:args.output.write_text(data)
    print(data)
