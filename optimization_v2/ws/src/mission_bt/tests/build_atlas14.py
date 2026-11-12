"""GNSS-anchored measured regional maps; heldout stations never enter map build.
The old map only seeds heading and conservatively reprojects old occupancy.
Every training station is checked against OTHER stations, not its own cloud.
"""
import sys,uuid,copy
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from common14 import *
from binding14 import canonical
from regional_math14 import SurfaceMap,rigid_fit,rotation
from regional_occupancy14 import build as build_occupancy
from collision_volume14 import INPUTS as VOLUME_INPUTS
sys.path.insert(0,str(BASE/'ws/src/gnss_supervisor/scripts'))
from alignment import geo

SURVEY=BASE/'results/S14/survey_20261112'
OUT=BASE/'maps/s14_regions'

def station(name,s):
    p=SURVEY/name;raw=p/'sensor_diagnostic';quality={round(q['stamp'],4):q['reported_mode'] for q in rows(raw/'quality.jsonl')}
    samples=[q for q in rows(raw/'gnss.jsonl') if 9<=q['t']<=21 and quality.get(round(q['t'],4))=='fixed' and q['status']>=0]
    if len(samples)<25:raise ValueError('GNSS_STATION_INSUFFICIENT '+name)
    antenna=np.median([geo.to_enu(q['llh'],[31.2304,121.4737,10.]) for q in samples],axis=0)-[28,14,0]
    hint=next(q['pose'] for q in read(p/'runtime.json')['events'] if q['event']=='initial_hint');hint=[*hint[:2],2*math.atan2(hint[5],hint[6])]
    clouds=[np.load(f)['points'] for f in sorted(raw.glob('cloud_*.npz'))]
    old=read(p/'registration_observation.json') if (p/'registration_observation.json').exists() else None
    if old and (len(old['matches'])<3 or old['map_scatter_m']>.05):old=None
    return dict(name=name,role=s['role'],antenna=antenna[:2],hint=hint,clouds=clouds,old=old)

def merged(stations,poses):
    points=[]
    for s in stations:
        if s['name'] not in poses:continue
        p=poses[s['name']]
        for cloud in s['clouds']:
            a=cloud[np.isfinite(cloud).all(1)&(cloud[:,2]>.2)&(np.linalg.norm(cloud,axis=1)<30)].copy();a[:,:2]=a[:,:2]@rotation(p[2]).T+p[:2];points.append(a)
    cloud=np.vstack(points);_,ids=np.unique(np.floor(cloud/.08).astype(int),axis=0,return_index=True)
    return cloud[np.sort(ids)]

def match(s,reference,seed):
    accepted=[];rejected=[]
    for index,raw in enumerate(s['clouds']):
        fits=[reference.align(raw,[*seed[:2],seed[2]+dyaw]) for dyaw in [-.15,0,.15]];good=[f for f in fits if f['valid']]
        if not good:rejected.append(dict(frame=index,reason='QUALITY_REJECTED',fits=fits));continue
        spread=max(math.dist(f['pose'][:2],good[0]['pose'][:2]) for f in good)
        ys=max(abs(math.atan2(math.sin(f['pose'][2]-good[0]['pose'][2]),math.cos(f['pose'][2]-good[0]['pose'][2]))) for f in good)
        if spread>.08 or ys>.015:rejected.append(dict(frame=index,reason='AMBIGUOUS_MULTI_START',fits=fits));continue
        accepted.append(min(good,key=lambda f:f['p95_m']))
    if len(accepted)<3:raise ValueError('REGIONAL_MATCH_INSUFFICIENT '+s['name']+' '+str(rejected))
    poses=np.array([f['pose'] for f in accepted]);p=np.median(poses,axis=0);p[2]=np.median(np.unwrap(poses[:,2]));scatter=float(np.linalg.norm(poses[:,:2]-p[:2],axis=1).max())
    if scatter>.05:raise ValueError('REGIONAL_MATCH_UNSTABLE '+s['name'])
    antenna=p[:2]+rotation(p[2])@[-.1,0.];error=float(np.linalg.norm(antenna-s['antenna']))
    return dict(name=s['name'],source=str((SURVEY/s['name']).relative_to(ROOT)),map_pose=p.tolist(),registered_antenna_xy=antenna.tolist(),site_antenna_xy=s['antenna'].tolist(),residual_m=error,temporal_scatter_m=scatter,accepted=accepted,rejected=rejected)

def main():
    OUT.mkdir(parents=True,exist_ok=True);cfg=read(BASE/'config/s14_survey_stations.json');base=read(BASE/'maps/frozen/manifest.json');m=plan();regions={};bindings={}
    for z in ['A','B']:
        stations=[station(n,s) for n,s in cfg['stations'].items() if s['region']==z];train=[s for s in stations if s['role']=='train'];held=[s for s in stations if s['role']=='heldout']
        eligible=[s['old'] for s in train if s['old']];x=np.array([s['raw_map_antenna_xy'] for s in eligible]);y=np.array([s['site_antenna_xy'] for s in eligible]);R,t=rigid_fit(x,y);heading=math.atan2(R[1,0],R[0,0]);center=x.mean(0)
        d=(x-center)@R.T;J=np.array([[[1,0,-p[1]],[0,1,p[0]]] for p in d]).reshape(-1,3);sigma2=max(.03**2,float(np.sum((y-(x@R.T+t))**2)/(2*len(x)-3)));covariance=sigma2*np.linalg.inv(J.T@J)
        poses={s['name']:[*(s['antenna']-rotation(s['old']['map_pose'][2]+heading)@[-.1,0.]),s['old']['map_pose'][2]+heading] for s in train if s['old']}
        iterations=[]
        for iteration in range(3):
            new={};checks=[]
            for s in train:
                # Other stations only: a bad station cannot validate itself.
                reference=SurfaceMap(merged([q for q in train if q['name']!=s['name']],poses));reference.prepare_normals()
                seed=poses.get(s['name'],s['hint']);v=match(s,reference,seed);checks.append(v)
                if v['residual_m']>=.08:raise ValueError('TRAIN_LEAVE_STATION_OUT_GNSS '+s['name']+' '+str(v['residual_m']))
                h=v['map_pose'][2];new[s['name']]=[*(s['antenna']-rotation(h)@[-.1,0.]),h]
            poses=new;iterations.append(checks);print(z,'leave-station-out',iteration,[round(v['residual_m'],4) for v in checks],flush=True)
        cloud=merged(train,poses);reference=SurfaceMap(cloud);reference.prepare_normals()
        heldout=[match(s,reference,s['hint']) for s in held] # No heldout GNSS in seeds or optimization.
        errors=np.array([v['residual_m'] for v in heldout]);training=[v['residual_m'] for v in iterations[-1]]
        route=np.array([p for s in m['zones'][z]['plans'][0]['segments'] for p in s['poses']]);extent=[(route[:,:2].min(0)-[28,14]-1).tolist(),(route[:,:2].max(0)-[28,14]+1).tolist()]
        passed=bool(len(heldout)==4 and max(errors)<.08 and np.sqrt(np.mean(errors**2))<.05 and max(training)<.08)
        cloudpath=OUT/(z+'_registered_cloud.ply')
        with cloudpath.open('w') as f:
            f.write('ply\nformat ascii 1.0\nelement vertex '+str(len(cloud))+'\nproperty float x\nproperty float y\nproperty float z\nend_header\n');np.savetxt(f,cloud,fmt='%.6f')
        bindings[str(cloudpath.relative_to(ROOT))]=sha(cloudpath)
        c=dict(rotation_yaw_rad=heading,translation_xy_m=t.tolist(),raw_center_xy=center.tolist(),registered_center_xy=(R@center+t).tolist(),parameter_covariance=covariance.tolist(),station_sigma_m=math.sqrt(sigma2),site_extent_xy=extent,train=iterations[-1],heldout=heldout,train_residuals_m=training,heldout_residuals_m=errors.tolist(),heldout_rmse_m=float(np.sqrt(np.mean(errors**2))),passed=passed,additional_position_sigma_m=.03,additional_yaw_sigma_rad=math.sqrt(covariance[2,2]),cloud_path=str(cloudpath.relative_to(ROOT)),cloud_frame='site_enu',map_build_poses=poses,training_iterations=iterations,legacy_heading_controls=[s['source'] for s in eligible],scope=f'{z}1 original motion envelope only; no six-cluster or connector claim')
        occupancy=build_occupancy(train,poses,OUT,z);c.update(occupancy)
        for relative in occupancy.values():bindings[relative]=sha(ROOT/relative)
        regions[z]=c;print(z,'heldout',errors,'passed',passed,flush=True)
        for s in stations:
            p=SURVEY/s['name']
            paths=[p/f for f in ['runtime.json','scene/world.sdf','scene/sensor_manifest.json','scene/vehicle.json','capture_inputs.json','sensor_diagnostic/gnss.jsonl','sensor_diagnostic/quality.jsonl']]+list((p/'sensor_diagnostic').glob('cloud_*.npz'))
            if (p/'registration_observation.json').exists():paths.append(p/'registration_observation.json')
            if (p/'registration_rejection.json').exists():paths.append(p/'registration_rejection.json')
            for f in paths:bindings[str(f.relative_to(ROOT))]=sha(f)
    for p in [BASE/'maps/frozen/manifest.json',BASE/'maps/frozen/map_cloud.ply',BASE/'maps/frozen/map.db',BASE/'config/sensors.yaml',BASE/'config/gnss_alignment.yaml',BASE/'config/s14_survey_stations.json',PKG/'scripts/regional_math14.py',PKG/'tests/fit_survey14.py',PKG/'scripts/survey_capture14.py',Path(__file__).resolve()]:bindings[str(p.relative_to(ROOT))]=sha(p)
    bindings[str((PKG/'scripts/regional_occupancy14.py').relative_to(ROOT))]=sha(PKG/'scripts/regional_occupancy14.py')
    for f in VOLUME_INPUTS+[PKG/'scripts/collision_volume14.py']:
        bindings[str(f.relative_to(ROOT))]=sha(f)
    seed=canonical(dict(regions=regions,bindings=bindings));map_id=str(uuid.uuid5(uuid.NAMESPACE_URL,'s14-regional-atlas:'+seed));epoch=str(uuid.uuid5(uuid.NAMESPACE_URL,'s14-enu-epoch:'+seed))
    payload=dict(schema='S14-REGIONAL-ATLAS/2',map_id=map_id,frame_epoch=epoch,source_map_revision=base['revision'],calibration_revision=base['calibration_revision'],frame='map',raw_frame='s06_frozen_slam',xy_definition='GNSS anchored training sensor surfaces in WGS84 ENU minus [28,14]; independent heldout scan matching',datum_llh=[31.2304,121.4737,10.],site_origin=[28.,14.,0.],datum_translation_sigma_m=.02,antenna_base_m=[-.1,0.,.8],regions=regions,bindings=bindings,passed=all(c['passed'] for c in regions.values()))
    payload['revision']=canonical(payload);dump(OUT/'manifest.json',payload)
    if not payload['passed']:raise ValueError('ATLAS_HELDOUT_FAILED; do not authorize motion')
    derived=copy.deepcopy(m);derived['base_plan_revision']=m['plan_revision'];derived['atlas_revision']=payload['revision'];derived['runtime_eligible_clusters']=['A1','B1'];derived['frame'].update(map_id=map_id,map_revision=payload['revision'],frame_epoch=epoch,source='S14 GNSS anchored measured regional atlas',nonrigid_map_error_corrected='separately rebuilt measured regions A1/B1 only',live_execution_requires='regional bounds, independent GNSS innovation and unchanged original safety gates')
    derived.pop('plan_revision');derived['plan_revision']=canonical(derived);planout=BASE/'plans/s14_registered';planout.mkdir(exist_ok=True);dump(planout/'mission_plan.json',derived);dump(planout/'manifest.json',dict(base_plan_revision=m['plan_revision'],plan_revision=derived['plan_revision'],atlas_revision=payload['revision'],files={'mission_plan.json':sha(planout/'mission_plan.json')}))
    dump(OUT/'geometry_invariance.json',dict(unchanged={k:derived[k]==m[k] for k in ['zones','connector_routes','task_revision']},base_plan_revision=m['plan_revision'],registered_plan_revision=derived['plan_revision'],original_fixed_points=21,original_fixed_area_m2=16.35945787959566))
if __name__=='__main__':main()
