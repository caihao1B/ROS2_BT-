"""Build corridor surfaces from sensor observations, with heldout stations.

Existing A/B maps seed registration. Each new train station is matched before
its clouds join the reference; GNSS independently gates the resulting pose.
Heldout station clouds never enter the final map or its occupancy raster.
"""
from common17 import *
import importlib.util,copy,uuid,numpy as np,math

def build():
    add14()
    from regional_math14 import SurfaceMap,rotation
    from regional_occupancy14 import build as occupancy
    sys.path.insert(0,str(BASE/'ws/src/map_manager/scripts'))
    from mapping_analysis import ply
    spec=importlib.util.spec_from_file_location('survey_math14',BASE/'ws/src/mission_bt/tests/build_atlas14.py');mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod)
    source=BASE/'maps/s17_regions/manifest.json';prior=read(source);out=RESULTS/'corridor_qualification';cfg=read(out/'stations.json');mod.SURVEY=out/'survey';original=read(BASE/'plans/validated/mission_plan.json')
    base_cloud=np.vstack([ply(ROOT/r['cloud_path']) for r in prior['regions'].values()]);train=[];poses={};checks={};failures={};bindings={str(source.relative_to(ROOT)):sha(source)}
    def station(name):
        p=out/'survey'/name
        if not (p/'runtime.json').exists():raise ValueError('STATION_NOT_CAPTURED '+name)
        runtime=read(p/'runtime.json');contacts=read(p/'evaluation/contacts.json')
        if not runtime['success'] or not runtime['database_unchanged'] or any(v for k,v in contacts.items() if not k.endswith('|ground::body::ground')):raise ValueError('STATION_MOTION_OR_CONTACT '+name)
        item=mod.station(name,cfg['stations'][name])
        paths=[p/f for f in ['runtime.json','evaluation/contacts.json','scene/world.sdf','sensor_diagnostic/gnss.jsonl','sensor_diagnostic/quality.jsonl']]+sorted((p/'sensor_diagnostic').glob('cloud_*.npz'))
        bindings.update({str(q.relative_to(ROOT)):sha(q) for q in paths});return item
    order=['C00','C10','C01','C09','C02','C08','C03','C07','C04','C06','C05']
    for name in order:
        try:
            s=station(name);cloud=np.vstack([base_cloud,mod.merged(train,poses)]) if train else base_cloud
            ref=SurfaceMap(cloud);ref.prepare_normals();v=mod.match(s,ref,s['hint'])
            if v['residual_m']>=.08:raise ValueError('TRAIN_GNSS_GATE '+name+' '+str(v['residual_m']))
            checks[name]=v;train.append(s);poses[name]=v['map_pose'];dump(out/'survey'/name/'registration_C17.json',v);print(name,round(v['residual_m'],5),flush=True)
        except ValueError as e:failures[name]=str(e);print(name,str(e)[:300],flush=True)
    cloud=np.vstack([base_cloud,mod.merged(train,poses)]) if train else base_cloud;ref=SurfaceMap(cloud);ref.prepare_normals();held=[]
    for name in ['H00','H01','H02','H03']:
        try:
            s=station(name);v=mod.match(s,ref,s['hint']);v['passed']=v['residual_m']<.08;held.append(v);checks[name]=v;dump(out/'survey'/name/'registration_C17.json',v);print(name,round(v['residual_m'],5),flush=True)
        except ValueError as e:failures[name]=str(e);print(name,str(e)[:300],flush=True)
    errors=[v['residual_m'] for v in held];passed=len(train)==11 and len(held)==4 and not failures and max(errors)<.08 and np.sqrt(np.mean(np.square(errors)))<.05
    result=dict(passed=bool(passed),train_count=len(train),heldout_count=len(held),failures=failures,errors_m=errors,checks=checks,truth_used_for_registration=False);dump(out/'qualification.json',result)
    if not passed:return result
    target=BASE/'maps/s17_full';target.mkdir(parents=True,exist_ok=True);cloudpath=target/'C_registered_cloud.ply'
    with cloudpath.open('w') as f:f.write('ply\nformat ascii 1.0\nelement vertex '+str(len(cloud))+'\nproperty float x\nproperty float y\nproperty float z\nend_header\n');np.savetxt(f,cloud,fmt='%.6f')
    # Measured ray occupancy includes previous A/B training observations, not
    # their heldout stations and not a simulator free-space oracle.
    old=[];allposes=dict(poses)
    for z,r in prior['regions'].items():
        for name,pose in r['map_build_poses'].items():
            record=next(v for v in r['train'] if v['name']==name);folder=ROOT/record['source'];label=z+'_'+name
            old.append(dict(name=label,clouds=[np.load(p)['points'] for p in sorted((folder/'sensor_diagnostic').glob('cloud_*.npz'))]));allposes[label]=pose
    raster=occupancy(old+train,allposes,target,'C')
    xy=np.array([q[:2] for r in original['connector_routes'] for s in r['segments'] for q in s['poses']])-[28,14];extent=[(xy.min(0)-2).tolist(),(xy.max(0)+2).tolist()]
    points=np.array([v['registered_antenna_xy'] for n,v in checks.items() if n.startswith('C')]);center=points.mean(0);delta=points-center;J=np.array([[[1,0,-p[1]],[0,1,p[0]]] for p in delta]).reshape(-1,3);covariance=.03**2*np.linalg.inv(J.T@J)
    region=dict(cloud_path=str(cloudpath.relative_to(ROOT)),cloud_frame='site_enu',site_extent_xy=extent,registered_center_xy=center.tolist(),raw_center_xy=center.tolist(),parameter_covariance=covariance.tolist(),rotation_yaw_rad=0.,translation_xy_m=[0.,0.],passed=True,train=[checks[n] for n in order],heldout=held,heldout_rmse_m=float(np.sqrt(np.mean(np.square(errors)))),map_build_poses=poses,scope='Sensor-qualified continuous connector plus A/B references; actual driving still required',**raster)
    atlas=copy.deepcopy(prior);atlas.pop('revision');atlas['source_atlas_revision']=prior['revision'];atlas['regions']['C']=region;atlas['bindings'].update(bindings)
    for p in [cloudpath,Path(__file__).resolve(),out/'stations.json',out/'qualification.json',*[ROOT/k for k in raster.values()]]:atlas['bindings'][str(p.relative_to(ROOT))]=sha(p)
    key=canonical(atlas);atlas['map_id']=str(uuid.uuid5(uuid.NAMESPACE_URL,'S17-full-atlas:'+key));atlas['frame_epoch']=str(uuid.uuid5(uuid.NAMESPACE_URL,'S17-full-epoch:'+key));atlas['revision']=canonical(atlas);dump(target/'manifest.json',atlas)
    return result
if __name__=='__main__':raise SystemExit(0 if build()['passed'] else 1)
