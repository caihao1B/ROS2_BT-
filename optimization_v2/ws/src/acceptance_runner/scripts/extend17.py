"""New atlas identity only after all extension stations pass unchanged gates."""
from common17 import *
import copy,uuid,numpy as np

def build():
    cfg=read(RESULTS/'qualification/stations.json');checks={}
    for name,s in cfg['stations'].items():
        p=RESULTS/'qualification/survey'/name/'qualification17.json'
        if not p.exists() or not read(p).get('passed'):raise ValueError('EXTENSION_NOT_QUALIFIED '+name)
        v=read(p)
        for k,h in v['input_hashes'].items():
            if sha(ROOT/k)!=h:raise ValueError('STATION_INPUT_CHANGED '+k)
        checks[name]=v
    original=read(BASE/'plans/validated/mission_plan.json');source=BASE/'maps/s14_regions/manifest.json';atlas=copy.deepcopy(read(source));atlas['source_atlas_revision']=atlas['revision'];atlas['bindings'][str(source.relative_to(ROOT))]=sha(source)
    for z,r in atlas['regions'].items():
        values=[v['residual_m'] for k,v in checks.items() if cfg['stations'][k]['region']==z]
        if len(values)!=8 or max(values)>=.08 or np.sqrt(np.mean(np.square(values)))>=.05:raise ValueError('EXTENSION_REGIONAL_GNSS_GATE '+z)
        xy=np.array([q[:2] for c in original['zones'][z]['plans'] for s in c['segments'] for q in s['poses']])-[28,14]
        r['site_extent_xy']=[(xy.min(0)-1).tolist(),(xy.max(0)+1).tolist()];r['scope']=z+'1–3 route bounding area; eight fresh heldout sensor stations; full task qualification remains pending';r['extension_heldout']=[checks[k] for k in checks if cfg['stations'][k]['region']==z]
    for name in checks:
        p=RESULTS/'qualification/survey'/name/'qualification17.json';atlas['bindings'][str(p.relative_to(ROOT))]=sha(p);atlas['bindings'].update(checks[name]['input_hashes'])
    for p in [PKG/'scripts/extend17.py',PKG/'scripts/qualify17.py',RESULTS/'qualification/stations.json']:atlas['bindings'][str(p.relative_to(ROOT))]=sha(p)
    atlas.pop('revision');key=canonical(atlas);atlas['map_id']=str(uuid.uuid5(uuid.NAMESPACE_URL,'S17-atlas:'+key));atlas['frame_epoch']=str(uuid.uuid5(uuid.NAMESPACE_URL,'S17-epoch:'+key));atlas['revision']=canonical(atlas)
    out=BASE/'maps/s17_regions';dump(out/'manifest.json',atlas)
    p=copy.deepcopy(original);p.pop('plan_revision');p['base_plan_revision']=original['plan_revision'];p['atlas_revision']=atlas['revision'];p['runtime_eligible_clusters']=['A1','A2','A3','B1','B2','B3'];p['eligibility_scope']='localization extension qualified; full missions under S17 test, no production completion claim';p['execution_routes']={}
    p['frame'].update(map_id=atlas['map_id'],map_revision=atlas['revision'],frame_epoch=atlas['frame_epoch'],source='S17 heldout-qualified extension of measured regional atlas')
    p['plan_revision']=canonical(p);dest=BASE/'plans/s17_registered';dump(dest/'mission_plan.json',p);dump(dest/'manifest.json',dict(files={'mission_plan.json':sha(dest/'mission_plan.json')}))
    return atlas,p
if __name__=='__main__':build()
