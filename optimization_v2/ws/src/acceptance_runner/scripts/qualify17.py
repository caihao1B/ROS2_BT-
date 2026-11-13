"""Validate frozen regional surfaces on newly observed heldout stations.

GNSS is used only to score a scan match. Neither placement nor GNSS initializes
the matcher; it receives the same deliberately coarse hint as an operator.
"""
from common17 import *
import importlib.util,numpy as np

def run(names=None):
    add14()
    from regional_math14 import SurfaceMap
    sys.path.insert(0,str(BASE/'ws/src/map_manager/scripts'))
    from mapping_analysis import ply
    path=BASE/'ws/src/mission_bt/tests/build_atlas14.py'
    spec=importlib.util.spec_from_file_location('atlas14_qualification',path);mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod)
    out=RESULTS/'qualification';mod.SURVEY=out/'survey';cfg=read(out/'stations.json');atlas=read(BASE/'maps/s14_regions/manifest.json');summary={};refs={}
    for name,s in cfg['stations'].items():
        if names and name not in names:continue
        p=mod.SURVEY/name
        if not (p/'runtime.json').exists():continue
        try:
            runtime=read(p/'runtime.json')
            if not runtime.get('success') or not runtime.get('database_unchanged'):raise ValueError('STATION_CAPTURE_UNSUCCESSFUL')
            contacts=read(p/'evaluation/contacts.json')
            collisions={k:v for k,v in contacts.items() if v and not k.endswith('|ground::body::ground')}
            if collisions:raise ValueError('SURVEY_OBSTACLE_CONTACT '+json.dumps(collisions))
            z=s['region']
            if z not in refs:
                refs[z]=SurfaceMap(ply(ROOT/atlas['regions'][z]['cloud_path']));refs[z].prepare_normals()
            station=mod.station(name,s);v=mod.match(station,refs[z],station['hint'])
            v.update(passed=v['residual_m']<.08,role='heldout',reference_cloud_sha256=sha(ROOT/atlas['regions'][z]['cloud_path']),input_hashes={str(x.relative_to(ROOT)):sha(x) for x in [p/'runtime.json',p/'evaluation/contacts.json',p/'scene/world.sdf',p/'sensor_diagnostic/gnss.jsonl',p/'sensor_diagnostic/quality.jsonl',*sorted((p/'sensor_diagnostic').glob('cloud_*.npz'))]})
        except Exception as e:v=dict(passed=False,error=str(e))
        dump(p/'qualification17.json',v);summary[name]={k:v[k] for k in ['passed','residual_m','temporal_scatter_m','error'] if k in v};print(name,json.dumps(summary[name]),flush=True)
    dump(out/'surface_qualification.json',dict(stations=summary,required=len(cfg['stations']),executed=len(summary),passed=len(summary)==len(cfg['stations']) and all(x['passed'] for x in summary.values()),scope='Station localization only, not cluster execution or corridor qualification'))
    return summary
if __name__=='__main__':run(sys.argv[1:] or None)
