"""Exercise real preflight API with wrong map/epoch/calibration/plan and payloads."""
import sys,tempfile,copy,shutil,json
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from s07_common import *
def run(out):
    out=Path(out);out.mkdir(parents=True,exist_ok=True);m=verify_bundle();request={k:m[k] for k in ['map_id','frame_epoch','revision','calibration_revision']};request['purpose']='DIAGNOSTIC_MOTION_FIXTURE';cases=[]
    def reject(name,fn,expected):
        try:fn();cases.append(dict(name=name,passed=False,reason='UNEXPECTED_ADMISSION',actuator_started=False))
        except ValueError as exc:cases.append(dict(name=name,passed=expected in str(exc),reason=str(exc),actuator_started=False))
    verify_bundle(request=request);cases.append(dict(name='correct_binding',passed=True,reason='VERIFIED',actuator_started=False))
    for key in ['map_id','frame_epoch','revision','calibration_revision']:
        bad=dict(request);bad[key]='OLD_OR_WRONG';reject('wrong_'+key,lambda bad=bad:verify_bundle(request=bad),'REQUEST_BINDING_MISMATCH: '+key)
    bad=dict(request);bad['purpose']='WORK_PLAN';reject('real_plan_not_yet_supported',lambda:verify_bundle(request=bad),'REAL_WORK_PLAN_NOT_IMPLEMENTED')
    # Copies only, never alter a protected map or its inode metadata.
    # Missing/changed payload is rejected before ROS starts.
    with tempfile.TemporaryDirectory(prefix='s07_contract_') as tmp:
        tmp=Path(tmp)
        for item in m['files']:shutil.copyfile(BASE/'maps/frozen'/item['path'],tmp/item['path'])
        def manifest(value):dump(tmp/'manifest.json',value)
        manifest(m);verify_bundle(tmp)
        changed=copy.deepcopy(m);changed['origin']['survey_reference'][0]+=1;manifest(changed);reject('manifest_tamper',lambda:verify_bundle(tmp),'MANIFEST_REVISION_MISMATCH')
        changed=copy.deepcopy(m);changed['calibration_revision']='wrong';changed['revision']=canonical({k:v for k,v in changed.items() if k!='revision'});manifest(changed);reject('map_calibration_mismatch',lambda:verify_bundle(tmp),'CALIBRATION_MISMATCH')
        manifest(m);target=tmp/'navigation.yaml';target.unlink();target.write_text('corrupt payload\n');reject('payload_tamper',lambda:verify_bundle(tmp),'MAP_CONTENT_CHANGED')
        target.unlink();reject('missing_payload',lambda:verify_bundle(tmp),'MISSING_OR_ESCAPED_FILE')
        reject('path_traversal',lambda:safe(tmp,'../map.db'),'UNSAFE_PATH')
    result=dict(passed=all(x['passed'] for x in cases),cases=cases,simulation_clock='N/A',contract_type='DIAGNOSTIC_MOTION_FIXTURE; real plan end-to-end test reserved for S12',map_id=m['map_id']);dump(out/'contracts.json',result);return result
if __name__=='__main__':
    r=run(sys.argv[1]);print(json.dumps(r,ensure_ascii=False));raise SystemExit(0 if r['passed'] else 1)
