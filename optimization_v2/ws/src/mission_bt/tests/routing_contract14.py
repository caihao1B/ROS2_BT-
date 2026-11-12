"""Fixed obligations, actual blocked old turn, new route and tamper refusal."""
import sys,copy
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from common14 import *
from binding14 import atlas,canonical,verify_derived,execution_cluster
from routing14 import verify_route,CollisionMap

def reject(fn):
    try:fn()
    except ValueError:return
    raise AssertionError('Expected rejection')

def main():
    original=base_plan();a=atlas();derived=read(BASE/'plans/s14_registered/mission_plan.json');verify_derived(derived,original,a,'B');ref=derived['execution_routes']['B1'];route=read(ROOT/ref['path']);validation=verify_route(route,original,a)
    assert derived['zones']==original['zones'] and route['required_area_m2']==16.35945787959566
    assert validation['coverage_fraction']>=1-1e-8 and validation['timing']['total_s']<=1440
    old=np.array(original['zones']['B']['plans'][0]['segments'][4]['poses']);old[:,:2]-=[28,14]
    assert not CollisionMap(ROOT/route['observation'],a['regions']['B']['site_extent_xy']).valid(old),'Old unsafe transit unexpectedly admitted'
    for mutation in ['denominator','work_removed','curvature','outside','return_removed','lineage']:
        bad=copy.deepcopy(route)
        if mutation=='denominator':bad['required_area_m2']-=1
        if mutation=='work_removed':
            s=next(s for s in bad['segments'] if s['tool_on']);s['kind']='TURN';s['tool_on']=False
        if mutation=='curvature':bad['segments'][0]['poses'][2][2]+=.5
        if mutation=='outside':bad['segments'][0]['poses'][2][0]+=100
        if mutation=='return_removed':bad['segments'].pop()
        if mutation=='lineage':bad['atlas_revision']='old'
        bad.pop('revision');bad['revision']=canonical(bad);reject(lambda:verify_route(bad,original,a))
    os.environ['S14_REGISTERED_REGION']='B'
    try:
        selected=execution_cluster('ackermann');assert selected['segments']==route['segments'];assert selected['coverage_check']==original['zones']['B']['plans'][0]['coverage_check']
    finally:os.environ.pop('S14_REGISTERED_REGION',None)
    print(json.dumps(dict(passed=True,original_denominators_unchanged=True,old_observed_collision_rejected=True,mutations_rejected=6,validation=validation,route_revision=route['revision'])))
if __name__=='__main__':main()
