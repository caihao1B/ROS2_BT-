"""Replay the real premature budget stop; pose/path progress must drive ETA."""
import sys,copy
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from common14 import *
from budget14 import estimate
f=read(PKG/'tests/fixtures/budget_progress.json')
for name,digest in f['sources'].items():assert sha(ROOT/name)==digest
e=estimate(f['job'],f['pose'],f['t'],True,f['nav_plan'])
assert e['source']=='REMAINING_NAV2_PATH' and 30<e['seconds']<f['remaining_s']-8,e
# A clock advance alone does not reduce the estimated remaining motion.
same=estimate(f['job'],f['pose'],f['t']+.1,True,f['nav_plan']);assert same['seconds']==e['seconds']
for change in ['future','wrong_job','wrong_frame','wrong_goal','wrong_heading']:
    n=copy.deepcopy(f['nav_plan'])
    if change=='future':n['received_t']=f['t']+1
    if change=='wrong_job':n['action_id']='different'
    if change=='wrong_frame':n['frame']='odom'
    if change=='wrong_goal':n['poses'][-1][0]+=1
    if change=='wrong_heading':n['poses'][-1][2]+=.5
    a=estimate(f['job'],f['pose'],f['t'],True,n)
    assert a['source']=='INITIAL_VALIDATED_SEGMENT' and a['seconds']==f['job']['planned_s']
before=estimate(f['job'],f['pose'],f['t'],False,f['nav_plan']);assert before['seconds']==e['seconds']
old=copy.deepcopy(f['nav_plan']);old['received_t']-=10
held=estimate(f['job'],f['pose'],f['t'],True,old);assert held['source']=='LAST_NAV2_GEOMETRY_FOR_ETA' and held['seconds']==e['seconds'] and held['purpose']=='ETA_ONLY_NOT_MOTION_PERMISSION'
h=read(PKG/'tests/fixtures/budget_action_handover.json')
for name,digest in h['sources'].items():assert sha(ROOT/name)==digest
handover=estimate(h['job'],h['pose'],h['t'],True,h['nav_plan']);assert handover['source']=='LAST_NAV2_GEOMETRY_FOR_ETA' and 0<handover['seconds']<h['remaining_s']-8
j=dict(id='work',kind='COVERAGE',poses=[[0,0,0],[1,0,0],[2,0,0]],planned_s=20.)
a=estimate(j,[.2,0],2,True);b=estimate(j,[1.2,0],10,True);assert a['seconds']>b['seconds']
assert estimate(j,[1,2],10,True)['seconds']==20.
print(json.dumps(dict(passed=True,real_premature_stop_replayed=True,previous_constant_s=150.,corrected=e,action_handover=handover,original_stop_reserve_s=8.,future_wrong_job_frame_goal_rejected=True)))
