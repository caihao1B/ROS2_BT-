"""Exercise production feedback accounting and cross-cluster residual retention."""
import sys,json
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'ws/src/mission_bt/scripts'))
from accounting14 import confirmed_points,FrozenSchedule
jobs=[dict(id='p1',kind='POINT_SERVICE'),dict(id='p2',kind='POINT_SERVICE')]
results=[dict(key='v:p1',state='SUCCEEDED',uncertain=False,quantity=2.1),dict(key='v:p1',state='SUCCEEDED',uncertain=False,quantity=2.1),dict(key='v:p2',state='SUCCEEDED',uncertain=True,quantity=2.0)]
assert confirmed_points(jobs,results,'v')==dict(required=2,confirmed=1,remaining=['v:p2'])
schedule=FrozenSchedule({'A1':['p1','p2'],'A2':['p3']});schedule.confirm('p1',results[0]);schedule.defer('A1');schedule.confirm('p3',dict(state='SUCCEEDED',uncertain=False))
s=schedule.snapshot();assert s['A1']['remaining']==['p2'] and s['A1']['required']==2 and s['A2']['confirmed']==1
print(json.dumps(s,ensure_ascii=False,indent=2));print('ACCOUNTING_AND_DEFERRAL_PASS')
