"""A completed/failed dispatch cannot consume the same stale BT command again."""
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from mission14 import Mission

m=Mission.__new__(Mission);m.done=[];m.replans=m.deferrals=m.patches=0;m.last_dispatch_token=None;m.blocked=False
d=dict(id=7,evidence=dict(obligation='first',completed_jobs=0,replans=0,deferrals=0,blocked=False))
assert m.claim_directive(d,dict(id='first'))
assert not m.claim_directive(d,dict(id='first')) # failure leaves counters unchanged
assert not m.claim_directive(d,dict(id='second')) # old blackboard cannot start next job
m.done.append('first');new=dict(id=7,evidence=dict(obligation='second',completed_jobs=1,replans=0,deferrals=0,blocked=False))
assert m.claim_directive(new,dict(id='second')) # same BT action, next acknowledged job
assert not m.claim_directive(new,dict(id='second'))
new['id']=9
assert m.claim_directive(new,dict(id='second')) # WAIT -> REPLAN authorizes a new attempt
m.replans=1
assert not m.claim_directive(new,dict(id='second'))
print('STALE_DIRECTIVE_REPLAY_REJECTED; NEXT_ACKNOWLEDGED_OBLIGATION_ALLOWED')

new['evidence']['replans']=1
m.blocked=True
assert not m.claim_directive(new,dict(id='second')) # old running REPLAN is not a fresh failure decision
new['evidence']['blocked']=True
assert m.claim_directive(new,dict(id='second'))
print('STALE_PRE_FAILURE_REPLAN_REJECTED')
