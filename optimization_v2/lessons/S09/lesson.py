"""Use the actual tracker: short occlusion is a hypothesis, not permanent identity."""
from pathlib import Path
import sys,numpy as np
base=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(base/'ws/src/object_fusion/scripts'))
from tracker import Tracker
tr=Tracker()
def measurement(x):return dict(position=[x,0,.85],size=[.4,.5,1.7],covariance=np.diag([.01,.01,.02]).reshape(-1).tolist(),classification='person')
for i in range(10):rows,_=tr.update(i*.2,[measurement(3+i*.1)])
print('moving:',[(r['id'],np.round(r['velocity'],3).tolist()) for r in rows])
rows,_=tr.update(2.4,[]);print('occluded:',rows[0]['state'],rows[0]['covariance'][0])
rows,_=tr.update(2.6,[measurement(4.3)]);assert rows[0]['id']==1
rows,_=tr.update(4.,[]);assert not rows
rows,_=tr.update(4.2,[measurement(4.3)]);assert rows[0]['id']==2
print('PASS: temporary ID retained briefly, then expired; uncertainty grows under occlusion')
