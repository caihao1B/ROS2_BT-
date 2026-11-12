"""Point32 transport must not spuriously reject the original B1 work polygon."""
import sys,tempfile,argparse
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from mission14 import Mission
from common14 import *
from shapely.geometry import shape,Polygon
from shapely.affinity import translate
sys.path.insert(0,str(BASE/'ws/src/tool_sim/scripts'))
from common10 import contract
from ledger_core import Ledger
test=Mission.__new__(Mission);test.a=argparse.Namespace(vehicle='ackermann');test.c=contract('ackermann');test.c.update({k:'test' for k in ['map_id','map_revision','frame_epoch','plan_revision','tool_revision','session']});p=cluster('ackermann');test.area=translate(shape(p['coverage_check']['covered']),xoff=-28,yoff=-14)
g=test.tool_goal(dict(task_id='B1',id='B1'))
d={k:getattr(g,k) for k in ['operation','task_id','idempotency_key','map_id','map_revision','frame_epoch','plan_revision','tool_revision','segment_id','requested_quantity','timeout_sim_s']};d.update(frame='map',target=[0,0,0],area=[[p.x,p.y] for p in g.target_area.points])
with tempfile.TemporaryDirectory() as td:
    ledger=Ledger(Path(td)/'ledger.sqlite',test.c);r=ledger.reserve(d,'attempt',0.);assert r['ok'],r;assert test.area.area==p['coverage_check']['domain_area_m2'];assert abs(test.area.area-g.requested_quantity)<1e-3;ledger.close()
print(json.dumps(dict(original_denominator_m2=test.area.area,transport_request_m2=g.requested_quantity,ledger_admitted=True,completion_credit=0)))
