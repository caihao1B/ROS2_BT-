"""Replay actual B1 boundary failure; preserve zero-area and feedback semantics."""
import sys,json,tempfile,copy
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from common14 import BASE
from ledger14 import Ledger
from ledger_geometry14 import area_union
from shapely.geometry import Polygon,GeometryCollection,LineString,Point,box,shape
from shapely import union,set_precision
from shapely.errors import GEOSException
sys.path.insert(0,str(BASE/'ws/src/work_ledger/scripts'))
from ledger_core import Ledger as OriginalLedger

fixture=json.loads((Path(__file__).parent/'fixtures/b1_boundary_observations.json').read_text())
def replay(cls,path,stale=False):
    ledger=cls(path,fixture['cfg']);assert ledger.reserve(fixture['goal'],fixture['attempt'],103.)['ok']
    results=[]
    try:
        for sample in fixture['samples']:
            results.append(ledger.observe(sample['feedback'],sample['context'],sample['now']+(2 if stale else 0)))
        snap=ledger.snapshot(fixture['goal']['idempotency_key']);return results,snap
    finally:ledger.close()

with tempfile.TemporaryDirectory() as td:
    root=Path(td)
    try:replay(OriginalLedger,root/'original.sqlite')
    except GEOSException as e:assert 'mixed-dimension' in str(e)
    else:raise AssertionError('Recorded production failure no longer reproduced')
    results,snap=replay(Ledger,root/'fixed.sqlite')
    assert snap['state']=='RUNNING' and snap['confirmed_quantity']>=0
    assert shape(snap['credit_geojson']).geom_type in ['Polygon','MultiPolygon']
    assert any(r['accepted'] for r in results)
    _,stale=replay(Ledger,root/'stale.sqlite',stale=True)
    assert stale['confirmed_quantity']==0

line=LineString([(0,0),(2,2)]);mixed=GeometryCollection([box(0,0,1,1),line,Point(4,4)])
assert area_union(Polygon(),line,.001).is_empty
assert area_union(mixed,box(.5,.5,1.5,1.5),.001).equals(union(box(0,0,1,1),box(.5,.5,1.5,1.5),grid_size=.001))
a=box(.0013,.0016,1.002,1.004);b=box(.502,.503,2.002,2.003)
assert area_union(a,b,.001).equals(union(a,set_precision(b,.001),grid_size=.001))
try:area_union(Polygon([(0,0),(1,1),(1,0),(0,1)]),b,.001)
except ValueError:pass
else:raise AssertionError('Invalid polygon must fail closed')
print(json.dumps(dict(recorded_failure_reproduced=True,replayed_feedback_samples=len(results),confirmed_area_m2=snap['confirmed_quantity'],zero_area_contacts_no_credit=True,stale_feedback_no_credit=True,ordinary_polygon_overlay_unchanged=True)))
