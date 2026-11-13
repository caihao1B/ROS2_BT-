"""A successful action label cannot substitute for the travelled route."""
import json,sys,tempfile,unittest
from pathlib import Path
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from inspection_audit17 import audit

class AuditContracts(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.base=Path(self.tmp.name);(self.base/'maps/prior').mkdir(parents=True);(self.base/'config').mkdir()
        polygon=dict(type='Polygon',coordinates=[[[-1,-1],[5,-1],[5,1],[-1,1],[-1,-1]]]);features=dict(type='FeatureCollection',features=[dict(type='Feature',geometry=polygon,properties={})])
        for f in ['maps/prior/workspaces.geojson','maps/prior/connector.geojson','config/perception_test_area.geojson']:(self.base/f).write_text(json.dumps(features))
        (self.base/'config/traversability.yaml').write_text('profiles:\n  diff:\n    footprint: [[-0.5,-0.4],[0.55,-0.4],[0.55,0.4],[-0.5,0.4]]\n')
        self.contract=dict(original_route=dict(segments=[dict(kind='CONNECTOR',poses=[[0,0,0],[2,0,0]]),dict(kind='CONNECTOR',poses=[[2,0,0],[4,0,0]])]))
    def tearDown(self):self.tmp.cleanup()
    def run_audit(self,positions):
        rows=[dict(t=i,xyz=[x,y,0],rpy=[0,0,0]) for i,(x,y) in enumerate(positions)]
        with patch('inspection_audit17.BASE',self.base):return audit(self.contract,rows,0.,'diff')
    def test_complete_ordered_route(self):self.assertTrue(self.run_audit([(0,0),(2,0),(4,0)])['passed'])
    def test_skipped_real_endpoint_rejected(self):self.assertFalse(self.run_audit([(0,0),(4,0)])['passed'])
    def test_wrong_order_rejected(self):self.assertFalse(self.run_audit([(0,0),(4,0),(2,0)])['passed'])
    def test_centre_inside_but_footprint_outside_rejected(self):
        r=self.run_audit([(0,0),(2,.8),(2,0),(4,0)]);self.assertFalse(r['passed']);self.assertEqual(r['outside_samples'],1)
if __name__=='__main__':unittest.main()
