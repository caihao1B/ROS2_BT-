import json
from pathlib import Path
import tempfile
import unittest

from residual_review import review


class ResidualReview(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.run = Path(self.temp.name)
        self.result = {'completed':[{'point':'p1'}], 'selected_points':2}
        self.jobs = [{'key':'frozen:p1','state':'SUCCEEDED','uncertain':False,'confirmed':3.1},
                     {'key':'frozen:p2','state':'CANCELLED','uncertain':True,'confirmed':1.}]
        self.write('mission.json', {'version':'frozen','map_revision':'map',
                   'clusters':[{'id':'A1','points':[{'id':p,'quantity':3.,'target':[i,1.]}
                                                    for i,p in enumerate(['p1','p2','p3'])]}]})
        self.write('spec.json',dict(task='spray',session='fixture',case='nominal',cluster='A1',point_limit=2))
        self.write('runtime.json',dict(passed=True))
        self.write('mission_result.json',self.result)
        self.analysis(self.result)

    def write(self, name, payload):
        (self.run/name).write_text(json.dumps(payload))

    def analysis(self, result):
        self.write('analysis_spray_r10.json',dict(task_result=result, passed=False, jobs=self.jobs))

    def test_partial_service_preserves_the_original_unselected_points(self):
        report=review(self.run)
        self.assertEqual(report['summary']['residual_points'],2)
        self.assertEqual([p['state'] for p in report['obligations']],
                         ['CONFIRMED','CHECK_UNCERTAINTY','NO_RECORDED_SERVICE'])
        self.assertFalse(report['obligations'][2]['selected'])
        self.assertFalse(report['automatic_dispatch'])

    def test_mismatched_evidence_is_not_reported_as_current(self):
        self.analysis(dict(self.result, selected_points=3))
        with self.assertRaisesRegex(ValueError,'different results'):
            review(self.run)
        self.jobs[0]['key']='old:p1'
        self.analysis(self.result)
        with self.assertRaisesRegex(ValueError,'version'):
            review(self.run)


if __name__=='__main__':
    unittest.main()
