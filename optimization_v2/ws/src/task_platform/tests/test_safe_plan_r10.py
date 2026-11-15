import unittest,copy,sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from common import read,edition
from safe_plan_r10 import validate,load_checked,requested_target,work_identity

class SafePlanContract(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.original=read(edition('R09')/'mission.json')
        cls.plan=read(edition('R10')/'mowing_safe_plan.json')
    def test_actual_plan_curvature_envelope_and_original_area(self):
        self.assertTrue(validate(load_checked(),self.original)['passed'])
    def test_cannot_erase_a_work_obligation(self):
        p=copy.deepcopy(self.plan);p['cluster']['original_work_segments'].pop()
        with self.assertRaisesRegex(ValueError,'ORIGINAL_WORK'):validate(p,self.original)
    def test_cannot_reduce_original_denominator(self):
        p=copy.deepcopy(self.plan);p['cluster']['work_area_m2']-=1
        with self.assertRaisesRegex(ValueError,'ORIGINAL_CLUSTER'):validate(p,self.original)
    def test_cannot_extend_original_time_budget(self):
        p=copy.deepcopy(self.plan);p['cluster']['budget_s']+=1
        with self.assertRaisesRegex(ValueError,'ORIGINAL_TIME'):validate(p,self.original)
    def test_fragments_share_original_job_and_target(self):
        fragments=[s for s in self.plan['cluster']['segments'] if s.get('work_key')=='B1:segment:16']
        self.assertGreater(len(fragments),1)
        self.assertEqual(work_identity(fragments[0]),work_identity(fragments[1]))
        self.assertEqual(fragments[0]['target_area'],fragments[1]['target_area'])
    def test_full_denominator_includes_wholly_unexecuted_strip(self):
        cluster=copy.deepcopy(self.plan['cluster']);selected=[s for s in cluster['segments'] if s.get('work_key')!='B1:segment:16']
        full=requested_target(cluster,selected,True);partial=requested_target(cluster,selected,False)
        self.assertGreater(full.area,partial.area)
        self.assertAlmostEqual(full.area,self.plan['original_work_target_union_m2'])
    def test_reference_has_not_been_replaced(self):
        self.assertEqual(len(self.original['clusters'][0]['segments']),34)
        self.assertEqual(self.plan['original_reference_status'],'PARTIAL')

if __name__=='__main__':unittest.main()
