import sys,unittest
from pathlib import Path
scripts=Path(__file__).resolve().parents[1]/'scripts';sys.path.insert(0,str(scripts))
from common17 import BASE
sys.path.insert(0,str(BASE/'ws/src/mission_bt/scripts'))
from inspection_budget17 import estimate

class Budget(unittest.TestCase):
    def setUp(self):
        self.a=[[0,0,0],[8,0,0]];self.b=[[8,0,0],[16,0,0]];self.c=[[16,0,0],[24,0,0]]
        self.job=dict(id='road',kind='INSPECTION',planned_s=220.,poses=[[0,0,0],[24,0,0]],reference_parts=[dict(id='p'+str(i),poses=p) for i,p in enumerate([self.a,self.b,self.c])])
    def test_elapsed_time_is_not_progress(self):
        a=estimate(self.job,[0,0],0,True,None);b=estimate(self.job,[0,0],400,True,None);self.assertEqual(a['seconds'],b['seconds']);self.assertAlmostEqual(a['seconds'],220.)
    def test_confirmed_parts_remove_only_completed_work(self):
        r=estimate(self.job,[16,0],240,True,None,['p0','p1']);self.assertAlmostEqual(r['seconds'],8/.12+20);self.assertEqual(r['pending_parts'],1)
    def test_live_detour_path_is_used_for_current_part(self):
        path=dict(frame='map',action_id='p2_resume_1',received_t=240,poses=[[20,0,0],[24,0,0]])
        r=estimate(self.job,[20,0],240,True,path,['p0','p1']);self.assertAlmostEqual(r['seconds'],4/.12+20)
    def test_other_part_plan_is_not_current_progress(self):
        path=dict(frame='map',action_id='p0',received_t=240,poses=[[20,0,0],[24,0,0]])
        r=estimate(self.job,[20,0],240,True,path,['p0','p1']);self.assertAlmostEqual(r['seconds'],8/.12+20)
    def test_all_confirmed_is_zero(self):self.assertEqual(estimate(self.job,[24,0],260,False,None,['p0','p1','p2'])['seconds'],0)
if __name__=='__main__':unittest.main()
