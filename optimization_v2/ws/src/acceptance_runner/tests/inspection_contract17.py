"""Regression: interrupted long FollowPath must not restart its pruned prefix."""
import sys,tempfile,unittest
from types import SimpleNamespace
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from common17 import *
from inspection17 import execute,route
from unittest.mock import patch

class Inspection(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory(dir=RESULTS,prefix='inspection_contract_');self.out=Path(self.tmp.name);self.calls=[]
        self.m=SimpleNamespace(a=SimpleNamespace(out=self.out,vehicle='diff',controller='DWB'),st=dict(t=0.),working=False,work_permission=lambda:False,c=dict(plan_revision='fixed'),log=lambda *a,**k:None,api=dict(revalidate=lambda:None,follow=self.follow))
        self.poses=[[0.,0.,0.],[1.,0.,0.],[2.,0.,0.],[3.,0.,0.]];self.job=dict(reference_parts=[dict(id='road_reference_0',poses=self.poses)])
    def follow(self,label,poses,nav):self.calls.append((label,poses,nav));return dict(status=4)
    def tearDown(self):self.tmp.cleanup()
    def test_dynamic_probe_keeps_both_chassis_route_direction(self):
        with patch.dict(os.environ,{'S17_BENCHMARK':'dynamic_prefix'}):
            forward=route('diff');reverse=route('ackermann')
        self.assertGreater(forward['end'][0],forward['start'][0]);self.assertLess(reverse['end'][0],reverse['start'][0])
        self.assertGreater(forward['benchmark_length_m'],20);self.assertGreater(reverse['benchmark_length_m'],20)
        self.assertAlmostEqual(forward['benchmark_length_m'],reverse['benchmark_length_m'],places=5)
    def test_only_original_suffix_is_submitted_after_interruption(self):
        dump(self.out/'progress.json',dict(id='road_reference_0_resume_1',remaining_poses=self.poses[2:]))
        execute(self.m,self.job,False);self.assertEqual(self.calls[0][1],self.poses[2:]);self.assertTrue(self.calls[0][2])
    def test_completed_part_is_not_dispatched_twice(self):
        execute(self.m,self.job,False);execute(self.m,self.job,False);self.assertEqual(len(self.calls),1)
    def test_modified_geometry_is_rejected(self):
        dump(self.out/'progress.json',dict(id='road_reference_0',remaining_poses=[[2.,.1,0.],[3.,0.,0.]]))
        with self.assertRaisesRegex(ValueError,'NOT_ORIGINAL_ROUTE_SUFFIX'):execute(self.m,self.job,False)
        self.assertEqual(self.calls,[])
    def test_unrelated_checkpoint_does_not_skip_route(self):
        dump(self.out/'progress.json',dict(id='another_road',remaining_poses=self.poses[2:]))
        execute(self.m,self.job,False);self.assertEqual(self.calls[0][1],self.poses)
if __name__=='__main__':unittest.main()
