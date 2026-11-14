import unittest,sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from common import *
from dynamics_r04 import phases
class Dynamics(unittest.TestCase):
 def test_01_geometry_preserved(self):
  for t in TASKS:
   a=read(edition('R02')/'vehicles'/f'{t}.json');b=read(edition('R04')/'vehicles'/f'{t}.json')
   for k in ['body','wheel','wheelbase','steering','footprint','safety','limits']:self.assertEqual(a[k],b[k])
 def test_02_calibration_and_version(self):
  for t in TASKS:
   c=read(edition('R04')/'vehicles'/f'{t}.json');self.assertEqual(c['config_hash'],canonical({k:v for k,v in c.items() if k!='config_hash'}));self.assertEqual(c['calibration']['drive_track_m'],c['calibration']['effective_track_m'])
 def test_03_loss_reject_rearm(self):
  for t in TASKS:
   c=read(edition('R04')/'vehicles'/f'{t}.json');p=phases(c);loss=next(i for i,s in enumerate(p) if s['name']=='command_loss');self.assertFalse(p[loss]['send']);self.assertEqual(p[loss+1]['name'],'latched_reject');self.assertEqual(p[loss+2]['mode'],'ARM')
 def test_04_plan_speeds_supported(self):
  q=read(edition('R04')/'qualification.json');self.assertTrue(q['passed'])
  for t,c in q['vehicles'].items():self.assertLessEqual(c['selected_work_mps'],.6);self.assertLessEqual(c['reverse_mps'],.3);self.assertFalse(c['full_navigation_qualified'])
if __name__=='__main__':unittest.main()
