import sys,unittest,tempfile
from pathlib import Path
from types import SimpleNamespace as N
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from inspection_recovery import completed_part,inspection_state,remaining_nominal,ROOT
class Recovery(unittest.TestCase):
 def mission(self):
  return N(inspection_completed_parts=['p'],admission_recovery=N(since=None,exhausted=False,resumes=5),replans=2,last_dispatch_token='old',log=lambda *a,**kw:None)
 def test_only_confirmed_new_progress_resets(self):
  m=self.mission();completed_part(m,{'id':'p'});self.assertEqual((m.replans,m.admission_recovery.resumes),(0,0));self.assertEqual(m.showcase_total_replans,2)
  m.replans=1;completed_part(m,{'id':'p'});self.assertEqual(m.replans,1)
 def test_failed_or_active_recovery_cannot_reset(self):
  m=self.mission()
  with self.assertRaises(ValueError):completed_part(m,{'id':'unknown'})
  m.admission_recovery.since=1
  with self.assertRaises(ValueError):completed_part(m,{'id':'p'})
 def test_measured_checkpoint_has_unfinished_road(self):
  p=ROOT/'optimization_v2/results/S17/showcase_A1_inspection_20261113_204730'
  if not p.exists():self.skipTest('local evidence not bundled')
  s=inspection_state(p);self.assertEqual(len(s['completed_parts']),6);self.assertGreater(remaining_nominal(p),300);self.assertLess(remaining_nominal(p),900)
if __name__=='__main__':unittest.main()
