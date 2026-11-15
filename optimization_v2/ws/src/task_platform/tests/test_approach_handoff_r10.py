import sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from approach_handoff_r10 import ApproachHandoff

POLICY=dict(current_radius_m=.10,visited_radius_m=.06,navigation_goal_m=.025,window_s=2.,stall_s=1.5,min_gap_s=0.,max_gap_s=.15,minimum_improvement_m=.005,translation_command_s=.5,minimum_samples=20)
class Handoff(unittest.TestCase):
    def feed(self,fn,velocity=.03):
        h=ApproachHandoff(POLICY);return [h.update(i*.03,[fn(i),0.],[0.,0.],True,velocity) for i in range(85)]
    def test_stalled_close_motion_only_proposes_stop(self):
        d=next(x for x in self.feed(lambda i:.045+(.001 if i%2 else 0)) if x)
        self.assertFalse(d['navigation_success']);self.assertFalse(d['work_admitted'])
    def test_normal_improvement_does_not_interrupt_navigation(self):
        self.assertFalse(any(self.feed(lambda i:.099-i*.0008)))
    def test_far_and_never_visited_close_are_not_handed_off(self):
        for d in [.11,.08,.024]:self.assertFalse(any(self.feed(lambda i:d)))
    def test_rotation_only_is_not_a_translation_stall(self):self.assertFalse(any(self.feed(lambda i:.04,velocity=0.)))
    def test_bad_or_discontinuous_observations_reset_evidence(self):
        for broken_t,qualified in [(1.5,False),(2.5,True),(float('nan'),True)]:
            h=ApproachHandoff(POLICY)
            for i in range(45):self.assertIsNone(h.update(i*.03,[.04,0.],[0.,0.],True,.03))
            self.assertIsNone(h.update(broken_t,[.04,0.],[0.,0.],qualified,.03))
            self.assertLess(len(h.rows),3)

if __name__=='__main__':unittest.main()
