import sys
import unittest
from pathlib import Path

scripts = Path(__file__).resolve().parents[1]/'scripts'
sys.path.insert(0,str(scripts))
from recovery_stop_candidate_r10 import RecoveryStopEvidence, source_for_candidate


class RecoveryStopTest(unittest.TestCase):
    def setUp(self):
        self.proof = RecoveryStopEvidence('run', 'tool', .1)

    def sample(self, t, **changes):
        row = dict(t=t, sensor_epoch='run', tool_revision='tool', healthy=True,
                   request_on=False, flow=0, rpm_rad_s=.01)
        self.proof.observe(dict(row, **changes))

    def gate(self, t):
        return dict(t=t, stopped=True, tool_stopped=True)

    def test_four_milliseconds_after_arm_cannot_confirm_stop(self):
        self.sample(975.20)
        self.assertFalse(self.proof.confirmed(975.216,975.220,self.gate(975.220)))
        for t in (975.23,975.28,975.34):
            self.sample(t)
        self.assertTrue(self.proof.confirmed(975.216,975.35,self.gate(975.35)))
        self.assertFalse(self.proof.confirmed(975.216,975.35,self.gate(975.20)))

    def test_gap_wrong_revision_or_motion_interrupt_proof(self):
        for bad in [dict(sensor_epoch='other'),dict(tool_revision='other'),dict(healthy=False),
                    dict(request_on=True),dict(flow=.1),dict(rpm_rad_s=.1)]:
            with self.subTest(bad=bad):
                self.setUp();self.sample(1.01);self.sample(1.06,**bad);self.sample(1.12)
                self.assertFalse(self.proof.confirmed(1.,1.13,self.gate(1.13)))
        self.setUp();self.sample(1.01);self.sample(1.22)
        self.assertFalse(self.proof.confirmed(1.,1.23,self.gate(1.23)))

    def test_actual_worker_specialization_parses(self):
        text=(scripts/'mission_mowing_r10.py').read_text()
        compile(source_for_candidate(text),'candidate_mowing_worker','exec')
