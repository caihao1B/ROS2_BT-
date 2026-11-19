"""Regression for the late A1 yaw drift, including rejection of unobservable stops."""
import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from stationary_gyro_r10 import StationaryYawBias


def stopped(stamp):
    return dict(t=stamp, mode="STOP", v=0., w=0., nav_idle=True, stopped=True)


class StationaryCalibrationTest(unittest.TestCase):
    def test_estimates_sensor_difference_and_keeps_real_rotation(self):
        tracker = StationaryYawBias()
        for i in range(1801):
            t = i * .01
            tracker.observe_gate(stopped(t))
            tracker.observe_wheel(t, 0., .003)
            tracker.update(t, -.004)
        self.assertAlmostEqual(tracker.offset, -.007, delta=.00015)
        self.assertAlmostEqual(-.004 - tracker.offset, .003, delta=.00015)

    def test_motion_sparse_data_and_large_fault_cannot_calibrate(self):
        for condition in ("drive", "locally_still_only", "turn", "rolling", "stale", "future", "fault", "slip"):
            with self.subTest(condition=condition):
                tracker = StationaryYawBias()
                for i in range(900):
                    t = i * .01
                    stamp = t + 10. if condition == "future" else t - .3 if condition == "stale" else t
                    gate = stopped(stamp)
                    if condition == "drive":
                        gate["mode"] = "DRIVE"
                    if condition == "locally_still_only":
                        gate["stopped"] = False
                    tracker.observe_gate(gate)
                    tracker.observe_wheel(stamp, .1 if condition == "rolling" else 0.,
                                          .08 if condition == "turn" else 0.)
                    tracker.update(t, -.2 if condition == "fault" else -.007, condition == "slip")
                self.assertEqual(tracker.updates, 0)
                self.assertEqual(tracker.offset, 0.)

    def test_only_fresh_evaluated_wheel_conflicts_veto(self):
        checked = dict(t=12., independent_motion=dict(available=True, wheel_valid=False, wheel_nis=10.))
        self.assertTrue(StationaryYawBias.wheel_disputed(12.2, checked))
        self.assertFalse(StationaryYawBias.wheel_disputed(12.7, checked))
        self.assertFalse(StationaryYawBias.wheel_disputed(11.9, checked))
        del checked['independent_motion']['wheel_nis']
        self.assertFalse(StationaryYawBias.wheel_disputed(12.2, checked))

    def test_gap_or_bad_samples_restart_stationary_dwell(self):
        tracker = StationaryYawBias()
        for i in range(500):
            t = i*.01
            if .4 < t < 1.: continue
            tracker.observe_gate(stopped(t))
            tracker.observe_wheel(t, 0., 0.)
            tracker.update(t, -.006)
            if t < 1.60:
                self.assertEqual(tracker.updates, 0)
        previous = tracker.offset
        self.assertEqual(tracker.update(0., math.nan), previous)
        self.assertEqual(tracker.update(5., math.inf), previous)


if __name__ == "__main__":
    unittest.main()
