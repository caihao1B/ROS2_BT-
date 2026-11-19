"""Deterministic regressions for the fresh stopped-observation and capture policy."""
import unittest
from spray_precision_r10 import LowSpeedCapture,StoppedWindow


def observe(window,now,stamp=None,stopped=True,epoch=0,xy=(0.,0.),valid=True):
    loc=dict(valid=valid,pose=[*xy,0.],odom_t=now if stamp is None else stamp,t=now,localization_epoch=epoch)
    return window.update(now,loc,dict(t=now,stopped=stopped,tool_stopped=True))


class PrecisionTest(unittest.TestCase):
    def test_settle_overlaps_full_stationary_evidence(self):
        w=StoppedWindow([0.,0.],1.5)
        for i in range(46):
            now=i*.04
            ok,metric=observe(w,now)
            if now<1.8:
                self.assertFalse(ok)
        self.assertTrue(ok)
        self.assertGreaterEqual(metric['samples'],20)

    def test_no_moving_samples_or_republished_time(self):
        w=StoppedWindow([0.,0.],1.5)
        for i in range(60):
            self.assertFalse(observe(w,i*.04,stopped=False)[0])
        self.assertFalse(observe(w,2.4,stamp=2.31)[0])
        self.assertEqual(w.history,[])
        for i in range(1,55):
            self.assertFalse(observe(w,2.4+i*.04,stamp=2.4)[0])

    def test_epoch_gap_and_single_bad_sample_reset(self):
        for reset in ('epoch','gap','moving','invalid'):
            w=StoppedWindow([0.,0.],1.5)
            for i in range(42):
                self.assertFalse(observe(w,i*.04)[0])
            t=2. if reset=='gap' else 1.68
            self.assertFalse(observe(w,t,stopped=reset!='moving',valid=reset!='invalid',epoch=int(reset=='epoch'))[0])
            self.assertLessEqual(len(w.history),1)

    def test_distant_stable_offset_and_scatter_cannot_admit(self):
        for off in [lambda i:.05,lambda i:(.025 if i%2 else -.025)]:
            w=StoppedWindow([0.,0.],1.5)
            for i in range(65):
                self.assertFalse(observe(w,i*.04,xy=(off(i),0))[0])

    def test_capture_requires_slow_multiple_samples(self):
        c=LowSpeedCapture(.025)
        def advance(i,v,xy):
            t=i*.035
            return c.update(t,[0.,0.],dict(t=t,odom_t=t,pose=[xy,0.,0.],localization_epoch=0,v=v),{'t':t},True)
        for i in range(15):
            self.assertIsNone(advance(i,.11,.005))
        for i in range(15,30):
            self.assertIsNone(advance(i,.05,.03))
        for i in range(30,34):
            self.assertIsNone(advance(i,.05,.022))
        r=advance(34,.05,.020)
        self.assertFalse(r['work_admitted'])
        self.assertFalse(r['navigation_success'])


if __name__=='__main__':
    unittest.main()
