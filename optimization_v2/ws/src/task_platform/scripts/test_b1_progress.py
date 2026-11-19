import gzip
import json
import math
from pathlib import Path
import unittest

from qualify_b1_progress import ContinuousRetryProgress
from replay_b1_progress import evaluate


DRIVE = {'allowed':True, 'mode':'DRIVE'}
HOLD = {'allowed':False, 'mode':'STOP'}


def apply(progress, observation, gate, gear=1):
    stamp=observation['t']
    progress.observe(observation, dict(gate,t=stamp,armed_direction=gear),stamp)


def segment(direction=1):
    return {'poses':[[i*.1,0.,0.] for i in range(25)] if direction==1 else
                    [[i*.1,0.,math.pi] for i in range(25)],'direction':direction}


def sample(t,x,odom_x=None,y=0.,yaw=0.,epoch=0):
    return dict(t=t, odom_t=t, valid=True, pose=[x,y,yaw],
                odom=[x if odom_x is None else odom_x, y, yaw], localization_epoch=epoch)


class B1ProgressTests(unittest.TestCase):
    def test_forward_reverse_with_sparse_vertices(self):
        for gear in (1,-1):
            progress=ContinuousRetryProgress(segment(gear),0.,.4)
            for i in range(101):
                apply(progress,sample(i*.05,i*.01,yaw=0. if gear==1 else math.pi),DRIVE,gear)
            self.assertAlmostEqual(progress.best,1.)
            self.assertTrue(progress.interrupted(5.))
            self.assertFalse(progress.interrupted(5.1))

    def test_stationary_odom_and_wrong_gear_do_not_rearm(self):
        for multiplier in (0,-1):
            progress=ContinuousRetryProgress(segment(),0.,.4)
            for i in range(101):
                apply(progress,sample(i*.05,i*.01,odom_x=multiplier*i*.01),DRIVE)
            self.assertEqual(progress.best,0)
            self.assertFalse(progress.interrupted(5.1))

    def test_closed_gate_wrong_heading_off_path_or_invalid_pose_do_not_rearm(self):
        for case in ('hold','heading','lateral','stale','invalid'):
            progress=ContinuousRetryProgress(segment(),0.,.4)
            for i in range(101):
                r=sample(i*.05,i*.01,y=.30 if case=='lateral' else 0.,yaw=.5 if case=='heading' else 0.)
                if case=='stale': r['odom_t']-=.25
                if case=='invalid': r['valid']=False
                apply(progress,r,HOLD if case=='hold' else DRIVE)
            self.assertEqual(progress.best,0.,case)

    def test_crossing_shortcut_epoch_and_future_or_duplicate_sample(self):
        path={'poses':[[0,0,0],[.2,0,0],[0,0,math.pi],[1,0,0]],'direction':1}
        progress=ContinuousRetryProgress(path,0.,.4)
        for i in range(51):
            apply(progress,sample(i*.05,min(.19,i*.01)),DRIVE)
        before=progress.best
        self.assertLessEqual(before,.2)
        for r in (sample(2.5,.95),dict(sample(2.55,.95),odom_t=3.),sample(2.6,.95,epoch=1)):
            apply(progress,r,DRIVE)
            self.assertEqual(before,progress.best)

    def test_stale_gate_future_evidence_wrong_direction_and_missing_samples(self):
        for case in ('stale','future','wrong_gear','gap','stale_pose'):
            progress=ContinuousRetryProgress(segment(),0.,.4)
            for i in range(51):
                t=i*(5. if case=='gap' else .1)
                r=sample(t,i*.022)
                gate=dict(DRIVE,armed_direction=-1 if case=='wrong_gear' else 1,
                          t=0. if case=='stale' else t+.1 if case=='future' else t)
                progress.observe(r,gate,t+.3 if case=='stale_pose' else t)
            self.assertFalse(progress.interrupted(8.),case)
            self.assertLess(progress.best,.10,case)

    def test_total_attempt_and_time_bounds_survive(self):
        progress=ContinuousRetryProgress(segment(),0.,.4)
        for i in range(18):
            self.assertFalse(progress.interrupted(i))
        with self.assertRaises(RuntimeError):
            progress.interrupted(18.)
        with self.assertRaises(RuntimeError):
            ContinuousRetryProgress(segment(),0.,.4).interrupted(250.)

    def test_frozen_full_attempt_replay(self):
        root=next(p for p in Path(__file__).resolve().parents if (p/'validation/r10_bounded_20261119/b1/progress_trace.json.gz').is_file())
        folder=root/'validation/r10_bounded_20261119/b1'
        j=evaluate(json.loads(gzip.decompress((folder/'progress_trace.json.gz').read_bytes())))
        self.assertEqual(j,json.loads((folder/'progress_replay.json').read_text()))
        old,new=j['counters'].values()
        self.assertTrue(old[-1]['consecutive_bound_reached'])
        self.assertEqual(old[-1]['path_progress_m'],0.)
        self.assertFalse(new[-1]['consecutive_bound_reached'])
        self.assertGreaterEqual(sum(r['resets_consecutive_counter'] for r in new),2)
        self.assertGreater(new[-1]['path_progress_m'],2*.8)
        self.assertEqual(len(old),len(new))
        self.assertEqual([r['reason'] for r in new],[r['reason'] for r in old])


if __name__=='__main__':
    unittest.main()
