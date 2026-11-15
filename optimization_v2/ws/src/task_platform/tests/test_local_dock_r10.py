import math
import sys
import unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from local_dock_r10 import propose,wrap,offset_observable,ready_for_stopped_observation


class LocalDockPath(unittest.TestCase):
    def history(self,xy):
        return [(i*.03,xy) for i in range(75)]

    def test_global_translation_becomes_fixed_local_displacement(self):
        p=propose(self.history([10.,20.]),[10.1,20.],[10.,20.,math.pi/2],[2.,3.,0.])
        self.assertAlmostEqual(p['odom_path'][-1][0],2.)
        self.assertAlmostEqual(p['odom_path'][-1][1],2.9)
        self.assertEqual(p['odom_path'][0],[2.,3.,0.])
        self.assertAlmostEqual(p['correction_m'],.1)

    def test_rotation_sweep_is_dense_and_shortest_at_wrap(self):
        h=math.pi-.01;p=propose(self.history([0.,0.]),[-.1,-.001],[0.,0.,h],[4.,5.,h])
        self.assertLess(abs(wrap(p['map_path'][1][2]-h)),.04)
        self.assertTrue(all(math.dist(a[:2],b[:2])<=.02000001 for a,b in zip(p['map_path'],p['map_path'][1:])))

    def test_refuses_large_correction_or_moving_reference(self):
        for target,current in [([.151,0.],[0.,0.,0.]),([.1,0.],[.031,0.,0.]),([0.,0.],[0.,0.,0.])]:
            with self.assertRaises(ValueError):propose(self.history([0.,0.]),target,current,[0.,0.,0.])

    def test_no_observation_does_not_create_a_motion(self):
        with self.assertRaises(ValueError):propose([], [.1,0.], [0.,0.,0.], [0.,0.,0.])

    def test_local_proximity_only_requests_a_real_stop(self):
        samples=[(i*.03,[.009,0.,0.]) for i in range(11)]
        self.assertTrue(ready_for_stopped_observation(samples,[0.,0.,1.],.30))
        self.assertFalse(ready_for_stopped_observation(samples,[0.,0.,1.],.46))
        self.assertFalse(ready_for_stopped_observation(samples[-2:],[0.,0.,1.],.30))
        self.assertFalse(ready_for_stopped_observation(samples[:-1]+[(.30,[.0101,0.,0.])],[0.,0.,1.],.30))
        self.assertFalse(ready_for_stopped_observation(samples[:3]+[(.30,[.01,0.,0.])]*8,[0.,0.,1.],.30))

    def test_stable_offset_is_ready_for_correction_but_not_work(self):
        from docking_r10 import stable_dock
        admitted,metrics=stable_dock(self.history([.06,0.]),[0.,0.])
        self.assertFalse(admitted);self.assertTrue(offset_observable(metrics))
        self.assertFalse(offset_observable(dict(metrics,median_distance_m=.0351,latest_distance_m=.031)))
        for key,value in [('samples',19),('span_s',1.79),('max_gap_s',.151),('position_scatter_p95_m',.0151),('median_distance_m',float('nan'))]:
            with self.subTest(key=key):self.assertFalse(offset_observable(dict(metrics,**{key:value})))
