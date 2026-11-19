import copy
import json
import sys
import unittest

import numpy as np

from spray_equal_length import minimum_turn_route
from spray_route_time import PROJECT
from spray_travel_replay import build as replay, REPORT

SCRIPTS = PROJECT / 'optimization_v2/ws/src/task_platform/scripts'
sys.path.insert(0, str(SCRIPTS))
from qualify_spray_travel_r10 import travel_points, verify_plan


class EqualLengthTravelTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.mission, cls.plan = verify_plan(certify=True)
        cls.original = next(c['points'] for c in cls.mission['clusters'] if c['id'] == 'A1')

    def config(self, count=56):
        ids = self.plan['ordered_ids']
        return {**self.mission, 'precision_session': {'ordered_ids': ids[:], 'selected_ids': ids[:count]}}

    def test_runtime_goals_and_every_metered_obligation(self):
        for count in (2, 4, 26, 56):
            cfg = self.config(count)
            original = copy.deepcopy(self.original)
            selected = cfg['precision_session']['selected_ids']
            result = travel_points(original, selected, cfg)
            expected = {p['id']: p for p in self.original}
            self.assertEqual([p['id'] for p in result], selected)
            for p in result:
                self.assertEqual({k:v for k,v in p.items() if k != 'yaw'},
                                 {k:v for k,v in expected[p['id']].items() if k != 'yaw'})
                self.assertAlmostEqual(p['yaw'], self.plan['ingress_yaw_rad'][p['id']])
            self.assertEqual(original, self.original)
        self.assertEqual(self.plan['candidate']['open_motion_m'], self.plan['original']['open_motion_m'])
        self.assertAlmostEqual(self.plan['candidate']['sum_abs_heading_change_rad'],
                               self.plan['certificate']['turn_lower_bound_rad'])
        self.assertEqual(sum(p['quantity'] for p in result), sum(p['quantity'] for p in self.original))

    def test_reject_changed_selection_orientation_contract_and_obligation(self):
        ids = self.plan['ordered_ids']
        cfg = self.config(4)
        for invalid in ([ids[0]], [ids[0], ids[2]], ids[1:5], ids[:3], [ids[0]]*4):
            with self.subTest(invalid=invalid), self.assertRaises(ValueError):
                travel_points(self.original, invalid, cfg)
        for invalid in ([*ids[1:], ids[0]], ids[:-1], [*ids[:-1], ids[0]], [*ids[:-1], 'absent']):
            cfg['precision_session']['ordered_ids'] = invalid
            with self.assertRaises(ValueError):
                travel_points(self.original, ids[:4], cfg)
        cfg = self.config(4)
        cfg['navigation_goal_orientation_required'] = True
        with self.assertRaises(ValueError):
            travel_points(self.original, ids[:4], cfg)

    def test_separate_cycle_is_not_a_route(self):
        # Four vertices can form an isolated cycle with one isolated start/end
        # edge. The initial degree-only MILP is feasible, but a coverage path is not.
        pts = np.array([[0.,0.],[2.,0.],[3.,0.],[3.,1.],[2.,1.],[0.,1.]])
        allowed = np.zeros((6,6), dtype=bool)
        for a,b in ((0,5),(1,2),(2,3),(3,4),(4,1)):
            allowed[a,b] = allowed[b,a] = True
        with self.assertRaisesRegex(ValueError, 'complete minimum-turn route'):
            minimum_turn_route(pts, allowed, 0., 8.)

    def test_replay_keeps_censored_cases_and_measured_scope(self):
        report = replay()
        self.assertEqual(report, json.loads(REPORT.read_text()))
        self.assertEqual(report['qualification']['new_route_closed_loop'], 'NOT_RUN')
        self.assertIsNone(report['qualification']['new_route_elapsed_sim_s'])
        for run in report['sample_medians']:
            self.assertTrue(run['censored'])
            self.assertLess(run['confirmed_points'], run['selected_points'])
        sample = report['sample_medians'][-1]['suffix_from_original_ordinal_14']
        self.assertGreater(sample['quarter_turn']['median_body_rotate_s'], sample['straight']['median_body_rotate_s'])


if __name__ == '__main__':
    unittest.main()
