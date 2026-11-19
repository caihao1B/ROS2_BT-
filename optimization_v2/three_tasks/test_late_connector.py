import math
import unittest
from shapely.geometry import box

from late_connector import measure


class OfflineConnectionGeometry(unittest.TestCase):
    envelope = [[-.3,-.2],[.3,-.2],[.3,.2],[-.3,.2]]
    area = box(-2,-2,2,2)

    def test_reverse_then_forward_counts_stops_with_adjacent_work(self):
        points = [[0.,0.,0.],[-.03,0.,0.],[-.01,0.,0.],[.025,0.,0.]]
        result = measure(points, self.envelope, self.area, (0,0))
        self.assertEqual(result['gear_sections'], 2)
        self.assertEqual(result['switches_between_forward_work_strips'], 2)
        self.assertLessEqual(result['continuous_envelope_clearance_lower_bound_m'], result['min_sampled_envelope_clearance_m'])

    def test_reject_wrong_frame_tangent_and_large_gaps(self):
        cases = [([[0,0,0],[.02,0,0]], (3,0)), ([[0,0,0],[0,.02,0]], (0,0)),
                 ([[0,0,0],[.2,0,0]], (0,0)), ([[0,0,0],[.02,0,math.pi]], (0,0))]
        for poses,offset in cases:
            with self.subTest(poses=poses,offset=offset), self.assertRaises(ValueError):
                measure(poses, self.envelope, self.area, offset)
