import json
import math
import unittest

import numpy as np
from shapely.geometry import LineString, shape

from spray_route_time import CANDIDATE, MISSION, model, moves


class PointOrderTest(unittest.TestCase):
    def setUp(self):
        self.mission = json.loads(MISSION.read_text())
        self.cluster = next(c for c in self.mission["clusters"] if c["id"] == "A1")
        self.candidate = json.loads(CANDIDATE.read_text())

    def test_same_work_same_endpoints_and_clearance(self):
        original = self.cluster["points"]
        targets = {p["id"]: np.array(p["target"]) + self.mission["map_offset"] for p in original}
        ordered = self.candidate["ordered_ids"]
        self.assertCountEqual(ordered, [p["id"] for p in original])
        self.assertEqual([ordered[0], ordered[-1]], [original[0]["id"], original[-1]["id"]])
        prior = shape(self.mission["allowed_prior"])
        radius = self.mission["footprint_margin_m"] + max(math.hypot(*v) for v in self.mission["vehicle"]["footprint"])
        for start, end in zip(ordered, ordered[1:]):
            leg = LineString((targets[start], targets[end]))
            self.assertTrue(prior.covers(leg))
            self.assertGreaterEqual(leg.distance(prior.boundary), radius)
        self.assertGreater(self.candidate["route_proxy_reduction_s"], 0)

    def test_solver_neighborhood_preserves_fixed_ends_and_service_ids(self):
        original = list(range(6))
        for move in moves(len(original)):
            self.assertEqual((move[0], move[-1]), (0, 5))
            self.assertEqual(sorted(move.tolist()), original)

    def test_rejects_an_unreachable_shortcut(self):
        _, _, _, report, _, _ = model(self.mission, self.cluster)
        order = np.arange(len(self.cluster["points"]))
        order[1], order[31] = order[31], order[1]
        with self.assertRaisesRegex(ValueError, "geometric envelope"):
            report(order)


if __name__ == "__main__":
    unittest.main()
