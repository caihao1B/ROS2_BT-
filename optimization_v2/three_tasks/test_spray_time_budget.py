import copy
import unittest

from spray_time_budget import analyze, common_prefix, r03, PLAN


class SprayTimeTest(unittest.TestCase):
    def setUp(self):
        self.plan = r03(PLAN)
        a, b = self.plan["points"][:2]
        self.a, self.b = a, b
        self.sample = {
            "session": "test", "mission_version": "test", "speed_cap_mps": .3, "yaw_cap_rps": .35,
            "closed": True, "end_sim_s": 18, "error": "stopped", "mission_report_passed": False,
            "wall_seconds_mission_report": 42, "report_completed_ids": [a],
            "event_counts": {"NAV_GOAL": 2, "BT_INTERRUPTS_NAV": 1},
            "events": [
                {"event": "NAV_GOAL", "t": 2, "point": a}, {"event": "NAV_SUCCESS", "t": 5, "point": a},
                {"event": "TOOL_RESULT", "t": 9, "point": a, "state": "SUCCEEDED", "remaining": 0, "uncertainty": False, "replayed": False},
                {"event": "TOOL_RESULT", "t": 10, "point": a, "state": "SUCCEEDED", "remaining": 0, "uncertainty": False, "replayed": True},
                {"event": "NAV_GOAL", "t": 12, "point": b}, {"event": "BT_INTERRUPTS_NAV", "t": 13},
                {"event": "TASK_STOP", "t": 17}
            ],
            "ledger": [[6, a, "RUNNING"], [9, a, "SUCCEEDED"], [14, b, "RUNNING"], [15, b, "CANCELLED"]]
        }

    def test_replays_cancellation_and_stopped_time(self):
        r = analyze(self.sample, self.plan)
        self.assertEqual(r["confirmed_points"], 1)
        self.assertEqual(r["attempted_unconfirmed"], [self.b])
        self.assertEqual(r["completed_cycle_stats"]["elapsed"]["p90_s"], 7)
        self.assertEqual(r["phase_totals_from_first_goal_s"],
                         dict(navigation=4, precision=1, tool=4, wait_recovery_handoff=7))
        self.assertEqual(r["point_cycles"][0]["phase_s"]["tool"], 3)

    def test_ledger_success_needed_to_close(self):
        changed = copy.deepcopy(self.sample)
        changed["ledger"][1][-1] = "CANCELLED"
        with self.assertRaisesRegex(ValueError, "ledger and mission"):
            analyze(changed, self.plan)

    def test_unfinished_reservation_and_common_prefix(self):
        changed = copy.deepcopy(self.sample)
        changed["closed"] = False
        changed["ledger"].pop()
        r = analyze(changed, self.plan)
        self.assertFalse(r["closed"])
        self.assertEqual(r["phase_totals_from_first_goal_s"]["tool"], 7)
        self.assertEqual(common_prefix([r, analyze(self.sample, self.plan)], self.plan["points"], self.plan["budget_sim_s"])[0]["same_points"], 1)

    def test_r03_includes_a_return_tail(self):
        self.assertGreater(self.plan["tail_after_last_point_estimate_s"], 0)
        self.assertEqual(len(self.plan["estimate_through_point_s"]), 56)


if __name__ == "__main__":
    unittest.main()
