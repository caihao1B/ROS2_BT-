"""A parked vehicle may consume only a current non-emergency decision after a cusp."""
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from qualify_b1_context import disposition


class SwitchEvidenceTest(unittest.TestCase):
    def setUp(self):
        self.before = 209.79
        self.segment = "B1:offline_connection:6:3"
        self.current = {"obligation_id": self.segment, "obligation_direction": -1,
                        "stamp": 210.2, "sensor_capture": 210.1}

    def decide(self, action, evidence):
        return disposition(action, evidence, self.segment, -1, self.before)

    def test_carried_over_replan_is_rejected_after_cusp(self):
        old = {**self.current, "obligation_id": "B1:offline_connection:6:2", "obligation_direction": 1}
        self.assertIsNone(self.decide("REPLAN", old))
        self.assertIsNone(self.decide("EXECUTE", {**self.current, "obligation_direction": 1}))
        self.assertIsNone(self.decide("REPLAN", {**self.current, "stamp": self.before}))
        self.assertIsNone(self.decide("REPLAN", {**self.current, "sensor_capture": 209.70}))

    def test_valid_current_obstruction_can_still_trigger_replanning(self):
        for action in ("REPLAN", "WAIT", "EXECUTE"):
            with self.subTest(action=action):
                self.assertEqual(self.decide(action, self.current), action)

    def test_emergency_and_lost_localization_are_immediate(self):
        for action in ("HUMAN", "STOP", "RECOVER_LOCALIZATION"):
            with self.subTest(action=action):
                self.assertEqual(self.decide(action, {}), action)


if __name__ == "__main__":
    unittest.main()
