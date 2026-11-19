import unittest

from r10_checkpoint import equal_work, full_work_time


def sample_run(name, times, complete):
    return {"session": name, "first_nav_t": 10, "scene_comparison_sha256": "same-fixture",
            "points": [{"id": f"point-{i}", "completed": ok, "confirmed": 3.05 if ok else .7,
                        "uncertain": False, "ledger_state": "SUCCEEDED" if ok else "CANCELLED",
                        "succeeded_t": t} for i, (ok,t) in enumerate(zip(complete,times))]}


class ComparisonTest(unittest.TestCase):
    def test_compares_contiguous_completed_doses_including_interpoint_time(self):
        a = sample_run("control", [16,50,79,None], [True,True,True,False])
        b = sample_run("repair", [18,43,66,95], [True,True,True,True])
        result = equal_work(a,b)
        self.assertEqual(result["consecutive_travel_and_service_cycles"], 2)
        self.assertEqual(result["before_travel_and_service_s"], 63)
        self.assertEqual(result["after_travel_and_service_s"], 48)

    def test_full_selection_uses_the_common_confirmed_prefix(self):
        original = [f"p-{i}" for i in range(56)]
        latest = {"session":"new", "selected_count":56, "total_original":56,
                  "points":[{"id":k, "succeeded_t":20+5*i, "completed":i!=3,
                             "ledger_state":"SUCCEEDED" if i!=3 else "CANCELLED", "uncertain":False}
                            for i,k in enumerate(original)]}
        old = {"session":"old", "point_cycles":[{"id":k,"end_sim_s":100+9*i,"confirmed":True}
                                                for i,k in enumerate(original[:8])]}
        history = {"r03_reference":{"points":original},"sessions":[old]}
        row = full_work_time(latest,history)["compared_sessions"][0]
        self.assertEqual((row["shared_consecutive_points"],row["control_sim_s"],row["current_sim_s"]), (3,18,10))
        self.assertEqual(row["difference_current_minus_control_s"],-8)
        latest["points"][2]["id"]="substitute"
        with self.assertRaises(ValueError):
            full_work_time(latest,history)

    def test_refuses_to_compare_different_initial_scenes(self):
        a = sample_run("first", [20], [True])
        b = {**a, "scene_comparison_sha256": "different"}
        with self.assertRaisesRegex(ValueError, "same physical scene"):
            equal_work(a,b)

    def test_counting_partial_quantity_as_a_completion_is_an_error(self):
        a = sample_run("first", [20], [False])
        a["points"][0]["completed"] = True
        with self.assertRaisesRegex(ValueError, "confirmed ledger outcome"):
            equal_work(a,a)


if __name__ == "__main__":
    unittest.main()
