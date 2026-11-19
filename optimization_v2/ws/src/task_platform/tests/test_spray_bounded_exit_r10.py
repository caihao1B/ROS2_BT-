"""Exercise the actual return branch with a confirmed hold and with missing stop evidence."""
import ast
import unittest
from pathlib import Path


class BoundedExitTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        path = Path(__file__).resolve().parents[1] / "scripts/mission_spray_r10.py"
        source = ast.parse(path.read_text())
        main = next(node for node in source.body if isinstance(node, ast.FunctionDef) and node.name == "main")
        branch = next(node for node in main.body if isinstance(node, ast.If)
                      and "LOCALIZATION_REVALIDATION_BOUND" in ast.unparse(node.test))
        # The worker is normally created inside a running ROS task. Extract only
        # its real exit branch for this fixture, retaining the production logic.
        fixture = ast.parse("def exit_code(error,final,events):\n    return 99")
        fixture.body[0].body.insert(0, branch)
        scope = {}
        exec(compile(fixture, str(path), "exec"), scope)
        cls.exit_code = staticmethod(scope["exit_code"])

    def test_controlled_exit_preserves_partial_work(self):
        final = dict(passed=False, t=6., final_gate=dict(t=5.98, stopped=True, tool_stopped=True))
        events = [dict(event="FINAL_TOOL_STOP", state="STOPPED")]
        self.assertEqual(self.exit_code(repr(RuntimeError("LOCALIZATION_REVALIDATION_BOUND")), final, events), 0)
        self.assertFalse(final["passed"])

    def test_refuse_unknown_or_unconfirmed_exit(self):
        for change in ("old", "body_moving", "tool_moving", "tool_ack_missing", "shutdown_error", "other_error"):
            with self.subTest(change=change):
                gate = dict(t=5.98, stopped=True, tool_stopped=True)
                events = [dict(event="FINAL_TOOL_STOP", state="STOPPED")]
                error = repr(RuntimeError("LOCALIZATION_REVALIDATION_BOUND"))
                if change == "old": gate["t"] = 5.
                if change == "body_moving": gate["stopped"] = False
                if change == "tool_moving": gate["tool_stopped"] = False
                if change == "tool_ack_missing": events = []
                if change == "shutdown_error": events.append(dict(event="SHUTDOWN_UNCONFIRMED"))
                if change == "other_error": error = "unhandled failure"
                self.assertNotEqual(self.exit_code(error, dict(passed=False, t=6., final_gate=gate), events), 0)


if __name__ == "__main__":
    unittest.main()
