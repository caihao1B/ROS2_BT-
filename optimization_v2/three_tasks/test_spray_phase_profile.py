import gzip
import json
import tempfile
import unittest
from pathlib import Path

from spray_phase_profile import DEFAULT_TRACE, PHASES, analyze, command_mode, sample_changes, tick


class PhaseProfileTest(unittest.TestCase):
    def trace(self):
        events = [
            dict(t=1., event='NAV_GOAL', point='a'),
            dict(t=2., event='BT_INTERRUPTS_NAV', reason='LOCALIZATION_ADMISSION_WAIT'),
            dict(t=3., event='NAV_GOAL', point='a'),
            dict(t=4., event='NAV_SUCCESS', point='a'),
            dict(t=6., event='DOCKING_SETTLED_CHECK', stable=True, point='a'),
            dict(t=8., event='TOOL_RESULT', state='SUCCEEDED', point='a', remaining=0., replayed=False, uncertainty=False),
            dict(t=8.1, event='TOOL_RESULT', state='SUCCEEDED', point='a', remaining=0., replayed=True, uncertainty=False),
            dict(t=8.2, event='NAV_GOAL', point='b'), dict(t=10., event='TASK_STOP')]
        def stream(changes):
            return dict(first_us=0, last_us=tick(11.), max_gap_us=tick(.05), changes=changes)
        return dict(schema='A1-PHASE-OBSERVATIONS/1', points=[['a', 3.], ['b', 3.], ['c', 3.]], selected=['a','b','c'],
                    events=events, session='synthetic', mission_version='v', source_sha256={},
                    cutoff_sim_s=11., terminal_error=None, mission_passed=False, confirmed_ids=['a'],
                    ledger=[['a', 3.]], controller_collision_warnings=0, commands=stream([[0, 'cruise']]),
                    body=stream([[0, 'translating']]), flow=stream([[0, None], [tick(6.2), 'a'], [tick(7.8), None]]))

    def test_flow_phase_sums_and_residual(self):
        report = analyze(self.trace())
        a, b, c = report['rows']
        self.assertEqual([a[k] for k in PHASES], [2., 2., 1., 1.6, .4])
        self.assertEqual((a['nav_attempts'], a['navigation_cruise_s'], a['body_translating_during_nav_s']), (2, 2., 2.))
        self.assertEqual((b['status'], b['elapsed_s'], b['travel_s'], b['other_s']), ('ATTEMPTED_INCOMPLETE', 2., 1.8, .2))
        self.assertEqual((c['status'], c['elapsed_s'], c['nav_attempts']), ('NOT_REACHED', None, None))
        self.assertEqual(report['observed_totals_s']['elapsed_s'] + report['start_before_first_goal_s'] +
                         report['cleanup_after_last_cycle_s'], report['end_sim_s'])

    def test_flow_over_cancel_takes_priority_and_never_leaks_into_another_cycle(self):
        trace = self.trace()
        trace['events'].insert(5, dict(t=7., event='BT_CANCEL_TOOL', reason='LOCALIZATION_ADMISSION_WAIT'))
        self.assertAlmostEqual(analyze(trace)['rows'][0]['localization_wait_s'], 1.2)
        trace['flow']['changes'][2][0] = tick(8.01)
        with self.assertRaisesRegex(ValueError, 'outside its own point'):
            analyze(trace)

    def test_retry_counts_only_observed_stopped_window(self):
        trace = self.trace()
        trace['events'][4:5] = [dict(t=4.2, event='DOCKING_SETTLED_CHECK', stable=True, point='a'),
                dict(t=4.5, event='RESIDUAL_RETRY', point='a'),
                dict(t=6., event='DOCKING_SETTLED_CHECK', stable=True, point='a', continuous_stopped_s=1.)]
        self.assertAlmostEqual(analyze(trace)['rows'][0]['precision_s'], 1.2)

    def test_inconsistent_ledgers_and_missing_flow_rejected(self):
        trace = self.trace()
        trace['ledger'][0][1] = 2.
        with self.assertRaisesRegex(ValueError, 'without quantity'):
            analyze(trace)
        trace['ledger'] = []
        with self.assertRaisesRegex(ValueError, 'persistent ledger disagree'):
            analyze(trace)
        trace = self.trace()
        trace['flow']['changes'] = [[0, None]]
        with self.assertRaisesRegex(ValueError, 'without quantity'):
            analyze(trace)

    def test_regression_missing_samples_and_command_cutoffs(self):
        for times in ((0., .16), (0., .05, .049)):
            with tempfile.TemporaryDirectory() as directory:
                path = Path(directory) / 'samples'
                path.write_text(''.join(json.dumps({'t': t})+'\n' for t in times))
                with self.assertRaises(ValueError):
                    sample_changes(path, lambda _: None)
        self.assertEqual([command_mode(v,w) for v,w in [(0,0), (0,.031), (.015,0), (.1,0), (.25,0)]],
                         ['still','rotate','slow','medium','cruise'])

    def test_published_replay_equals_report(self):
        report = analyze(json.loads(gzip.decompress(DEFAULT_TRACE.read_bytes())))
        self.assertEqual(report, json.loads(DEFAULT_TRACE.with_name('point_phase_profile.json').read_bytes()))
        self.assertEqual(report['status_counts'], {'CONFIRMED':26, 'ATTEMPTED_INCOMPLETE':1, 'NOT_REACHED':29})
        self.assertFalse(report['formal_credit'])


if __name__ == '__main__':
    unittest.main()
