import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from stage_r10 import matches_case


class ScopeReceiptTests(unittest.TestCase):
    def test_full_mowing_requires_original_start_cluster_and_budget(self):
        full = dict(task='mowing', cluster='B1', case='nominal', segment_limit=0,
                    segment_start=0, sim_seconds=1800)
        self.assertTrue(matches_case('mowing_full_B1', full))
        for key, value in [('cluster', 'B2'), ('case', 'startup_perception_loss'),
                           ('segment_limit', 5), ('segment_start', 19), ('sim_seconds', 3600),
                           ('diagnostic_only', True), ('candidate_profile', 'late-warm')]:
            with self.subTest(key=key):
                self.assertFalse(matches_case('mowing_full_B1', dict(full, **{key: value})))

    def test_spray_fault_pilots_keep_declared_headless_and_recorded_budgets(self):
        fault = dict(task='spray', cluster='A1', case='tool_jam', point_limit=3)
        for seconds in (240, 300):
            self.assertTrue(matches_case('spray_tool_jam', dict(fault, sim_seconds=seconds)))
        self.assertFalse(matches_case('spray_full_A1', dict(fault, sim_seconds=1800)))

class HistoricalEvidenceTests(unittest.TestCase):
    def test_new_edition_cannot_overwrite_an_archived_success(self):
        from tempfile import TemporaryDirectory
        from unittest.mock import patch
        import json
        from recheck_evidence_r10 import current_analysis
        with TemporaryDirectory() as tmp:
            run = Path(tmp)
            (run/'worker.py').write_text('new edition')
            (run/'spec.json').write_text(json.dumps({'implementation_bindings': {'worker.py': 'old'}}))
            original = '{"passed":true,"task_result":{"selected_points":3}}'
            (run/'analysis_spray_r10.json').write_text(original)
            def examine(path):
                self.fail('Historical data was reanalyzed with different code')
            examine.__module__ = 'analyze_spray_r10'
            with patch('recheck_evidence_r10.ROOT', run):
                verdict = current_analysis(run, examine)
            self.assertFalse(verdict['passed'])
            self.assertTrue(verdict['recorded_passed'])
            self.assertEqual(verdict['binding_changes'], ['worker.py'])
            self.assertEqual((run/'analysis_spray_r10.json').read_text(), original)

    def test_recomputation_leaves_sealed_results_and_database_sidecars_unchanged(self):
        import json
        from hashlib import sha256
        from tempfile import TemporaryDirectory
        from unittest.mock import patch
        from recheck_evidence_r10 import current_analysis
        with TemporaryDirectory() as temp:
            root=Path(temp)
            run=root/'closed';run.mkdir()
            (root/'worker.py').write_bytes(b'qualified')
            (run/'spec.json').write_text(json.dumps({'implementation_bindings':{
                'worker.py':sha256(b'qualified').hexdigest()}}))
            (run/'analysis_r08.json').write_text('original')
            (run/'ledger').mkdir()
            (run/'ledger/ledger.sqlite').write_bytes(b'db')
            (run/'ledger/ledger.sqlite-wal').write_bytes(b'wal')
            (run/'ledger/ledger.sqlite-shm').write_bytes(b'protected')
            def examine(path):
                self.assertNotEqual(path, run)
                self.assertEqual((path/'ledger/ledger.sqlite-wal').read_bytes(), b'wal')
                (path/'ledger/ledger.sqlite-shm').write_bytes(b'updated')
                (path/'analysis_r08.json').write_text('recheck')
                return {'passed':True}
            examine.__module__='analyze_r08'
            with patch('recheck_evidence_r10.ROOT',root):
                self.assertTrue(current_analysis(run,examine)['passed'])
            self.assertEqual((run/'analysis_r08.json').read_text(),'original')
            self.assertEqual((run/'ledger/ledger.sqlite-shm').read_bytes(),b'protected')

    def test_empty_binding_manifest_is_unverified(self):
        import json
        from tempfile import TemporaryDirectory
        from recheck_evidence_r10 import current_analysis
        with TemporaryDirectory() as tmp:
            run = Path(tmp)
            (run/'spec.json').write_text(json.dumps({'implementation_bindings': {}}))
            def examine(path):
                self.fail('Unbound evidence must not get current credit')
            examine.__module__ = 'analyze_mowing_r10'
            result = current_analysis(run, examine)
            self.assertFalse(result['passed'])
            self.assertFalse(result['binding_manifest_present'])
