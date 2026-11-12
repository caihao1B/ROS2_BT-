"""Index/receipt guards only. These tests do not validate the robot stages."""
import contextlib
import importlib.util
import io
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

MODULE = Path(__file__).resolve().parents[1] / 'stepctl.py'
spec = importlib.util.spec_from_file_location('stepctl', MODULE)
ctl = importlib.util.module_from_spec(spec)
spec.loader.exec_module(ctl)


class StepIndexTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.root = Path(self.tmp.name)
        self.root_patch = patch.object(ctl, 'ROOT', self.root)
        self.root_patch.start()
        self.plan = {'revision': 'test.1'}
        self.steps = {
            'S01': {'id':'S01', 'title':'fixture one', 'depends_on':[], 'checks':[{'id':'C1'}]},
            'S02': {'id':'S02', 'title':'fixture two', 'depends_on':['S01'], 'checks':[{'id':'C2'}]}
        }

    def tearDown(self):
        self.root_patch.stop()
        self.tmp.cleanup()

    def write_file(self, name, content='fixture'):
        path = self.root / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(content)
        return {'path':name, 'sha256':ctl.digest(path)}

    def write_receipt(self, sid, data):
        path = ctl.receipt_path(sid)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(data))

    def valid_receipt(self, sid):
        artifacts = [self.write_file(f'docs/optimization_v2/{sid}/{name}.md')
                     for name in ('TECHNICAL','CODE_WALKTHROUGH','MANUAL_CHECK','RESULTS')]
        artifacts += [self.write_file(f'optimization_v2/lessons/{sid}/{name}')
                      for name in ('lesson.cpp','lesson.py','README.md')]
        report = f'optimization_v2/results/{sid}/fixture/index.html'
        artifacts.append(self.write_file(report))
        data = dict(schema_version=1, step=sid, plan_revision='test.1', run_id='fixture',
                    checked_at_utc='2000-01-01T00:00:00Z', automated_status='PASS', check_exit_code=0,
                    checks={c['id']:'PASS' for c in self.steps[sid]['checks']},
                    report_path=report, artifacts=artifacts,
                    source_files=[self.write_file(f'implementation/{sid}.py')],
                    dependency_receipts={dep:ctl.digest(ctl.receipt_path(dep)) for dep in self.steps[sid]['depends_on']})
        self.write_receipt(sid, data)
        return data

    def state(self, sid='S01'):
        return ctl.inspect(sid, self.plan, self.steps)[0]

    def test_missing_is_not_run(self):
        self.assertEqual(self.state(), 'NOT_RUN')

    def test_empty_pass_rejected(self):
        self.write_receipt('S01', dict(schema_version=1, step='S01', plan_revision='test.1', automated_status='PASS'))
        self.assertEqual(self.state(), 'INVALID')

    def test_malformed_json_shape_rejected(self):
        self.write_receipt('S01', [])
        self.assertEqual(self.state(), 'INVALID')
        data = self.valid_receipt('S01')
        data['checks'] = []
        self.write_receipt('S01', data)
        self.assertEqual(self.state(), 'INVALID')

    def test_source_change_invalidates(self):
        self.valid_receipt('S01')
        self.assertEqual(self.state(), 'PASS')
        self.write_file('implementation/S01.py', 'changed')
        self.assertEqual(self.state(), 'STALE')

    def test_missing_learning_artifact_rejected(self):
        data = self.valid_receipt('S01')
        data['artifacts'] = [a for a in data['artifacts'] if not a['path'].endswith('lesson.cpp')]
        self.write_receipt('S01', data)
        self.assertEqual(self.state(), 'INVALID')

    def test_dependency_change_invalidates(self):
        parent = self.valid_receipt('S01')
        self.valid_receipt('S02')
        self.assertEqual(self.state('S02'), 'PASS')
        parent['run_id'] = 'fixture-rerun'
        self.write_receipt('S01', parent)
        self.assertEqual(self.state('S02'), 'STALE')

    def test_outside_file_rejected(self):
        data = self.valid_receipt('S01')
        data['source_files'][0]['path'] = '../outside'
        self.write_receipt('S01', data)
        self.assertEqual(self.state(), 'INVALID')

    def test_failed_run_is_not_pass(self):
        data = self.valid_receipt('S01')
        data['automated_status'] = 'FAIL'
        data['check_exit_code'] = 1
        self.write_receipt('S01', data)
        self.assertEqual(self.state(), 'FAIL')

    def test_next_step_requires_current_valid_pass(self):
        with patch.object(ctl, 'load_plan', return_value=(self.plan,self.steps)):
            with contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(ctl.main(['next','01']),1)
                self.valid_receipt('S01')
                self.assertEqual(ctl.main(['next','01']),0)
                self.write_file('implementation/S01.py','changed')
                self.assertEqual(ctl.main(['next','01']),1)

    def test_stage_number_and_missing_handler(self):
        self.assertEqual(ctl.stage_id('6'), 'S06')
        self.assertEqual(ctl.stage_id('s18'), 'S18')
        with self.assertRaises(ValueError): ctl.stage_id('19')
        with patch.object(ctl, 'load_plan', return_value=(self.plan,self.steps)):
            with contextlib.redirect_stderr(io.StringIO()):
                self.assertEqual(ctl.main(['demo','01','--sim-seconds','30','--wall-timeout','60']), 3)

    def test_handler_is_blocked_until_dependency_passes(self):
        self.write_file('optimization_v2/stages/S02/run.sh', 'exit 99\n')
        with patch.object(ctl, 'load_plan', return_value=(self.plan,self.steps)):
            with contextlib.redirect_stderr(io.StringIO()):
                self.assertEqual(ctl.main(['test','02','--sim-seconds','30','--wall-timeout','60']), 4)

    def test_timeout_does_not_stop_unrelated_process(self):
        unrelated = subprocess.Popen([sys.executable,'-c','import time; time.sleep(30)'], start_new_session=True)
        try:
            with contextlib.redirect_stderr(io.StringIO()):
                rc = ctl.bounded_run([sys.executable,'-c','import signal,time; signal.signal(signal.SIGINT, lambda *a: exit(0)); time.sleep(30)'], 0.2)
            self.assertEqual(rc,124)
            self.assertIsNone(unrelated.poll())
        finally:
            unrelated.terminate()
            unrelated.wait(timeout=3)


class PlanContractTest(unittest.TestCase):
    def test_all_stages_and_prerequisites(self):
        plan = json.loads((MODULE.parent/'plan/steps.json').read_text())
        self.assertEqual(len(plan['steps']),18)
        seen=set()
        root=MODULE.parent.parent
        for n, s in enumerate(plan['steps'],1):
            self.assertEqual(s['id'],f'S{n:02d}')
            self.assertTrue(set(s['depends_on']).issubset(seen))
            self.assertTrue((root/s['prompt_path']).is_file())
            self.assertTrue(s['checks'])
            self.assertTrue(s['cases'])
            for case in s['cases']:
                self.assertLessEqual(case['sim_seconds'],1800)
                self.assertIn(case['name'],s['manual_markdown'])
            seen.add(s['id'])


if __name__ == '__main__':
    unittest.main()
