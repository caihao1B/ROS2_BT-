"""Exercise the real R01 profile and shutdown validators on a disposable input copy."""
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]
APP = Path('optimization_v2/three_tasks')
SCRIPTS = Path('optimization_v2/ws/src/task_runtime/scripts')
LOCK = APP / 'config/shutdown.json'


class WorkspaceBinding(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.cleanup = tempfile.TemporaryDirectory(prefix='three-task-binding-')
        cls.addClassCleanup(cls.cleanup.cleanup)
        cls.root = Path(cls.cleanup.name)
        profiles = [APP / 'config/tasks' / f'{t}.json' for t in ('spray', 'inspection', 'mowing')]
        inputs = {SCRIPTS / name for name in ('common_r1.py', 'registry_r1.py', 'lifecycle_r1.py')}
        inputs.update(profiles)
        for path in profiles:
            profile = json.loads((ROOT / path).read_text())
            inputs.update(Path(item['path']) for item in profile['bindings'].values())
        for path in inputs:
            source = (ROOT / path).resolve()
            if not source.is_relative_to(ROOT) or path.is_absolute() or '..' in path.parts:
                raise ValueError(f'Input outside project: {path}')
            dest = cls.root / path
            dest.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(source, dest)
        cls.original = (cls.root / LOCK).read_bytes()
        cls.lock = json.loads(cls.original)
        candidate = ROOT / APP / 'build/candidate' / Path(cls.lock['preload']).name
        cls.target = cls.root / APP / 'build' / candidate.name
        cls.target.parent.mkdir(parents=True)
        cls.binary = candidate.read_bytes()
        cls.binary_sha = hashlib.sha256(cls.binary).hexdigest()
        item = next(x for x in cls.lock['libraries'] if x['path'] == cls.lock['preload'])
        if cls.binary_sha != item['sha256']:
            raise ValueError('Candidate differs from the original shutdown library hash')

    def setUp(self):
        (self.root / LOCK).write_bytes(self.original)
        self.target.write_bytes(self.binary)

    def call(self, body):
        # Isolated interpreter imports unchanged production modules copied from this checkout.
        setup = f'import sys; sys.path.insert(0, {str(self.root / SCRIPTS)!r}); '
        return subprocess.run([sys.executable, '-I', '-B', '-c', setup + body],
                              capture_output=True, text=True, timeout=15)

    def local_lock(self):
        lock = json.loads(self.original)
        item = next(item for item in lock['libraries'] if item['path'] == lock['preload'])
        item['path'] = item['resolved'] = lock['preload'] = str(self.target)
        (self.root / LOCK).write_text(json.dumps(lock, indent=2) + '\n')
        return lock

    def test_original_profiles_accept_original_lock(self):
        result = self.call('from registry_r1 import TASKS, load_task; [load_task(t) for t in TASKS]')
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_relocated_lock_is_rejected_by_every_task(self):
        self.local_lock()
        abi = self.call('from lifecycle_r1 import validate; validate()')
        self.assertEqual(abi.returncode, 0, abi.stderr)
        for task in ('spray', 'inspection', 'mowing'):
            with self.subTest(task=task):
                result = self.call(f'from registry_r1 import load_task; load_task({task!r})')
                self.assertNotEqual(result.returncode, 0)
                self.assertIn('TASK_INPUT_CHANGED shutdown', result.stderr)

    def test_different_candidate_is_rejected(self):
        self.local_lock()
        self.target.write_bytes(self.binary + b'changed')
        result = self.call('from lifecycle_r1 import validate; validate()')
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('SHUTDOWN_ABI_OR_LIBRARY_CHANGED', result.stderr)

    def test_different_system_hash_is_rejected(self):
        lock = self.local_lock()
        next(lib for lib in lock['libraries'] if lib['path'] != lock['preload'])['sha256'] = '0' * 64
        (self.root / LOCK).write_text(json.dumps(lock))
        result = self.call('from lifecycle_r1 import validate; validate()')
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('SHUTDOWN_ABI_OR_LIBRARY_CHANGED', result.stderr)


if __name__ == '__main__':
    unittest.main(verbosity=2)
