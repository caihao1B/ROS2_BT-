"""Keep the original result of a run when its exact implementation is no longer installed."""
from pathlib import Path
import shutil
from tempfile import TemporaryDirectory

from common import ROOT, read, sha


def current_analysis(run, analyze):
    spec = read(run/'spec.json')
    bindings = spec.get('implementation_bindings', {})
    stale = [path for path, digest in bindings.items()
             if not (ROOT/path).is_file() or sha(ROOT/path) != digest]
    filename = 'analysis_' + analyze.__module__.removeprefix('analyze_') + '.json'
    if bindings and not stale:
        # The qualified analyzers also emit figures and JSON. Recompute separately
        # so checking a still-current recording cannot rewrite sealed evidence.
        products = {filename, 'coverage.geojson', 'trajectory.npz', 'errors.npz'}
        with TemporaryDirectory(prefix='task-recheck-') as temporary:
            scratch = Path(temporary)
            for item in run.iterdir():
                if item.name in products:
                    continue
                target = scratch/item.name
                if item.name != 'ledger':
                    target.symlink_to(item.resolve(), target_is_directory=item.is_dir())
                    continue
                target.mkdir()
                for entry in item.iterdir():
                    if entry.name.startswith('ledger.sqlite'):
                        # A completed writer may have left committed records in the WAL.
                        # SQLite may update its shared-memory sidecar even in read mode.
                        if entry.name != 'ledger.sqlite-shm':
                            shutil.copyfile(entry, target/entry.name)
                    else:
                        (target/entry.name).symlink_to(entry.resolve(), target_is_directory=entry.is_dir())
            return analyze(scratch)

    recorded = read(run/filename) if (run/filename).is_file() else {}
    checks = dict(recorded.get('checks', {}), current_implementation_bindings=False)
    return dict(recorded, passed=False, full_connector_passed=False, checks=checks,
                recorded_passed=recorded.get('passed'), binding_changes=stale,
                binding_manifest_present=bool(bindings),
                task_result=recorded.get('task_result', {}),
                scope=recorded.get('scope', 'Saved run with superseded implementation'))
