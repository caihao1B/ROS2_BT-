"""Bounded stationary cluster admission probe, not task acceptance."""
import sys, fcntl, signal
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from runtime14 import *

if __name__ == '__main__':
    kind = sys.argv[1]
    lock = (ROOT / 'native_20261108/runs/session.lock').open('a')
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    out = BASE / 'results/S14' / ('probe_' + kind + '_' + str(os.getpid()))
    print(out, flush=True)
    result = run_case(out, 'zone_a', 'calibration_cluster', 55, 200,
                      kind=kind, probe=True)
    print(json.dumps(read(out / 'online/live.json'), ensure_ascii=False), flush=True)
    raise SystemExit(0 if result['success'] else 1)
