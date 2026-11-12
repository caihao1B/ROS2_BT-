"""Replay observed outages and verify recovery cannot swallow real failures."""
import ast
import json
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from admission_recovery14 import AdmissionRecovery, observation_interruption

root = Path(__file__).resolve().parents[2]
upstream = ast.parse((root / 'navigation_adapter/scripts/client13.py').read_text())
fn = next(n for n in ast.walk(upstream) if isinstance(n, ast.FunctionDef) and n.name == 'follow_resuming')
assert any(isinstance(n, ast.Call) and isinstance(n.func, ast.Name) and n.func.id == 'range'
           and len(n.args) == 1 and isinstance(n.args[0], ast.Constant) and n.args[0].value == 12
           for n in ast.walk(fn)), 'review recovery bound if S13 retry contract changes'
fixture = json.loads((Path(__file__).parent / 'fixtures/observation_recovery.json').read_text())
for w in fixture['windows']:
    recovery = AdmissionRecovery(1., 45.)
    recovery.begin(w['interrupted_t'])
    restored = None
    for e in w['records']:
        state = recovery.update(e['t'], all(e[k] for k in ['platform_valid', 'costmaps_ready', 'tool_stopped']))
        if state == 'RESTORED':
            restored = e['t']; break
    assert restored is not None and restored - w['interrupted_t'] < 12., (w['interrupted_t'], restored)
    print('ACTUAL_OBSERVATION_RECOVERY', w['interrupted_t'], restored, 'NO_GLOBAL_REPLAN')

r = AdmissionRecovery(1., 45.)
r.begin(0)
assert r.update(10, True) == 'CLEAR_HOLD'
assert r.update(10.9, False) == 'WAITING'
assert r.update(11, True) == 'CLEAR_HOLD'
assert r.update(11.9, True) == 'CLEAR_HOLD'
assert r.update(12, True) == 'RESTORED'
for i in range(11):
    t = 20 + 3*i
    r.begin(t); assert r.update(t, True) == 'CLEAR_HOLD'
    assert r.update(t + 1., True) == 'RESTORED'
r.begin(100); assert r.update(100, True) == 'EXHAUSTED'
assert r.resumes == 12
r = AdmissionRecovery(1., 45.); r.begin(2.)
assert r.update(46.99, False) == 'WAITING'
assert r.update(47., True) == 'EXHAUSTED'  # arrival at deadline cannot grant motion
r = AdmissionRecovery(1., 45.); r.begin(20.)
assert r.update(19., True) == 'EXHAUSTED'
assert observation_interruption('SHORT_OBSTRUCTION_WAIT', False, dict(reason='STALE_GEOMETRY_STOP'))
for reason in ['INPUT_STALE', 'BT_WATCHDOG', 'SAFETY_STOP', 'BUDGET_STOP', 'NAV_FAILED 105']:
    assert not observation_interruption(reason, False, {})
assert not observation_interruption('SHORT_OBSTRUCTION_WAIT', False, dict(geometry_fresh=True,boundary_valid=True,reason='GEOMETRIC_PROTECTIVE_STOP'))
assert not observation_interruption('SHORT_OBSTRUCTION_WAIT', True, {})
print('ADMISSION_RECOVERY_BOUNDED; FLICKER_CLOCK_TIMEOUT_AND_REAL_OBSTACLE_GUARDS_PASS')
