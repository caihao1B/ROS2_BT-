"""Exercise the real mower heartbeat and native stopped revalidation transaction."""
from pathlib import Path
import hashlib

from common import APP, read, dump, sha
from mission_mowing_r10 import worker_heartbeat


def mower_message(stamp, phase, plan_revision, revision_mode):
    state = dict(t=stamp, phase=phase, nav_idle=True, complete=False,
                 nav_failed=False, replans=0, deferrals=0, alternative=False,
                 index=19, segment="TURN_TOOL_OFF", segment_id="B1:safe_turn:5:3",
                 direction=1, work_complete=False)
    message = worker_heartbeat(state, None, plan_revision)
    if revision_mode == "missing":
        message.pop("task_version", None)
    elif revision_mode == "wrong":
        message["task_version"] = "old-session"
    return message


if __name__ == '__main__':
    template = Path(__file__).with_name('probe_revalidation_r10.py')
    source = template.read_text()
    # Exercise the same revision contract as the live /r07/mission publisher.
    reject_mismatch = """moving=False
  for bad_revision in ['missing','wrong']:
   revision_mode=bad_revision
   for _ in range(36):tick()
   checks['reject_'+bad_revision+'_worker_version']=not status[-1]['recovery_ready'] and not call(prepare).success
  revision_mode='actual';until(lambda:status[-1]['recovery_ready'],8.)"""
    replacements = [
        ('results/R10/revalidation_probe', 'results/R10/mowing_revalidation_probe'),
        ("'spray/manager.json'", "'mowing/manager.json'"),
        ("t=0.;yaw=0.;worker='READY';", "revision_mode='actual';t=0.;yaw=0.;worker='READY';"),
        ("worker=dict(t=t,nav_idle=True,phase=worker,task_version=cfg['plan_revision'])",
         "worker=mower_message(t,worker,cfg['plan_revision'],revision_mode)"),
        ("moving=False;until(lambda:status[-1]['recovery_ready'],8.)", reject_mismatch),
    ]
    for original, updated in replacements:
        assert source.count(original) == 1
        source = source.replace(original, updated)
    namespace = dict(globals(), __name__='mowing_transaction_probe')
    exec(compile(source, str(template), 'exec'), namespace)
    code = namespace['run']()
    path = APP / 'results/R10/mowing_revalidation_probe/result.json'
    report = read(path)
    report.update(template_sha256=sha(template),
                  worker_sha256=sha(Path(__file__).with_name('mission_mowing_r10.py')),
                  derived_source_sha256=hashlib.sha256(source.encode()).hexdigest())
    dump(path, report)
    raise SystemExit(code)
