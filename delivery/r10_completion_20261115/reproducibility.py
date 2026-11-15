"""Regenerate only into a temporary directory; do not touch active binaries/config."""
import sys,tempfile,json,shutil
from pathlib import Path
from unittest.mock import patch
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'optimization_v2/ws/src/task_platform/scripts'))
import prepare_r10,bridge_r10,global_filter_r10,spray_global_r10,precision_supervisor_r10
from common import APP,PKG,edition,sha,dump
with tempfile.TemporaryDirectory(prefix='r10_config_regeneration_') as folder:
    target=Path(folder)
    shutil.copy2(edition('R10')/'mowing_safe_plan.json',target/'mowing_safe_plan.json')
    with patch.object(prepare_r10,'edition',lambda stage:target if stage=='R10' else edition(stage)),patch.object(bridge_r10,'build',lambda:None),patch.object(global_filter_r10,'build',lambda:None),patch.object(spray_global_r10,'build',lambda:None),patch.object(precision_supervisor_r10,'build',lambda:None):
        prepare_r10.build()
    checks={str(p.relative_to(target)):p.read_bytes()==(edition('R10')/p.relative_to(target)).read_bytes() for p in target.rglob('*') if p.is_file()}
report=dict(passed=all(checks.values()),checks=checks,generator_sha256=sha(PKG/'scripts/prepare_r10.py'),scope='Fresh isolated regeneration compared to active R10 inputs; native build suppressed to preserve running binary; native behavior tested separately')
dump(APP/'results/R10/config_reproducibility.json',report);print(json.dumps(report))
if not report['passed']:raise SystemExit(2)
