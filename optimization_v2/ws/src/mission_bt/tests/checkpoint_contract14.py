"""Synthetic unit fixtures only: stale/corrupt/incomplete evidence cannot resume."""
import sys,tempfile
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from checkpoint14 import *

case='nominal_spray_cluster';binding={'unit_source':'hash'};dependencies={'unit_dependency':'hash'}
result=dict(passed=True,completed=True,sim_seconds=100,mission=dict(required_points=21,confirmed_points=21))
assert not complete(case,dict(result,mission=dict(required_points=2,confirmed_points=2)))
assert not complete(case,dict(result,completed=False))
with tempfile.TemporaryDirectory(prefix='unit_checkpoint_',dir=BASE/'results/S14') as directory:
    root=Path(directory);source=root/'prior'/case;source.mkdir(parents=True);(source/'synthetic.txt').write_text('UNIT FIXTURE, NOT A ROBOT RUN')
    save(source,case,result,binding,dependencies)
    assert reuse([source.parent],root/'reused'/case,case,binding,dependencies)
    assert (root/'reused'/case/'synthetic.txt').stat().st_ino==(source/'synthetic.txt').stat().st_ino
    assert reuse([source.parent],root/'stale'/case,case,{'unit_source':'changed'},dependencies) is None
    (source/'synthetic.txt').write_text('changed evidence')
    assert reuse([source.parent],root/'corrupt'/case,case,binding,dependencies) is None
print('INCOMPLETE_DENOMINATOR_STALE_VERSION_CORRUPT_EVIDENCE_REJECTED')
