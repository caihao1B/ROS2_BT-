"""Retain bounded whole-task grids without changing the passive R09 observer."""
from pathlib import Path
source=Path(__file__).with_name('diagnostics_r09.py').read_text()
for before,after in [("t-st['risk_last']<1.","t-st['risk_last']<5."),("st['risk_count']>=80","st['risk_count']>=400")]:
    if source.count(before)!=1:raise RuntimeError('READONLY_DIAGNOSTIC_TEMPLATE_CHANGED')
    source=source.replace(before,after)
exec(compile(source,str(Path(__file__).with_name('diagnostics_r09.py')),'exec'))
