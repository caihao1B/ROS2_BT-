"""Production Python SQLite contract examples, using disposable data only."""
from pathlib import Path
import sys,tempfile
BASE=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(BASE/'ws/src/experience_memory/tests'))
from contracts16 import run
with tempfile.TemporaryDirectory() as d:
 result=run(Path(d)/'checks.json')
 print('PASS:',len(result['checks']),'normal/failure contracts; original database untouched')
 print(result['evidence']['growth'])
