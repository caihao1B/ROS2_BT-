"""Read-only lesson: changing one task parameter produces another version."""
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[3]/'optimization_v2/ws/src/task_runtime/scripts'))
from common_r1 import read,APP,canonical
profile=read(APP/'config/tasks/spray.json')
body={k:v for k,v in profile.items() if k!='version'}
assert canonical(body)==profile['version']
print('recorded:',canonical(body))
body['seed']+=1
print('changed: ',canonical(body))
assert canonical(body)!=profile['version']
print('In-memory comparison only; no configuration was written.')
