"""S17 paths, atomic journals and content addressed experiment identities."""
from pathlib import Path
import hashlib, json, os, sys, time
ROOT = Path(__file__).resolve().parents[5]
BASE = ROOT / 'optimization_v2'
PKG = Path(__file__).resolve().parents[1]
EVAL = BASE / 'evaluation'
RESULTS = BASE / 'results/S17'

def read(p): return json.loads(Path(p).read_text())
def sha(p):
    with Path(p).open('rb') as stream:return hashlib.file_digest(stream,'sha256').hexdigest()
def canonical(x): return hashlib.sha256(json.dumps(x, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()).hexdigest()
def dump(p, d):
    p = Path(p); p.parent.mkdir(parents=True, exist_ok=True)
    temp = p.with_name(p.name + '.tmp.' + str(os.getpid()))
    with temp.open('w') as f:
        json.dump(d, f, indent=2, ensure_ascii=False, allow_nan=False); f.write('\n'); f.flush(); os.fsync(f.fileno())
    os.replace(temp, p)
def files(p): return sorted(x for x in Path(p).rglob('*') if x.is_file() and '__pycache__' not in x.parts)
def rows(p): return [json.loads(s) for s in Path(p).read_text().splitlines() if s.strip()] if Path(p).exists() else []
def add14(): sys.path.insert(0, str(BASE / 'ws/src/mission_bt/scripts'))
def source_binding(scope='all'):
    # A report or a new offline study cannot invalidate already executed robot
    # trials. Bind the complete runtime/evaluator closure for formal runs;
    # experimental dispatchers conservatively bind the entire package.
    if scope=='formal':
        names=['common17.py','inputs17.py','overlay17.py','one17.py','resume17.py','inspection17.py','inspection_obstacles17.py','risk_adapter17.py','prediction_cost17.py','evaluate17.py','binding_check17.py','statistics17.py','checkpoint17.py','temporal_join17.py','dynamic_envelope17.py','inspection_audit17.py','inspection_budget17.py','map_memory17.py']
        code=[PKG/'scripts'/n for n in names]+files(PKG/'include')+files(PKG/'src')+[PKG/'CMakeLists.txt',PKG/'package.xml']
    elif scope=='all':code=files(PKG)+files(BASE/'stages/S17')
    else:raise ValueError('UNKNOWN_SOURCE_SCOPE')
    paths = code + [EVAL/'acceptance_matrix.yaml', EVAL/'seeds.json', BASE/'scripts/build_s17.sh']
    if (BASE/'build_s17/monitor').exists():paths.append(BASE/'build_s17/monitor')
    if (BASE/'ws/install/acceptance_runner/lib/acceptance_runner/gnss_supervisor17').exists():paths.append(BASE/'ws/install/acceptance_runner/lib/acceptance_runner/gnss_supervisor17')
    if (EVAL/'active_inputs17.json').exists():paths.append(EVAL/'active_inputs17.json')
    return {str(p.relative_to(ROOT)): sha(p) for p in paths}
def dependencies(): return {s:sha(BASE/f'results/{s}/latest.json') for s in ['S14','S15','S16']}
