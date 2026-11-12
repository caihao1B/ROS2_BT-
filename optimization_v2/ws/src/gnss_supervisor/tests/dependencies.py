"""Read actual installed package versions; do not silently accept a changed lock."""
import sys,subprocess,json
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from common08 import *
lock=read(BASE/'config/s08_dependency_lock.json');actual={};mismatches=[]
for package,expected in lock['packages'].items():
    r=subprocess.run(['dpkg-query','-W','-f=${Version}',package],capture_output=True,text=True,timeout=5);actual[package]=r.stdout.strip()
    if r.returncode or actual[package]!=expected:mismatches.append(package)
result=dict(passed=not mismatches,packages=actual,mismatches=mismatches,source='actual dpkg-query; no installation changes');dump(Path(sys.argv[1]),result);print(json.dumps(result));raise SystemExit(0 if result['passed'] else 1)
