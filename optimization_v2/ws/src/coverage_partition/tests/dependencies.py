"""Verify the exact locally used runtime; no package installation or downloads."""
import sys,subprocess,platform
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from common11 import *
import shapely,numpy,yaml

def versions():
    packages=['libmetis5','libmetis-dev','nlohmann-json3-dev']
    return dict(python=platform.python_version(),shapely=shapely.__version__,geos=shapely.geos_version_string,numpy=numpy.__version__,yaml=yaml.__version__,
      metis_binary=json.loads(subprocess.check_output([str(BIN),'--version'],text=True)),
      packages={p:subprocess.check_output(['dpkg-query','-W','-f=${Version}',p],text=True) for p in packages},
      metis_header_sha256=sha('/usr/include/metis.h'),metis_library_sha256=sha('/usr/lib/x86_64-linux-gnu/libmetis.so'))
if __name__=='__main__':
    actual=versions();locked=read(BASE/'config/s11_dependency_lock.json');passed=actual==locked['versions']
    dump(Path(sys.argv[1]),dict(passed=passed,versions=actual));print(json.dumps(dict(passed=passed,versions=actual)));raise SystemExit(0 if passed else 1)
