import sys,subprocess,platform
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from common12 import *
import shapely,scipy

def versions():
    return dict(python=platform.python_version(),shapely=shapely.__version__,geos=shapely.geos_version_string,numpy=np.__version__,scipy=scipy.__version__,
       packages={p:subprocess.check_output(['dpkg-query','-W','-f=${Version}',p],text=True) for p in ['ros-jazzy-fields2cover','ros-jazzy-ortools-vendor','libgdal-dev','libmetis-dev']},
       f2c_header_sha256=sha('/opt/ros/jazzy/include/fields2cover.h'),ortools_header_sha256=sha('/opt/ros/jazzy/opt/ortools_vendor/include/ortools/constraint_solver/routing.h'))
if __name__=='__main__':
    actual=versions();passed=actual==read(BASE/'config/s12_dependency_lock.json')['versions'];dump(Path(sys.argv[1]),dict(passed=passed,versions=actual));print(json.dumps(dict(passed=passed,versions=actual)));raise SystemExit(0 if passed else 1)
