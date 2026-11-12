import sys,json,sqlite3,subprocess
from pathlib import Path
import shapely,numpy
root=Path(__file__).resolve().parents[5];base=root/'optimization_v2'
packages=['ros-jazzy-rclcpp-action','ros-jazzy-rosidl-default-generators','libsqlite3-dev','python3-numpy','ros-jazzy-gz-sim-vendor','nlohmann-json3-dev']
versions={}
for p in packages:
 r=subprocess.run(['dpkg-query','-W','-f=${Version}',p],capture_output=True,text=True)
 if r.returncode:raise RuntimeError('Missing package '+p)
 versions[p]=r.stdout
actual=dict(packages=versions,sqlite=sqlite3.sqlite_version,shapely=shapely.__version__,geos=shapely.geos_version_string,numpy=numpy.__version__,python=sys.version.split()[0],venv=str(base/'venvs/scene'),new_system_packages_installed=[])
lock=base/'config/s10_dependency_lock.json'
if '--capture' in sys.argv:lock.write_text(json.dumps(actual,indent=2)+'\n')
else:
 assert json.loads(lock.read_text())==actual,'Environment differs from locked versions'
 if len(sys.argv)>1:Path(sys.argv[1]).write_text(json.dumps(actual,indent=2)+'\n')
 print('PASS dependency lock')
