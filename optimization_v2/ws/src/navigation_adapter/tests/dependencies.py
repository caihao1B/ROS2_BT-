import sys,json,subprocess
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from common13 import *
from registered_map13 import validate_registration
lock=read(BASE/'references/S13/dependency_lock.json');actual={}
for p,expected in lock['packages'].items():
 actual[p]=subprocess.check_output(['dpkg-query','-W','-f=${Version}','ros-jazzy-'+p],text=True)
 if actual[p]!=expected:raise ValueError('DEPENDENCY_VERSION_CHANGED '+p)
c=validate_registration();dump(Path(sys.argv[1]),dict(packages=actual,registration_heldout_residuals_m=c['heldout_residuals_m'],map_revision=c['source_map_revision'],controller_speed_mps=.12,world_sensor_seed=41,mppi_cross_process_seed='not exposed by installed upstream implementation'))
print('PASS: installed versions and held-out regional registration bindings verified')
