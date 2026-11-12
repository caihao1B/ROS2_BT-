"""S12 local inputs; reuse S11 geometry without changing its frozen source/output."""
import sys,os,json,hashlib,math,time,subprocess,importlib.util
from pathlib import Path
from functools import lru_cache
import numpy as np
import yaml
from shapely.geometry import Polygon,Point,LineString,shape,mapping,box
from shapely import union_all,set_precision
ROOT=Path(__file__).resolve().parents[5];BASE=ROOT/'optimization_v2';PKG=BASE/'ws/src/coverage_planner';BIN=BASE/'ws/install/coverage_planner/lib/coverage_planner/planner_backend'
sys.path.insert(0,str(BASE/'ws/src/coverage_partition/scripts'))
from common11 import read,dump,sha,identity,files,row,feature,collection
from geometry11 import parameters as partition_parameters,load_domain,build_graph,polygons
from partition11 import solve as partition_solve,verify as partition_verify,repair
from products11 import validate_bundle as validate_partition_bundle
spec=importlib.util.spec_from_file_location('legacy_dubins',ROOT/'offline_coverage/motion.py');dubins=importlib.util.module_from_spec(spec);spec.loader.exec_module(dubins)
def config():return yaml.safe_load((BASE/'config/planning.yaml').read_text())
def backend(request,out,name):
    out.mkdir(parents=True,exist_ok=True);inp=out/(name+'_request.json');dest=out/(name+'_result.json');dump(inp,request)
    t=time.monotonic();args=[str(BIN),str(inp),str(dest)];proc=subprocess.Popen(args,stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True)
    try:stdout,stderr=proc.communicate(timeout=25)
    except BaseException:
        proc.terminate()
        try:proc.communicate(timeout=3)
        except subprocess.TimeoutExpired:proc.kill();proc.communicate()
        raise
    r=subprocess.CompletedProcess(args,proc.returncode,stdout,stderr)
    with (out/'backend_commands.jsonl').open('a') as f:f.write(json.dumps(dict(command=r.args,exit_code=r.returncode,wall_s=time.monotonic()-t,stderr=r.stderr))+'\n')
    if r.returncode:raise ValueError('BACKEND_FAILED '+r.stderr)
    return read(dest)
def wrap(a):return math.atan2(math.sin(a),math.cos(a))
def inputs():
    from common11 import inputs as s11inputs
    return s11inputs()+[BASE/'config/planning.yaml',BASE/'config/gnss_alignment.yaml',ROOT/'offline_coverage/motion.py']+files(BASE/'plans/candidate')
@lru_cache(maxsize=2)
def body_local(vehicle):
    v=yaml.safe_load((BASE/f'config/vehicles/{vehicle}.yaml').read_text());t=yaml.safe_load((BASE/'config/tool_profiles.yaml').read_text())['spray' if vehicle=='diff' else 'mower'];x,y=t['mount_xyz'][:2];a,b=np.array(t['collision_size'][:2])/2
    pts=np.vstack([v['footprint'],[[x-a,y-b],[x+a,y-b],[x+a,y+b],[x-a,y+b]]]);return np.asarray(Polygon(pts).convex_hull.exterior.coords)[:-1],v,t

@lru_cache(maxsize=2)
def effective_speed(vehicle):
    q=read(BASE/'config/vehicles/qualified_profiles.json')['vehicles'][vehicle]
    loc=yaml.safe_load((BASE/'config/localization_supervisor.yaml').read_text())
    tool=yaml.safe_load((BASE/'config/tool_contract.yaml').read_text())
    return min(q['selected_work_mps'],loc['nominal_speed'],tool['work_max_speed'])
