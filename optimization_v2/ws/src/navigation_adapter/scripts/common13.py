import os,sys,json,math,time,hashlib,yaml
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[5];BASE=ROOT/'optimization_v2';PKG=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(BASE/'ws/src/coverage_planner/scripts'))
sys.path.insert(0,str(BASE/"ws/src/coverage_partition/scripts"))
from common11 import read,dump,sha,identity,files,row

def policy():return yaml.safe_load((BASE/'config/controller_profiles.yaml').read_text())
def plan():
 from contracts12 import check_binding
 from products12 import validate_products
 validate_products(BASE/'plans/validated');m=read(BASE/'plans/validated/mission_plan.json');check_binding(m);return m

def rows(p):return [json.loads(s) for s in Path(p).read_text().splitlines() if s.strip()] if Path(p).exists() else []
def yaw(q):return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
def stamp(m):return m.header.stamp.sec+m.header.stamp.nanosec*1e-9
