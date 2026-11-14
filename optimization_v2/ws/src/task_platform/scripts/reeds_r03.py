"""Signed native Reeds-Shepp primitives; explicit stationary gear boundaries."""
from common import *
import math,importlib.util
spec=importlib.util.spec_from_file_location('r03_motion',ROOT/'offline_coverage/motion.py');motion=importlib.util.module_from_spec(spec);spec.loader.exec_module(motion)
BINARY=APP/'build/r03/r03_reeds_shepp'
def query(start,goals,radius,out):
 out.mkdir(parents=True,exist_ok=True);request=out/'rs_request.json';response=out/'rs_response.json'
 dump(request,dict(radius=radius,queries=[dict(id=i,start=start,goal=g) for i,g in enumerate(goals)]))
 p=subprocess.run([str(BINARY),str(request),str(response)],capture_output=True,text=True,timeout=15)
 if p.returncode:raise ValueError('RS_BACKEND_FAILED '+p.stderr)
 return read(response)['paths']
def sample(start,primitives,ds=.10):
 chunks=[];current=list(start)
 for curvature,length in primitives:
  if abs(length)<1e-9:continue
  n=max(1,math.ceil(abs(length)/ds));poses=[current]
  for _ in range(n):poses.append(motion.step(poses[-1],curvature,length/n))
  chunks.append(dict(poses=poses,direction=1 if length>0 else -1,curvature=curvature,length_m=abs(length),stop_before=True,stop_after=True));current=poses[-1]
 return chunks
