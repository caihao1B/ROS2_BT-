"""Continuous SE(2) endpoint connector using the existing native OMPL solver.

This is a kinematic candidate only. The caller must validate the original prior,
full body, actual Nav2 costmap, and live perception before any motion.
"""
from common import *
import math

BINARY=APP/'build/r03/r03_reeds_shepp'

def launch(start,goal,radius,out):
 out.mkdir(parents=True,exist_ok=True);request=out/'request.json';response=out/'response.json';dump(request,dict(radius=radius,queries=[dict(id=0,start=start,goal=goal)]))
 child=subprocess.Popen([str(BINARY),str(request),str(response)],stdout=subprocess.PIPE,stderr=subprocess.PIPE)
 return child,response

def sample(start,response,spacing=.04):
 record=read(response)['paths'][0];points=[list(start)];gears=[]
 for curvature,length in record['primitives']:
  n=max(1,math.ceil(abs(length)/spacing));ds=length/n;gears.append(1 if length>0 else -1)
  for _ in range(n):
   x,y,h=points[-1];nh=h+curvature*ds
   if abs(curvature)<1e-12:nx=x+ds*math.cos(h);ny=y+ds*math.sin(h)
   else:nx=x+(math.sin(nh)-math.sin(h))/curvature;ny=y+(math.cos(h)-math.cos(nh))/curvature
   points.append([nx,ny,math.atan2(math.sin(nh),math.cos(nh))])
 return points,dict(solver='OMPL ReedsSheppStateSpace',length_m=record['length_m'],primitives=record['primitives'],gear_sequence=gears,spacing_m=spacing)
