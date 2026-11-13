from common16 import *
import os
import runtime15
once=runtime15.once
original=runtime15.adapt
# Fresh module per stage process; preserve S15 file bytes and original safety nodes.
def adapt(source):
 source=original(source)
 return once(source,"str(BASE/'ws/src/motion_prediction/scripts/prediction_node15.py')","str(BASE/'ws/src/experience_memory/scripts/node16.py')")
def run_case(out,scenario,group,snapshot,seconds=100,wall=360,gui=False,fault='none'):
 old=runtime15.adapt;runtime15.adapt=adapt
 keys=dict(S16_SNAPSHOT=str(snapshot),S16_CASE=str(out/'prediction'),S16_FAULT=fault);saved={k:os.environ.get(k) for k in keys};os.environ.update(keys)
 try:
  result=runtime15.run_case(out,scenario,group,'test',seconds,wall,gui,fault='none')
  result['memory_fault']=fault;dump(out/'runtime.json',result);return result
 finally:
  runtime15.adapt=old
  for k,v in saved.items():
   if v is None:os.environ.pop(k,None)
   else:os.environ[k]=v
