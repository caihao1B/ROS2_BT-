"""R10 reuses frozen R09 physical geometry and substitutes only task/tool version."""
from common import *
from scene_r09 import compose as physical_scene
from geometry_r02 import write
from xml.etree import ElementTree as E

def compose(out,task,session,rtf=.4,cluster='B1',case='nominal',segment_start=0):
 c=physical_scene(out,task,session,rtf,cluster,case)
 tool=read(edition('R10')/'mowing/tool.json');tool['session']=session;dump(out/'tool.json',tool)
 root=E.parse(out/'world.sdf').getroot();plugin=root.find("world/model[@name='vehicle']/plugin[@name='s10::ToolActuator']/config");assert plugin is not None;plugin.text=json.dumps(tool)
 cfg=read(edition('R10')/'mowing/mission.json');route=next(x for x in cfg['clusters'] if x['cluster']==cluster)
 if cfg.get('safe_execution_plan'):
  if segment_start:raise ValueError('OLD_TERRAIN_DIAGNOSTIC_NOT_APPLICABLE_TO_SAFE_PLAN')
  start=route['segments'][0]['poses_world'][0];pose=root.find("world/model[@name='vehicle']/pose");v=[float(x) for x in pose.text.split()];v[0],v[1],v[5]=start;pose.text=' '.join(map(str,v))
  dump(out/'fixture.json',dict(spawn=start,online_access=False,coarse_operator_map_hint=[start[0]-28+.5,start[1]-14-.5,start[2]+.05],scope='Initial spawn at approved safe execution entry; original world and localization map unchanged',safe_plan_revision=cfg['safe_execution_plan']['revision']))
 if segment_start:
  if case!='terrain_rejoin_pilot' or segment_start!=12:raise ValueError('UNDECLARED_DIAGNOSTIC_START')
  cfg=read(edition('R10')/'mowing/mission.json');route=next(x for x in cfg['clusters'] if x['cluster']==cluster);start=route['segments'][segment_start]['poses_world'][0]
  pose=root.find("world/model[@name='vehicle']/pose");v=[float(x) for x in pose.text.split()];v[0],v[1],v[5]=start;pose.text=' '.join(map(str,v))
  dump(out/'fixture.json',dict(spawn=start,online_access=False,coarse_operator_map_hint=[start[0]-28+.5,start[1]-14-.5,start[2]+.05],scope='Separate diagnostic starts at public original segment 12; no full-cluster credit or runtime pose reset'))
 write(out/'world.sdf',root)
 return c
