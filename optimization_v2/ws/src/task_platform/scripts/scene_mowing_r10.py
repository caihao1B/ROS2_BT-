"""R10 reuses frozen R09 physical geometry and substitutes only task/tool version."""
from common import *
from scene_r09 import compose as physical_scene
from geometry_r02 import write
from xml.etree import ElementTree as E

def compose(out,task,session,rtf=.4,cluster='B1',case='nominal'):
 c=physical_scene(out,task,session,rtf,cluster,case)
 tool=read(edition('R10')/'mowing/tool.json');tool['session']=session;dump(out/'tool.json',tool)
 root=E.parse(out/'world.sdf').getroot();plugin=root.find("world/model[@name='vehicle']/plugin[@name='s10::ToolActuator']/config");assert plugin is not None;plugin.text=json.dumps(tool);write(out/'world.sdf',root)
 return c
