"""Gazebo composition only; no online node imports simulator poses."""
from common10 import *
from xml.etree import ElementTree as E
import mapping_runtime as baseline
from generate import el,values,write

def compose(out,kind,case,session):
 baseline.compose(out,kind,'mapping_full',session);root=E.parse(out/'world.sdf').getroot();w=root.find('world');v=w.find("model[@name='vehicle']");w.find('physics/real_time_factor').text='0.5';v.find('pose').text='30.5 15.3 0.025 0 0 0.15'
 c=contract(kind);m=verify_bundle();c.update(session=session,map_id=m['map_id'],map_revision=m['revision'],frame_epoch=m['frame_epoch'],plan_revision=canonical(dict(case=case,route='diagnostic short straight, operator-selected targets, not S11 coverage plan',revision=1)))
 rotor=el(v,'link',name='s10_rotor');el(rotor,'pose','0 0 -0.055 0 0 0',relative_to='tool_link');inertia=el(rotor,'inertial');el(inertia,'mass',c['rotor_mass_kg']);ii=el(inertia,'inertia')
 for tag,val in [('ixx',.0001),('iyy',.0001),('izz',.0002),('ixy',0),('ixz',0),('iyz',0)]:el(ii,tag,val)
 vis=el(rotor,'visual',name='visible_rotor');el(el(el(vis,'geometry'),'box'),'size','.12 .02 .015');mat=el(vis,'material');el(mat,'diffuse','0.1 0.6 0.9 1')
 j=el(v,'joint',name='s10_motor_joint',type='revolute');el(j,'parent','tool_link');el(j,'child','s10_rotor');axis=el(j,'axis');el(axis,'xyz','0 0 1');lim=el(axis,'limit');el(lim,'lower',-1e16);el(lim,'upper',1e16);el(lim,'effort',10);el(lim,'velocity',30)
 plugin=el(v,'plugin',filename='libs10_tool.so',name='s10::ToolActuator');el(plugin,'config',json.dumps(c));write(out/'world.sdf',root);dump(out/'tool.json',c);return [2.,2.,0.]
