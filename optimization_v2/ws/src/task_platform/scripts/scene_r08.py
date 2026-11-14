"""Simulator-only rig and declared moving cart; online code cannot read fixture."""
from common import *
from rig_r05 import compose as sensor_scene
from geometry_r02 import el,values,write
from xml.etree import ElementTree as E
import copy,yaml,numpy as np

def compose(out,task,session,rtf=.4,case='nominal'):
 cfg=read(edition('R08')/'mission.json');start=cfg['route_world'][0];c=sensor_scene(out,task,session,start=start,rtf=rtf)
 for file,extra in [('icp.yaml',dict(scan_cloud_max_points=0,guess_frame_id='wheel_imu_odom')),('fused.yaml',dict(smooth_lagged_data=True,history_length=2.))]:
  d=yaml.safe_load((out/file).read_text());d['/**']['ros__parameters'].update(extra);(out/file).write_text(yaml.safe_dump(d,sort_keys=False))
 root=E.parse(out/'world.sdf').getroot();world=root.find('world');v=world.find("model[@name='vehicle']");main=v.find(".//sensor[@name='lidar']")
 for unit in cfg['near_sensor']['units']:
  near=copy.deepcopy(main);near.set('name','r08_near_'+unit['name']);near.find('topic').text='/r07/near_'+unit['name'];near.find('pose').text=values([*unit['xyz'],0,0,unit['yaw']]);near.find('lidar/scan/horizontal/samples').text=str(cfg['near_sensor']['horizontal']);near.find('lidar/scan/vertical/samples').text=str(cfg['near_sensor']['vertical']);near.find('lidar/scan/horizontal/min_angle').text=str(-np.pi/2);near.find('lidar/scan/horizontal/max_angle').text=str(np.pi/2);near.find('lidar/scan/vertical/min_angle').text=str(cfg['near_sensor']['vertical_min']);near.find('lidar/scan/vertical/max_angle').text=str(cfg['near_sensor']['vertical_max']);near.find('lidar/range/min').text='.03';near.find('lidar/range/max').text=str(cfg['near_sensor']['range_m']);noise=el(near.find('lidar'),'noise');el(noise,'type','gaussian');el(noise,'mean',0);el(noise,'stddev',cfg['near_sensor']['noise_m']);v.find("link[@name='base_link']").append(near)
 fixture=dict(spawn=start,online_access=False,coarse_operator_map_hint=[start[0]-28+.5,start[1]-14-.5,start[2]+.05],case=case)
 if case in ['leader','blocked']:
  xy=np.array(cfg['route_world'])[:,:2];arc=np.r_[0,np.cumsum(np.linalg.norm(np.diff(xy,axis=0),axis=1))]
  def point(s):return [float(np.interp(s,arc,xy[:,j])) for j in range(2)]+[.6]
  timeline=[(0,4.5),(20,4.5),(60,10.5),(69,10.5),(149,22.5),(300,45.15)] if case=='leader' else [(0,4.5),(400,4.5)]
  cart=el(world,'model',name='r08_cart');el(cart,'pose',values([*point(4.5),0,0,0]));link=el(cart,'link',name='body');el(link,'gravity','false');ii=el(link,'inertial');el(ii,'mass',40);inertia=el(ii,'inertia')
  for tag,value in [('ixx',5),('iyy',5),('izz',2),('ixy',0),('ixz',0),('iyz',0)]:el(inertia,tag,value)
  for tag in ['visual','collision']:
   obj=el(link,tag,name='cart_'+tag);geo=el(obj,'geometry');box=el(geo,'box');el(box,'size','.5 .5 1.2')
   if tag=='visual':mat=el(obj,'material');el(mat,'diffuse','.9 .45 .08 1');el(mat,'ambient','.9 .45 .08 1')
  plugin=el(world,'plugin',filename='libs15_actor.so',name='s15sim::ScriptedMotion');actor=el(plugin,'actor');el(actor,'name','r08_cart')
  for t,s in timeline:el(actor,'waypoint',values([t,*point(s)]))
  fixture['actor_waypoints']=[[t,*point(s)] for t,s in timeline]
 write(out/'world.sdf',root);dump(out/'fixture.json',fixture);dump(out/'assembly.json',dict(base_vehicle_hash=c['config_hash'],tool_capability='ABSENT',near_sensor=cfg['near_sensor']));return c
