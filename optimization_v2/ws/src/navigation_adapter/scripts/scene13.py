"""Simulation composition only. Public route/fixture geometry, never an online pose source."""
from common13 import *
from xml.etree import ElementTree as E
sys.path.insert(0,str(BASE/'ws/src/tool_sim/scripts'))
from scene10 import compose as tool_scene
from generate import write
import copy

def compose(out,kind,case,session):
 hint=tool_scene(out,kind,'navigation_only',session);root=E.parse(out/'world.sdf').getroot();w=root.find('world');v=w.find("model[@name='vehicle']")
 w.find('physics/real_time_factor').text='0.4'
 plug=v.find("plugin[@name='s03::WheelDrive']")
 if plug is None:raise ValueError('S03_DRIVE_PLUGIN_MISSING')
 plug.set('name','s13::WheelDrive');plug.set('filename','libs13_drive.so')
 # A second measured optical channel fills the ground sampling gap. It does
 # not change the S04 lidar or its localization/calibration stream.
 main=v.find(".//sensor[@name='lidar']");near=copy.deepcopy(main);near.set('name','s13_near_lidar');near.find('topic').text='/s13/near'
 near.find('lidar/scan/horizontal/samples').text='128';near.find('lidar/scan/vertical/samples').text='32'
 near.find('lidar/scan/vertical/min_angle').text='-1.10';near.find('lidar/scan/vertical/max_angle').text='-0.30';near.find('lidar/range/max').text='6.0'
 noise=E.SubElement(near.find('lidar'),'noise');E.SubElement(noise,'type').text='gaussian';E.SubElement(noise,'mean').text='0';E.SubElement(noise,'stddev').text='0.01'
 next(link for link in v.findall('link') if main in list(link)).append(near)
 if case=='near_obstacle':
  from generate import el
  box=el(w,'model',name='s13_near_low_obstacle');el(box,'static','true');x=30.5+1.1*math.cos(.15);y=15.3+1.1*math.sin(.15);el(box,'pose',f'{x} {y} .04 0 0 .15');body=el(box,'link',name='body')
  for tag in ['collision','visual']:el(el(el(el(body,tag,name=tag),'geometry'),'box'),'size','.3 .6 .08')
 if case in ['inspect','diff_point_and_inspect']:
  route=read(BASE/'maps/prior/connector.geojson')['features'][1]['geometry']['coordinates'];a,b=route[:2];h=math.atan2(b[1]-a[1],b[0]-a[0]);v.find('pose').text=f'{a[0]} {a[1]} .025 0 0 {h}';hint=[20.5,28.5,-.4]
 if case.startswith('calibration_'):
  from connector12 import road_geometry
  from common12 import config
  poses=road_geometry(config())[0];dist=np.r_[0,np.cumsum(np.linalg.norm(np.diff(np.array(poses)[:,:2],axis=0),axis=1))];arc=float(case.split('_')[1]);point=poses[int(np.argmin(abs(dist-arc)))];v.find('pose').text=f'{point[0]} {point[1]} .025 0 0 {point[2]}';hint=[point[0]-28-.5,point[1]-14+.5,point[2]+.1]
 write(out/'world.sdf',root)
 c=read(out/'tool.json');c['plan_revision']=plan()['plan_revision'];dump(out/'tool.json',c);return hint
