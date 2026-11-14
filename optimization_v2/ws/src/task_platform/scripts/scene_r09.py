"""Simulation assembly only; live mission nodes never import this module."""
from common import *
from rig_r05 import compose as sensor_scene
from geometry_r02 import el,values,write
from xml.etree import ElementTree as E
import copy,yaml

def compose(out,task,session,rtf=.6,cluster='B1',case='nominal'):
    cfg=read(edition('R09')/'mission.json');route=next(x for x in cfg['clusters'] if x['cluster']==cluster)
    c=sensor_scene(out,task,session,start=route['start'],rtf=rtf)
    icp=yaml.safe_load((out/'icp.yaml').read_text());icp['/**']['ros__parameters'].update(scan_cloud_max_points=0,guess_frame_id='wheel_imu_odom');(out/'icp.yaml').write_text(yaml.safe_dump(icp,sort_keys=False))
    fused=yaml.safe_load((out/'fused.yaml').read_text());fused['/**']['ros__parameters'].update(smooth_lagged_data=True,history_length=2.);(out/'fused.yaml').write_text(yaml.safe_dump(fused,sort_keys=False))
    dump(out/'fixture.json',dict(spawn=route['start'],online_access=False,coarse_operator_map_hint=[route['start'][0]-28+.5,route['start'][1]-14-.5,route['start'][2]+.05]))
    root=E.parse(out/'world.sdf').getroot();v=root.find("world/model[@name='vehicle']")
    main=v.find(".//sensor[@name='lidar']")
    from rig_r08 import apply_scene
    apply_scene(main,out,cfg)
    for unit in cfg['near_sensor']['units']:
        near=copy.deepcopy(main);near.set('name','r07_near_'+unit['name']);near.find('topic').text='/r07/near_'+unit['name'];near.find('pose').text=values([*unit['xyz'],0,unit.get('pitch',0.),unit['yaw']])
        near.find('lidar/scan/horizontal/samples').text=str(cfg['near_sensor']['horizontal']);near.find('lidar/scan/vertical/samples').text=str(cfg['near_sensor']['vertical'])
        near.find('lidar/scan/horizontal/min_angle').text=str(-__import__('math').pi/2);near.find('lidar/scan/horizontal/max_angle').text=str(__import__('math').pi/2)
        near.find('lidar/scan/vertical/min_angle').text=str(cfg['near_sensor']['vertical_min']);near.find('lidar/scan/vertical/max_angle').text=str(cfg['near_sensor']['vertical_max']);near.find('lidar/range/min').text='.03';near.find('lidar/range/max').text=str(cfg['near_sensor']['range_m'])
        noise=el(near.find('lidar'),'noise');el(noise,'type','gaussian');el(noise,'mean',0);el(noise,'stddev',cfg['near_sensor']['noise_m']);v.find("link[@name='base_link']").append(near)
    tool=read(edition('R09')/'tool.json');tool['session']=session;dump(out/'tool.json',tool)
    rotor=el(v,'link',name='s10_rotor');el(rotor,'pose','0 0 -0.055 0 0 0',relative_to='tool_link');inert=el(rotor,'inertial');el(inert,'mass',tool['rotor_mass_kg']);ii=el(inert,'inertia')
    for tag,val in [('ixx',.0001),('iyy',.0001),('izz',.0002),('ixy',0),('ixz',0),('iyz',0)]:el(ii,tag,val)
    j=el(v,'joint',name='s10_motor_joint',type='revolute');el(j,'parent','tool_link');el(j,'child','s10_rotor');axis=el(j,'axis');el(axis,'xyz','0 0 1');lim=el(axis,'limit')
    for k,val in [('lower',-1e16),('upper',1e16),('effort',10),('velocity',30)]:el(lim,k,val)
    plugin=el(v,'plugin',filename='libs10_tool_r09.so',name='s10::ToolActuator');el(plugin,'config',json.dumps(tool));
    if case=='blocked':
        import math
        start=route['segments'][0]['poses_world'][0];x=start[0]+4.8*math.cos(start[2]);y=start[1]+4.8*math.sin(start[2]);ob=el(root.find('world'),'model',name='r09_persistent_barrier');el(ob,'static','true');el(ob,'pose',values([x,y,.6,0,0,start[2]]));link=el(ob,'link',name='body')
        for tag in ['visual','collision']:
            a=el(link,tag,name=tag);box=el(el(a,'geometry'),'box');el(box,'size','.6 4.5 1.2')
        fixture=E.Element('sdf',version='1.9');root.find('world').remove(ob);fixture.append(ob);write(out/'persistent_barrier.sdf',fixture);dump(out/'barrier_fixture.json',dict(trigger='after observed localization admission and at least 0.30m measured travel',position_world=[x,y,.6],size_m=[.6,4.5,1.2],purpose='Physical persistent obstruction injected after startup; sensor-only navigation; startup occlusion is a separate retained failure case'))
    write(out/'world.sdf',root)
    dump(out/'assembly.json',dict(base_vehicle_hash=c['config_hash'],added_rotor_mass_kg=tool['rotor_mass_kg'],purpose='abstract physical work feedback fixture; no arm',near_sensor=cfg['near_sensor']))
    return c
