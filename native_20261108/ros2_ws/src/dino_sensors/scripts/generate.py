#!/usr/bin/env python3
import copy,hashlib,json,pathlib,struct,xml.etree.ElementTree as E
import numpy as np
from scipy.spatial import ConvexHull
P=pathlib.Path(__file__).resolve().parents[1];D=P.parent/'dino_description';C=P.parent/'dino_control';O=P/'generated';O.mkdir(exist_ok=True)
g=json.loads((D/'config/geometry.json').read_text());p=json.loads((P/'config/sensors.json').read_text());drive=json.loads((C/'generated/contract.json').read_text())
p.update(geometry_sha256=hashlib.sha256((D/'config/geometry.json').read_bytes()).hexdigest(),rear_x=drive['rear_x'],sensor_poses=g['sensors'])
(O/'contract.json').write_text(json.dumps(p,indent=2)+'\n')
def el(parent,tag,text=None,**attrs):
    e=E.SubElement(parent,tag,attrs)
    if text is not None:e.text=str(text)
    return e
def write(path,root):E.indent(root,space='  ');E.ElementTree(root).write(path,encoding='utf-8',xml_declaration=True)
# New launch tree only. Existing control/description packages stay untouched.
u=E.parse(C/'generated/dino_drive.urdf').getroot()
j=u.find("joint[@name='base_footprint_to_base_link']");j.find('parent').set('link','base_link');j.find('child').set('link','base_footprint');j.set('name','base_link_to_footprint')
write(O/'dino_sensors.urdf',u)
shapes=[]
for part in g['parts']:
    if part['name']=='wheel':continue
    link={'body':'base_link','FL':'front_left_carrier_link','FR':'front_right_carrier_link','RL':'rear_left_carrier_link','RR':'rear_right_carrier_link'}[part['name']]
    for collision in part['collisions']:
        data=(D/collision['mesh']).read_bytes();count=struct.unpack_from('<I',data,80)[0]
        dtype=np.dtype([('normal','<f4',(3,)),('v','<f4',(3,3)),('attr','<u2')])
        verts=np.frombuffer(data,offset=84,count=count,dtype=dtype)['v'].reshape(-1,3).astype(float)
        equations=ConvexHull(np.unique(verts,axis=0)).equations
        # Drop duplicate coplanar equations; each retained normal is unit length.
        equations=np.unique(np.round(equations,9),axis=0)
        shapes.append({'type':'convex','link':link,'source':collision['mesh'],'sha256':hashlib.sha256(data).hexdigest(),'planes':equations.tolist()})
for name in ('front_left','front_right','rear_left','rear_right'):
    shapes.append({'type':'wheel_y','link':name+'_wheel_link','radius':g['wheel']['radius_m'],'width':g['wheel']['width_m']})
(O/'self_geometry.json').write_text(json.dumps(shapes))
def source_world():
    root=E.parse(C/'generated/drive_full.sdf').getroot()
    for sensor in root.findall('.//sensor'):
        if sensor.get('type')=='gpu_lidar':
            for element in sensor.findall('.//horizontal/samples'):element.text=str(p['lidar_horizontal_samples'])
    return root
root=source_world();world=root.find('world');world.set('name','dino_sensor_validation')
# Four known walls for range verification only. Interior faces x/y = +/-6 m.
for axis in (0,1):
    for sign in (-1,1):
        m=el(world,'model',name=f'calibration_wall_{axis}_{sign}');el(m,'static','true');pos=[0,0,1];pos[axis]=sign*6.05;el(m,'pose',' '.join(map(str,pos+[0,0,0])))
        link=el(m,'link',name='wall');size=[12.2,12.2,2];size[axis]=.1
        for kind in ('visual','collision'):
            obj=el(link,kind,name=kind);el(el(el(obj,'geometry'),'box'),'size',' '.join(map(str,size)))
            if kind=='visual':mat=el(obj,'material');el(mat,'ambient','.8 .4 .2 1');el(mat,'diffuse','.8 .4 .2 1')
write(O/'validation_dual.sdf',root)
single=copy.deepcopy(root);base=single.find("world/model[@name='dino']/link[@name='base_link']")
base.remove(base.find("sensor[@name='lidar_rear_link_sensor']"));write(O/'validation_single.sdf',single)
# Operational world preserves the accepted large empty drive field, without measurement walls.
write(O/'runtime_dual.sdf',source_world())
single_runtime=source_world();base=single_runtime.find("world/model[@name='dino']/link[@name='base_link']");base.remove(base.find("sensor[@name='lidar_rear_link_sensor']"));write(O/'runtime_single.sdf',single_runtime)
bridge=(C/'generated/bridge_full.yaml').read_text();(O/'bridge.yaml').write_text(bridge)
state=[0.0]*15;state[0]=drive['rear_x']
ekf=f'''dino_ekf:
  ros__parameters:
    use_sim_time: true
    frequency: 30.0
    sensor_timeout: 0.15
    two_d_mode: true
    publish_tf: false
    publish_acceleration: false
    print_diagnostics: true
    odom_frame: odom
    world_frame: odom
    base_link_frame: rear_axle_link
    transform_timeout: 0.0
    transform_time_offset: 0.0
    reset_on_time_jump: false
    initial_state: {state}
    twist0: /dino/estimator/wheel_twist
    twist0_config: [false, false, false, false, false, false, true, true, false, false, false, false, false, false, false]
    twist0_queue_size: 10
    imu0: /dino/estimator/imu
    imu0_config: [false, false, false, false, false, false, false, false, false, false, false, true, false, false, false]
    imu0_queue_size: 10
    imu0_differential: false
    imu0_relative: false
    imu0_remove_gravitational_acceleration: false
'''
(O/'ekf.yaml').write_text(ekf)
print(json.dumps({'status':'PASS','self_filter_shapes':len(shapes),'geometry_sha256':p['geometry_sha256'],'raw_model_changed':False}))
