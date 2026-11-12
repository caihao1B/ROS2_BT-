"""Simulator-side composer. Does not edit frozen S02/S03 assets or drive code."""
import sys,math
from xml.etree import ElementTree as E
from common import BASE,settings,dump,config_hash
sys.path.insert(0,str(BASE/'ws/src/robot_description/scripts'))
from generate import world,el,values,write
def generate(out,kind,session):
    out.mkdir(parents=True,exist_ok=True);cfg=settings()
    # Test lane faces the pump house; the rig sees the actual agricultural world.
    c=world(out,kind,session,[27,22,0],cfg['real_time_factor'])
    root=E.parse(out/'world.sdf').getroot();w=root.find('world');v=w.find("model[@name='vehicle']");body=v.find("link[@name='base_link']")
    w.find('physics/max_step_size').text=str(cfg['physics_step_s'])
    # Restore S02's actual moving actors: S04 sensor observations include motion.
    original=E.parse(BASE/'assets/worlds/irregular_farm.sdf').getroot().find('world')
    motion=original.find("plugin[@name='s02::ScriptedMotion']")
    for actor in motion.findall('actor'):
        name=actor.findtext('name');old=w.find(f"model[@name='{name}']");w.remove(old);w.append(original.find(f"model[@name='{name}']"))
    w.append(motion)
    for filename,name in [('imu','Imu'),('navsat','NavSat')]:el(w,'plugin',filename='gz-sim-'+filename+'-system',name='gz::sim::systems::'+name)
    sph=el(w,'spherical_coordinates');lat,lon,height=cfg['gnss']['origin']
    for k,val in [('surface_model','EARTH_WGS84'),('world_frame_orientation','ENU'),('latitude_deg',lat),('longitude_deg',lon),('elevation',height),('heading_deg',0)]:el(sph,k,val)
    def sensor(name,kind,c,xyz):
        s=el(body,'sensor',name=name,type=kind);el(s,'pose',values([*xyz,0,0,0]));el(s,'update_rate',c['rate_hz']);el(s,'always_on','true');el(s,'topic','/s04/raw/'+name);return s
    ld=cfg['lidar'];s=sensor('lidar','gpu_lidar',ld,ld['xyz']);l=el(s,'lidar');scan=el(l,'scan')
    for key,n,lo,hi in [('horizontal',ld['horizontal'],-math.pi,math.pi),('vertical',ld['vertical'],ld['vertical_min_rad'],ld['vertical_max_rad'])]:
        e=el(scan,key)
        for k,val in [('samples',n),('resolution',1),('min_angle',lo),('max_angle',hi)]:el(e,k,val)
    r=el(l,'range')
    for k,val in [('min',ld['min_m']),('max',ld['max_m']),('resolution',.01)]:el(r,k,val)
    for side,sign in [('left',1),('right',-1)]:
        cam=cfg['stereo'];xyz=list(cam['xyz']);xyz[1]+=sign*cam['baseline_m']/2;s=sensor(side,'camera',cam,xyz);e=el(s,'camera');el(e,'horizontal_fov',cam['hfov_rad']);im=el(e,'image')
        for k,val in [('width',cam['width']),('height',cam['height']),('format','R8G8B8')]:el(im,k,val)
        cl=el(e,'clip');el(cl,'near',.05);el(cl,'far',70)
    sensor('imu','imu',cfg['imu'],cfg['imu']['xyz'])
    sensor('gnss','navsat',cfg['gnss'],cfg['gnss']['xyz'])
    plugin=el(v,'plugin',filename='libs04_joint_observer.so',name='s04::JointObserver');el(plugin,'rate',cfg['wheel']['rate_hz'])
    write(out/'world.sdf',root)
    c.pop('test_fixture',None);dump(out/'vehicle.json',c)
    dump(out/'sensor_manifest.json',dict(config_hash=config_hash(),settings=cfg,vehicle=c['id'],mount_mass='included in existing body mass; no new collision geometry',map_source='S02 s02.2',dynamic_actors='enabled',initial_pose=c['initial_pose']))
    return c
