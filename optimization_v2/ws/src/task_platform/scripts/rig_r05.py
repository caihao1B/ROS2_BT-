"""Rig-specific simulation, extrinsics and observation adapters; original assets immutable."""
from common import *
import copy,math,yaml,shutil
from xml.etree import ElementTree as E
from geometry_r02 import world,el,values,write

def replace_one(s,a,b):
 if s.count(a)!=1:raise ValueError('UPSTREAM_SOURCE_CHANGED '+a[:90])
 return s.replace(a,b)

def build():
 require('R04');out=edition('R05');atlas_path=BASE/'maps/s17_full/manifest.json';atlas=read(atlas_path)
 for path,digest in atlas['bindings'].items():
  if sha(ROOT/path)!=digest:raise ValueError('ATLAS_SOURCE_CHANGED '+path)
 base=yaml.safe_load((BASE/'config/sensors.yaml').read_text());source=read(BASE/'maps/frozen/manifest.json')
 for task in TASKS:
  c=read(edition('R04')/'vehicles'/f'{task}.json');rig=copy.deepcopy(base);rig['schema']='R05-SENSORS/1';rig['task']=task;rig['lidar']['vertical']=64;rig['mapping_filter']=dict(range_max_m=35.,voxel_m=.12,ground_stride=4,ground_band_m=.18,max_points=6000,reason='retain observed elevated structures for multi-height rigs; fixed surface quality gates unchanged')
  mounts={'spray':[[.1,0,.78],[.25,0,.66],[0,0,.34],[-.1,0,.80]],'inspection':[[.1,0,.55],[.28,0,.46],[0,0,.18],[-.1,0,.58]],'mowing':[[1.1,0,1.50],[2.25,0,1.30],[.819,0,1.0],[.6,0,1.55]]}[task]
  for key,xyz in zip(['lidar','stereo','imu','gnss'],mounts):rig[key]['xyz']=xyz
  xs=[p[0] for p in c['footprint']];ys=[p[1] for p in c['footprint']];top=c['body']['center'][2]+c['body']['size'][2]/2
  rig['self_filter']=dict(x_min=min(xs)-.05,x_max=max(xs)+.05,y_min=min(ys)-.05,y_max=max(ys)+.05,z_max=max(top+.05,c['wheel']['radius']*2+.05));rig['vehicle_hash']=c['config_hash'];rig['revision']=canonical({k:v for k,v in rig.items() if k!='revision'});dump(out/task/'sensors.json',rig)
  slam=yaml.safe_load((BASE/'config/slam_localization.yaml').read_text());slam['/**']['ros__parameters'].update({'RGBD/ProximityGlobalScanMap':'true','RGBD/ProximityPathFilteringRadius':'3','RGBD/ProximityMaxPaths':'10'});(out/task/'slam.yaml').write_text(yaml.safe_dump(slam,sort_keys=False))
  binding=dict(localization_backend='regional_surface_map',atlas_path=str(atlas_path.relative_to(ROOT)),atlas_sha256=sha(atlas_path),reference_cloud_path=atlas['regions']['C']['cloud_path'],reference_cloud_sha256=sha(ROOT/atlas['regions']['C']['cloud_path']),slam_config_path=str((out/task/'slam.yaml').relative_to(ROOT)),slam_config_sha256=sha(out/task/'slam.yaml'),schema='R05-RIG-MAP/1',task=task,vehicle_path=str((edition('R04')/'vehicles'/f'{task}.json').relative_to(ROOT)),vehicle_sha256=sha(edition('R04')/'vehicles'/f'{task}.json'),rig_path=str((out/task/'sensors.json').relative_to(ROOT)),rig_sha256=sha(out/task/'sensors.json'),rig_revision=rig['revision'],source_manifest_sha256=sha(BASE/'maps/frozen/manifest.json'),source_map_id=source['map_id'],source_revision=source['revision'],source_database_sha256=source['source_database_sha256'],map_frame='map',world_source_sha256=sha(BASE/'assets/worlds/irregular_farm.sdf'),scope='New rig uses existing S17 registered observed cloud; RTAB local ICP retained, native old-DB global branch remains a separate failed diagnostic; no inherited localization PASS')
  binding['revision']=canonical(binding);dump(out/task/'binding.json',binding)
 return out

def overlay(task,out):
 rig=read(edition('R05')/task/'sensors.json');out.mkdir(parents=True,exist_ok=True);s04=BASE/'ws/src/sensor_sim_adapter/scripts';s05=BASE/'ws/src/localization_guard/scripts';s07=BASE/'ws/src/global_pose_manager/scripts';bindings={}
 def put(name,source,text):
  bindings[str(source.relative_to(ROOT))]=sha(source);(out/name).write_text(text)
 for name in ['common.py','adapter.py','fanout.py']:
  src=s04/name;s=src.read_text()
  if name=='common.py':
   s=replace_one(s,"ROOT=Path(__file__).resolve().parents[5]",'ROOT=Path('+repr(str(ROOT))+')');s=replace_one(s,"def settings():return yaml.safe_load((BASE/'config/sensors.yaml').read_text())",'def settings():return read('+repr(str(edition('R05')/task/'sensors.json'))+')')
  put(name,src,s)
 mask=rig['self_filter'];expr=f"(xyz[:,0]>{mask['x_min']})&(xyz[:,0]<{mask['x_max']})&(xyz[:,1]>{mask['y_min']})&(xyz[:,1]<{mask['y_max']})&(xyz[:,2]<{mask['z_max']})"
 for src,name in [(s05/'cloud_filter.py','cloud_filter.py'),(s07/'localization_preprocessor.py','preprocessor.py')]:
  s=src.read_text();s='import sys\nsys.path.insert(0,'+repr(str(src.parent))+')\n'+s
  if name=='cloud_filter.py':
   s=replace_one(s,'np.array([.1,0,.78],np.float32)',f"np.array({rig['lidar']['xyz']!r},np.float32)");s=replace_one(s,'(xyz[:,0]>-.65)&(xyz[:,0]<1.5)&(np.abs(xyz[:,1])<.65)&(xyz[:,2]<1.0)',expr)
  else:
   s=replace_one(s,'+[.1,0,.78]','+'+repr(rig['lidar']['xyz']));s=replace_one(s,'(xyz[:,0]>-.65)&(xyz[:,0]<1.5)&(np.abs(xyz[:,1])<.65)&(xyz[:,2]<1.)',expr)
   s=replace_one(s,'np.floor(xyz/.18)', 'np.floor(xyz/'+repr(rig['mapping_filter']['voxel_m'])+')')
   s=replace_one(s,'            if len(xyz)>6000:', "            ground=np.abs(xyz[:,2])<.18;keep=~ground;keep[np.flatnonzero(ground)[::4]]=True;xyz=xyz[keep]\n            if len(xyz)>6000:")
  put(name,src,s)
 src=s05/'icp_gate.py';s=src.read_text();s='import sys\nsys.path.insert(0,'+repr(str(s05))+')\n'+s
 s=replace_one(s,'from nav_msgs.msg import Odometry','from nav_msgs.msg import Odometry\nfrom sensor_msgs.msg import Imu')
 s=replace_one(s,'    now=[0.]', "    now=[0.];motion=dict(wheel_t=-100.,wheel_v=None,imu_t=-100.,gyro=None,quiet_since=None)\n    def wheel_observed(m):motion.update(wheel_t=stamp(m),wheel_v=m.twist.twist.linear.x)\n    def imu_observed(m):motion.update(imu_t=stamp(m),gyro=float(np.linalg.norm([m.angular_velocity.x,m.angular_velocity.y,m.angular_velocity.z])))\n    n.create_subscription(TwistWithCovarianceStamped,'/s04/localization/wheel',wheel_observed,qos_profile_sensor_data)\n    n.create_subscription(Imu,'/s05/inputs/imu',imu_observed,qos_profile_sensor_data)\n    def stationary(t):\n        quiet=abs(t-motion['wheel_t'])<.20 and abs(t-motion['imu_t'])<.20 and motion['wheel_v'] is not None and abs(motion['wheel_v'])<.01 and motion['gyro'] is not None and motion['gyro']<.02\n        if not quiet:motion['quiet_since']=None;return False\n        if motion['quiet_since'] is None:motion['quiet_since']=t\n        return t-motion['quiet_since']>=1.\n")
 s=replace_one(s,'        consistency_v=None;increment_valid=False','        consistency_v=None;increment_valid=False;stationary_conflict=False;quiet=stationary(t)')
 s=replace_one(s,'                if np.linalg.norm(velocity)<1.2 and np.linalg.norm(angular)<.8:', '                stationary_conflict=quiet and (np.linalg.norm(velocity)>.02 or np.linalg.norm(angular)>.03)\n                if np.linalg.norm(velocity)<1.2 and np.linalg.norm(angular)<.8 and not stationary_conflict:')
 s=replace_one(s,'row=dict(t=t,valid=', 'row=dict(stationary_window=quiet,stationary_conflict=bool(stationary_conflict),t=t,valid=')
 put('icp_gate.py',src,s)
 src=BASE/'ws/src/mission_bt/scripts/regional_math14.py';s=replace_one(src.read_text(),'(np.linalg.norm(raw,axis=1)<30)','(np.linalg.norm(raw,axis=1)<'+repr(rig['mapping_filter']['range_max_m'])+')');put('regional_math.py',src,s)
 src=s07/'localization_quality.py';s=src.read_text();s='import sys\nsys.path.insert(0,'+repr(str(s07))+')\nsys.path.insert(0,'+repr(str(PKG/'scripts'))+')\nfrom contract_r05 import verify_runtime_contract\n'+s
 s=replace_one(s,'manifest=verify_bundle(frozen,read(a.request))','manifest=verify_runtime_contract(read(a.request))');s=replace_one(s,"BASE/'config/sensors.yaml'",'Path('+repr(str(edition('R05')/task/'sensors.json'))+')');put('quality.py',src,s)
 dump(out/'adaptation.json',dict(task=task,rig_revision=rig['revision'],upstream=bindings,changes=['rig-mounted sensor extrinsics and model self bounds','new rig/map qualification contract; original frozen checks retained'],generated={p.name:sha(p) for p in out.glob('*.py')}))
 return out

def compose(out,task,session,start=None,rtf=.6):
 mission=read(edition('R03')/'mission.json');entry=mission['task_plans'][task][0]['start'];poses={task:(entry,[entry[0]-28+.5,entry[1]-14-.5,entry[2]+.05])};start=start or entry
 c=read(edition('R04')/'vehicles'/f'{task}.json');rig=read(edition('R05')/task/'sensors.json');world(out,c,session,full=True,start=start,rtf=rtf);root=E.parse(out/'world.sdf').getroot();w=root.find('world');v=w.find("model[@name='vehicle']");body=v.find("link[@name='base_link']")
 for fn,n in [('imu','Imu'),('navsat','NavSat')]:el(w,'plugin',filename='gz-sim-'+fn+'-system',name='gz::sim::systems::'+n)
 sph=el(w,'spherical_coordinates');lat,lon,h=rig['gnss']['origin']
 for k,val in [('surface_model','EARTH_WGS84'),('world_frame_orientation','ENU'),('latitude_deg',lat),('longitude_deg',lon),('elevation',h),('heading_deg',0)]:el(sph,k,val)
 def sensor(name,kind,c,xyz):
  s=el(body,'sensor',name=name,type=kind);el(s,'pose',values([*xyz,0,0,0]));el(s,'update_rate',c['rate_hz']);el(s,'always_on','true');el(s,'topic','/s04/raw/'+name);return s
 ld=rig['lidar'];s=sensor('lidar','gpu_lidar',ld,ld['xyz']);l=el(s,'lidar');scan=el(l,'scan')
 for key,n,lo,hi in [('horizontal',ld['horizontal'],-math.pi,math.pi),('vertical',ld['vertical'],ld['vertical_min_rad'],ld['vertical_max_rad'])]:
  e=el(scan,key)
  for k,val in [('samples',n),('resolution',1),('min_angle',lo),('max_angle',hi)]:el(e,k,val)
 r=el(l,'range')
 for k,val in [('min',ld['min_m']),('max',ld['max_m']),('resolution',.01)]:el(r,k,val)
 for side,sign in [('left',1),('right',-1)]:
  cam=rig['stereo'];xyz=list(cam['xyz']);xyz[1]+=sign*cam['baseline_m']/2;s=sensor(side,'camera',cam,xyz);e=el(s,'camera');el(e,'horizontal_fov',cam['hfov_rad']);im=el(e,'image')
  for k,val in [('width',cam['width']),('height',cam['height']),('format','R8G8B8')]:el(im,k,val)
  cl=el(e,'clip');el(cl,'near',.05);el(cl,'far',70)
 sensor('imu','imu',rig['imu'],rig['imu']['xyz']);sensor('gnss','navsat',rig['gnss'],rig['gnss']['xyz']);p=el(v,'plugin',filename='libs04_joint_observer.so',name='s04::JointObserver');el(p,'rate',rig['wheel']['rate_hz']);write(out/'world.sdf',root)
 local=yaml.safe_load((BASE/'config/local_estimator.yaml').read_text());icp=yaml.safe_load((BASE/'config/icp_odometry.yaml').read_text());fused=local['s05_fused']['ros__parameters'];wi=copy.deepcopy(fused)
 for k in list(wi):
  if k.startswith('twist1'):del wi[k]
 wi['publish_tf']=False;wi['twist0']='/s05/inputs/wheel_baseline'
 for name,params in [('fused',fused),('wheel_imu',wi),('icp',icp['s05_icp']['ros__parameters'])]:(out/(name+'.yaml')).write_text(yaml.safe_dump({'/**':{'ros__parameters':params}},sort_keys=False))
 dump(out/'sensor_manifest.json',rig);dump(out/'fixture.json',dict(spawn=list(start),online_access=False,coarse_operator_map_hint=poses[task][1]));return c
