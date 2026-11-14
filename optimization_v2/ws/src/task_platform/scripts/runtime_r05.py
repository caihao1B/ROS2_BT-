"""Bounded real sensor / native RTAB sessions with owned teardown and process restart."""
from common import *
import time,signal,shutil
from rig_r05 import compose,overlay
from lifecycle_r1 import environment

def run(task,seconds=120,rtf=.6):
 require('R04');lock=lock_session();out=APP/'results/R05'/(task+'_'+stamp());out.mkdir(parents=True);session=out.name;start=time.monotonic();c=compose(out/'scene',task,session,rtf=rtf);adapt=overlay(task,out/'overlay');binding=read(edition('R05')/task/'binding.json');source=read(BASE/'maps/frozen/manifest.json')
 spec=dict(task=task,session=session,sim_seconds=seconds,wall_budget=480,operator_hint=read(out/'scene/fixture.json')['coarse_operator_map_hint'],localization_backend=binding['localization_backend'],rig_revision=binding['rig_revision'],binding_revision=binding['revision']);spec['online_probe_sha256']=sha(PKG/'scripts/observer_r05.py');spec['runtime_sha256']=sha(Path(__file__));dump(out/'spec.json',spec);dump(out/'request.json',dict(map_id=source['map_id'],rig_binding_revision=binding['revision'],localization_backend=binding['localization_backend'],rig_revision=binding['rig_revision']))
 # Native RTAB opens only this disposable copy; original DB stays read-only.
 db=out/'localization.db';shutil.copyfile(BASE/'maps/frozen/map.db',db)
 env=os.environ.copy();env.update(ROS_DOMAIN_ID='119',GZ_PARTITION='r05_'+session,R05_BINDING=str(edition('R05')/task/'binding.json'),R05_ADAPTER=str(adapt),OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1');env['GZ_SIM_SYSTEM_PLUGIN_PATH']=':'.join([str(APP/'build/platform'),*[str(BASE/'ws/install'/p/'lib') for p in ['sensor_sim_adapter','chassis_bridge','scenario_world']],env.get('GZ_SIM_SYSTEM_PLUGIN_PATH','')]);raw=env.copy();raw['ROS_DOMAIN_ID']='121';ev=env.copy();ev['ROS_DOMAIN_ID']='120';children=[];records=[];logs=[];forced=[];error=None;server=None;slam=None;retired=set()
 def launch(role,args,e=env):
  log=(out/(role+'.log')).open('w');logs.append(log);p=subprocess.Popen(args,env=environment(role,e,out),cwd=ROOT,stdout=log,stderr=subprocess.STDOUT,start_new_session=True);children.append(p);records.append(dict(role=role,pid=p.pid,args=args,domain=e['ROS_DOMAIN_ID'],partition=e['GZ_PARTITION']));return p
 def gz(args):return subprocess.run(['gz',*args],env=env,capture_output=True,text=True,timeout=7)
 def start_slam(name):return launch(name,[str(BASE/'venvs/scene/bin/python'),str(PKG/'scripts/regional_r05.py'),'--out',str(out/('quality_restarted' if name.endswith('restarted') else 'quality')),'--request',str(out/'request.json')])
 s05=BASE/'ws/src/localization_guard/scripts';bridge='/opt/ros/jazzy/lib/ros_gz_bridge/parameter_bridge'
 try:
  server=launch('server',['gz','sim','--force-version','8','-s','--seed','41',str(out/'scene/world.sdf')]);deadline=time.monotonic()+45
  while time.monotonic()<deadline:
   if server.poll() is not None:raise RuntimeError('SERVER_FAILED')
   if '/world/irregular_farm/control' in gz(['service','-l']).stdout:break
   time.sleep(.2)
  else:raise TimeoutError('WORLD_STARTUP')
  launch('online_bridge',[bridge,'/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock','/s03/encoders@std_msgs/msg/String[gz.msgs.StringMsg','/s03/actuator/request@std_msgs/msg/String]gz.msgs.StringMsg'])
  launch('raw_bridge',[bridge,'/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock','/s04/raw/lidar/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked','/s04/raw/left@sensor_msgs/msg/Image[gz.msgs.Image','/s04/raw/right@sensor_msgs/msg/Image[gz.msgs.Image','/s04/raw/imu@sensor_msgs/msg/Imu[gz.msgs.IMU','/s04/raw/gnss@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat','/s04/raw/joints@std_msgs/msg/String[gz.msgs.StringMsg'],raw)
  launch('evaluation_bridge',[bridge,'/s03/evaluation/pose@std_msgs/msg/String[gz.msgs.StringMsg','/s03/evaluation/contacts@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts'],ev);launch('evaluator',['/usr/bin/python3',str(BASE/'ws/src/chassis_bridge/scripts/evaluator.py'),'--out',str(out/'evaluation')],ev)
  for mode in ['motion','vision']:launch('generator_'+mode,['/usr/bin/python3',str(adapt/'adapter.py'),'--vehicle',str(out/'scene/vehicle.json'),'--out',str(out/('generation_'+mode)),'--seed','41','--profile','nominal','--streams',mode],raw)
  launch('fanout',['/usr/bin/python3',str(adapt/'fanout.py')]);launch('cloud',['/usr/bin/python3',str(adapt/'cloud_filter.py'),'--out',str(out/'cloud')])
  launch('icp',['/opt/ros/jazzy/lib/rtabmap_odom/icp_odometry','--ros-args','-r','__node:=s05_icp','-r','__ns:=/s05/icp','--params-file',str(out/'scene/icp.yaml'),'-r','scan_cloud:=/s05/icp/cloud','-r','scan:=/s05/unused_scan','-r','odom:=/s05/icp/raw','-r','odom_info:=/s05/icp/info','-r','/tf:=/s05/internal/icp_tf','--log-level','warn'])
  for role,script in [('icp_gate','icp_gate.py'),('inputs','input_adapter.py'),('odom_output','odom_output.py')]:launch(role,['/usr/bin/python3',str(adapt/script if role=='icp_gate' else s05/script),'--out',str(out/role)])
  for name,topic in [('fused','/s05/raw/fused'),('wheel_imu','/s05/raw/wheel_imu')]:launch(name,['/opt/ros/jazzy/lib/robot_localization/ekf_node','--ros-args','-r','__node:=s05_'+name,'-r','__ns:=/s05/'+name,'--params-file',str(out/('scene/'+name+'.yaml')),'-r','odometry/filtered:='+topic,'--log-level','warn','-r','/tf:=/s05/internal/'+name+'_tf'])
  launch('mapping_inputs',['/usr/bin/python3',str(adapt/'preprocessor.py'),'--out',str(out/'mapping_inputs')]);slam=start_slam('slam_initial');observer=launch('observer',['/usr/bin/python3',str(PKG/'scripts/observer_r05.py'),'--out',str(out)])
  ready=['generation_motion/ready','generation_vision/ready','cloud/ready','icp_gate/ready','inputs/ready','odom_output/ready','mapping_inputs/ready','quality/ready','observer_ready','evaluation/ready'];deadline=time.monotonic()+40
  while time.monotonic()<deadline:
   if any(p.poll() is not None for p in children):raise RuntimeError('NODE_STARTUP_FAILED')
   if all((out/p).exists() for p in ready):break
   time.sleep(.15)
  else:raise TimeoutError('NODES_NOT_READY')
  time.sleep(1.5);r=gz(['service','-s','/world/irregular_farm/control','--reqtype','gz.msgs.WorldControl','--reptype','gz.msgs.Boolean','--timeout','3000','--req','pause: false'])
  if r.returncode or 'true' not in r.stdout:raise RuntimeError('UNPAUSE_FAILED')
  restarted=False
  while time.monotonic()-start<spec['wall_budget']:
   if observer.poll() is not None:
    if observer.returncode:raise RuntimeError('OBSERVER_NOT_COMPLETE')
    break
   if any(p.poll() is not None for p in children if p not in retired):raise RuntimeError('ESSENTIAL_NODE_EXITED')
   if not restarted and (out/'restart_requested.json').exists():
    request=read(out/'restart_requested.json');slam.send_signal(signal.SIGINT);slam.wait(timeout=20);retired.add(slam)
    if slam.returncode!=0:raise RuntimeError('SLAM_RESTART_CLOSE_FAILED')
    slam=start_slam('slam_restarted');restarted=True;dump(out/'restart_done.json',dict(t=read(out/'live.json')['t'],prior_pose_source=request.get('source'),previous_exit=0))
   time.sleep(.08)
  else:raise TimeoutError('WALL_WATCHDOG')
 except BaseException as exc:error=repr(exc)
 finally:
  for role in ['observer','slam_restarted','slam_initial']:
   for rec,p in zip(records,children):
    if rec['role']==role and p.poll() is None:
     p.send_signal(signal.SIGINT)
     try:p.wait(timeout=15 if role.startswith('slam') else 4)
     except subprocess.TimeoutExpired:forced.append(role);p.kill();p.wait(timeout=3)
  time.sleep(.5)
  for rec,p in zip(records,children):
   if rec['role']!='server' and p.poll() is None:p.send_signal(signal.SIGINT)
  for rec,p in zip(records,children):
   if rec['role']!='server':
    try:p.wait(timeout=7)
    except subprocess.TimeoutExpired:forced.append(rec['role']);p.kill();p.wait(timeout=3)
  if server and server.poll() is None:
   try:gz(['service','-s','/server_control','--reqtype','gz.msgs.ServerControl','--reptype','gz.msgs.Boolean','--timeout','3000','--req','stop: true']);server.wait(timeout=15)
   except (subprocess.TimeoutExpired,OSError):forced.append('server');server.send_signal(signal.SIGINT)
   if server.poll() is None:
    try:server.wait(timeout=3)
    except subprocess.TimeoutExpired:server.kill();server.wait(timeout=3)
  for f in logs:f.close()
  for rec,p in zip(records,children):rec['exit']=p.returncode
  dump(out/'runtime.json',dict(passed=not error and not forced and all(p.returncode==0 for p in children),error=error,forced=forced,processes=records,wall_seconds=time.monotonic()-start,original_db_unchanged=sha(BASE/'maps/frozen/map.db')==source['source_database_sha256']))
  if db.exists():dump(out/'disposable_db.json',dict(sha256=sha(db),size=db.stat().st_size,source=source['source_database_sha256'],purpose='native localization temporary copy; original immutable'));db.unlink()
 dump(APP/'results/R05'/(task+'_latest.json'),dict(path=str(out.relative_to(ROOT))));print(out,flush=True);return out
