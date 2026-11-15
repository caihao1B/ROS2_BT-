from transport_r08 import configure_transport,transport_environment
"""Bounded real sensor / native RTAB sessions with owned teardown and process restart."""
from common import *
import time,signal,shutil
from overlay_r08 import overlay
from scene_mowing_r10 import compose as original_compose
from lifecycle_r1 import environment

def compose(out,task,session,**kwargs):
 from visual_r11 import apply,camera
 c=original_compose(out,task,session,**kwargs)
 original=out/'untextured.sdf';(out/'world.sdf').replace(original)
 apply(original,out/'world.sdf',task);camera(out/'world.sdf',task)
 dump(out/'recording_camera.json',dict(topic='/r12/observer/image',domain=120,autonomy_input=False,source='actual renderer',material_geometry_contract=str(out/'world.visual.json')))
 return c


def run(task="mowing",seconds=420,rtf=.4,cluster="B1",segment_limit=5,case="nominal",segment_start=0):
 require('R09');lock=lock_session();out=APP/'results/R12'/(task+'_'+stamp());out.mkdir(parents=True);session=out.name;start=time.monotonic();c=compose(out/'scene',task,session,rtf=rtf,cluster=cluster,case=case,segment_start=segment_start);adapt=overlay(task,out/'overlay');binding=read(edition('R05')/task/'binding.json');source=read(BASE/'maps/frozen/manifest.json')
 spec=dict(task=task,session=session,sim_seconds=seconds,wall_budget=max(600,int(seconds/min(rtf,.25))+90),cluster=cluster,segment_limit=segment_limit,segment_start=segment_start,case=case,operator_hint=read(out/'scene/fixture.json')['coarse_operator_map_hint'],localization_backend=binding['localization_backend'],rig_revision=binding['rig_revision'],binding_revision=binding['revision']);spec['online_probe_sha256']=sha(PKG/'scripts/mission_mowing_r10.py');spec['runtime_sha256']=sha(Path(__file__));bound=[APP/'build/r03/r03_reeds_shepp',PKG/'r03_backend/reeds_shepp.cpp']+list((PKG/'scripts').glob('*_r07.py'))+list((PKG/'scripts').glob('*_r08.py'))+list((PKG/'scripts').glob('*_r09.py'))+[PKG/'scripts'/name for name in ['runtime_mowing_r10.py', 'mission_mowing_r10.py', 'scene_mowing_r10.py', 'groups_mowing_r10.py', 'recovery_mowing_r10.py', 'coverage_rejoin_r10.py', 'arrival_mowing_r10.py', 'ompl_endpoint_r10.py', 'control_mowing_r10.py', 'obstruction_memory_r10.py', 'risk_mowing_r10.py', 'near_mowing_r10.py', 'self_capture_mowing_r10.py', 'diagnostics_mowing_r10.py', 'reference_spray_r10.py', 'revalidation_spray_r10.py', 'progress_spray_r10.py', 'start_work_r10.py', 'ledger_mowing_r10.py', 'analyze_mowing_r10.py', 'safe_plan_r10.py', 'planning_envelope_r10.py', 'prepare_safe_plan_r10.py', 'common.py']]+list((edition('R10')/'mowing').rglob('*'))+list((PKG/'r07_nav').glob('*'))+list((PKG/'r08_backend').glob('*'))+list((PKG/'r08_nav').glob('*'))+list((PKG/'r09_backend').glob('*'))+[APP/'build/r09_source/terrain.cpp',APP/'build/r09_source/tool_action_server.cpp',APP/'build/r09_source/ToolActuator.cc',APP/'build/r09/libs10_tool_r09.so',APP/'build/r09/r09_tool_action_server']+[APP/'build/r08_source/terrain.cpp',APP/'build/r08_source/global_manager.cpp',APP/'build/r08/r08_global_manager',APP/'build/r08_nav_install/lib/libr08_observed_prior.so']+[APP/'build/r09/r09_terrain',APP/'build/r08/r08_supervisor',APP/'build/r07_nav_install/lib/libr07_exact_goal.so',BASE/'ws/src/work_ledger/scripts/ledger_core.py'];bound += [PKG/'scripts/bridge_r10.py',PKG/'r10_backend/tool_bridge.cpp',APP/'build/r10_bridge/r10_tool_bridge'];bound += [PKG/'scripts/global_filter_r10.py',PKG/'r10_backend/correction_filter.hpp',PKG/'r10_backend/stopped_revalidation.inc',APP/'build/r10_global_source/global_manager.cpp',APP/'build/r10_global/r10_global_manager'];bound += [edition('R10')/'mowing_safe_plan.json',edition('R09')/'mission.json',PKG/'r10_backend/safe_route.cpp',APP/'build/r10_safe_route/r10_safe_route'];bound += [Path(__file__),PKG/'scripts/record_r12.py',PKG/'scripts/visual_r11.py']+list((edition('R11')/'assets').glob('*'));spec['implementation_bindings']={str(p.relative_to(ROOT)):sha(p) for p in bound if p.is_file()};dump(out/'spec.json',spec);dump(out/'request.json',dict(map_id=source['map_id'],rig_binding_revision=binding['revision'],localization_backend=binding['localization_backend'],rig_revision=binding['rig_revision']))
 cfg=read((edition('R10')/'mowing')/'mission.json');dump(out/'mission.json',cfg);bt=read(edition('R06')/'mowing.json');bt.update(session=session,task_version=cfg['version'],rig_revision=cfg['rig']['revision']);dump(out/'bt.json',bt)
 from rig_r08 import bind_adapter
 bind_adapter(adapt,cfg);spec.update(rig_revision=cfg['rig']['revision'],base_calibration_revision=binding['rig_revision']);dump(out/'spec.json',spec)
 env=os.environ.copy();env.update(ROS_DOMAIN_ID='119',GZ_PARTITION='r10_mowing_'+session,R05_BINDING=str(edition('R05')/task/'binding.json'),R05_ADAPTER=str(adapt),OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1');configure_transport(env,out);env['GZ_SIM_SYSTEM_PLUGIN_PATH']=':'.join([str(APP/'build/r09'),str(APP/'build/platform'),*[str(BASE/'ws/install'/p/'lib') for p in ['sensor_sim_adapter','chassis_bridge','scenario_world','motion_prediction','tool_sim']],env.get('GZ_SIM_SYSTEM_PLUGIN_PATH','')]);env['AMENT_PREFIX_PATH']=str(APP/'build/r08_nav_install')+':'+str(APP/'build/r07_nav_install')+':'+env.get('AMENT_PREFIX_PATH','');env['LD_LIBRARY_PATH']=str(APP/'build/r08_nav_install/lib')+':'+str(APP/'build/r07_nav_install/lib')+':'+env.get('LD_LIBRARY_PATH','');raw=env.copy();raw['ROS_DOMAIN_ID']='121';ev=env.copy();ev['ROS_DOMAIN_ID']='120';children=[];records=[];logs=[];forced=[];error=None;server=None;slam=None;retired=set();auxiliary=set();nav_started=False;nav_shutdown=None;fault_sent=False;paused_risk=None;resume_risk_at=None
 def launch(role,args,e=env,aux=False):
  log=(out/(role+'.log')).open('w');logs.append(log);p=subprocess.Popen(args,env=environment(role,transport_environment(role,e),out),cwd=ROOT,stdout=log,stderr=subprocess.STDOUT,start_new_session=True);children.append(p);
  if aux:auxiliary.add(p)
  records.append(dict(role=role,pid=p.pid,args=args,domain=e['ROS_DOMAIN_ID'],partition=e['GZ_PARTITION'],dds_profile=transport_environment(role,e).get('FASTRTPS_DEFAULT_PROFILES_FILE','DEFAULT')));return p
 def gz(args):return subprocess.run(['gz',*args],env=env,capture_output=True,text=True,timeout=7)
 def start_slam(name):return launch(name,[str(BASE/'venvs/scene/bin/python'),str(PKG/'scripts/regional_r09.py'),'--out',str(out/('quality_restarted' if name.endswith('restarted') else 'quality')),'--request',str(out/'request.json')])
 s05=BASE/'ws/src/localization_guard/scripts';bridge='/opt/ros/jazzy/lib/ros_gz_bridge/parameter_bridge'
 try:
  server=launch('server',['gz','sim','--force-version','8','-s','--seed','41',str(out/'scene/world.sdf')]);deadline=time.monotonic()+45
  while time.monotonic()<deadline:
   if server.poll() is not None:raise RuntimeError('SERVER_FAILED')
   if '/world/irregular_farm/control' in gz(['service','-l']).stdout:break
   time.sleep(.2)
  else:raise TimeoutError('WORLD_STARTUP')
  launch('online_bridge',[bridge,'/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock','/s03/encoders@std_msgs/msg/String[gz.msgs.StringMsg','/s03/actuator/request@std_msgs/msg/String]gz.msgs.StringMsg',*['/r07/near_'+u['name']+'/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked' for u in cfg['near_sensor']['units']]])
  launch('tool_bridge',[str(APP/'build/r10_bridge/r10_tool_bridge')])
  launch('raw_bridge',[bridge,'/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock','/s04/raw/lidar/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked','/s04/raw/left@sensor_msgs/msg/Image[gz.msgs.Image','/s04/raw/right@sensor_msgs/msg/Image[gz.msgs.Image','/s04/raw/imu@sensor_msgs/msg/Imu[gz.msgs.IMU','/s04/raw/gnss@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat','/s04/raw/joints@std_msgs/msg/String[gz.msgs.StringMsg'],raw)
  launch('evaluation_bridge',[bridge,'/s03/evaluation/pose@std_msgs/msg/String[gz.msgs.StringMsg','/s03/evaluation/contacts@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts','/s10/evaluation/effect@std_msgs/msg/String[gz.msgs.StringMsg'],ev);launch('evaluator',['/usr/bin/python3',str(BASE/'ws/src/chassis_bridge/scripts/evaluator.py'),'--out',str(out/'evaluation')],ev)
  launch('video_bridge',[bridge,'/r12/observer/image@sensor_msgs/msg/Image[gz.msgs.Image'],ev)
  launch('video_recorder',[str(BASE/'venvs/scene/bin/python'),str(PKG/'scripts/record_r12.py'),'--out',str(out/'video')],ev)
  for mode in ['motion','vision']:launch('generator_'+mode,['/usr/bin/python3',str(adapt/'adapter.py'),'--vehicle',str(out/'scene/vehicle.json'),'--out',str(out/('generation_'+mode)),'--seed','41','--profile','nominal','--streams',mode],raw)
  launch('fanout',['/usr/bin/python3',str(PKG/'scripts/near_mowing_r10.py'),'--out',str(out/'near')]);launch('cloud',['/usr/bin/python3',str(adapt/'cloud_filter.py'),'--out',str(out/'cloud')])
  launch('icp',['/opt/ros/jazzy/lib/rtabmap_odom/icp_odometry','--ros-args','-r','__node:=s05_icp','-r','__ns:=/s05/icp','--params-file',str(out/'scene/icp.yaml'),'-r','scan_cloud:=/s05/icp/cloud','-r','scan:=/s05/unused_scan','-r','odom:=/s05/icp/raw','-r','odom_info:=/s05/icp/info','-r','/tf:=/s05/internal/icp_tf','--log-level','warn'])
  for role,script in [('icp_gate','icp_gate.py'),('inputs','input_adapter.py'),('odom_output','odom_output.py')]:launch(role,['/usr/bin/python3',str(adapt/script if role in ['icp_gate','inputs'] else s05/script),'--out',str(out/role)])
  for name,topic in [('fused','/s05/raw/fused'),('wheel_imu','/s05/raw/wheel_imu')]:launch(name,['/opt/ros/jazzy/lib/robot_localization/ekf_node','--ros-args','-r','__node:=s05_'+name,'-r','__ns:=/s05/'+name,'--params-file',str(out/('scene/'+name+'.yaml')),'-r','odometry/filtered:='+topic,'--log-level','warn','-r','/tf:=/s05/internal/'+name+'_tf'])
  launch('icp_motion_guess',['/usr/bin/python3',str(PKG/'scripts/guess_r07.py'),'--out',str(out/'icp_motion_guess')])
  launch('mapping_inputs',['/usr/bin/python3',str(PKG/'scripts/map_inputs_r08.py'),'--out',str(out/'mapping_inputs')]);slam=start_slam('slam_initial');
  launch('global_sources',[str(BASE/'venvs/scene/bin/python'),str(PKG/'scripts/sources_r07.py'),'--out',str(out/'global_sources'),'--config',str(out/'mission.json')])
  launch('global_manager',[str(APP/'build/r10_global/r10_global_manager'),'--ros-args','-p','use_sim_time:=true','-p','policy_file:='+str((edition('R10')/'mowing')/'manager.json')])
  launch('reference',['/usr/bin/python3',str(PKG/'scripts/reference_spray_r10.py'),'--out',str(out/'reference'),'--config',str(out/'mission.json')])
  launch('terrain',[str(APP/'build/r09/r09_terrain'),'--ros-args','-p','use_sim_time:=true','-p','config_file:='+str((edition('R10')/'mowing')/'terrain.json')])
  launch('risk',[str(BASE/'venvs/scene/bin/python'),str(PKG/'scripts/risk_mowing_r10.py'),'--out',str(out/'risk'),'--config',str(out/'mission.json')])
  launch('control',['/usr/bin/python3',str(PKG/'scripts/control_mowing_r10.py'),'--out',str(out/'control'),'--config',str(out/'mission.json'),'--session',session])
  launch('bt',[str(APP/'build/r08/r08_supervisor'),'--ros-args','-p','use_sim_time:=true','-p','config_file:='+str(out/'bt.json'),'-p','tree_file:='+str(PKG/'r08_backend/common.xml'),'-p','log_file:='+str(out/'decisions.jsonl')])
  for package,name in [('nav2_map_server','map_server'),('nav2_controller','controller_server'),('nav2_planner','planner_server'),('nav2_bt_navigator','bt_navigator'),('nav2_lifecycle_manager','lifecycle_manager')]:
   launch(name,['/opt/ros/jazzy/lib/'+package+'/'+name,'--ros-args','-r','__node:='+name,'--params-file',str((edition('R10')/'mowing')/'nav2.yaml'),'-r','cmd_vel:=/r07/nav_cmd_vel','--log-level','warn'])
  launch('tool_observer',['/usr/bin/python3',str(PKG/'scripts/tool_observer_r09.py'),'--out',str(out/'tool_observer'),'--config',str(out/'scene/tool.json')])
  launch('ledger',[str(BASE/'venvs/scene/bin/python'),str(PKG/'scripts/ledger_mowing_r10.py'),'--out',str(out/'ledger'),'--config',str(out/'scene/tool.json')])
  launch('tool_server',[str(APP/'build/r09/r09_tool_action_server'),'--ros-args','-p','config_file:='+str(out/'scene/tool.json')])
  launch('tool_evaluator',[str(BASE/'venvs/scene/bin/python'),str(PKG/'scripts/tool_evaluator_r09.py'),'--out',str(out/'tool_evaluation'),'--config',str(out/'scene/tool.json')],ev)
  launch('diagnostics',['/usr/bin/python3',str(PKG/'scripts/diagnostics_mowing_r10.py'),'--out',str(out/'diagnostics')])
  observer=launch('observer',[str(BASE/'venvs/scene/bin/python'),str(PKG/'scripts/mission_mowing_r10.py'),'--out',str(out)])
  ready=['video/ready','generation_motion/ready','generation_vision/ready','cloud/ready','icp_gate/ready','inputs/ready','odom_output/ready','icp_motion_guess/ready','mapping_inputs/ready','quality/ready','observer_ready','evaluation/ready','near/ready','reference/ready','global_sources/ready','risk/ready','control/ready','tool_observer/ready','ledger/ready','tool_evaluation/ready'];deadline=time.monotonic()+40
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
   if any(p.poll() is not None for p in children if p not in retired and p not in auxiliary):raise RuntimeError('ESSENTIAL_NODE_EXITED')
   reference=out/'reference/live.json'
   if not nav_started and reference.exists():
    try:ready_ref=read(reference)
    except (ValueError,OSError):ready_ref={}
    if ready_ref.get('valid') and ready_ref.get('t',0)>12:
     launch('nav_startup',['ros2','service','call','/lifecycle_manager/manage_nodes','nav2_msgs/srv/ManageLifecycleNodes','{command: 0}'],aux=True);nav_started=True
   if case=='startup_perception_loss' and (out/'live.json').exists():
    try:live=read(out/'live.json')
    except (ValueError,OSError):live={}
    if not fault_sent and live.get('phase')=='START_WORK':
     paused_risk=next(p for rec,p in zip(records,children) if rec['role']=='risk');paused_risk.send_signal(signal.SIGSTOP);resume_risk_at=live['t']+1.2;fault_sent=True
     dump(out/'startup_fault.json',dict(t=live['t'],until=resume_risk_at,pid=paused_risk.pid,phase=live['phase'],method='SIGSTOP owned risk producer; all health timeouts and BT gates unchanged',resumed=False))
    if paused_risk is not None and live.get('t',0)>=resume_risk_at:
     paused_risk.send_signal(signal.SIGCONT);d=read(out/'startup_fault.json');d.update(resumed=True,resumed_t=live['t']);dump(out/'startup_fault.json',d);paused_risk=None
   if case=='blocked' and not fault_sent and (out/'live.json').exists():
    try:live=read(out/'live.json')
    except (ValueError,OSError):live={}
    if live.get('phase')=='NAVIGATING' and (live.get('gate')or{}).get('progress_m',0)>.30:
     fixture=out/'scene/persistent_barrier.sdf';injected=gz(['service','-s','/world/irregular_farm/create','--reqtype','gz.msgs.EntityFactory','--reptype','gz.msgs.Boolean','--timeout','3000','--req','sdf_filename: "'+str(fixture)+'" allow_renaming: false']);ok=injected.returncode==0 and 'true' in injected.stdout
     dump(out/'barrier_injection.json',dict(passed=ok,observed_trigger_t=live.get('t'),observed_progress_m=(live.get('gate')or{}).get('progress_m'),output=injected.stdout,stderr=injected.stderr,fixture_sha256=sha(fixture),truth_used_online=False))
     if not ok:raise RuntimeError('PHYSICAL_BARRIER_INJECTION_FAILED')
     fault_sent=True
   time.sleep(.08)
  else:raise TimeoutError('WALL_WATCHDOG')
 except BaseException as exc:error=repr(exc)
 finally:
  if paused_risk is not None and paused_risk.poll() is None:paused_risk.send_signal(signal.SIGCONT)
  for role in ['observer']:
   for rec,p in zip(records,children):
    if rec['role']==role and p.poll() is None:
     p.send_signal(signal.SIGINT)
     try:p.wait(timeout=20)
     except subprocess.TimeoutExpired:forced.append(role);p.kill();p.wait(timeout=3)
  try:
   shutdown=subprocess.run(['ros2','service','call','/lifecycle_manager/manage_nodes','nav2_msgs/srv/ManageLifecycleNodes','{command: 4}'],env=env,cwd=ROOT,capture_output=True,text=True,timeout=15)
   nav_shutdown=dict(exit=shutdown.returncode,output=shutdown.stdout[-2000:],stderr=shutdown.stderr[-1000:],passed=shutdown.returncode==0 and 'success=True' in shutdown.stdout);dump(out/'nav_shutdown.json',nav_shutdown)
  except BaseException as exc:nav_shutdown=dict(passed=False,error=repr(exc));dump(out/'nav_shutdown.json',nav_shutdown)
  gz(['service','-s','/world/irregular_farm/control','--reqtype','gz.msgs.WorldControl','--reptype','gz.msgs.Boolean','--timeout','2000','--req','pause: true'])
  time.sleep(.5)
  from shutdown_r08 import stop_ros
  stop_ros(records,children,forced,out)
  if server and server.poll() is None:
   try:gz(['service','-s','/server_control','--reqtype','gz.msgs.ServerControl','--reptype','gz.msgs.Boolean','--timeout','3000','--req','stop: true']);server.wait(timeout=15)
   except (subprocess.TimeoutExpired,OSError):forced.append('server');server.send_signal(signal.SIGINT)
   if server.poll() is None:
    try:server.wait(timeout=3)
    except subprocess.TimeoutExpired:server.kill();server.wait(timeout=3)
  for f in logs:f.close()
  for rec,p in zip(records,children):rec['exit']=p.returncode
  dump(out/'runtime.json',dict(passed=not error and not forced and bool(nav_shutdown and nav_shutdown['passed']) and all(p.returncode==0 for p in children),error=error,forced=forced,processes=records,wall_seconds=time.monotonic()-start,original_db_unchanged=sha(BASE/'maps/frozen/map.db')==source['source_database_sha256']))
 dump(APP/'results/R12'/(task+'_'+case+'_latest.json'),dict(path=str(out.relative_to(ROOT))));print(out,flush=True);return out
