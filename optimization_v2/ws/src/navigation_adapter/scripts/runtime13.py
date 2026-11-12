"""Bounded localization sessions; reuse S03/S04/S05 and S06 observation nodes.
The sensor/localization startup descriptors match the qualified S06 pipeline.
Only simulation composition can see fixture spawn; online nodes do not read it.
"""
import argparse,os,sys,signal,subprocess,time,copy
from xml.etree import ElementTree as E
import psutil,shutil
import common13
from middleware13 import local_dds
sys.path.insert(0,str(common13.BASE/"ws/src/tool_sim/scripts"))
import common10
import common09
import common08
from s07_common import *
from common08 import validate
S08=BASE/"ws/src/gnss_supervisor/scripts"
import mapping_runtime as baseline
SCRIPTS=BASE/'ws/src/localization_guard/scripts';MAPPING=BASE/'ws/src/map_manager/scripts';LOCAL=PKG/'scripts'
def s08_inputs():
    paths=[S08/x for x in ['common08.py','alignment.py','source_adapter.py','runtime.py','observer.py','motion.py']]+[BASE/'ws/src/gnss_supervisor/src/gnss_supervisor.cpp']+[BASE/'config'/x for x in ['gnss_alignment.yaml','localization_supervisor.yaml','fault_cases/localization.yaml']]+[BASE/'ws/install/gnss_supervisor/lib/gnss_supervisor/gnss_supervisor']
    return {str(p.relative_to(ROOT)):digest(p) for p in paths if '__pycache__' not in p.parts}
from scene13 import compose
S09=BASE/"ws/src/terrain_perception/scripts"
FUSION=BASE/"ws/src/object_fusion/scripts"
def capture09():
    paths=[S09/x for x in ['runtime.py','scene.py','common09.py','observer.py','motion.py','costmap_adapter.py','dynamic_evaluator.py']]+list((BASE/'ws/src/object_fusion/scripts').glob('*.py'))+list((BASE/'ws/src/terrain_perception/src').glob('*.cpp'))+list((BASE/'ws/src/terrain_perception/include').rglob('*.hpp'))+[BASE/'config'/x for x in ['traversability.yaml','costmap.yaml','perception_test_area.geojson','detector_lock.yaml','rviz/perception.rviz']]+[BASE/'ws/install/terrain_perception/lib/terrain_perception/terrain_node',BASE/'ws/install/terrain_perception/lib/libobserved_risk_layer.so',BASE/'ws/install/terrain_perception/lib/libs09_actor.so']+[p for p in (BASE/'assets/perception').rglob('*') if p.is_file()]+[p for p in (BASE/'ws/src/object_fusion/vendor').rglob('*') if p.is_file() and '__pycache__' not in p.parts]
    return {str(p.relative_to(ROOT)):digest(p) for p in paths}

def run_case(out,pose_name,case,seconds,wall,seed=41,gui=False,kind="diff",controller=None):
    out=Path(out);out.mkdir(parents=True,exist_ok=True);manifest=verify_bundle();request={k:manifest[k] for k in ['map_id','frame_epoch','revision','calibration_revision']};request['purpose']='DIAGNOSTIC_MOTION_FIXTURE';dump(out/'request.json',request);verify_bundle(request=request)
    dump(out/'capture_inputs.json',capture_inputs());dump(out/'s08_inputs.json',s08_inputs());db=BASE/'maps/frozen/map.db';db_before=digest(db);session=out.parent.name+'_'+out.name;hint=compose(out/'scene',kind,case,session);dump(out/'terrain.json',yaml.safe_load((BASE/'config/traversability.yaml').read_text()));dump(out/'s09_inputs.json',capture09())
    validate();manager=yaml.safe_load((BASE/'config/localization_supervisor.yaml').read_text());manager.update(map_id=manifest['map_id'],diagnostic_fixture=True);dump(out/'manager.json',manager)
    env=local_dds(os.environ);env.update(ROS_DOMAIN_ID='119',GZ_PARTITION='s13_'+session);env['GZ_SIM_SYSTEM_PLUGIN_PATH']=':'.join(str(BASE/'ws/install'/p/'lib') for p in ['sensor_sim_adapter','chassis_bridge','scenario_world','terrain_perception','tool_sim','navigation_adapter'])+':'+env.get('GZ_SIM_SYSTEM_PLUGIN_PATH','');raw=env.copy();raw['ROS_DOMAIN_ID']='121';evaluation=env.copy();evaluation['ROS_DOMAIN_ID']='120';children=[];logs=[];commands=[];retired=set();auxiliary=set();start=time.monotonic();resources=[];events=[];result=dict(case=case,pose_name=pose_name,seconds=seconds,seed=seed,success=False,gui=gui,vehicle=kind)
    def launch(name,args,e=env,aux=False):
        log=(out/(name+'.log')).open('w');logs.append(log);p=subprocess.Popen(args,cwd=ROOT,env=e,stdout=log,stderr=subprocess.STDOUT);children.append(p);commands.append(dict(role=name,args=args,domain=int(e['ROS_DOMAIN_ID']),partition=e['GZ_PARTITION'],**baseline.identity(p.pid)))
        if aux:auxiliary.add(p)
        return p
    def gz(args):return subprocess.run(['gz',*args],env=env,capture_output=True,text=True,timeout=7)
    def start_slam(name):
        return launch(name,['/opt/ros/jazzy/lib/rtabmap_slam/rtabmap','--ros-args','-r','__node:=s07_slam','-r','__ns:=/s07/rtabmap','--params-file',str(BASE/'config/slam_localization.yaml'),'-p','database_path:='+str(db),'-r','rgbd_image:=/s07/inputs/stereo','-r','scan_cloud:=/s07/inputs/cloud','-r','odom:=/s07/inputs/odom','-r','/tf:=/s07/internal/rtabmap_tf'])
    def set_hint(pose7,label,t):
        cov=[0.]*36
        for i in [0,7,14]:cov[i]=1.
        for i in [21,28,35]:cov[i]=.25
        message=dict(header=dict(frame_id='map'),pose=dict(pose=dict(position=dict(zip(['x','y','z'],pose7[:3])),orientation=dict(zip(['x','y','z','w'],pose7[3:]))),covariance=cov))
        launch(label,['ros2','topic','pub','--once','/s07/rtabmap/initialpose','geometry_msgs/msg/PoseWithCovarianceStamped',json.dumps(message)],aux=True);events.append(dict(t=t,event=label,pose=pose7,source='declared coarse operator prior' if label!='restart_hint' else 'last verified estimated pose; not simulator truth'))
    def hint7(h):return [h[0],h[1],0.,*Rotation.from_euler('z',h[2]).as_quat().tolist()]
    try:
        server=launch('server',['gz','sim','--force-version','8','-s','--seed',str(seed),str(out/'scene/world.sdf')]);deadline=time.monotonic()+40
        while time.monotonic()<deadline:
            if server.poll() is not None:raise RuntimeError('Gazebo startup failed')
            if '/world/irregular_farm/control' in gz(['service','-l']).stdout:break
            time.sleep(.3)
        else:raise TimeoutError('world not ready')
        bridge='/opt/ros/jazzy/lib/ros_gz_bridge/parameter_bridge'
        launch('online_bridge',[bridge,'/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock','/s03/encoders@std_msgs/msg/String[gz.msgs.StringMsg','/s03/actuator/request@std_msgs/msg/String]gz.msgs.StringMsg','/s03/front_scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan','/s10/tool/command@std_msgs/msg/String]gz.msgs.StringMsg','/s10/tool/fault@std_msgs/msg/String]gz.msgs.StringMsg','/s10/tool/raw_feedback@std_msgs/msg/String[gz.msgs.StringMsg','/s13/near/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked','--ros-args','-r','/clock:=/s13/clock_input','-r','/s13/near/points:=/s13/near/raw'])
        launch('raw_bridge',[bridge,'/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock','/s04/raw/lidar/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked','/s04/raw/left@sensor_msgs/msg/Image[gz.msgs.Image','/s04/raw/right@sensor_msgs/msg/Image[gz.msgs.Image','/s04/raw/imu@sensor_msgs/msg/Imu[gz.msgs.IMU','/s04/raw/gnss@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat','/s04/raw/joints@std_msgs/msg/String[gz.msgs.StringMsg','--ros-args','-r','/clock:=/s13/clock_input'],raw)
        for name,e in [('clock_online',env),('clock_raw',raw)]:launch(name,[str(BASE/'ws/install/navigation_adapter/lib/navigation_adapter/clock_relay')],e)
        launch('evaluation_bridge',[bridge,'/s03/evaluation/pose@std_msgs/msg/String[gz.msgs.StringMsg','/s03/evaluation/contacts@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts','/s10/evaluation/effect@std_msgs/msg/String[gz.msgs.StringMsg'],evaluation)
        launch('tool_evaluator',[str(BASE/'venvs/scene/bin/python'),str(common10.PKG/'scripts/evaluator10.py'),'--out',str(out/'tool_evaluation'),'--config',str(out/'scene/tool.json')],evaluation);launch('evaluator',['/usr/bin/python3',str(BASE/'ws/src/chassis_bridge/scripts/evaluator.py'),'--out',str(out/'evaluation')],evaluation)
        drive=launch('drive',['/usr/bin/python3',str(common13.PKG/'scripts/drive_extension.py'),'--config',str(out/'scene/vehicle.json'),'--out',str(out/'drive')])
        for mode in ['motion','vision']:
            launch('generator_'+mode,['/usr/bin/python3',str(SCRIPTS/'sensors.py'),'--vehicle',str(out/'scene/vehicle.json'),'--out',str(out/('generation_'+mode)),'--seed',str(seed),'--streams',mode]+(['--faults'] if case=='faults' else ['--imu-only'] if case=='imu_loss' else []),raw)
        launch('near_field',['/usr/bin/python3',str(common13.PKG/'scripts/near_field13.py'),'--out',str(out/'near_field')])
        launch('cloud',['/usr/bin/python3',str(SCRIPTS/'cloud_filter.py'),'--out',str(out/'cloud')])
        launch('icp',['/opt/ros/jazzy/lib/rtabmap_odom/icp_odometry','--ros-args','-r','__node:=s05_icp','-r','__ns:=/s05/icp','--params-file',str(out/'scene/icp.yaml'),'-r','scan_cloud:=/s05/icp/cloud','-r','scan:=/s05/unused_scan','-r','odom:=/s05/icp/raw','-r','odom_info:=/s05/icp/info','-r','/tf:=/s05/internal/icp_tf','--log-level','warn'])
        launch('icp_gate',['/usr/bin/python3',str(SCRIPTS/'icp_gate.py'),'--out',str(out/'icp_gate')])
        launch('inputs',['/usr/bin/python3',str(SCRIPTS/'input_adapter.py'),'--out',str(out/'inputs')])
        for name,topic in [('fused','/s05/raw/fused'),('wheel_imu','/s05/raw/wheel_imu')]:launch(name,['/opt/ros/jazzy/lib/robot_localization/ekf_node','--ros-args','-r','__node:=s05_'+name,'-r','__ns:=/s05/'+name,'--params-file',str(out/('scene/'+name+'.yaml')),'-r','odometry/filtered:='+topic,'--log-level','warn']+['-r','/tf:=/s05/internal/'+name+'_tf'])
        launch('odom_output',['/usr/bin/python3',str(SCRIPTS/'odom_output.py'),'--out',str(out/'odom_output')])
        launch('health',[str(BASE/'ws/install/localization_guard/lib/localization_guard/health_guard'),'--ros-args','--params-file',str(out/'scene/health.yaml')])
        launch('mapping_inputs',['/usr/bin/python3',str(LOCAL/'localization_preprocessor.py'),'--out',str(out/'mapping_inputs')]+(['--occlusion'] if case=='restart_occlusion' else []))
        slam=start_slam('slam');launch('quality',['/usr/bin/python3',str(LOCAL/'localization_quality.py'),'--out',str(out/'quality'),'--request',str(out/'request.json')])
        registered=case in ['inspect','diff_point_and_inspect'];source_process=launch('source_adapter',['/usr/bin/python3',str(common13.PKG/'scripts/source_registered13.py') if registered else str(S08/'source_adapter.py'),'--out',str(out/'sources'),'--case','normal'])
        if registered:launch('registration',[str(BASE/'venvs/scene/bin/python'),str(common13.PKG/'scripts/registered_map13.py'),'--out',str(out/'registration')])
        launch('manager',[str(BASE/'ws/install/gnss_supervisor/lib/gnss_supervisor/gnss_supervisor'),'--ros-args','-p','use_sim_time:=true','-p','policy_file:='+str(out/'manager.json')])
        terrain=read(out/'terrain.json');footprint=read(out/'scene/vehicle.json')['footprint'];terrain['self_mask']=[min(p[0] for p in footprint)-.05,max(p[0] for p in footprint)+.05,min(p[1] for p in footprint)-.05,max(p[1] for p in footprint)+.05];dump(out/'terrain.json',terrain)
        launch('terrain',[str(BASE/'ws/install/navigation_adapter/lib/navigation_adapter/terrain_s13'),'--ros-args','-p','use_sim_time:=true','-p','config_file:='+str(out/'terrain.json')])
        launch('fusion',['/usr/bin/python3',str(FUSION/'fusion_node.py'),'--out',str(out/'fusion'),'--case',case])
        launch('cost_adapter',[str(BASE/'venvs/scene/bin/python'),str(S09/'costmap_adapter.py'),'--out',str(out/'cost_adapter'),'--vehicle',kind])
        launch('prior_cost',[str(BASE/'venvs/scene/bin/python'),str(common13.PKG/'scripts/prior_cost13.py'),'--out',str(out/'prior_cost'),'--vehicle',kind]+(['--registered'] if registered else []))
        nav_config=BASE/f'config/nav2_{kind}.yaml'
        for role,package,binary in [('controller','nav2_controller','controller_server'),('planner','nav2_planner','planner_server'),('navigator','nav2_bt_navigator','bt_navigator')]:
            args=[f'/opt/ros/jazzy/lib/{package}/{binary}','--ros-args','--params-file',str(nav_config)]
            if role=='controller':args+=['-r','cmd_vel:=/s13/nav_cmd']
            launch(role,args)
        launch('nav_lifecycle',['/opt/ros/jazzy/lib/nav2_lifecycle_manager/lifecycle_manager','--ros-args','-p','use_sim_time:=true','-p','autostart:=true','-p','node_names:=[controller_server,planner_server,bt_navigator]','-p','bond_timeout:=5.0'])
        for name,script,python in [('ledger',common10.LEDGER/'scripts/ledger_node.py',str(BASE/'venvs/scene/bin/python')),('tool_online',common10.PKG/'scripts/online_adapter.py',str(BASE/'venvs/scene/bin/python'))]:
            launch(name,[python,str(script),'--out',str(out/name),'--config',str(out/'scene/tool.json')])
        gate=common13.policy();gate.update(vehicle=kind,wheel_radius_m=read(out/'scene/vehicle.json')['wheel']['radius'],plan_revision=common13.plan()['plan_revision'],sim_deadline=seconds,log_file=str(out/'gate.jsonl'));dump(out/'gate.json',gate)
        driver=launch('motion',[str(BASE/'ws/install/navigation_adapter/lib/navigation_adapter/command_gate'),'--ros-args','-p','use_sim_time:=true','-p','config_file:='+str(out/'gate.json')])
        controller=controller or ('DWB' if kind=='diff' else 'RPP')
        client=launch('client',[str(BASE/'venvs/scene/bin/python'),str(common13.PKG/'scripts/client13.py'),'--out',str(out/'client'),'--vehicle',kind,'--controller',controller,'--case',case,'--seconds',str(seconds)])
        launch('tool_server',[str(BASE/'ws/install/tool_sim/lib/tool_sim/tool_action_server'),'--ros-args','-p','config_file:='+str(out/'scene/tool.json')])
        observer=launch('observer',['/usr/bin/python3',str(common13.PKG/'scripts/observer13.py'),'--out',str(out/'online'),'--seconds',str(seconds)])
        ready=['drive/bridge_ready','generation_motion/ready','generation_vision/ready','cloud/ready','icp_gate/ready','inputs/ready','odom_output/ready','mapping_inputs/ready','sources/ready','fusion/ready','cost_adapter/ready','quality/ready','online/ready','evaluation/ready','tool_evaluation/ready','tool_online/ready','ledger/ready','client/ready','prior_cost/ready'];deadline=time.monotonic()+35
        if registered:ready.append('registration/ready')
        while time.monotonic()<deadline:
            if any(p.poll() is not None for p in children):raise RuntimeError('essential node exited during startup: '+str([(c['role'],p.returncode) for c,p in zip(commands,children) if p.poll() is not None and p not in auxiliary and p not in retired]))
            if all((out/f).exists() for f in ready):break
            time.sleep(.2)
        else:raise TimeoutError('nodes not ready')
        if gui:
            view=yaml.safe_load((BASE/'config/rviz/navigation.rviz').read_text());view['Visualization Manager']['Views']['Current']['Focal Point']=dict(X=4.,Y=0.,Z=.5);rviz_config=out/'scene/localization.rviz';rviz_config.write_text(yaml.safe_dump(view,sort_keys=False));launch('rviz',['rviz2','-d',str(rviz_config),'--ros-args','-p','use_sim_time:=true'])
        time.sleep(2);u=gz(['service','-s','/world/irregular_farm/control','--reqtype','gz.msgs.WorldControl','--reptype','gz.msgs.Boolean','--timeout','4000','--req','pause: false'])
        if u.returncode or 'true' not in u.stdout:raise RuntimeError('unpause failed')
        actions=set();last_sample=0.;last_ack=-100.;restart_pose=None;procs={};shot=False
        while time.monotonic()-start<wall:
            live=read(out/'online/live.json') if (out/'online/live.json').exists() else dict(t=0,status=None,pose=None);t=live['t']
            if t>4 and 'initial' not in actions:
                actions.add('initial');set_hint(hint7([12.,12.,1.] if case=='wrong_initial' else hint),'initial_hint',t)
            for recovery_time in [25,40]:
                label='repeat_coarse_hint_'+str(recovery_time)
                if t>recovery_time and label not in actions and not (live.get('status') or {}).get('has_transform',False) and (live.get('tool') or {}).get('stopped_confirmed') and not (live.get('tool') or {}).get('active'):
                    actions.add(label);set_hint(hint7(hint),label,t)
            if case=='wrong_initial' and t>30 and 'corrected' not in actions:actions.add('corrected');set_hint(hint7(hint),'corrected_coarse_hint',t)
            if case=='restart_occlusion' and t>40 and 'stop_slam' not in actions:
                actions.add('stop_slam');restart_pose=live['pose']['pose'] if live['pose'] else hint7(hint);slam.send_signal(signal.SIGINT);slam.wait(timeout=15);retired.add(slam);events.append(dict(t=t,event='slam_stopped',exit_code=slam.returncode))
            if case=='restart_occlusion' and t>48 and 'restart_slam' not in actions:
                actions.add('restart_slam');slam=start_slam('slam_restarted');events.append(dict(t=t,event='slam_new_process',pid=slam.pid))
            if case=='restart_occlusion' and t>51 and 'restart_hint' not in actions:actions.add('restart_hint');set_hint(restart_pose,'restart_hint',t)
            control=out/'client/runtime_request.json'
            if control.exists():
                req=read(control);key='control_'+req['id']
                if key not in actions:
                    actions.add(key);events.append(dict(t=t,event='runtime_control',request=req))
                    if req['operation']=='localization_loss':
                        source_process.send_signal(signal.SIGSTOP);source_restore_t=t+5.;events.append(dict(t=t,event='source_observations_paused',pid=source_process.pid))
                    elif req['operation']=='restart_nav':
                        if live.get('navigation_active') or not (live.get('tool') or {}).get('stopped_confirmed'):raise RuntimeError('RESTART_REQUIRES_NO_ACTIVE_TASK_AND_TOOL_STOPPED')
                        selected=[(c,p) for c,p in zip(commands,children) if c['role'] in ['nav_lifecycle','navigator','planner','controller'] and p not in retired]
                        for c,p in selected:
                            if p.poll() is None:p.send_signal(signal.SIGINT)
                        for c,p in selected:
                            try:p.wait(timeout=4)
                            except subprocess.TimeoutExpired:p.terminate();p.wait(timeout=3)
                            retired.add(p)
                        for c,p in selected:
                            args=[arg.replace('autostart:=true','autostart:=false') if c['role']=='nav_lifecycle' else arg for arg in c['args']]+(['-p',f's13_startup_sim_time_floor:={t:.9f}'] if c['role'] in ['controller','planner'] else [])
                            launch(c['role']+'_restarted',args)
                        dump(out/'client/runtime_reply.json',dict(id=req['id'],t=t,operation='new_nav_processes_started'));events.append(dict(t=t,event='nav_processes_restarted',startup_clock_floor=t))
            if 'source_restore_t' in locals() and t>=source_restore_t:
                source_process.send_signal(signal.SIGCONT);events.append(dict(t=t,event='source_observations_resumed'));del source_restore_t
            status=live['status']
            if not live.get('navigation_active',False) and (live.get('tool') or {}).get('stopped_confirmed') and not (live.get('tool') or {}).get('active') and status and status['confirmation_count']>=3 and status['plan_revalidation_required'] and t-last_ack>5:
                last_ack=t;launch('ack_'+str(int(t)),['ros2','service','call','/s08/acknowledge_diagnostic_hold','std_srvs/srv/Trigger','{}'],aux=True);events.append(dict(t=t,event='diagnostic_ack_request',purpose='No active work task; testing stopped re-admission only'))
            if gui and not shot and t>25 and live['pose']:result['rviz_capture']=baseline.s04.screenshot_rviz(out/'rviz.png');shot=True
            if (out/'client/result.json').exists() and t>read(out/'client/result.json')['t']+3:
                result['success']=read(out/'client/result.json')['completed'];result['sim_seconds_observed']=t;break
            if observer.poll() is not None:result['success']=observer.returncode==0 and (out/'client/result.json').exists();break
            if any(p.poll() is not None for p in children if p not in auxiliary and p not in retired and p is not driver and p is not client):raise RuntimeError('essential node exited')
            if driver.poll() not in [None,0]:raise RuntimeError('motion driver failed')
            if time.monotonic()-last_sample>1:
                last_sample=time.monotonic();owned={}
                for child in children:
                    try:
                        proc=psutil.Process(child.pid)
                        for q in [proc]+proc.children(recursive=True):owned[q.pid]=q
                    except psutil.Error:pass
                rss=0;cpu=0.
                for pid,q in owned.items():
                    try:rss+=q.memory_info().rss;ct=q.cpu_times();cpu+=ct.user+ct.system
                    except psutil.Error:pass
                resources.append(dict(t=t,wall=last_sample,owned_rss_bytes=rss,owned_cpu_seconds=cpu));dump(out/'live_resources.json',resources[-1])
                if rss>8589934592:raise RuntimeError('memory watchdog exceeded')
                if shutil.disk_usage(out).free<2*1024**3:raise RuntimeError('disk free space below 2 GiB')
            time.sleep(.05)
        else:raise TimeoutError('case wall watchdog')
    except BaseException as exc:result.update(error=repr(exc),interrupted=isinstance(exc,KeyboardInterrupt))
    finally:
        if 'source_process' in locals() and source_process.poll() is None:source_process.send_signal(signal.SIGCONT)
        # Stop the task producer before STOP_WORK, so shutdown cannot enqueue a new job.
        for descriptor,child in zip(commands,children):
            if descriptor['role']=='client' and child.poll() is None:
                child.send_signal(signal.SIGINT)
                try:child.wait(timeout=3)
                except subprocess.TimeoutExpired:child.terminate();child.wait(timeout=3)
        if 'driver' in locals() and driver.poll() is None:
            driver.send_signal(signal.SIGINT)
            try:driver.wait(timeout=3)
            except subprocess.TimeoutExpired:pass
        shutdown_path=out/'client/shutdown.json'
        result['tool_shutdown']=read(shutdown_path) if shutdown_path.exists() else dict(state='UNCONFIRMED',uncertainty_pending=True)
        # End the sole tool command owner before waiting for slow SLAM/database shutdown.
        # The Gazebo wall watchdog independently removes ON if the action acknowledgement was lost.
        for descriptor,child in zip(commands,children):
            if descriptor['role']=='tool_server' and child.poll() is None:
                child.send_signal(signal.SIGINT)
                try:child.wait(timeout=2)
                except subprocess.TimeoutExpired:child.terminate();child.wait(timeout=2)
        time.sleep(2.2)
        if 'slam' in locals() and slam.poll() is None:
            slam.send_signal(signal.SIGINT)
            try:slam.wait(timeout=20)
            except subprocess.TimeoutExpired:result['shutdown_error']='SLAM close timeout'
        baseline.s04.s03.stop_all(children)
        for f in logs:f.close()
        for c,p in zip(commands,children):c['exit_code']=p.returncode
        result.update(commands=commands,events=events,wall_seconds=time.monotonic()-start,database_before=db_before,database_after=digest(db));result['database_unchanged']=result['database_before']==result['database_after'];dump(out/'runtime.json',result);dump(out/'resources.json',resources)
    print(json.dumps({k:v for k,v in result.items() if k not in ['commands']},ensure_ascii=False),flush=True)
    if result.get('interrupted'):raise KeyboardInterrupt
    return result
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True);p.add_argument('--pose',default='zone_a');p.add_argument('--case',default='normal');p.add_argument('--seconds',type=float,default=65);p.add_argument('--wall',type=float,default=200);p.add_argument('--gui',action='store_true');p.add_argument('--vehicle',default='diff');p.add_argument('--controller');a=p.parse_args();raise SystemExit(0 if run_case(a.out,a.pose,a.case,a.seconds,a.wall,gui=a.gui,kind=a.vehicle,controller=a.controller)['success'] else 1)
