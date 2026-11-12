"""Bounded localization sessions; reuse S03/S04/S05 and S06 observation nodes.
The sensor/localization startup descriptors match the qualified S06 pipeline.
Only simulation composition can see fixture spawn; online nodes do not read it.
"""
import argparse,os,sys,signal,subprocess,time,copy
from xml.etree import ElementTree as E
import psutil,shutil
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
from scene10 import compose
S09=BASE/"ws/src/terrain_perception/scripts"
FUSION=BASE/"ws/src/object_fusion/scripts"
def capture09():
    paths=[S09/x for x in ['runtime.py','scene.py','common09.py','observer.py','motion.py','costmap_adapter.py','dynamic_evaluator.py']]+list((BASE/'ws/src/object_fusion/scripts').glob('*.py'))+list((BASE/'ws/src/terrain_perception/src').glob('*.cpp'))+list((BASE/'ws/src/terrain_perception/include').rglob('*.hpp'))+[BASE/'config'/x for x in ['traversability.yaml','costmap.yaml','perception_test_area.geojson','detector_lock.yaml','rviz/perception.rviz']]+[BASE/'ws/install/terrain_perception/lib/terrain_perception/terrain_node',BASE/'ws/install/terrain_perception/lib/libobserved_risk_layer.so',BASE/'ws/install/terrain_perception/lib/libs09_actor.so']+[p for p in (BASE/'assets/perception').rglob('*') if p.is_file()]+[p for p in (BASE/'ws/src/object_fusion/vendor').rglob('*') if p.is_file() and '__pycache__' not in p.parts]
    return {str(p.relative_to(ROOT)):digest(p) for p in paths}

def run_case(out,pose_name,case,seconds,wall,seed=41,gui=False,kind="diff"):
    out=Path(out);out.mkdir(parents=True,exist_ok=True);manifest=verify_bundle();request={k:manifest[k] for k in ['map_id','frame_epoch','revision','calibration_revision']};request['purpose']='DIAGNOSTIC_MOTION_FIXTURE';dump(out/'request.json',request);verify_bundle(request=request)
    dump(out/'capture_inputs.json',capture_inputs());dump(out/'s08_inputs.json',s08_inputs());db=BASE/'maps/frozen/map.db';db_before=digest(db);session=out.parent.name+'_'+out.name;hint=compose(out/'scene',kind,case,session);dump(out/'terrain.json',yaml.safe_load((BASE/'config/traversability.yaml').read_text()));dump(out/'s09_inputs.json',capture09())
    validate();manager=yaml.safe_load((BASE/'config/localization_supervisor.yaml').read_text());manager.update(map_id=manifest['map_id'],diagnostic_fixture=True);dump(out/'manager.json',manager)
    env=os.environ.copy();env.update(ROS_DOMAIN_ID='119',GZ_PARTITION='s10_'+session);env['GZ_SIM_SYSTEM_PLUGIN_PATH']=':'.join(str(BASE/'ws/install'/p/'lib') for p in ['sensor_sim_adapter','chassis_bridge','scenario_world','terrain_perception','tool_sim'])+':'+env.get('GZ_SIM_SYSTEM_PLUGIN_PATH','');raw=env.copy();raw['ROS_DOMAIN_ID']='121';evaluation=env.copy();evaluation['ROS_DOMAIN_ID']='120';children=[];logs=[];commands=[];retired=set();auxiliary=set();start=time.monotonic();resources=[];events=[];result=dict(case=case,pose_name=pose_name,seconds=seconds,seed=seed,success=False,gui=gui,vehicle=kind)
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
        launch('online_bridge',[bridge,'/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock','/s03/encoders@std_msgs/msg/String[gz.msgs.StringMsg','/s03/actuator/request@std_msgs/msg/String]gz.msgs.StringMsg','/s03/front_scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan','/s10/tool/command@std_msgs/msg/String]gz.msgs.StringMsg','/s10/tool/fault@std_msgs/msg/String]gz.msgs.StringMsg','/s10/tool/raw_feedback@std_msgs/msg/String[gz.msgs.StringMsg'])
        launch('raw_bridge',[bridge,'/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock','/s04/raw/lidar/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked','/s04/raw/left@sensor_msgs/msg/Image[gz.msgs.Image','/s04/raw/right@sensor_msgs/msg/Image[gz.msgs.Image','/s04/raw/imu@sensor_msgs/msg/Imu[gz.msgs.IMU','/s04/raw/gnss@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat','/s04/raw/joints@std_msgs/msg/String[gz.msgs.StringMsg'],raw)
        launch('evaluation_bridge',[bridge,'/s03/evaluation/pose@std_msgs/msg/String[gz.msgs.StringMsg','/s03/evaluation/contacts@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts','/s10/evaluation/effect@std_msgs/msg/String[gz.msgs.StringMsg'],evaluation)
        launch('tool_evaluator',[str(BASE/'venvs/scene/bin/python'),str(common10.PKG/'scripts/evaluator10.py'),'--out',str(out/'tool_evaluation'),'--config',str(out/'scene/tool.json')],evaluation);launch('evaluator',['/usr/bin/python3',str(BASE/'ws/src/chassis_bridge/scripts/evaluator.py'),'--out',str(out/'evaluation')],evaluation)
        drive=launch('drive',['/usr/bin/python3',str(SCRIPTS/'drive_adapter.py'),'--config',str(out/'scene/vehicle.json'),'--out',str(out/'drive')])
        for mode in ['motion','vision']:
            launch('generator_'+mode,['/usr/bin/python3',str(SCRIPTS/'sensors.py'),'--vehicle',str(out/'scene/vehicle.json'),'--out',str(out/('generation_'+mode)),'--seed',str(seed),'--streams',mode]+(['--faults'] if case=='faults' else ['--imu-only'] if case=='imu_loss' else []),raw)
        launch('fanout',['/usr/bin/python3',str(BASE/'ws/src/sensor_sim_adapter/scripts/fanout.py')])
        launch('cloud',['/usr/bin/python3',str(SCRIPTS/'cloud_filter.py'),'--out',str(out/'cloud')])
        launch('icp',['/opt/ros/jazzy/lib/rtabmap_odom/icp_odometry','--ros-args','-r','__node:=s05_icp','-r','__ns:=/s05/icp','--params-file',str(out/'scene/icp.yaml'),'-r','scan_cloud:=/s05/icp/cloud','-r','scan:=/s05/unused_scan','-r','odom:=/s05/icp/raw','-r','odom_info:=/s05/icp/info','-r','/tf:=/s05/internal/icp_tf','--log-level','warn'])
        launch('icp_gate',['/usr/bin/python3',str(SCRIPTS/'icp_gate.py'),'--out',str(out/'icp_gate')])
        launch('inputs',['/usr/bin/python3',str(SCRIPTS/'input_adapter.py'),'--out',str(out/'inputs')])
        for name,topic in [('fused','/s05/raw/fused'),('wheel_imu','/s05/raw/wheel_imu')]:launch(name,['/opt/ros/jazzy/lib/robot_localization/ekf_node','--ros-args','-r','__node:=s05_'+name,'-r','__ns:=/s05/'+name,'--params-file',str(out/('scene/'+name+'.yaml')),'-r','odometry/filtered:='+topic,'--log-level','warn']+['-r','/tf:=/s05/internal/'+name+'_tf'])
        launch('odom_output',['/usr/bin/python3',str(SCRIPTS/'odom_output.py'),'--out',str(out/'odom_output')])
        launch('health',[str(BASE/'ws/install/localization_guard/lib/localization_guard/health_guard'),'--ros-args','--params-file',str(out/'scene/health.yaml')])
        launch('mapping_inputs',['/usr/bin/python3',str(LOCAL/'localization_preprocessor.py'),'--out',str(out/'mapping_inputs')]+(['--occlusion'] if case=='restart_occlusion' else []))
        slam=start_slam('slam');launch('quality',['/usr/bin/python3',str(LOCAL/'localization_quality.py'),'--out',str(out/'quality'),'--request',str(out/'request.json')])
        launch('source_adapter',['/usr/bin/python3',str(S08/'source_adapter.py'),'--out',str(out/'sources'),'--case','normal']);launch('manager',[str(BASE/'ws/install/gnss_supervisor/lib/gnss_supervisor/gnss_supervisor'),'--ros-args','-p','use_sim_time:=true','-p','policy_file:='+str(out/'manager.json')])
        launch('terrain',[str(BASE/'ws/install/terrain_perception/lib/terrain_perception/terrain_node'),'--ros-args','-p','use_sim_time:=true','-p','config_file:='+str(out/'terrain.json')])
        launch('fusion',['/usr/bin/python3',str(FUSION/'fusion_node.py'),'--out',str(out/'fusion'),'--case',case])
        launch('cost_adapter',[str(BASE/'venvs/scene/bin/python'),str(S09/'costmap_adapter.py'),'--out',str(out/'cost_adapter'),'--vehicle',kind])
        costcfg=yaml.safe_load((BASE/'config/costmap.yaml').read_text())
        for chassis in ['diff','ackermann']:
            params=dict(use_sim_time=True,global_frame=costcfg['frame'],robot_base_frame='base_link',update_frequency=5.0,publish_frequency=5.0,rolling_window=True,width=int(costcfg['size_m']),height=int(costcfg['size_m']),resolution=float(costcfg['resolution_m']),origin_x=-15.,origin_y=-15.,track_unknown_space=True,always_send_full_costmap=True,footprint=str(yaml.safe_load((BASE/f'config/vehicles/{chassis}.yaml').read_text())['footprint']),plugins=['observed','inflation'],observed=dict(plugin='s09::ObservedRiskLayer',topic='/s09/risk/'+chassis,max_age=float(costcfg['stale_source_s'])),inflation=dict(plugin='nav2_costmap_2d::InflationLayer',inflation_radius=.8 if chassis=='diff' else 1.6,cost_scaling_factor=4.,inflate_unknown=False,inflate_around_unknown=False))
            config=out/f'nav2_{chassis}.yaml';config.write_text(yaml.safe_dump({'/**':{'ros__parameters':params}}));launch('nav2_'+chassis,['/opt/ros/jazzy/lib/nav2_costmap_2d/nav2_costmap_2d','--ros-args','-r','__ns:=/s09_nav2/'+chassis,'--params-file',str(config)])
        for name,script,python in [('ledger',common10.LEDGER/'scripts/ledger_node.py',str(BASE/'venvs/scene/bin/python')),('tool_online',common10.PKG/'scripts/online_adapter.py',str(BASE/'venvs/scene/bin/python')),('client',common10.PKG/'scripts/client10.py',str(BASE/'venvs/scene/bin/python'))]:
            launch(name,[python,str(script),'--out',str(out/name),'--config',str(out/'scene/tool.json')])
        launch('tool_server',[str(BASE/'ws/install/tool_sim/lib/tool_sim/tool_action_server'),'--ros-args','-p','config_file:='+str(out/'scene/tool.json')])
        observer=launch('observer',['/usr/bin/python3',str(common10.PKG/'scripts/observer10.py'),'--out',str(out/'online'),'--seconds',str(seconds)]);driver=launch('motion',['/usr/bin/python3',str(common10.PKG/'scripts/motion10.py'),'--out',str(out/'motion'),'--seconds',str(seconds),'--case',case])
        ready=['drive/bridge_ready','generation_motion/ready','generation_vision/ready','cloud/ready','icp_gate/ready','inputs/ready','odom_output/ready','mapping_inputs/ready','sources/ready','fusion/ready','cost_adapter/ready','quality/ready','online/ready','motion/ready','evaluation/ready','tool_evaluation/ready','tool_online/ready','ledger/ready','client/ready'];deadline=time.monotonic()+35
        while time.monotonic()<deadline:
            if any(p.poll() is not None for p in children):raise RuntimeError('essential node exited during startup')
            if all((out/f).exists() for f in ready):break
            time.sleep(.2)
        else:raise TimeoutError('nodes not ready')
        if gui:
            view=yaml.safe_load((BASE/'config/rviz/perception.rviz').read_text());view['Visualization Manager']['Views']['Current']['Focal Point']=dict(X=4.,Y=0.,Z=.5);rviz_config=out/'scene/localization.rviz';rviz_config.write_text(yaml.safe_dump(view,sort_keys=False));launch('rviz',['rviz2','-d',str(rviz_config),'--ros-args','-p','use_sim_time:=true'])
        time.sleep(2);u=gz(['service','-s','/world/irregular_farm/control','--reqtype','gz.msgs.WorldControl','--reptype','gz.msgs.Boolean','--timeout','4000','--req','pause: false'])
        if u.returncode or 'true' not in u.stdout:raise RuntimeError('unpause failed')
        actions=set();last_sample=0.;last_ack=-100.;restart_pose=None;procs={};shot=False
        while time.monotonic()-start<wall:
            live=read(out/'online/live.json') if (out/'online/live.json').exists() else dict(t=0,status=None,pose=None);t=live['t']
            if t>2 and 'nav2_configure' not in actions:
                actions.add('nav2_configure')
                for chassis in ['diff','ackermann']:launch('configure_'+chassis,['ros2','lifecycle','set','/s09_nav2/'+chassis+'/costmap','configure'],aux=True)
            if t>8 and 'nav2_activate' not in actions:
                actions.add('nav2_activate')
                for chassis in ['diff','ackermann']:launch('activate_'+chassis,['ros2','lifecycle','set','/s09_nav2/'+chassis+'/costmap','activate'],aux=True)
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
            status=live['status']
            if (live.get('tool') or {}).get('stopped_confirmed') and not (live.get('tool') or {}).get('active') and status and status['confirmation_count']>=3 and status['plan_revalidation_required'] and t-last_ack>5:
                last_ack=t;launch('ack_'+str(int(t)),['ros2','service','call','/s08/acknowledge_diagnostic_hold','std_srvs/srv/Trigger','{}'],aux=True);events.append(dict(t=t,event='diagnostic_ack_request',purpose='No active work task; testing stopped re-admission only'))
            if gui and not shot and t>25 and live['pose']:result['rviz_capture']=baseline.s04.screenshot_rviz(out/'rviz.png');shot=True
            if (out/'client/result.json').exists() and t>read(out/'client/result.json')['t']+3:
                result['success']=read(out/'client/result.json')['completed'];result['sim_seconds_observed']=t;break
            if observer.poll() is not None:result['success']=observer.returncode==0 and (out/'client/result.json').exists();break
            if any(p.poll() is not None for p in children if p not in auxiliary and p not in retired and p is not driver):raise RuntimeError('essential node exited')
            if driver.poll() not in [None,0]:raise RuntimeError('motion driver failed')
            if time.monotonic()-last_sample>1:
                last_sample=time.monotonic();owned={}
                for child in children:
                    try:
                        proc=psutil.Process(child.pid)
                        for q in [proc]+proc.children(recursive=True):owned[q.pid]=q
                    except psutil.Error:pass
                rss=0
                for pid,q in owned.items():
                    try:rss+=q.memory_info().rss
                    except psutil.Error:pass
                resources.append(dict(t=t,wall=last_sample,owned_rss_bytes=rss));dump(out/'live_resources.json',resources[-1])
                if rss>8589934592:raise RuntimeError('memory watchdog exceeded')
                if shutil.disk_usage(out).free<2*1024**3:raise RuntimeError('disk free space below 2 GiB')
            time.sleep(.05)
        else:raise TimeoutError('case wall watchdog')
    except BaseException as exc:result.update(error=repr(exc),interrupted=isinstance(exc,KeyboardInterrupt))
    finally:
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
    p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True);p.add_argument('--pose',default='zone_a');p.add_argument('--case',default='normal');p.add_argument('--seconds',type=float,default=65);p.add_argument('--wall',type=float,default=200);p.add_argument('--gui',action='store_true');p.add_argument('--vehicle',default='diff');a=p.parse_args();raise SystemExit(0 if run_case(a.out,a.pose,a.case,a.seconds,a.wall,gui=a.gui,kind=a.vehicle)['success'] else 1)
