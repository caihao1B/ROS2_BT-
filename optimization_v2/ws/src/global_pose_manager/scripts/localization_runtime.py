"""Bounded localization sessions; reuse S03/S04/S05 and S06 observation nodes.
The sensor/localization startup descriptors match the qualified S06 pipeline.
Only simulation composition can see fixture spawn; online nodes do not read it.
"""
import argparse,os,sys,signal,subprocess,time,copy
from xml.etree import ElementTree as E
import psutil,shutil
from s07_common import *
import mapping_runtime as baseline
SCRIPTS=BASE/'ws/src/localization_guard/scripts';MAPPING=BASE/'ws/src/map_manager/scripts';LOCAL=PKG/'scripts'
def compose(out,pose_name,session):
    baseline.compose(out,'diff','mapping_full',session)
    fixture=yaml.safe_load((BASE/'config/relocalization_cases.yaml').read_text())['poses'][pose_name]
    root=E.parse(out/'world.sdf').getroot();x,y,yaw=fixture['spawn'];root.find("world/model[@name='vehicle']/pose").text=f'{x} {y} .025 0 0 {yaw}';E.indent(root);E.ElementTree(root).write(out/'world.sdf',encoding='utf-8',xml_declaration=True)
    dump(out/'fixture.json',dict(pose_name=pose_name,spawn=fixture['spawn'],online_access=False))
    return fixture['operator_hint']
def run_case(out,pose_name,case,seconds,wall,seed=41,gui=False):
    out=Path(out);out.mkdir(parents=True,exist_ok=True);manifest=verify_bundle();request={k:manifest[k] for k in ['map_id','frame_epoch','revision','calibration_revision']};request['purpose']='DIAGNOSTIC_MOTION_FIXTURE';dump(out/'request.json',request);verify_bundle(request=request)
    dump(out/'capture_inputs.json',capture_inputs());db=BASE/'maps/frozen/map.db';db_before=digest(db);session=out.parent.name+'_'+out.name;hint=compose(out/'scene',pose_name,session)
    manager=yaml.safe_load((BASE/'config/map_contract.yaml').read_text())['manager'];manager.update(map_id=manifest['map_id'],diagnostic_fixture=True);dump(out/'manager.json',manager)
    env=os.environ.copy();env.update(ROS_DOMAIN_ID='119',GZ_PARTITION='s07_'+session);env['GZ_SIM_SYSTEM_PLUGIN_PATH']=':'.join(str(BASE/'ws/install'/p/'lib') for p in ['sensor_sim_adapter','chassis_bridge','scenario_world'])+':'+env.get('GZ_SIM_SYSTEM_PLUGIN_PATH','');raw=env.copy();raw['ROS_DOMAIN_ID']='121';evaluation=env.copy();evaluation['ROS_DOMAIN_ID']='120';children=[];logs=[];commands=[];retired=set();auxiliary=set();start=time.monotonic();resources=[];events=[];result=dict(case=case,pose_name=pose_name,seconds=seconds,seed=seed,success=False,gui=gui)
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
        launch('online_bridge',[bridge,'/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock','/s03/encoders@std_msgs/msg/String[gz.msgs.StringMsg','/s03/actuator/request@std_msgs/msg/String]gz.msgs.StringMsg','/s03/front_scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'])
        launch('raw_bridge',[bridge,'/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock','/s04/raw/lidar/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked','/s04/raw/left@sensor_msgs/msg/Image[gz.msgs.Image','/s04/raw/right@sensor_msgs/msg/Image[gz.msgs.Image','/s04/raw/imu@sensor_msgs/msg/Imu[gz.msgs.IMU','/s04/raw/gnss@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat','/s04/raw/joints@std_msgs/msg/String[gz.msgs.StringMsg'],raw)
        launch('evaluation_bridge',[bridge,'/s03/evaluation/pose@std_msgs/msg/String[gz.msgs.StringMsg','/s03/evaluation/contacts@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts'],evaluation)
        launch('evaluator',['/usr/bin/python3',str(BASE/'ws/src/chassis_bridge/scripts/evaluator.py'),'--out',str(out/'evaluation')],evaluation)
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
        launch('manager',[str(BASE/'ws/install/global_pose_manager/lib/global_pose_manager/global_pose_manager'),'--ros-args','-p','use_sim_time:=true','-p','policy_file:='+str(out/'manager.json')])
        observer=launch('observer',['/usr/bin/python3',str(LOCAL/'localization_observer.py'),'--out',str(out/'online'),'--seconds',str(seconds)]);driver=launch('motion',['/usr/bin/python3',str(LOCAL/'localization_motion.py'),'--out',str(out/'motion'),'--seconds',str(seconds),'--case',case])
        ready=['drive/bridge_ready','generation_motion/ready','generation_vision/ready','cloud/ready','icp_gate/ready','inputs/ready','odom_output/ready','mapping_inputs/ready','quality/ready','online/ready','motion/ready','evaluation/ready'];deadline=time.monotonic()+35
        while time.monotonic()<deadline:
            if any(p.poll() is not None for p in children):raise RuntimeError('essential node exited during startup')
            if all((out/f).exists() for f in ready):break
            time.sleep(.2)
        else:raise TimeoutError('nodes not ready')
        if gui:
            view=yaml.safe_load((BASE/'config/rviz/localization.rviz').read_text());view['Visualization Manager']['Views']['Current']['Focal Point']=dict(X=hint[0],Y=hint[1],Z=1.);rviz_config=out/'scene/localization.rviz';rviz_config.write_text(yaml.safe_dump(view,sort_keys=False));launch('rviz',['rviz2','-d',str(rviz_config),'--ros-args','-p','use_sim_time:=true'])
        time.sleep(2);u=gz(['service','-s','/world/irregular_farm/control','--reqtype','gz.msgs.WorldControl','--reptype','gz.msgs.Boolean','--timeout','4000','--req','pause: false'])
        if u.returncode or 'true' not in u.stdout:raise RuntimeError('unpause failed')
        actions=set();last_sample=0.;last_ack=-100.;restart_pose=None;procs={};shot=False
        while time.monotonic()-start<wall:
            live=read(out/'online/live.json') if (out/'online/live.json').exists() else dict(t=0,status=None,pose=None);t=live['t']
            if t>4 and 'initial' not in actions:
                actions.add('initial');set_hint(hint7([12.,12.,1.] if case=='wrong_initial' else hint),'initial_hint',t)
            if case=='wrong_initial' and t>30 and 'corrected' not in actions:actions.add('corrected');set_hint(hint7(hint),'corrected_coarse_hint',t)
            if case=='restart_occlusion' and t>40 and 'stop_slam' not in actions:
                actions.add('stop_slam');restart_pose=live['pose']['pose'] if live['pose'] else hint7(hint);slam.send_signal(signal.SIGINT);slam.wait(timeout=15);retired.add(slam);events.append(dict(t=t,event='slam_stopped',exit_code=slam.returncode))
            if case=='restart_occlusion' and t>48 and 'restart_slam' not in actions:
                actions.add('restart_slam');slam=start_slam('slam_restarted');events.append(dict(t=t,event='slam_new_process',pid=slam.pid))
            if case=='restart_occlusion' and t>51 and 'restart_hint' not in actions:actions.add('restart_hint');set_hint(restart_pose,'restart_hint',t)
            status=live['status']
            if status and status['state']=='LOCALIZED' and status['plan_revalidation_required'] and t-last_ack>5:
                last_ack=t;launch('ack_'+str(int(t)),['ros2','service','call','/s07/acknowledge_diagnostic_hold','std_srvs/srv/Trigger','{}'],aux=True);events.append(dict(t=t,event='diagnostic_ack_request',purpose='No active work task; testing stopped re-admission only'))
            if gui and not shot and t>25 and live['pose']:result['rviz_capture']=baseline.s04.screenshot_rviz(out/'rviz.png');shot=True
            if observer.poll() is not None:result['success']=observer.returncode==0;break
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
        if 'driver' in locals() and driver.poll() is None:
            driver.send_signal(signal.SIGINT)
            try:driver.wait(timeout=3)
            except subprocess.TimeoutExpired:pass
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
    p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True);p.add_argument('--pose',default='zone_a');p.add_argument('--case',default='normal');p.add_argument('--seconds',type=float,default=65);p.add_argument('--wall',type=float,default=200);p.add_argument('--gui',action='store_true');a=p.parse_args();raise SystemExit(0 if run_case(a.out,a.pose,a.case,a.seconds,a.wall,gui=a.gui)['success'] else 1)
