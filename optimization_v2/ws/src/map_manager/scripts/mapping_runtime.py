"""Bounded S06 orchestration; reuse frozen S03/S04/S05 nodes and sensor generator."""
import argparse,copy,importlib.util,os,signal,subprocess,sys,time
from xml.etree import ElementTree as E
import psutil
from s06_common import *
import sys
sys.path.insert(0,str(BASE/"ws/src/localization_guard/scripts"))
from support import policy
from mapping_scene import compose
sys.path.insert(0,str(BASE/'ws/src/sensor_sim_adapter/scripts'))
from generate_world import generate
spec=importlib.util.spec_from_file_location('s04_runtime',BASE/'ws/src/sensor_sim_adapter/scripts/runtime.py');s04=importlib.util.module_from_spec(spec);spec.loader.exec_module(s04)
sys.path.insert(0,str(BASE/'ws/src/project_baseline/scripts'));from session_owner import identity
SCRIPTS=BASE/'ws/src/localization_guard/scripts'
MAPPING=PKG/'scripts'
def run_case(out,kind,case,seconds,wall,seed=41,gui=False):
    out=Path(out);out.mkdir(parents=True,exist_ok=True);session=out.parent.name+'_'+out.name;compose(out/'scene',kind,case,session);env=os.environ.copy();env.update(ROS_DOMAIN_ID='119',GZ_PARTITION='s06_'+session);env['GZ_SIM_SYSTEM_PLUGIN_PATH']=':'.join(str(BASE/'ws/install'/p/'lib') for p in ['sensor_sim_adapter','chassis_bridge','scenario_world'])+':'+env.get('GZ_SIM_SYSTEM_PLUGIN_PATH','');raw=env.copy();raw['ROS_DOMAIN_ID']='121';evaluation=env.copy();evaluation['ROS_DOMAIN_ID']='120';children=[];logs=[];commands=[];start=time.monotonic();resources=[];result=dict(vehicle=kind,case=case,seconds=seconds,seed=seed,success=False,gui=gui)
    dump(out/'capture_inputs.json',capture_inputs())
    def launch(name,args,e=env):
        log=(out/(name+'.log')).open('w');logs.append(log);p=subprocess.Popen(args,cwd=ROOT,env=e,stdout=log,stderr=subprocess.STDOUT);children.append(p);commands.append(dict(role=name,args=args,domain=int(e['ROS_DOMAIN_ID']),partition=e['GZ_PARTITION'],**identity(p.pid)));return p
    def gz(args):return subprocess.run(['gz',*args],env=env,capture_output=True,text=True,timeout=7)
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
        launch('mapping_inputs',['/usr/bin/python3',str(MAPPING/'mapping_inputs.py'),'--out',str(out/'mapping_inputs')])
        db=out/'mapping.db'
        slam=launch('slam',['/opt/ros/jazzy/lib/rtabmap_slam/rtabmap','--ros-args','-r','__node:=s06_slam','-r','__ns:=/s06/rtabmap','--params-file',str(BASE/'config/slam_mapping.yaml'),'-p','database_path:='+str(db),'-r','rgbd_image:=/s06/mapping/stereo','-r','scan_cloud:=/s06/mapping/cloud','-r','odom:=/s06/mapping/odom'])
        manager=launch('manager',[str(BASE/'ws/install/map_manager/lib/map_manager/mapping_manager'),'--ros-args','-p','use_sim_time:=true'])
        observer=launch('observer',['/usr/bin/python3',str(MAPPING/'mapping_observer.py'),'--out',str(out/'online'),'--seconds',str(seconds)])
        driver=launch('motion',['/usr/bin/python3',str(MAPPING/'mapping_motion.py'),'--out',str(out/'motion'),'--seconds',str(seconds),'--case',case])
        ready=['drive/bridge_ready','generation_motion/ready','generation_vision/ready','cloud/ready','icp_gate/ready','inputs/ready','odom_output/ready','online/ready','motion/ready','evaluation/ready','mapping_inputs/ready'];deadline=time.monotonic()+30
        while time.monotonic()<deadline:
            if any(p.poll() is not None for p in children):raise RuntimeError('essential node exited during startup')
            if all((out/f).exists() for f in ready):break
            time.sleep(.2)
        else:raise TimeoutError('nodes not ready')
        if gui:launch('rviz',['rviz2','-d',str(BASE/'config/rviz/mapping.rviz'),'--ros-args','-p','use_sim_time:=true'])
        time.sleep(2);u=gz(['service','-s','/world/irregular_farm/control','--reqtype','gz.msgs.WorldControl','--reptype','gz.msgs.Boolean','--timeout','4000','--req','pause: false'])
        if u.returncode or 'true' not in u.stdout:raise RuntimeError('unpause failed')
        last=0.;procs={};psutil.cpu_percent();shot=False;reset_sent=False;actions=set();service_calls=[];progress=None
        while time.monotonic()-start<wall:
            now=time.monotonic()
            if case=='reset' and not reset_sent and (out/'online/live.json').exists() and read(out/'online/live.json')['t']>16:
                reset_sent=True
                reset=gz(['service','-s','/world/irregular_farm/control','--reqtype','gz.msgs.WorldControl','--reptype','gz.msgs.Boolean','--timeout','4000','--req','reset: {all: true}'])
                result['clock_reset']={'returncode':reset.returncode,'response':reset.stdout}
                gz(['service','-s','/world/irregular_farm/control','--reqtype','gz.msgs.WorldControl','--reptype','gz.msgs.Boolean','--timeout','4000','--req','pause: false'])
            if (out/'online/live.json').exists():
                current=read(out/'online/live.json')['t']
                for at,action in [(25,'pause'),(32,'resume'),(45,'save'),(55,'resume')]:
                    if current>at and at not in actions:
                        actions.add(at)
                        service_calls.append(launch('service_'+str(at),['ros2','service','call','/s06/manager/'+action,'std_srvs/srv/Trigger','{}']))
            if gui and not shot and (out/'online/live.json').exists() and read(out/'online/live.json')['t']>40:result['rviz_capture']=s04.screenshot_rviz(out/'rviz.png');shot=True
            if observer.poll() is not None:result['success']=observer.returncode==0;break
            if driver.poll() not in [None,0]:raise RuntimeError('mapping route driver failed')
            if any(p.poll() is not None for p in children if p is not driver and p not in service_calls):raise RuntimeError('essential node exited')
            if now-last>1:
                last=now;owned={}
                if case=='mapping_full' and (out/'motion/live.json').exists():
                    live=read(out/'motion/live.json')
                    if live['odom'] and live['t']>65 and not live['goal']:
                        if progress is None or np.linalg.norm(np.array(live['odom'][:2])-progress[1])>.2:progress=(live['t'],np.array(live['odom'][:2]))
                        elif live['t']-progress[0]>30:raise RuntimeError('ROUTE_STALLED: '+live['reason'])
                for child in children:
                    try:
                        p=psutil.Process(child.pid)
                        for proc in [p]+p.children(recursive=True):owned[proc.pid]=proc
                    except psutil.Error:pass
                rss=0;cpu=0
                for pid,proc in owned.items():
                    try:saved=procs.setdefault(pid,proc);rss+=saved.memory_info().rss;cpu+=saved.cpu_percent()
                    except psutil.Error:pass
                gpu=subprocess.run(['nvidia-smi','--query-gpu=utilization.gpu,memory.used','--format=csv,noheader,nounits'],capture_output=True,text=True,timeout=3);disk=sum(p.stat().st_size for p in out.rglob('*') if p.is_file());resources.append(dict(wall=now,owned_rss=rss,owned_cpu_one_core_percent=cpu,host_cpu_percent=psutil.cpu_percent(),gpu=gpu.stdout.strip(),case_disk_bytes=disk))
                if rss>policy()['limits']['owned_rss_bytes'] or disk>1073741824:raise RuntimeError('resource watchdog exceeded')
            time.sleep(.05)
        else:raise TimeoutError('case wall watchdog')
    except BaseException as exc:result.update(error=repr(exc),interrupted=isinstance(exc,KeyboardInterrupt))
    finally:
        if 'driver' in locals() and driver.poll() is None:
            driver.send_signal(signal.SIGINT)
            try:driver.wait(timeout=3)
            except subprocess.TimeoutExpired:pass
        time.sleep(2.2)
        # Pause and flush RTAB-Map before taking down sensor/clock processes.
        if 'slam' in locals() and slam.poll() is None:
            for action in ['pause','backup']:
                try:
                    p=subprocess.run(['timeout','20','ros2','service','call','/s06/rtabmap/s06_slam/'+action,'std_srvs/srv/Empty','{}'],env=env,capture_output=True,text=True,timeout=22)
                    (out/('final_'+action+'.log')).write_text(p.stdout+p.stderr)
                except Exception as exc:result['save_error']=repr(exc)
            slam.send_signal(signal.SIGINT)
            try:slam.wait(timeout=20)
            except subprocess.TimeoutExpired:result['save_error']='SLAM close timeout'
        s04.s03.stop_all(children)
        for f in logs:f.close()
        for cmd,p in zip(commands,children):cmd['exit_code']=p.returncode
        result.update(commands=commands,wall_seconds=time.monotonic()-start);dump(out/'runtime.json',result);dump(out/'resources.json',resources)
    print(json.dumps({k:v for k,v in result.items() if k!='commands'},ensure_ascii=False),flush=True)
    if result.get('interrupted'):raise KeyboardInterrupt
    return result
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True);p.add_argument('--vehicle',default='diff');p.add_argument('--case',default='mapping_loop');p.add_argument('--seconds',type=float,default=40);p.add_argument('--wall',type=float,default=180);p.add_argument('--gui',action='store_true');a=p.parse_args();raise SystemExit(0 if run_case(a.out,a.vehicle,a.case,a.seconds,a.wall,gui=a.gui)['success'] else 1)
