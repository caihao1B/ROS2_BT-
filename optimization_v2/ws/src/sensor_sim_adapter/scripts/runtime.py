"""Serial, owned, bounded simulations with separated raw/online/evaluation domains."""
import argparse,importlib.util,json,os,signal,subprocess,sys,time
import psutil
from common import *
from generate_world import generate
sys.path.insert(0,str(BASE/'ws/src/project_baseline/scripts'));from session_owner import identity
spec=importlib.util.spec_from_file_location('s03_runtime',BASE/'ws/src/chassis_bridge/scripts/runtime.py');s03=importlib.util.module_from_spec(spec);spec.loader.exec_module(s03)
SCRIPTS=PKG/'scripts'
def screenshot_rviz(path):
    import re
    from PIL import ImageGrab
    tree=subprocess.run(['xwininfo','-root','-tree'],capture_output=True,text=True,timeout=5).stdout
    matches=[line for line in tree.splitlines() if 'rviz' in line.lower() and '0x' in line]
    if not matches:raise RuntimeError('RViz window absent')
    win=re.search(r'0x[0-9a-f]+',matches[0]).group();s=subprocess.run(['xwininfo','-id',win],capture_output=True,text=True,timeout=5).stdout
    x,y,w,h=[int(re.search(k+r':\s*(-?\d+)',s).group(1)) for k in ['Absolute upper-left X','Absolute upper-left Y','Width','Height']]
    ImageGrab.grab(bbox=(x,y,x+w,y+h)).save(path);return dict(window_id=win,geometry=[x,y,w,h],capture='actual RViz client')
def run_case(out,kind,profile,seconds,wall,seed=41,gui=False):
    out=Path(out);out.mkdir(parents=True,exist_ok=True);session=out.parent.name+'_'+out.name;generate(out/'scene',kind,session);cfg=settings()
    env=os.environ.copy();env.update(ROS_DOMAIN_ID='119',GZ_PARTITION='s04_'+session);env['GZ_SIM_SYSTEM_PLUGIN_PATH']=':'.join(str(BASE/'ws/install'/p/'lib') for p in ['sensor_sim_adapter','chassis_bridge','scenario_world'])+':'+env.get('GZ_SIM_SYSTEM_PLUGIN_PATH','')
    raw=env.copy();raw['ROS_DOMAIN_ID']='121';evaluation=env.copy();evaluation['ROS_DOMAIN_ID']='120';children=[];logs=[];commands=[];start=time.monotonic();mon=[];result=dict(vehicle=kind,profile=profile,seconds=seconds,seed=seed,success=False,gui=gui,config_hash=config_hash())
    def launch(name,args,e=env):
        log=(out/(name+'.log')).open('w');logs.append(log);p=subprocess.Popen(args,cwd=ROOT,env=e,stdout=log,stderr=subprocess.STDOUT);children.append(p);commands.append(dict(role=name,args=args,domain=int(e['ROS_DOMAIN_ID']),partition=e['GZ_PARTITION'],**identity(p.pid)));return p
    def gz(args):return subprocess.run(['gz',*args],env=env,capture_output=True,text=True,timeout=7)
    try:
        server=launch('server',['gz','sim','--force-version','8','-s','--seed',str(seed),str(out/'scene/world.sdf')]);deadline=time.monotonic()+45
        while time.monotonic()<deadline:
            if server.poll() is not None:raise RuntimeError('Gazebo failed')
            if '/world/irregular_farm/control' in gz(['service','-l']).stdout:break
            time.sleep(.3)
        else:raise TimeoutError('world startup')
        bridge='/opt/ros/jazzy/lib/ros_gz_bridge/parameter_bridge'
        launch('online_bridge',[bridge,'/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock','/s03/encoders@std_msgs/msg/String[gz.msgs.StringMsg','/s03/actuator/request@std_msgs/msg/String]gz.msgs.StringMsg'])
        launch('raw_bridge',[bridge,'/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock','/s04/raw/lidar/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked','/s04/raw/left@sensor_msgs/msg/Image[gz.msgs.Image','/s04/raw/right@sensor_msgs/msg/Image[gz.msgs.Image','/s04/raw/imu@sensor_msgs/msg/Imu[gz.msgs.IMU','/s04/raw/gnss@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat','/s04/raw/joints@std_msgs/msg/String[gz.msgs.StringMsg'],raw)
        launch('evaluation_bridge',[bridge,'/s03/evaluation/pose@std_msgs/msg/String[gz.msgs.StringMsg','/s03/evaluation/contacts@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts'],evaluation)
        launch('evaluator',['/usr/bin/python3',str(BASE/'ws/src/chassis_bridge/scripts/evaluator.py'),'--out',str(out/'evaluation')],evaluation)
        drive=launch('drive',['/usr/bin/python3',str(BASE/'ws/src/chassis_bridge/scripts/drive_bridge.py'),'--config',str(out/'scene/vehicle.json'),'--out',str(out/'drive')])
        adapter=launch('generator',['/usr/bin/python3',str(SCRIPTS/'adapter.py'),'--vehicle',str(out/'scene/vehicle.json'),'--out',str(out/'generation'),'--seed',str(seed),'--profile',profile,'--streams','motion'],raw)
        vision=launch('vision_generator',['/usr/bin/python3',str(SCRIPTS/'adapter.py'),'--vehicle',str(out/'scene/vehicle.json'),'--out',str(out/'vision'),'--seed',str(seed),'--profile',profile,'--streams','vision'],raw)
        fanout=launch('fanout',['/usr/bin/python3',str(SCRIPTS/'fanout.py')])
        observer=launch('observer',['/usr/bin/python3',str(SCRIPTS/'observe.py'),'--out',str(out/'online'),'--seconds',str(seconds)])
        driver=launch('motion',['/usr/bin/python3',str(SCRIPTS/'motion.py'),'--out',str(out/'motion'),'--seconds',str(seconds)])
        ready=['drive/bridge_ready','generation/ready','vision/ready','online/ready','motion/ready','evaluation/ready'];deadline=time.monotonic()+25
        while time.monotonic()<deadline:
            if any(p.poll() is not None for p in [drive,adapter,vision,fanout,observer,driver]):raise RuntimeError('node startup failed')
            if all((out/f).exists() for f in ready):break
            time.sleep(.2)
        else:raise TimeoutError('nodes not ready')
        if gui:
            launch('rviz',['rviz2','-d',str(BASE/'config/rviz/sensors.rviz'),'--ros-args','-p','use_sim_time:=true'])
            launch('live_panel',['/usr/bin/python3',str(SCRIPTS/'live_panel.py'),'--online',str(out/'online')])
        time.sleep(1.5);u=gz(['service','-s','/world/irregular_farm/control','--reqtype','gz.msgs.WorldControl','--reptype','gz.msgs.Boolean','--timeout','4000','--req','pause: false'])
        if u.returncode or 'true' not in u.stdout:raise RuntimeError('unpause failed')
        last=0.;processes={};psutil.cpu_percent();shot=False;browser=False
        while time.monotonic()-start<wall:
            now=time.monotonic()
            if gui and not shot and now-start>18:result['rviz_capture']=screenshot_rviz(out/'rviz.png');shot=True
            if gui and shot and not browser and (out/'online/live_url.txt').exists():subprocess.run(['xdg-open',(out/'online/live_url.txt').read_text()],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL,timeout=5);browser=True
            if observer.poll() is not None:result['success']=observer.returncode==0;break
            if any(p.poll() is not None for p in [server,drive,adapter,vision,fanout]):raise RuntimeError('essential process exited')
            if now-last>1:
                last=now;owned={}
                for child in children:
                    try:
                        p=psutil.Process(child.pid)
                        for proc in [p]+p.children(recursive=True):owned[proc.pid]=proc
                    except psutil.Error:pass
                rss=0;cpu=0
                for pid,proc in owned.items():
                    try:
                        saved=processes.setdefault(pid,proc);rss+=saved.memory_info().rss;cpu+=saved.cpu_percent()
                    except psutil.Error:pass
                gpu=subprocess.run(['nvidia-smi','--query-gpu=utilization.gpu,memory.used','--format=csv,noheader,nounits'],capture_output=True,text=True,timeout=3)
                mon.append(dict(wall=now,elapsed_wall=now-start,owned_rss=rss,owned_cpu_one_core_percent=cpu,host_cpu_percent=psutil.cpu_percent(),gpu=gpu.stdout.strip()))
                if rss>cfg['limits']['owned_rss_bytes']:raise MemoryError('owned RSS exceeded configured ceiling')
                disk=sum(p.stat().st_size for p in out.rglob('*') if p.is_file())
                mon[-1]['case_disk_bytes']=disk
                if disk>cfg['limits']['case_disk_bytes']:raise RuntimeError('case disk watchdog exceeded configured ceiling')
            time.sleep(.05)
        else:raise TimeoutError('case wall timeout')
    except BaseException as exc:result.update(error=repr(exc),interrupted=isinstance(exc,KeyboardInterrupt))
    finally:
        if 'driver' in locals() and driver.poll() is None:
            driver.send_signal(signal.SIGINT)
            try:driver.wait(timeout=3)
            except subprocess.TimeoutExpired:pass
        time.sleep(2.2);s03.stop_all(children)
        for f in logs:f.close()
        for cmd,p in zip(commands,children):cmd['exit_code']=p.returncode
        result.update(commands=commands,wall_seconds=time.monotonic()-start);dump(out/'runtime.json',result);dump(out/'resources.json',mon)
    print(json.dumps({k:v for k,v in result.items() if k!='commands'},ensure_ascii=False),flush=True)
    if result.get('interrupted'):raise KeyboardInterrupt
    return result
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True);p.add_argument('--vehicle',default='diff');p.add_argument('--profile',default='nominal');p.add_argument('--seconds',type=float,default=25);p.add_argument('--wall',type=float,default=150);p.add_argument('--gui',action='store_true');a=p.parse_args();raise SystemExit(0 if run_case(a.out,a.vehicle,a.profile,a.seconds,a.wall,gui=a.gui)['success'] else 1)
