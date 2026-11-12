"""Owned bounded S03 simulations. S02 source/assets stay immutable."""
import argparse,json,math,os,signal,subprocess,sys,time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[5];BASE=ROOT/'optimization_v2';SCRIPTS=Path(__file__).resolve().parent
sys.path.insert(0,str(BASE/'ws/src/robot_description/scripts'));from generate import world
sys.path.insert(0,str(BASE/'ws/src/project_baseline/scripts'));from session_owner import identity
def stop_all(children):
    for p in reversed(children):
        if p.poll() is None:p.send_signal(signal.SIGINT)
    end=time.monotonic()+5
    while time.monotonic()<end and any(p.poll() is None for p in children):time.sleep(.1)
    for p in children:
        if p.poll() is None:p.terminate()
    for p in children:
        try:p.wait(timeout=2)
        except subprocess.TimeoutExpired:p.kill();p.wait(timeout=2)
def gui_config(path,start):
    x,y,yaw=start;cx=x-5*math.cos(yaw)+3*math.sin(yaw);cy=y-5*math.sin(yaw)-3*math.cos(yaw)
    hidden='<gz-gui><property key="state" type="string">floating</property><property key="width" type="double">5</property><property key="height" type="double">5</property><property key="showTitleBar" type="bool">false</property></gz-gui>'
    path.write_text(f'''<dialog name="quick_start" show_again="false"/><window><width>1200</width><height>800</height><dialog_on_exit>false</dialog_on_exit></window>
<plugin filename="MinimalScene" name="3D View"><gz-gui><property type="bool" key="showTitleBar">false</property><property type="string" key="state">docked</property></gz-gui><engine>ogre</engine><scene>scene</scene><ambient_light>.6 .6 .6</ambient_light><background_color>.68 .81 .91</background_color><camera_clip><near>.1</near><far>500</far></camera_clip><camera_pose>{cx} {cy} 3.5 0 .45 {yaw+.45}</camera_pose></plugin>
<plugin filename="GzSceneManager" name="Scene Manager">{hidden}</plugin><plugin filename="InteractiveViewControl" name="View control">{hidden}</plugin><plugin filename="CameraTracking" name="Camera Tracking">{hidden}<follow_target>vehicle</follow_target><follow_offset>-5 -3 3.5</follow_offset><follow_pgain>.08</follow_pgain></plugin>''')
def screenshot(path):
    # Reuse tested viewport capture, without importing the unrelated S02 run executor.
    import importlib.util
    spec=importlib.util.spec_from_file_location('s02_capture',BASE/'ws/src/scenario_world/scripts/runtime.py');m=importlib.util.module_from_spec(spec)
    sys.path.insert(0,str(BASE/'ws/src/scenario_world/scripts'));spec.loader.exec_module(m);return m.screenshot(path)
def run_case(out,kind,case,seconds=300,wall=300,gui=False,rtf=2,seed=41):
    out=Path(out);out.mkdir(parents=True,exist_ok=True);route=BASE/'maps/prior/connector.geojson';points=json.loads(route.read_text())['features'][1]['geometry']['coordinates']
    start=[70,12,0]
    if case=='connector':start=[*points[0],math.atan2(points[1][1]-points[0][1],points[1][0]-points[0][0])]
    elif case=='slope':start=[37,63.5,0]
    session=out.parent.name+'_'+out.name;c=world(out/'scene',kind,session,start,rtf)
    env=os.environ.copy();env['GZ_PARTITION']='s03_'+session;env['ROS_DOMAIN_ID']='119';env['GZ_SIM_SYSTEM_PLUGIN_PATH']=str(BASE/'ws/install/chassis_bridge/lib')+':'+str(BASE/'ws/install/scenario_world/lib')+':'+env.get('GZ_SIM_SYSTEM_PLUGIN_PATH','')
    ev=env.copy();ev['ROS_DOMAIN_ID']='120';children=[];logs=[];commands=[];pids={};begin=time.monotonic();result=dict(vehicle=kind,case=case,seed=seed,rtf_target=rtf,requested_sim_seconds=seconds,wall_timeout=wall,gui=gui,success=False,config_hash=c['config_hash'],session=session)
    def launch(name,args,e=env):
        log=(out/(name+'.log')).open('w');logs.append(log);p=subprocess.Popen(args,cwd=ROOT,env=e,stdout=log,stderr=subprocess.STDOUT);children.append(p);pids[name]=identity(p.pid);commands.append(dict(name=name,argv=args,pid=p.pid,domain=e['ROS_DOMAIN_ID'],partition=e['GZ_PARTITION']));return p
    def gz(args):return subprocess.run(['gz',*args],env=env,capture_output=True,text=True,timeout=7)
    try:
        server=launch('server',['gz','sim','--force-version','8','-s','--seed',str(seed),str(out/'scene/world.sdf')])
        deadline=time.monotonic()+45
        while time.monotonic()<deadline:
            if server.poll() is not None:raise RuntimeError('server startup failure')
            if '/world/irregular_farm/control' in gz(['service','-l']).stdout:break
            time.sleep(.3)
        else:raise TimeoutError('world not ready')
        bridge='/opt/ros/jazzy/lib/ros_gz_bridge/parameter_bridge'
        launch('clock_bridge',[bridge,'/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock','/s03/front_scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'])
        launch('feedback_bridge',[bridge,'/s03/encoders@std_msgs/msg/String[gz.msgs.StringMsg'])
        launch('request_bridge',[bridge,'/s03/actuator/request@std_msgs/msg/String]gz.msgs.StringMsg'])
        launch('evaluation_bridge',[bridge,'/s03/evaluation/pose@std_msgs/msg/String[gz.msgs.StringMsg','/s03/evaluation/contacts@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts'],ev)
        evaluation=launch('evaluator',['/usr/bin/python3',str(SCRIPTS/'evaluator.py'),'--out',str(out/'evaluation')],ev)
        drive=launch('drive',['/usr/bin/python3',str(SCRIPTS/'drive_bridge.py'),'--config',str(out/'scene/vehicle.json'),'--out',str(out/'online')])
        (out/'owned_pids.json').write_text(json.dumps(pids,indent=2))
        driver=launch('driver',['/usr/bin/python3',str(SCRIPTS/'test_driver.py'),'--config',str(out/'scene/vehicle.json'),'--case',case,'--out',str(out/'online'),'--seconds',str(seconds),'--wall',str(wall-15),'--pids',str(out/'owned_pids.json'),'--route',str(route)])
        deadline=time.monotonic()+20
        while time.monotonic()<deadline:
            if driver.poll() is not None or drive.poll() is not None:raise RuntimeError('online node startup failed')
            if (out/'online/driver_ready').exists() and (out/'online/bridge_ready').exists() and (out/'evaluation/ready').exists():break
            time.sleep(.2)
        else:raise TimeoutError('online/evaluator startup timeout')
        gp=None
        if gui:
            gui_config(out/'gui.config',start);gp=launch('gui',['gz','sim','--force-version','8','-g','--render-engine-gui','ogre','--gui-config',str(out/'gui.config')])
        time.sleep(1.5);u=gz(['service','-s','/world/irregular_farm/control','--reqtype','gz.msgs.WorldControl','--reptype','gz.msgs.Boolean','--timeout','4000','--req','pause: false']);(out/'unpause.log').write_text(u.stdout+u.stderr)
        if u.returncode or 'true' not in u.stdout:raise RuntimeError('unpause failed')
        if gui:
            gz(['service','-s','/gui/follow','--reqtype','gz.msgs.StringMsg','--reptype','gz.msgs.Boolean','--timeout','2000','--req','data: "vehicle"'])
            gz(['service','-s','/gui/follow/offset','--reqtype','gz.msgs.Vector3d','--reptype','gz.msgs.Boolean','--timeout','2000','--req','x: -3 y: -2 z: 2'])
        shot=False;active=time.monotonic()
        while time.monotonic()-begin<wall:
            if server.poll() is not None:raise RuntimeError('server exited')
            if evaluation.poll() is not None:raise RuntimeError('evaluator exited')
            if drive.poll() is not None and case!='adapter_loss':raise RuntimeError('drive bridge exited')
            if gp and gp.poll() is not None:raise RuntimeError('GUI exited before completion')
            if gui and not shot and time.monotonic()-active>8:result['screenshot']=screenshot(out/'gazebo.png');shot=True
            if driver.poll() is not None:break
            time.sleep(.2)
        else:raise TimeoutError('case watchdog expired')
        result['driver_exit_code']=driver.returncode;result['success']=driver.returncode==0
    except BaseException as exc:
        result['error']=repr(exc);result['interrupted']=isinstance(exc,KeyboardInterrupt)
    finally:
        # Driver's signal handler sends STOP; keep bridge/physics alive for braking.
        if 'driver' in locals() and driver.poll() is None:
            driver.send_signal(signal.SIGINT)
            try:driver.wait(timeout=3)
            except subprocess.TimeoutExpired:pass
        time.sleep(2.2/max(1,rtf));stop_all(children)
        for f in logs:f.close()
        for row,p in zip(commands,children):row['exit_code']=p.returncode
        result['commands']=commands;result['wall_seconds']=time.monotonic()-begin;(out/'runtime.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k not in ['commands']},ensure_ascii=False),flush=True)
    if result.get('interrupted'):raise KeyboardInterrupt
    return result
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True);p.add_argument('--vehicle',choices=['diff','ackermann'],required=True);p.add_argument('--case',default='bench');p.add_argument('--seconds',type=float,default=300);p.add_argument('--wall',type=float,default=300);p.add_argument('--gui',action='store_true');a=p.parse_args()
    raise SystemExit(0 if run_case(a.out,a.vehicle,a.case,a.seconds,a.wall,a.gui)['success'] else 1)
