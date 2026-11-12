"""Bounded owned child processes. Never kill unrelated Gazebo / ROS sessions."""
import json,os,re,signal,subprocess,sys,time
from pathlib import Path
from generate_scene import ROOT,BASE,generate
SCRIPTS=Path(__file__).resolve().parent
def gui_config(path,view):
    pose='80 6 160 0 1.267 1.5708' if view=='overview' else '18 17 7 0 .35 .65'
    near=2 if view=='overview' else .1
    hidden='<gz-gui><property key="state" type="string">floating</property><property key="width" type="double">5</property><property key="height" type="double">5</property><property key="showTitleBar" type="bool">false</property></gz-gui>'
    path.write_text(f'''<dialog name="quick_start" show_again="false"/><window><width>1200</width><height>800</height><dialog_on_exit>false</dialog_on_exit></window>
<plugin filename="MinimalScene" name="3D View"><gz-gui><property type="bool" key="showTitleBar">false</property><property type="string" key="state">docked</property></gz-gui><engine>ogre</engine><scene>scene</scene><ambient_light>.6 .6 .6</ambient_light><background_color>.68 .81 .91</background_color><camera_clip><near>{near}</near><far>500</far></camera_clip><camera_pose>{pose}</camera_pose></plugin>
<plugin filename="GzSceneManager" name="Scene Manager">{hidden}</plugin><plugin filename="InteractiveViewControl" name="View control">{hidden}</plugin><plugin filename="CameraTracking" name="Camera Tracking">{hidden}</plugin>
<plugin filename="WorldControl" name="World control"><gz-gui><property type="string" key="state">floating</property><property key="width" type="double">250</property><property key="height" type="double">70</property><property key="showTitleBar" type="bool">false</property><anchors target="3D View"><line own="left" target="left"/><line own="bottom" target="bottom"/></anchors></gz-gui><play_pause>true</play_pause><step>true</step><start_paused>true</start_paused><use_event>true</use_event></plugin>''')
def screenshot(path):
    from PIL import ImageGrab
    tree=subprocess.run(['xwininfo','-root','-tree'],capture_output=True,text=True,timeout=5).stdout
    lines=[l for l in tree.splitlines() if '"gz-sim-gui"' in l]
    if not lines:raise RuntimeError('Gazebo client window not found')
    win=re.search(r'0x[0-9a-f]+',lines[0]).group()
    s=subprocess.run(['xwininfo','-id',win],capture_output=True,text=True,timeout=5).stdout
    values=[int(re.search(k+r':\s*(-?\d+)',s).group(1)) for k in ['Absolute upper-left X','Absolute upper-left Y','Width','Height']]
    x,y,w,h=values
    # Client-only viewport: never capture a full desktop or unrelated application.
    ImageGrab.grab(bbox=(x+4,y+110,x+w-4,y+h-90)).save(path)
    return dict(window_id=win,client_geometry=values,capture='actual Gazebo viewport',wall_utc=time.time())
def run_case(out,seed=41,variant='base',seconds=18,wall=180,fixture=False,gui=False,view='overview'):
    out=Path(out);out.mkdir(parents=True,exist_ok=True);scene=out/'scene';info=generate(scene,seed,variant,fixture)
    env=os.environ.copy();env['GZ_PARTITION']='s02_'+out.parent.name+'_'+out.name
    env['GZ_SIM_SYSTEM_PLUGIN_PATH']=str(BASE/'ws/install/scenario_world/lib')+':'+env.get('GZ_SIM_SYSTEM_PLUGIN_PATH','')
    env['ROS_DOMAIN_ID']='119';children=[];logs=[];rows=[];start=time.monotonic()
    def launch(name,args,e=None):
        log=(out/(name+'.log')).open('w');logs.append(log)
        p=subprocess.Popen(args,stdout=log,stderr=subprocess.STDOUT,env=e or env,cwd=ROOT)
        children.append(p);rows.append(dict(name=name,pid=p.pid,argv=args,domain=(e or env)['ROS_DOMAIN_ID'],partition=env['GZ_PARTITION']));return p
    def cmd(args,timeout=7):return subprocess.run(args,env=env,capture_output=True,text=True,timeout=timeout)
    result=dict(scene=info,seed=seed,variant=variant,fixture=fixture,requested_sim_seconds=seconds,wall_timeout=wall,gui=gui,view=view,success=False)
    try:
        server=launch('server',['gz','sim','--force-version','8','-s','--seed',str(seed),info['world']])
        deadline=time.monotonic()+50
        while time.monotonic()<deadline:
            if server.poll() is not None:raise RuntimeError('Gazebo server exited')
            if '/world/irregular_farm/control' in cmd(['gz','service','-l']).stdout:break
            time.sleep(.4)
        else:raise TimeoutError('Gazebo world control service not ready')
        online=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock','/s02/probe/lidar/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked','/s02/probe/image@sensor_msgs/msg/Image[gz.msgs.Image']
        evaluation=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock','/s02/evaluation/dynamic_poses@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V','/s02/evaluation/contacts@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts']
        bridge='/opt/ros/jazzy/lib/ros_gz_bridge/parameter_bridge'
        launch('online_bridge',[bridge,*online])
        ev=env.copy();ev['ROS_DOMAIN_ID']='120';launch('evaluation_bridge',[bridge,*evaluation],ev)
        p1=launch('online',['/usr/bin/python3',str(SCRIPTS/'online_probe.py'),'--config',str(scene/'config/sensor_config.json'),'--out',str(out/'online'),'--seconds',str(seconds),'--wall',str(wall-10)])
        p2=launch('evaluation',['/usr/bin/python3',str(SCRIPTS/'evaluation_probe.py'),'--truth',str(scene/'evaluation/truth_manifest.json'),'--out',str(out/'evaluation'),'--seconds',str(seconds),'--wall',str(wall-10)],ev)
        deadline=time.monotonic()+20
        while time.monotonic()<deadline:
            if p1.poll() is not None or p2.poll() is not None:raise RuntimeError('probe initialization failed')
            if all((out/f'{n}/ready').exists() for n in ['online','evaluation']):break
            time.sleep(.2)
        else:raise TimeoutError('probe subscriptions not ready')
        gp=None
        if gui:
            gui_config(out/'gui.config',view)
            gp=launch('gui',['gz','sim','--force-version','8','-g','--render-engine-gui','ogre','--gui-config',str(out/'gui.config')])
        time.sleep(2)
        unpause=cmd(['gz','service','-s','/world/irregular_farm/control','--reqtype','gz.msgs.WorldControl','--reptype','gz.msgs.Boolean','--timeout','5000','--req','pause: false'])
        (out/'unpause.log').write_text(unpause.stdout+unpause.stderr)
        if unpause.returncode or 'true' not in unpause.stdout:raise RuntimeError('world did not unpause')
        (out/'runtime_topics.txt').write_text(cmd(['gz','topic','-l']).stdout)
        active=time.monotonic();shot=False
        while time.monotonic()-start<wall:
            if server.poll() is not None:raise RuntimeError('server failed during run')
            if gp and gp.poll() is not None:raise RuntimeError('GUI closed before bounded run completed')
            if gui and not shot and time.monotonic()-active>7:
                result['screenshot']=screenshot(out/'gazebo.png');shot=True
            if p1.poll() is not None and p2.poll() is not None:break
            time.sleep(.2)
        else:raise TimeoutError('simulation wall watchdog expired')
        result['observer_exit_codes']=[p1.returncode,p2.returncode]
        result['success']=p1.returncode==0 and p2.returncode==0
    except BaseException as exc:
        result['error']=repr(exc)
        if isinstance(exc,KeyboardInterrupt):result['interrupted']=True
    finally:
        for p in reversed(children):
            if p.poll() is None:p.send_signal(signal.SIGINT)
        deadline=time.monotonic()+6
        while time.monotonic()<deadline and any(p.poll() is None for p in children):time.sleep(.1)
        for p in children:
            if p.poll() is None:p.terminate()
        for p in children:
            try:p.wait(timeout=2)
            except subprocess.TimeoutExpired:p.kill();p.wait(timeout=2)
        for l in logs:l.close()
        for row,p in zip(rows,children):row['exit_code']=p.returncode
        result['wall_seconds']=time.monotonic()-start;result['commands']=rows
        (out/'runtime.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k not in ['commands','scene']},ensure_ascii=False),flush=True)
    if result.get('interrupted'):raise KeyboardInterrupt
    return result
if __name__=='__main__':
    import argparse
    p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True);p.add_argument('--seconds',type=float,default=18);p.add_argument('--fixture',action='store_true');p.add_argument('--gui',action='store_true');p.add_argument('--view',default='overview');a=p.parse_args()
    raise SystemExit(0 if run_case(a.out,seconds=a.seconds,fixture=a.fixture,gui=a.gui,view=a.view)['success'] else 1)
