"""Bounded, unarmed Gazebo preview. Observer images never enter autonomy."""
from common18 import *
import argparse, fcntl, subprocess, signal, time, math
from xml.etree import ElementTree as E
from visual18 import apply

def capture(path):
    import rclpy
    import numpy as np
    from sensor_msgs.msg import Image
    from rclpy.qos import qos_profile_sensor_data
    from PIL import Image as PILImage
    rclpy.init();node=rclpy.create_node('s18_preview_only');frames=[]
    def receive(m):
        if m.encoding not in ['rgb8','bgr8']:return
        a=np.frombuffer(bytes(m.data),np.uint8).reshape(m.height,m.step)[:,:m.width*3].reshape(m.height,m.width,3)
        if m.encoding=='bgr8':a=a[:,:,::-1]
        frames.append((a.copy(),m.header.stamp.sec+m.header.stamp.nanosec/1e9))
        del frames[:-3]
    node.create_subscription(Image,'/s18/preview/image',receive,qos_profile_sensor_data)
    until=time.monotonic()+60
    while time.monotonic()<until and (len(frames)<3 or frames[-1][1]<2.):rclpy.spin_once(node,timeout_sec=.1)
    node.destroy_node();rclpy.shutdown()
    if not frames:raise RuntimeError('NO_REAL_CAMERA_FRAME')
    PILImage.fromarray(frames[-1][0]).save(path)
    dump(path.with_suffix('.json'),dict(source='actual Gazebo observer camera',sim_time=frames[-1][1],
        image_sha256=sha(path),scope='UNARMED_PRESENTATION_ONLY',autonomy_input=False))

def preview(kind):
    lock=(ROOT/'native_20261108/runs/session.lock').open('a');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    sys.path.insert(0,str(BASE/'ws/src/robot_description/scripts'))
    from generate import world
    out=BASE/'showcase/presentation_v1'/kind;out.mkdir(parents=True,exist_ok=True)
    session='s18_preview_'+stamp();start=[126,60,0] if kind=='ackermann' else [29,27,0]
    world(out/'original',kind,session,start,1.)
    apply(out/'original/world.sdf',out/'skin.sdf')
    tree=E.parse(out/'skin.sdf');body=tree.find("world/model[@name='vehicle']/link[@name='base_link']")
    # A distinct observer sensor is added only to the preview fixture, after the
    # skin-only equality check. It is not part of the original sensor pipeline.
    body.append(E.fromstring('''<sensor name="s18_observer" type="camera"><pose>-2.6 -3.4 3.2 0 .55 .83</pose><update_rate>3</update_rate><always_on>true</always_on><topic>/s18/preview/image</topic><camera><horizontal_fov>1.05</horizontal_fov><image><width>1280</width><height>720</height><format>R8G8B8</format></image><clip><near>.1</near><far>180</far></clip></camera></sensor>'''))
    tree.write(out/'preview.sdf',encoding='utf-8',xml_declaration=True)
    env=os.environ.copy();env.update(GZ_PARTITION=session,ROS_DOMAIN_ID='120',OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1')
    env['GZ_SIM_SYSTEM_PLUGIN_PATH']=str(BASE/'ws/install/chassis_bridge/lib')+':'+str(BASE/'ws/install/scenario_world/lib')
    processes=[];logs=[];started=time.monotonic();error=None
    def launch(name,args):
        log=(out/(name+'.log')).open('w');logs.append(log)
        p=subprocess.Popen(args,env=env,cwd=ROOT,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        processes.append((name,p));return p
    def interrupted(*_):raise KeyboardInterrupt()
    old={s:signal.signal(s,interrupted) for s in [signal.SIGINT,signal.SIGTERM,signal.SIGALRM]}
    signal.setitimer(signal.ITIMER_REAL,85)
    try:
        server=launch('server',['gz','sim','--force-version','8','-s','-r','--headless-rendering','--seed','18141',str(out/'preview.sdf')])
        launch('bridge',['/opt/ros/jazzy/lib/ros_gz_bridge/parameter_bridge','/s18/preview/image@sensor_msgs/msg/Image[gz.msgs.Image'])
        recorder=launch('capture',['/usr/bin/python3',str(Path(__file__).resolve()),'--capture',str(out/'preview.png')])
        while recorder.poll() is None:
            if server.poll() is not None:raise RuntimeError('GAZEBO_EXIT_BEFORE_CAPTURE')
            time.sleep(.2)
        if recorder.returncode:raise RuntimeError('CAMERA_CAPTURE_FAILED')
    except (Exception,KeyboardInterrupt) as exc:error=repr(exc)
    finally:
        signal.setitimer(signal.ITIMER_REAL,0)
        for sig in old:signal.signal(sig,signal.SIG_IGN)
        for name,p in reversed(processes):
            # Every launched group is fresh and owned by this preview; no global
            # ROS/Gazebo process lookup or signaling is used.
            if p.poll() is None:os.killpg(p.pid,signal.SIGINT)
            try:p.wait(timeout=8)
            except subprocess.TimeoutExpired:
                try:os.killpg(p.pid,signal.SIGKILL)
                except ProcessLookupError:pass
                p.wait(timeout=5)
        for f in logs:f.close()
        for sig,handler in old.items():signal.signal(sig,handler)
    result=dict(vehicle=kind,status='PASS_STATIC_PREVIEW' if error is None else 'FAIL',error=error,
        wall_seconds=time.monotonic()-started,partition=session,
        process_exit_codes={name:p.returncode for name,p in processes},
        commanded_motion=False,autonomous_qualification='NOT_RUN',sensor_equivalence='NOT_QUALIFIED',
        skin_physics_unchanged=read(out/'skin.visual.json')['nonvisual_unchanged'],
        note='Parked actor fixture; observer camera is additional. Physics equality applies to skin.sdf only.')
    dump(out/'result.json',result);print(json.dumps(result,ensure_ascii=False));return 0 if error is None else 2

if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('--vehicle',choices=['diff','ackermann']);ap.add_argument('--capture',type=Path);a=ap.parse_args()
    if a.capture:capture(a.capture)
    elif a.vehicle:raise SystemExit(preview(a.vehicle))
    else:ap.error('--vehicle required')
