"""Real official-camera A/B exit probe. No robot commands or upstream writes."""
from common_r1 import *
import subprocess,time,signal,fcntl,argparse

def run(pin,mode='service',project=False,compat=False):
    lock=(ROOT/'native_20261108/runs/session.lock').open('a');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    out=APP/'results/R01'/('exit_'+('compat' if compat else 'pinned' if pin else 'original')+'_'+mode+('_project' if project else '_minimal')+'_'+stamp());out.mkdir(parents=True)
    library='/usr/lib/x86_64-linux-gnu/libboost_thread.so.1.83.0'
    env=os.environ.copy();env.pop('LD_PRELOAD',None);env['GZ_PARTITION']='r01_'+out.name
    env['GZ_SIM_SYSTEM_PLUGIN_PATH']=str(BASE/'ws/install/chassis_bridge/lib')+':'+str(BASE/'ws/install/scenario_world/lib')
    server_env=env.copy()
    if pin:server_env['LD_PRELOAD']=library
    if compat:server_env['LD_PRELOAD']=str(APP/'build/libr01_shutdown_compat.so')
    world=BASE/'showcase/presentation_v1/ackermann/preview.sdf' if project else PKG/'config/official_camera.sdf'
    world_name='irregular_farm' if project else 'r01_camera';topic='/s18/preview/image' if project else '/r01/camera'
    commands=[];started=time.monotonic();p=None;forced=False;error=None
    def call(args,timeout=8):
        r=subprocess.run(args,env=env,capture_output=True,text=True,timeout=timeout)
        commands.append(dict(argv=args,code=r.returncode,stdout=r.stdout if len(r.stdout)<20000 else '[camera payload omitted; sha256='+hashlib.sha256(r.stdout.encode()).hexdigest()+']',stderr=r.stderr));return r
    try:
        with (out/'server.log').open('w') as log:
            p=subprocess.Popen(['gz','sim','--force-version','8','-s','-r','--headless-rendering',str(world)],env=server_env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            deadline=time.monotonic()+20
            while time.monotonic()<deadline:
                if p.poll() is not None:raise RuntimeError('SERVER_START_FAILED')
                if '/world/'+world_name+'/control' in call(['gz','service','-l']).stdout:break
                time.sleep(.2)
            else:raise TimeoutError('WORLD_NOT_READY')
            clock=call(['gz','topic','-e','-t','/clock','-n','2','--json-output'])
            if clock.returncode or 'sim' not in clock.stdout:raise RuntimeError('NO_REAL_SIM_CLOCK')
            # Require actual rendered data, not only a registered sensor topic.
            camera=call(['gz','topic','-e','-t',topic,'-n','1','--json-output'])
            if camera.returncode or 'width' not in camera.stdout:raise RuntimeError('NO_REAL_CAMERA_FRAME')
            if mode=='service':
                r=call(['gz','service','-s','/server_control','--reqtype','gz.msgs.ServerControl','--reptype','gz.msgs.Boolean','--timeout','2000','--req','stop: true'])
                if r.returncode or 'true' not in r.stdout:raise RuntimeError('STOP_NOT_ACKNOWLEDGED')
            else:p.send_signal(signal.SIGINT)
            p.wait(timeout=12)
    except BaseException as exc:error=repr(exc)
    finally:
        if p and p.poll() is None:
            forced=True;os.killpg(p.pid,signal.SIGINT)
            try:p.wait(timeout=5)
            except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait(timeout=5)
    # Keep image sample in compact JSON, sufficient to establish actual rendering.
    dump(out/'commands.json',commands)
    result=dict(compat=compat,pinned=pin,mode=mode,project=project,library=library,library_sha256=sha(library),world_sha256=sha(world),
        server_exit=p.returncode if p else None,forced=forced,error=error,wall_s=time.monotonic()-started,
        passed=bool(p and p.returncode==0 and not forced and error is None))
    dump(out/'result.json',result);print(json.dumps(dict(out=str(out),**result)),flush=True);return result

if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('--pin',action='store_true');ap.add_argument('--compat',action='store_true');ap.add_argument('--project',action='store_true');ap.add_argument('--mode',choices=['service','signal'],default='service');a=ap.parse_args()
    raise SystemExit(0 if run(a.pin,a.mode,a.project,a.compat)['passed'] else 2)
