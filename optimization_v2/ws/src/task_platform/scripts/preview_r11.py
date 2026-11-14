"""Paired unarmed real renderer benchmark, with bounded owned teardown."""
from common import *
import argparse,time,signal,shutil
from lifecycle_r1 import environment
from visual_r11 import camera

def capture(out):
 import rclpy,numpy as np
 from sensor_msgs.msg import Image
 from rclpy.qos import qos_profile_sensor_data
 from PIL import Image as PILImage
 out=Path(out);rclpy.init();n=rclpy.create_node('r11_parked_camera');frames=[];latest=None;begin=time.monotonic()
 def receive(m):
  nonlocal latest
  if m.encoding not in ['rgb8','bgr8']:return
  a=np.frombuffer(bytes(m.data),np.uint8).reshape(m.height,m.step)[:,:m.width*3].reshape(m.height,m.width,3)
  if m.encoding=='bgr8':a=a[:,:,::-1]
  latest=a.copy();frames.append((m.header.stamp.sec+m.header.stamp.nanosec*1e-9,time.monotonic()))
 n.create_subscription(Image,'/r12/observer/image',receive,qos_profile_sensor_data)
 while time.monotonic()-begin<70 and (not frames or frames[-1][0]<10):rclpy.spin_once(n,timeout_sec=.05)
 samples=[x for x in frames if x[0]>=2];passed=len(samples)>=30 and samples[-1][0]>=9.5
 if latest is not None:PILImage.fromarray(latest).save(out/'preview.png')
 result=dict(passed=passed,frames=len(frames),sample_frames=len(samples),source='actual Gazebo observer camera',autonomous_motion=False)
 if len(samples)>1:result.update(real_time_factor=(samples[-1][0]-samples[0][0])/(samples[-1][1]-samples[0][1]),received_wall_fps=(len(samples)-1)/(samples[-1][1]-samples[0][1]),first_sim=samples[0][0],last_sim=samples[-1][0])
 dump(out/'capture.json',result);n.destroy_node();rclpy.try_shutdown();return 0 if passed else 2

def run(task):
 lock=lock_session();base=APP/'results/R11'/task;results={}
 for variant in ['original','material']:
  out=base/variant;out.mkdir(parents=True,exist_ok=True);source=edition('R11')/task/('original/world.sdf' if variant=='original' else 'world.sdf');shutil.copyfile(source,out/'world.sdf');camera(out/'world.sdf',task)
  env=os.environ.copy();env.update(ROS_DOMAIN_ID='120',GZ_PARTITION='r11_'+task+'_'+variant+'_'+stamp(),GZ_SIM_SYSTEM_PLUGIN_PATH=':'.join([str(APP/'build/r09'),str(APP/'build/platform'),*[str(BASE/'ws/install'/p/'lib') for p in ['sensor_sim_adapter','chassis_bridge','scenario_world','motion_prediction']],env.get('GZ_SIM_SYSTEM_PLUGIN_PATH','')]))
  children=[];logs=[];error=None;forced=[];began=time.monotonic()
  def launch(role,args):
   f=(out/(role+'.log')).open('w');logs.append(f);p=subprocess.Popen(args,env=environment(role,env,out),cwd=ROOT,stdout=f,stderr=subprocess.STDOUT,start_new_session=True);children.append((role,p));return p
  def gz(args):return subprocess.run(['gz',*args],env=env,capture_output=True,text=True,timeout=7)
  try:
   server=launch('server',['gz','sim','--force-version','8','-s','-r','--headless-rendering','--seed','41',str(out/'world.sdf')]);bridge=launch('camera_bridge',['/opt/ros/jazzy/lib/ros_gz_bridge/parameter_bridge','/r12/observer/image@sensor_msgs/msg/Image[gz.msgs.Image']);rec=launch('capture',[str(BASE/'venvs/scene/bin/python'),str(Path(__file__).resolve()),'--capture',str(out)])
   while rec.poll() is None and time.monotonic()-began<85:
    if server.poll() is not None:raise RuntimeError('SERVER_EXITED')
    time.sleep(.1)
   if rec.poll()!=0:raise RuntimeError('CAPTURE_NOT_COMPLETED')
  except BaseException as ex:error=repr(ex)
  finally:
   try:gz(['service','-s','/world/irregular_farm/control','--reqtype','gz.msgs.WorldControl','--reptype','gz.msgs.Boolean','--timeout','2000','--req','pause: true'])
   except Exception:pass
   for role,p in reversed(children):
    if p.poll() is None:
     if role=='server':
      try:gz(['service','-s','/server_control','--reqtype','gz.msgs.ServerControl','--reptype','gz.msgs.Boolean','--timeout','2000','--req','stop: true'])
      except Exception:pass
     else:p.send_signal(signal.SIGINT)
     try:p.wait(timeout=12)
     except subprocess.TimeoutExpired:forced.append(role);p.kill();p.wait(timeout=3)
   for f in logs:f.close()
  cap=read(out/'capture.json') if (out/'capture.json').exists() else {}
  results[variant]=dict(passed=bool(cap.get('passed')) and not error and not forced and all(p.returncode==0 for _,p in children),capture=cap,error=error,forced=forced,process_exits={role:p.returncode for role,p in children},wall_seconds=time.monotonic()-began,scene_sha256=sha(source));dump(out/'result.json',results[variant])
 ratio=results['material'].get('capture',{}).get('real_time_factor',0)/max(results['original'].get('capture',{}).get('real_time_factor',0),1e-9);result=dict(task=task,passed=all(x['passed'] for x in results.values()) and ratio>=.8,variants=results,material_to_original_rtf_ratio=ratio,minimum_ratio=.8,material_contract=read(edition('R11')/task/'world.visual.json'),scope='Paired parked 10-sim-second render test; no autonomy equivalence claim');dump(base/'comparison.json',result);return result

if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--capture',type=Path);a=p.parse_args();raise SystemExit(capture(a.capture))
