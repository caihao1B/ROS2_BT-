from common import *
import signal,time,math
from geometry_r02 import world
from lifecycle_r1 import environment

def phases(c):
 result=[dict(name='settle',mode='STOP',duration=2.,v=0.,w=0.)]
 k=c['steering']['max_curvature'] if c['kind']=='ackermann' else .8;v=.6 if c['kind']=='ackermann' else .4
 for name,speed,curve,duration in [('straight',.4,0,6),('left',v,k,32 if c['kind']=='ackermann' else 24),('right',v,-k,32 if c['kind']=='ackermann' else 24),('reverse',-.2,0,6),('spin',0,.5,5)]:
  result += [dict(name='arm_'+name,mode='ARM',duration=.7,v=0.,w=0.),dict(name=name,mode='DRIVE',duration=duration,v=speed,w=curve if name=='spin' else speed*curve),dict(name='stop_'+name,mode='STOP',duration=3.,v=0.,w=0.)]
 return result

def run(task,phase_list=None,stage='R02',rtf=1.,config=None):
 lock=lock_session();c=config or read(edition('R02')/'vehicles'/f'{task}.json');out=APP/'results'/stage/(task+'_'+stamp());out.mkdir(parents=True);session=out.name
 if canonical({k:v for k,v in c.items() if k!='config_hash'})!=c['config_hash']:raise ValueError('VEHICLE_HASH_MISMATCH')
 spec=dict(task=task,session=session,phases=phase_list or phases(c),wall_budget=240,vehicle_hash=c['config_hash']);dump(out/'spec.json',spec);world(out/'scene',c,session,rtf=rtf)
 env=os.environ.copy();env.update(GZ_PARTITION='r02_'+session,ROS_DOMAIN_ID='119',GZ_SIM_SYSTEM_PLUGIN_PATH=str(APP/'build/platform'))
 evaluator=env.copy();evaluator['ROS_DOMAIN_ID']='120';children=[];records=[];logs=[];error=None;start=time.monotonic();forced=[]
 def launch(role,args,e):
  log=(out/(role+'.log')).open('w');logs.append(log);p=subprocess.Popen(args,env=e,cwd=ROOT,stdout=log,stderr=subprocess.STDOUT,start_new_session=True);children.append(p);records.append(dict(role=role,pid=p.pid,args=args,domain=e['ROS_DOMAIN_ID'],partition=e['GZ_PARTITION']));return p
 def gz(args):
  return subprocess.run(['gz',*args],env=env,capture_output=True,text=True,timeout=6)
 server=None
 try:
  server=launch('server',['gz','sim','--force-version','8','-s',str(out/'scene/world.sdf')],environment('server',env,out))
  deadline=time.monotonic()+25
  while time.monotonic()<deadline:
   if server.poll() is not None:raise RuntimeError('SERVER_FAILED')
   if '/world/irregular_farm/control' in gz(['service','-l']).stdout:break
   time.sleep(.1)
  else:raise TimeoutError('WORLD_NOT_READY')
  bridge='/opt/ros/jazzy/lib/ros_gz_bridge/parameter_bridge'
  launch('online_bridge',[bridge,'/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock','/s03/encoders@std_msgs/msg/String[gz.msgs.StringMsg','/s03/actuator/request@std_msgs/msg/String]gz.msgs.StringMsg'],env)
  launch('evaluation_bridge',[bridge,'/s03/evaluation/pose@std_msgs/msg/String[gz.msgs.StringMsg','/s03/evaluation/contacts@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts'],evaluator)
  launch('evaluator',['/usr/bin/python3',str(BASE/'ws/src/chassis_bridge/scripts/evaluator.py'),'--out',str(out/'evaluation')],evaluator)
  driver=launch('driver',['/usr/bin/python3',str(PKG/'scripts/driver_r02.py'),'--out',str(out)],env)
  deadline=time.monotonic()+15
  while time.monotonic()<deadline:
   if (out/'driver_ready').exists() and (out/'evaluation/ready').exists():break
   time.sleep(.1)
  else:raise TimeoutError('DRIVER_NOT_READY')
  time.sleep(.5);r=gz(['service','-s','/world/irregular_farm/control','--reqtype','gz.msgs.WorldControl','--reptype','gz.msgs.Boolean','--timeout','3000','--req','pause: false'])
  if r.returncode or 'true' not in r.stdout:raise RuntimeError('UNPAUSE_FAILED')
  driver.wait(timeout=max(1,spec['wall_budget']-(time.monotonic()-start)-20))
  if driver.returncode:raise RuntimeError('DRIVER_DID_NOT_COMPLETE')
 except BaseException as exc:error=repr(exc)
 finally:
  # Stop producer first, let bounded wheel actuator brake before world shutdown.
  for rec,p in zip(records,children):
   if rec['role']=='driver' and p.poll() is None:
    p.send_signal(signal.SIGINT)
    try:p.wait(timeout=3)
    except subprocess.TimeoutExpired:forced.append('driver');p.terminate();p.wait(timeout=3)
  time.sleep(.5)
  for rec,p in zip(records,children):
   if rec['role']!='server' and p.poll() is None:p.send_signal(signal.SIGINT)
  for rec,p in zip(records,children):
   if rec['role']!='server':
    try:p.wait(timeout=5)
    except subprocess.TimeoutExpired:forced.append(rec['role']);p.kill();p.wait(timeout=3)
  if server and server.poll() is None:
   try:gz(['service','-s','/server_control','--reqtype','gz.msgs.ServerControl','--reptype','gz.msgs.Boolean','--timeout','2000','--req','stop: true']);server.wait(timeout=12)
   except (subprocess.TimeoutExpired,OSError):forced.append('server');server.send_signal(signal.SIGINT)
   if server.poll() is None:
    try:server.wait(timeout=3)
    except subprocess.TimeoutExpired:server.kill();server.wait(timeout=3)
  for f in logs:f.close()
  for rec,p in zip(records,children):rec['exit']=p.returncode
  dump(out/'runtime.json',dict(error=error,forced=forced,processes=records,wall_seconds=time.monotonic()-start,session=session,passed=not error and not forced and all(p.returncode==0 for p in children)))
 dump(APP/'results'/stage/(task+'_latest.json'),dict(path=str(out.relative_to(ROOT))));print(out,flush=True);return out
