#!/usr/bin/env python3
"""Measured sensors and local estimate; truth is read only by this test."""
import argparse,collections,copy,datetime,hashlib,json,math,os,pathlib,signal,subprocess,sys,tarfile,time,threading
os.environ['OPENBLAS_NUM_THREADS']='1'
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data,QoSProfile,DurabilityPolicy,ReliabilityPolicy
from sensor_msgs.msg import LaserScan,Imu,NavSatFix
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer,TransformListener,TransformException
from gz.transport13 import Node as GNode
from gz.msgs10.pose_v_pb2 import Pose_V

P=pathlib.Path(__file__).resolve().parents[1];R=P.parents[2];G=P/'generated'
pa=argparse.ArgumentParser();pa.add_argument('output');pa.add_argument('--dual',action='store_true');pa.add_argument('--trajectory',action='store_true');args=pa.parse_args()
A=pathlib.Path(args.output);A.mkdir(parents=True,exist_ok=False)
os.environ['ROS_DOMAIN_ID']=str(120+os.getpid()%30);os.environ['GZ_PARTITION']='dino_sensors_'+str(os.getpid())
os.environ['LIBGL_ALWAYS_SOFTWARE']='1';os.environ['QT_QPA_PLATFORM']='xcb';os.environ['DISPLAY']=':0'
manifest={str(p.relative_to(P)):hashlib.sha256(p.read_bytes()).hexdigest() for p in P.rglob('*') if p.is_file() and '__pycache__' not in p.parts}
(A/'source_manifest.json').write_text(json.dumps(manifest,indent=2))
with tarfile.open(A/'source_snapshot.tar.gz','w:gz') as tf:
    for rel in manifest:tf.add(P/rel,arcname='dino_sensors/'+rel)
rows=(A/'samples.jsonl').open('w');record_lock=threading.Lock();commands=[];owned={};logs=[];start=time.monotonic();phase='startup';sim=0.;last={};counts=collections.Counter();tf_owners={};tf_checks=collections.Counter();pending=[];negative=[];subscriptions=[]
def record(kind,**data):
    with record_lock:rows.write(json.dumps({'wall':time.monotonic()-start,'sim':sim,'phase':phase,'kind':kind,**data},allow_nan=False)+'\n')
def launch(name,cmd):
    f=(A/(name+'.log')).open('w');logs.append(f);p=subprocess.Popen(cmd,cwd=R,stdout=f,stderr=subprocess.STDOUT,start_new_session=True);owned[name]=p;commands.append({'name':name,'pid':p.pid,'command':cmd,'started_utc':datetime.datetime.now(datetime.timezone.utc).isoformat()});return p
def stop(name):
    p=owned[name]
    if p.poll() is None:
        os.killpg(p.pid,signal.SIGINT)
        try:p.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(p.pid,signal.SIGTERM)
            try:p.wait(timeout=4)
            except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait(timeout=3)
    for c in commands:
        if c['pid']==p.pid:c['exit_code']=p.returncode
def qyaw(q):return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
def stamp(m):return m.header.stamp.sec+m.header.stamp.nanosec*1e-9
def clock_cb(m):
    global sim
    sim=m.clock.sec+1e-9*m.clock.nanosec
def scan_cb(key):
    def cb(m):
        last[key]=m;counts[key]+=1;record('scan',topic=key,t=stamp(m),frame=m.header.frame_id,angle_min=m.angle_min,angle_increment=m.angle_increment,range_min=m.range_min,range_max=m.range_max,
             scan_time=m.scan_time,time_increment=m.time_increment,ranges=[float(v) if math.isfinite(v) else None for v in m.ranges])
        if key.endswith('raw'):pending.append((m.header.frame_id,m.header.stamp,time.monotonic(),phase))
    return cb
def odom_cb(key):
    def cb(m):
        last[key]=m;counts[key]+=1;p=m.pose.pose
        record(key,t=stamp(m),frame=m.header.frame_id,child=m.child_frame_id,x=p.position.x,y=p.position.y,yaw=qyaw(p.orientation),v=m.twist.twist.linear.x,vy=m.twist.twist.linear.y,w=m.twist.twist.angular.z,cov=list(m.pose.covariance))
    return cb
def imu_cb(m):
    last['imu']=m;counts['imu']+=1;record('imu',t=stamp(m),frame=m.header.frame_id,omega=[m.angular_velocity.x,m.angular_velocity.y,m.angular_velocity.z],acc=[m.linear_acceleration.x,m.linear_acceleration.y,m.linear_acceleration.z],orientation=[m.orientation.x,m.orientation.y,m.orientation.z,m.orientation.w],cov=list(m.angular_velocity_covariance))
def pose_cb(m):
    for p in m.pose:
        if p.name=='dino':
            last['truth']=True;record('truth',t=p.header.stamp.sec+1e-9*p.header.stamp.nsec,x=p.position.x,y=p.position.y,z=p.position.z,yaw=qyaw(p.orientation))
def tf_cb(m,info):
    # Jazzy MessageInfo omits publisher GID. Record edges; attribute using graph
    # publishers plus the generated URDF/launch contract, never invent a GID.
    gid='observed_edge_no_gid'
    for tr in m.transforms:
        edge=tr.header.frame_id+'->'+tr.child_frame_id;tf_owners.setdefault(edge,set()).add(gid)
def negative_cb(m):negative.append(m);record('negative_status',**json.loads(m.data))
def gnss_cb(m):
    record('gnss',t=stamp(m),frame=m.header.frame_id,latitude=m.latitude,longitude=m.longitude,altitude=m.altitude,status=m.status.status,cov=list(m.position_covariance),covariance_type=m.position_covariance_type)
rclpy.init();n=Node('dino_sensor_evaluator',parameter_overrides=[Parameter('use_sim_time',value=True)]);buf=Buffer(node=n);listener=TransformListener(buf,n)
pub=n.create_publisher(Twist,'/dino/cmd_vel',1)
subscriptions.extend([n.create_subscription(Clock,'/clock',clock_cb,20),n.create_subscription(Imu,'/dino/imu',imu_cb,qos_profile_sensor_data),
    n.create_subscription(NavSatFix,'/dino/gnss/fix',gnss_cb,qos_profile_sensor_data),
    n.create_subscription(Odometry,'/dino/wheel/odometry',odom_cb('wheel'),20),n.create_subscription(Odometry,'/dino/odometry/filtered',odom_cb('fused'),20),
    n.create_subscription(String,'/dino/sensors/filter_status',lambda m:record('filter_status',**json.loads(m.data)),50),
    n.create_subscription(String,'/dino/estimator/status',lambda m:record('estimator_status',**json.loads(m.data)),50),
    n.create_subscription(TFMessage,'/tf',tf_cb,100),n.create_subscription(TFMessage,'/tf_static',tf_cb,QoSProfile(depth=10,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE))])
for k in ('front','rear') if args.dual else ('front',):
    for suffix,topic in [('raw','scan'),('filtered','scan_filtered')]:subscriptions.append(n.create_subscription(LaserScan,'/dino/lidar/'+k+'/'+topic,scan_cb(k+'_'+suffix),qos_profile_sensor_data))
gn=GNode();gn.subscribe(Pose_V,'/evaluation/dino/poses',pose_cb)
def spin():
    rclpy.spin_once(n,timeout_sec=.003)
    for frame,st,wall,origin_phase in list(pending):
        prefix='startup_' if origin_phase in ('startup','settle') else ''
        try:
            buf.lookup_transform('odom',frame,Time.from_msg(st));tf_checks[prefix+'stamped_lookup_pass']+=1;pending.remove((frame,st,wall,origin_phase))
        except TransformException as exc:
            if time.monotonic()-wall>1.5:
                tf_checks[prefix+'stamped_lookup_fail']+=1;pending.remove((frame,st,wall,origin_phase))
                record('tf_lookup_failure',origin_phase=origin_phase,frame=frame,stamp=st.sec+st.nanosec*1e-9,error=str(exc))
def motion(name,duration,v=0.,w=0.):
    global phase
    phase=name;record('phase_start',v=v,w=w,duration=duration);end=sim+duration;deadline=time.monotonic()+max(60,duration*15);lastsend=0.;lastprint=0.
    while sim<end:
        if time.monotonic()>deadline:raise TimeoutError(name)
        if time.monotonic()-lastsend>.02:
            msg=Twist();msg.linear.x=float(v);msg.angular.z=float(w);pub.publish(msg);lastsend=time.monotonic();record('command',v=v,w=w)
        spin()
        if time.monotonic()-lastprint>15:print(json.dumps({'phase':phase,'sim':sim,'wall':time.monotonic()-start,'counts':dict(counts)}),flush=True);lastprint=time.monotonic()
        if owned['launch'].poll() is not None:raise RuntimeError('launch exited')
    rows.flush()
result={'status':'FAIL','invocation':sys.argv,'dual':args.dual,'trajectory':args.trajectory,'started_utc':datetime.datetime.now(datetime.timezone.utc).isoformat()}
try:
    launch('launch',['ros2','launch','dino_sensors','sensors.launch.py','dual_lidar:='+str(args.dual).lower(),'validation_world:='+str(not args.trajectory).lower()])
    deadline=time.monotonic()+60
    while not all(k in last for k in ('truth','front_raw','front_filtered','wheel','imu','fused')):
        spin()
        if time.monotonic()>deadline:raise TimeoutError('missing startup: '+str(last.keys()))
    motion('settle',2);motion('static',6)
    motion('straight',10 if args.trajectory else 5,.5 if args.trajectory else .25,0);motion('stop',2)
    motion('left',16 if args.trajectory else 8,.5 if args.trajectory else .3,.125 if args.trajectory else .075);motion('stop_left',3)
    if args.trajectory:motion('right',16,.5,-.125);motion('stop_right',3)
    # Dedicated filter with no TF subscription. No real input or TF is modified.
    launch('negative_filter',['ros2','run','dino_sensors','scan_filter.py','--ros-args','-r','__node:=dino_no_tf_test','-p','use_sim_time:=true',
        '-p','generated_dir:='+str(G),'-p','raw_front:=/test/no_tf_raw','-p','filtered_front:=/test/no_tf_filtered',
        '-r','/tf:=/test/empty_tf','-r','/tf_static:=/test/empty_tf_static','-r','/dino/sensors/filter_status:=/test/no_tf_status'])
    testpub=n.create_publisher(LaserScan,'/test/no_tf_raw',1);invalid_outputs=[]
    subscriptions.extend([n.create_subscription(LaserScan,'/test/no_tf_filtered',lambda m:invalid_outputs.append(m),qos_profile_sensor_data),n.create_subscription(String,'/test/no_tf_status',negative_cb,20)])
    motion('negative_ready',2);m=copy.deepcopy(last['front_raw']);m.header.stamp=n.get_clock().now().to_msg();testpub.publish(m);motion('negative_missing_tf',2)
    m=copy.deepcopy(last['front_raw']);m.header.frame_id='wrong_frame';testpub.publish(m);motion('negative_wrong_frame',1)
    m=copy.deepcopy(last['front_raw']);m.header.stamp.sec=0;m.header.stamp.nanosec=0;testpub.publish(m);motion('negative_zero_stamp',.5)
    m=copy.deepcopy(last['front_raw']);m.time_increment=.0001;testpub.publish(m);motion('negative_rolling_scan',.5)
    reasons={json.loads(x.data)['reason'] for x in negative}
    result['negative_checks']={'no_fallback_output':len(invalid_outputs)==0,'drop_reported':bool(reasons & {'TF_MISSING_DROP','STALE_OR_FUTURE_SCAN'}),'wrong_frame_rejected':'FRAME_REJECTED' in reasons,'zero_stamp_rejected':'ZERO_STAMP_REJECTED' in reasons,'rolling_scan_not_falsely_accepted':'ROLLING_SCAN_REQUIRES_DESKEW' in reasons}
    graph={topic:[{'node':i.node_name,'type':i.topic_type} for i in n.get_publishers_info_by_topic(topic)] for topic in ['/tf','/tf_static','/dino/odometry/filtered','/dino/wheel/odometry','/dino/cmd_vel']}
    result['publisher_graph']=graph;result['tf_attribution_method']='Observed edges + publisher graph + generated URDF/launch contract; Jazzy callback has no publisher GID';result['status']='PASS' if all(result['negative_checks'].values()) else 'FAIL'
except BaseException as e:
    import traceback
    result['error']=str(e);result['traceback']=traceback.format_exc()
finally:
    phase='cleanup';pub.publish(Twist())
    for name in reversed(list(owned)):stop(name)
    for f in logs:f.close()
    result.update(sim_s=sim,wall_s=time.monotonic()-start,counts=dict(counts),tf_owners={k:list(v) for k,v in tf_owners.items()},tf_checks=dict(tf_checks))
    rows.flush();rows.close();n.destroy_node();rclpy.try_shutdown()
    (A/'execution.json').write_text(json.dumps(result,indent=2));(A/'commands.json').write_text(json.dumps({'commands':commands,'environment':{k:os.environ.get(k) for k in ['ROS_DOMAIN_ID','GZ_PARTITION','LIBGL_ALWAYS_SOFTWARE','DISPLAY','OPENBLAS_NUM_THREADS']},'exit_signal_note':'own process groups stopped with SIGINT after recording'},indent=2))
    print(json.dumps(result,indent=2),flush=True)
sys.exit(0 if result['status']=='PASS' else 1)
