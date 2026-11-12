"""Online-only observation QA, bounded recording and observable health diagnostics."""
import argparse,copy,json,math,os,signal,time,queue,threading
from collections import Counter,deque
import numpy as np
import rclpy,rosbag2_py
from rclpy.serialization import serialize_message
from rclpy.qos import qos_profile_sensor_data,QoSProfile,DurabilityPolicy,ReliabilityPolicy
from sensor_msgs.msg import PointCloud2,Image,Imu,NavSatFix,CameraInfo,JointState
from sensor_msgs_py import point_cloud2
from geometry_msgs.msg import TwistWithCovarianceStamped
from rosgraph_msgs.msg import Clock
from diagnostic_msgs.msg import DiagnosticArray,DiagnosticStatus,KeyValue
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from PIL import Image as PILImage
from common import *
OBS={'points':(PointCloud2,'lidar/points'),'left':(Image,'stereo/left/image_rect'),'right':(Image,'stereo/right/image_rect'),'imu':(Imu,'imu/data'),'wheel':(TwistWithCovarianceStamped,'wheel/twist'),'joints':(JointState,'wheel/joint_states'),'gnss':(NavSatFix,'gnss/fix'),'left_info':(CameraInfo,'stereo/left/camera_info'),'right_info':(CameraInfo,'stereo/right/camera_info'),'quality':(String,'gnss/quality'),'pair':(String,'perception/pair'),'localization_points':(PointCloud2,'localization/points'),'perception_points':(PointCloud2,'perception/points')}
class Observe:
    def __init__(self,out,seconds):
        self.c=settings();self.out=out;out.mkdir(parents=True,exist_ok=True);self.n=rclpy.create_node('s04_observation_audit');self.t=0.;self.seconds=seconds;self.frames={};self.count=Counter();self.last={};self.stamps={k:[] for k in OBS};self.log=(out/'observations.jsonl').open('w');self.samples=Counter();self.info={};self.tf={};self.live={};self.recent=deque(maxlen=1200);self.last_png={};self.errors=[];self.epoch=0;self.last_gnss=None;self.gnss_jump_until=0.;self.diagnostics=[];self.writer=None;self.bag_bytes=0;self.bag_counts=Counter();self.bag_closed=False;self.cached_tf=None
        self.media_queue=queue.Queue(maxsize=4);self.media_thread=threading.Thread(target=self.media_worker,daemon=True);self.media_thread.start();self.media_drops=0;self.cloud_preview=[]
        self.n.create_subscription(Clock,'/clock',self.clock,qos_profile_sensor_data)
        calq=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        for key,(typ,topic) in OBS.items():
            qos=calq if key.endswith('info') else QoSProfile(depth=100 if key in ['imu','wheel','joints'] else 30 if 'points' in key else 10,reliability=ReliabilityPolicy.BEST_EFFORT)
            self.n.create_subscription(typ,'/s04/'+topic,lambda m,k=key:self.receive(k,m),qos)
        self.n.create_subscription(TFMessage,'/tf_static',self.transforms,calq)
        self.diag_pub=self.n.create_publisher(DiagnosticArray,'/s04/diagnostics',10)
        self.n.create_timer(.5,self.health)
        (out/'ready').write_text('online observations only')
    def media_worker(self):
        while True:
            job=self.media_queue.get()
            try:
                if job is None:return
                data,paths=job
                image=PILImage.fromarray(data)
                for path in paths:image.save(path,compress_level=1)
            finally:self.media_queue.task_done()
    def clock(self,m):
        t=m.clock.sec+m.clock.nanosec*1e-9
        if t<self.t:self.epoch+=1;self.last.clear()
        self.t=t
        if self.writer and t>self.c['limits']['record_start_s']+self.c['limits']['record_duration_s']+.5:self.close_bag()
    def transforms(self,m):
        self.cached_tf=m
        for t in m.transforms:self.tf[t.child_frame_id]=dict(parent=t.header.frame_id,xyz=[t.transform.translation.x,t.transform.translation.y,t.transform.translation.z],quaternion=[t.transform.rotation.x,t.transform.rotation.y,t.transform.rotation.z,t.transform.rotation.w])
    def open_bag(self):
        self.writer=rosbag2_py.SequentialWriter();self.writer.open(rosbag2_py.StorageOptions(uri=str(self.out/'sample_bag'),storage_id='sqlite3'),rosbag2_py.ConverterOptions('cdr','cdr'))
        for i,(key,(typ,topic)) in enumerate(OBS.items(),1):
            if key not in ['points','left','right','imu','wheel','gnss','left_info','right_info','joints']:continue
            typename=typ.__module__.split('.')[0]+'/msg/'+typ.__name__;self.writer.create_topic(rosbag2_py.TopicMetadata(id=i,name='/s04/'+topic,type=typename,serialization_format='cdr'))
        self.writer.create_topic(rosbag2_py.TopicMetadata(id=99,name='/tf_static',type='tf2_msgs/msg/TFMessage',serialization_format='cdr'))
        if self.cached_tf:
            data=serialize_message(self.cached_tf);self.writer.write('/tf_static',data,int(self.t*1e9));self.bag_bytes+=len(data)
    def close_bag(self):
        self.writer=None;self.bag_closed=True
    def record(self,key,msg,t):
        limits=self.c['limits']
        if self.bag_closed or not limits['record_start_s']<=t<=limits['record_start_s']+limits['record_duration_s'] or key not in ['points','left','right','imu','wheel','gnss','left_info','right_info','joints']:return
        if self.writer is None:self.open_bag()
        data=serialize_message(msg)
        if self.bag_bytes+len(data)>limits['record_bytes']-1048576:self.close_bag();return
        self.writer.write('/s04/'+OBS[key][1],data,int(t*1e9));self.bag_bytes+=len(data);self.bag_counts[key]+=1
    def receive(self,key,m):
        if key in ['quality','pair']:
            d=json.loads(m.data);t=d['stamp'];row=dict(t=t,key=key,arrival_sim=self.t,wall=time.monotonic(),**{k:v for k,v in d.items() if k!='stamp'})
        else:
            t=stamp(m);row=dict(t=t,key=key,arrival_sim=self.t,wall=time.monotonic(),frame=m.header.frame_id)
            self.frames.setdefault(key,set()).add(m.header.frame_id)
            if t<self.last.get(key,-1):self.errors.append('rewind '+key)
            self.record(key,m,t)
            if key in ['points','localization_points','perception_points']:
                row.update(height=m.height,width=m.width,point_step=m.point_step,size=len(m.data))
                capture=key=='points' and self.samples[key]<self.c['limits']['snapshots'] and t>5+25*self.samples[key]
                if key=='points' and (capture or self.count[key]%10==0):
                    xyz=point_cloud2.read_points_numpy(m,field_names=['x','y','z'],skip_nans=False).reshape(-1,3);valid=np.isfinite(xyz).all(axis=1);xyz=xyz[valid];angles=np.arctan2(xyz[:,2],np.hypot(xyz[:,0],xyz[:,1]));row.update(finite=len(xyz),vertical_bins=len(np.unique(np.round(angles,3))),z_span=float(np.ptp(xyz[:,2])))
                    self.cloud_preview=np.round(xyz[::max(1,len(xyz)//1800)],3).tolist()
                    if capture:np.savez_compressed(self.out/f'cloud_{self.samples[key]}.npz',xyz=xyz,t=t);self.samples[key]+=1
            elif key in ['left','right']:
                row.update(width=m.width,height=m.height,encoding=m.encoding,step=m.step)
                if t-self.last_png.get(key,-100)>1:
                    a=np.frombuffer(m.data,np.uint8).reshape(m.height,m.step)[:,:m.width*3].reshape(m.height,m.width,3);row['pixel_std']=float(a.std());paths=[self.out/(key+'_latest.png')];self.last_png[key]=t
                    if self.samples[key]<3 and t>5+25*self.samples[key]:paths.append(self.out/f'{key}_{self.samples[key]}.png');self.samples[key]+=1
                    try:self.media_queue.put_nowait((a.copy(),paths))
                    except queue.Full:self.media_drops+=1
            elif key=='imu':row.update(gyro=[m.angular_velocity.x,m.angular_velocity.y,m.angular_velocity.z],accel=[m.linear_acceleration.x,m.linear_acceleration.y,m.linear_acceleration.z],gyro_cov=list(m.angular_velocity_covariance),accel_cov=list(m.linear_acceleration_covariance),orientation_cov_0=m.orientation_covariance[0])
            elif key=='wheel':row.update(v=m.twist.twist.linear.x,w=m.twist.twist.angular.z,cov=list(m.twist.covariance))
            elif key=='joints':row.update(names=list(m.name),position=list(m.position),velocity=list(m.velocity))
            elif key=='gnss':
                enu=to_enu([m.latitude,m.longitude,m.altitude],self.c['gnss']['origin']);row.update(enu=enu.tolist(),status=int(m.status.status),cov=list(m.position_covariance))
                if self.last_gnss:
                    pt,pos=self.last_gnss;dt=t-pt
                    if 0<dt<.5 and np.linalg.norm(enu[:2]-pos[:2])>max(1.,3*dt):self.gnss_jump_until=self.t+3
                self.last_gnss=(t,enu)
            elif key.endswith('info'):
                row.update(k=list(m.k),p=list(m.p),r=list(m.r),d=list(m.d),width=m.width,height=m.height);self.info[key]=row
        self.last[key]=t;self.stamps[key].append(t);self.count[key]+=1
        self.live[key]=row;self.log.write(json.dumps(row,separators=(',',':'))+'\n')
        if key in ['gnss','wheel','imu'] and (key=='gnss' or self.count[key]%(20 if key=='imu' else 10)==0):self.recent.append(row)
    def health(self):
        reasons=[]
        for k,limit in [('points',.5),('left',.4),('right',.4),('imu',.1),('wheel',.1),('gnss',.7)]:
            if self.t-self.last.get(k,-100)>limit:reasons.append(k+'_STALE')
        if self.t<self.gnss_jump_until:reasons.append('GNSS_POSITION_JUMP')
        msg=DiagnosticArray();set_stamp(msg,self.t);d=DiagnosticStatus();d.name='s04/observation_health';d.hardware_id='simulated_sensor_rig';d.level=DiagnosticStatus.WARN if reasons else DiagnosticStatus.OK;d.message=','.join(reasons) if reasons else 'OBSERVATIONS_CURRENT'
        d.values=[KeyValue(key=k,value=str(self.count[k])) for k in OBS];msg.status=[d];self.diag_pub.publish(msg)
        self.diagnostics.append(dict(t=self.t,reasons=reasons));dump(self.out/'live.json',dict(t=self.t,diagnostics=reasons,counts=dict(self.count),latest=self.live,recent=list(self.recent),cloud=self.cloud_preview))
    def finish(self,complete):
        self.close_bag();self.media_queue.put(None);self.media_thread.join(timeout=5);self.log.close();graph={t:types for t,types in self.n.get_topic_names_and_types()}
        dump(self.out/'result.json',dict(completed=complete,counts=dict(self.count),frames={k:sorted(v) for k,v in self.frames.items()},graph=graph,subscriptions={t:[i.node_name for i in self.n.get_subscriptions_info_by_topic(t)] for t in graph},publishers={t:[i.node_name for i in self.n.get_publishers_info_by_topic(t)] for t in graph},calibration=self.info,tf_static=self.tf,errors=self.errors,diagnostics=self.diagnostics,bag_bytes=self.bag_bytes,bag_counts=dict(self.bag_counts),allowed_read_files=[str(BASE/'config/sensors.yaml')],domain=119))
        self.n.destroy_node()
def main():
    p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True);p.add_argument('--seconds',type=float,required=True);a=p.parse_args()
    if os.environ.get('ROS_DOMAIN_ID')!='119':raise ValueError('wrong online domain')
    rclpy.init();o=Observe(a.out,a.seconds);stop=[False]
    def end(s,f):stop[0]=True
    signal.signal(signal.SIGINT,end);signal.signal(signal.SIGTERM,end)
    try:
        while not stop[0] and o.t<a.seconds:rclpy.spin_once(o.n,timeout_sec=.01)
    finally:o.finish(not stop[0] and o.t>=a.seconds);rclpy.try_shutdown()
if __name__=='__main__':main()
