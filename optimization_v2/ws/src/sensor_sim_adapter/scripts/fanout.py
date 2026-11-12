"""Online-only calibration, exact stereo pairing and deterministic branch decimation."""
import copy,json,math,signal
from collections import OrderedDict
import rclpy
from rclpy.qos import qos_profile_sensor_data,QoSProfile,DurabilityPolicy,ReliabilityPolicy
from sensor_msgs.msg import PointCloud2,Image,Imu,NavSatFix,CameraInfo
from geometry_msgs.msg import TwistWithCovarianceStamped,TransformStamped
from std_msgs.msg import String
from tf2_ros import StaticTransformBroadcaster
from common import *
class Fanout:
    def __init__(self):
        self.c=settings();self.n=rclpy.create_node('s04_sensor_fanout');self.count=0;self.left=OrderedDict();self.right=OrderedDict();self.stats=dict(paired=0,unpaired_dropped=0,max_pair_error=0.)
        q=QoSProfile(depth=20,reliability=ReliabilityPolicy.BEST_EFFORT);calq=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.info={s:self.n.create_publisher(CameraInfo,f'/s04/stereo/{s}/camera_info',calq) for s in ['left','right']}
        self.pl=self.n.create_publisher(PointCloud2,'/s04/localization/points',q);self.pp=self.n.create_publisher(PointCloud2,'/s04/perception/points',q)
        self.images={s:self.n.create_publisher(Image,'/s04/perception/'+s,q) for s in ['left','right']};self.pairs=self.n.create_publisher(String,'/s04/perception/pair',10)
        self.n.create_subscription(PointCloud2,'/s04/lidar/points',self.cloud,q)
        for s in ['left','right']:self.n.create_subscription(Image,f'/s04/stereo/{s}/image_rect',lambda m,s=s:self.image(m,s),q)
        for typ,topic,key in [(Imu,'imu/data','imu'),(TwistWithCovarianceStamped,'wheel/twist','wheel'),(NavSatFix,'gnss/fix','gnss')]:
            pub=self.n.create_publisher(typ,'/s04/localization/'+key,q);self.n.create_subscription(typ,'/s04/'+topic,lambda m,p=pub:p.publish(m),q)
        self.tf=StaticTransformBroadcaster(self.n);transforms=[]
        for child,xyz,rpy in [('lidar_link',self.c['lidar']['xyz'],[0,0,0]),('imu_link',self.c['imu']['xyz'],[0,0,0]),('gnss_link',self.c['gnss']['xyz'],[0,0,0])]+[(s+'_optical',[self.c['stereo']['xyz'][0],sign*self.c['stereo']['baseline_m']/2,self.c['stereo']['xyz'][2]],[-math.pi/2,0,-math.pi/2]) for s,sign in [('left',1),('right',-1)]]:
            tr=TransformStamped();tr.header.frame_id='base_link';tr.child_frame_id=child;tr.transform.translation.x,tr.transform.translation.y,tr.transform.translation.z=map(float,xyz)
            roll,pitch,yaw=rpy;cr,sr,cp,sp,cy,sy=math.cos(roll/2),math.sin(roll/2),math.cos(pitch/2),math.sin(pitch/2),math.cos(yaw/2),math.sin(yaw/2)
            qv=[sr*cp*cy-cr*sp*sy,cr*sp*cy+sr*cp*sy,cr*cp*sy-sr*sp*cy,cr*cp*cy+sr*sp*sy]
            tr.transform.rotation.x,tr.transform.rotation.y,tr.transform.rotation.z,tr.transform.rotation.w=qv;transforms.append(tr)
        self.tf.sendTransform(transforms)
    def cloud(self,m):
        self.pl.publish(m);self.count+=1
        if self.count%2:return
        # Preserve organized vertical rings and stamp; select alternate azimuth samples.
        out=copy.deepcopy(m);a=np.frombuffer(m.data,np.uint8).reshape(m.height,m.width,m.point_step);out.data=np.ascontiguousarray(a[:,::2,:]).tobytes();out.width=m.width//2;out.row_step=out.width*out.point_step;self.pp.publish(out)
    def image(self,m,side):
        c=self.c['stereo'];info=CameraInfo();info.header=copy.deepcopy(m.header);info.width=m.width;info.height=m.height;info.distortion_model='plumb_bob';info.d=[0.]*5
        fx=c['width']/(2*math.tan(c['hfov_rad']/2));cx=c['width']/2;cy=c['height']/2;info.k=[fx,0.,cx,0.,fx,cy,0.,0.,1.];info.r=[1.,0.,0.,0.,1.,0.,0.,0.,1.];info.p=[fx,0.,cx,-fx*c['baseline_m'] if side=='right' else 0.,0.,fx,cy,0.,0.,0.,1.,0.];self.info[side].publish(info)
        key=(m.header.stamp.sec,m.header.stamp.nanosec);mine=self.left if side=='left' else self.right;other=self.right if side=='left' else self.left;mine[key]=m
        if key in other:
            a=self.left.pop(key);b=self.right.pop(key);self.images['left'].publish(a);self.images['right'].publish(b);err=abs(stamp(a)-stamp(b));self.stats['paired']+=1;self.stats['max_pair_error']=max(self.stats['max_pair_error'],err)
            out=String();out.data=json.dumps(dict(stamp=stamp(a),right_stamp=stamp(b),error_s=err,**self.stats));self.pairs.publish(out)
        while len(mine)>self.c['branches']['stereo_pair_queue']:mine.popitem(last=False);self.stats['unpaired_dropped']+=1
def main():
    if __import__('os').environ.get('ROS_DOMAIN_ID')!='119':raise ValueError('online domain required')
    rclpy.init();f=Fanout();stop=[False]
    def end(s,fr):stop[0]=True
    signal.signal(signal.SIGINT,end);signal.signal(signal.SIGTERM,end)
    try:
        while not stop[0]:rclpy.spin_once(f.n,timeout_sec=.05)
    finally:f.n.destroy_node();rclpy.try_shutdown()
if __name__=='__main__':main()
