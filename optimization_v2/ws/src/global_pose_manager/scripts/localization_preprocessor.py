"""S07 transport adapter derived from qualified S06 message assembly.
Shares the unchanged temporal_filter; publishes RGBD reliably without a relay.
The disclosed optical/lidar-cover fixture affects only this localization branch.
"""
import argparse,signal,time,copy
from collections import deque
import rclpy
from rclpy.qos import qos_profile_sensor_data,QoSProfile,DurabilityPolicy,ReliabilityPolicy
from sensor_msgs.msg import PointCloud2,Image,CameraInfo
from sensor_msgs_py import point_cloud2
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from std_msgs.msg import Header
from rtabmap_msgs.msg import RGBDImage
from s07_common import *
from mapping_inputs import temporal_filter
from scipy.spatial.transform import Slerp
def interpolate_odom(messages,t):
    before=[m for m in messages if stamp(m)<=t];after=[m for m in messages if stamp(m)>=t]
    if not before or not after:return None
    a,b=before[-1],after[0];ta,tb=stamp(a),stamp(b)
    if tb-ta>.10:return None
    out=copy.deepcopy(a);out.header.stamp.sec=int(t);out.header.stamp.nanosec=int(round((t-int(t))*1e9))
    if tb-ta<1e-9:return out
    ratio=(t-ta)/(tb-ta);pa,qa=pose(a);pb,qb=pose(b);xyz=pa*(1-ratio)+pb*ratio;q=Slerp([ta,tb],Rotation.from_quat([qa,qb]))([t]).as_quat()[0]
    out.pose.pose.position.x,out.pose.pose.position.y,out.pose.pose.position.z=xyz.tolist();out.pose.pose.orientation.x,out.pose.pose.orientation.y,out.pose.pose.orientation.z,out.pose.pose.orientation.w=q.tolist()
    out.pose.covariance=(np.array(a.pose.covariance)*(1-ratio)+np.array(b.pose.covariance)*ratio).tolist()
    return out
def main():
    ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path,required=True);ap.add_argument('--occlusion',action='store_true');a=ap.parse_args();a.out.mkdir(parents=True,exist_ok=True)
    rclpy.init();n=rclpy.create_node('s07_preprocessor');st=dict(stop=False,last=-1.,saved=-20.,packets=0)
    images={s:deque(maxlen=60) for s in ['left','right']};infos={};clouds=deque(maxlen=30);odoms=deque(maxlen=150);history=deque(maxlen=5)
    log=(a.out/'frames.jsonl').open('w');cp=n.create_publisher(PointCloud2,'/s07/inputs/cloud',10);ip=n.create_publisher(RGBDImage,'/s07/inputs/stereo',QoSProfile(depth=5,reliability=ReliabilityPolicy.RELIABLE));op=n.create_publisher(Odometry,'/s07/inputs/odom',10)
    tf_input=n.create_publisher(TFMessage,'/s07/internal/rtabmap_tf',100)
    def forward_local_tf(m):
        transforms=[tr for tr in m.transforms if tr.header.frame_id=='odom' and tr.child_frame_id=='base_link']
        if transforms:tf_input.publish(TFMessage(transforms=transforms))
    n.create_subscription(TFMessage,'/tf',forward_local_tf,100)
    calq=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    n.create_subscription(PointCloud2,'/s04/localization/points',lambda m:clouds.append(m),qos_profile_sensor_data);n.create_subscription(Odometry,'/s05/odometry',lambda m:odoms.append(m),50)
    for side in images:
        n.create_subscription(Image,'/s04/stereo/'+side+'/image_rect',lambda m,s=side:images[s].append(m),qos_profile_sensor_data)
        n.create_subscription(CameraInfo,f'/s04/stereo/{side}/camera_info',lambda m,s=side:infos.update({s:m}),calq)
    def process():
        if len(infos)<2 or not odoms or not all(images.values()):return
        while clouds:
            m=clouds[0];t=stamp(m)
            if t<5 or t-st['last']<.25-1e-4:clouds.popleft();continue
            if stamp(odoms[-1])<t+.08 or min(stamp(images[s][-1]) for s in images)<t+.07:return
            clouds.popleft();right_by_stamp={stamp(x):x for x in images['right']};paired=[x for x in images['left'] if stamp(x) in right_by_stamp]
            if not paired:continue
            left=min(paired,key=lambda x:abs(stamp(x)-t));right=right_by_stamp[stamp(left)];odom=min(odoms,key=lambda x:abs(stamp(x)-t));delta=max(abs(stamp(left)-t),abs(stamp(odom)-t))
            if delta>.04 or abs(stamp(right)-stamp(left))>1e-6:log.write(json.dumps(dict(t=t,rejected='SYNC_BOUND',delta=delta))+'\n');continue
            cloud_odom=interpolate_odom(odoms,t);aligned_odom=interpolate_odom(odoms,stamp(left))
            if cloud_odom is None or aligned_odom is None:continue
            st['last']=t;begin=time.monotonic();p,q=pose(cloud_odom);image_p,image_q=pose(aligned_odom)
            xyz=point_cloud2.read_points_numpy(m,field_names=['x','y','z'],skip_nans=False).reshape(-1,3);ranges=np.linalg.norm(xyz,axis=1);xyz=xyz[np.isfinite(xyz).all(axis=1)&(ranges>1)&(ranges<35)]+[.1,0,.78]
            xyz=xyz[~((xyz[:,0]>-.65)&(xyz[:,0]<1.5)&(np.abs(xyz[:,1])<.65)&(xyz[:,2]<1.))]
            if len(xyz):_,ids=np.unique(np.floor(xyz/.18).astype(np.int32),axis=0,return_index=True);xyz=xyz[np.sort(ids)]
            if len(xyz)>6000:xyz=xyz[np.linspace(0,len(xyz)-1,6000,dtype=int)]
            filtered,world,keep=temporal_filter(xyz,p,q,history,t)
            if len(history)<3 or np.count_nonzero(filtered[:,2]>.18)<100:
                log.write(json.dumps(dict(t=t,rejected='TEMPORAL_WARMUP_OR_INSUFFICIENT_STABLE_3D',kept=len(filtered)))+'\n');continue
            blocked=a.occlusion and 70<=t<85
            aligned=Rotation.from_quat(image_q).inv().apply(transform(filtered,p,q)-image_p)
            h=Header();h.stamp=left.header.stamp;h.frame_id='base_link';cp.publish(point_cloud2.create_cloud_xyz32(h,np.empty((0,3),np.float32) if blocked else aligned.astype('float32')))
            packet=RGBDImage();packet.header=left.header;packet.rgb=left;packet.depth=right;packet.rgb_camera_info=infos['left'];packet.depth_camera_info=infos['right']
            if blocked:
                packet.rgb=copy.deepcopy(left);packet.depth=copy.deepcopy(right);packet.rgb.data=bytes(len(packet.rgb.data));packet.depth.data=bytes(len(packet.depth.data))
            aligned_odom.header.stamp=left.header.stamp;ip.publish(packet);op.publish(aligned_odom);st['packets']+=1
            record=dict(t=t,output_epoch=stamp(left),alignment="Rigid whole-snapshot ego motion to original camera time; interpolated odometry, unchanged embedded image stamps",occluded=blocked,image_t=stamp(left),odom_t=stamp(odom),pair_error_s=abs(stamp(left)-stamp(right)),max_sync_error_s=delta,raw=len(xyz),kept=len(filtered),rejected=len(xyz)-len(filtered),processing_s=time.monotonic()-begin,xyz=p.tolist(),q=q.tolist())
            if t-st['saved']>=5:
                st['saved']=t;name=f'cloud_{t:.3f}.npz';np.savez_compressed(a.out/name,raw=xyz.astype('float32'),keep=keep,xyz=p,q=q,t=t);record['snapshot']=name
                if st['packets']<5:
                    from PIL import Image as PILImage
                    PILImage.fromarray(np.frombuffer(left.data,np.uint8).reshape(left.height,left.width,3)).save(a.out/'first_camera.png')
            log.write(json.dumps(record,separators=(',',':'))+'\n');log.flush();dump(a.out/'live.json',record)
    def end(s,f):st['stop']=True
    signal.signal(signal.SIGINT,end);signal.signal(signal.SIGTERM,end);(a.out/'ready').write_text('observations only')
    try:
        while not st['stop']:rclpy.spin_once(n,timeout_sec=.005);process()
    finally:log.close();dump(a.out/'result.json',dict(packets=st['packets'],last=st['last']));n.destroy_node();rclpy.try_shutdown()
if __name__=='__main__':main()
