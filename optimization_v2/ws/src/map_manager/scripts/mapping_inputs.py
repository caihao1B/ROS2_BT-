"""Observed-only stereo/cloud synchronizer and conservative temporal map filter.
No world model, actor labels, GNSS or evaluation subscribers are permitted here.
"""
import argparse,signal,time
from collections import deque
import rclpy
from rclpy.qos import qos_profile_sensor_data,QoSProfile,DurabilityPolicy,ReliabilityPolicy
from sensor_msgs.msg import PointCloud2,Image,CameraInfo
from sensor_msgs_py import point_cloud2
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from rtabmap_msgs.msg import RGBDImage
from scipy.spatial import cKDTree
from s06_common import *
def temporal_filter(xyz,p,q,history,t):
    world=transform(xyz,p,q)
    # Preserve broad ground support. Two previous independent observations must
    # support above-ground surfaces within 0.25 m; this is not semantic removal.
    support=np.zeros(len(xyz),np.int8)
    for old_t,old in history:
        if .15<t-old_t<3.0 and len(old):support+=(cKDTree(old).query(world,k=1)[0]<.25)
    keep=(xyz[:,2]<.18)|(support>=2)
    history.append((t,world));return xyz[keep],world,keep
def main():
    ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path,required=True);a=ap.parse_args();a.out.mkdir(parents=True,exist_ok=True)
    rclpy.init();n=rclpy.create_node('s06_mapping_inputs');st=dict(stop=False,last=-1.,saved=-20.,packets=0)
    images={s:deque(maxlen=60) for s in ['left','right']};infos={};clouds=deque(maxlen=30);odoms=deque(maxlen=150);history=deque(maxlen=5)
    log=(a.out/'frames.jsonl').open('w');cp=n.create_publisher(PointCloud2,'/s06/mapping/cloud',qos_profile_sensor_data);ip=n.create_publisher(RGBDImage,'/s06/mapping/stereo',qos_profile_sensor_data);op=n.create_publisher(Odometry,'/s06/mapping/odom',10)
    calq=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    n.create_subscription(PointCloud2,'/s04/localization/points',lambda m:clouds.append(m),qos_profile_sensor_data);n.create_subscription(Odometry,'/s05/odometry',lambda m:odoms.append(m),50)
    for side in images:
        n.create_subscription(Image,'/s04/perception/'+side,lambda m,s=side:images[s].append(m),qos_profile_sensor_data)
        n.create_subscription(CameraInfo,f'/s04/stereo/{side}/camera_info',lambda m,s=side:infos.update({s:m}),calq)
    def process():
        if len(infos)<2 or not odoms or not all(images.values()):return
        while clouds:
            m=clouds[0];t=stamp(m)
            if t<5 or t-st['last']<.5-1e-4:clouds.popleft();continue
            if stamp(odoms[-1])<t+.08 or min(stamp(images[s][-1]) for s in images)<t+.07:return
            clouds.popleft();right_by_stamp={stamp(x):x for x in images['right']};paired=[x for x in images['left'] if stamp(x) in right_by_stamp]
            if not paired:continue
            left=min(paired,key=lambda x:abs(stamp(x)-t));right=right_by_stamp[stamp(left)];odom=min(odoms,key=lambda x:abs(stamp(x)-t));delta=max(abs(stamp(left)-t),abs(stamp(odom)-t))
            if delta>.04 or abs(stamp(right)-stamp(left))>1e-6:log.write(json.dumps(dict(t=t,rejected='SYNC_BOUND',delta=delta))+'\n');continue
            st['last']=t;begin=time.monotonic();p,q=pose(odom)
            xyz=point_cloud2.read_points_numpy(m,field_names=['x','y','z'],skip_nans=False).reshape(-1,3);ranges=np.linalg.norm(xyz,axis=1);xyz=xyz[np.isfinite(xyz).all(axis=1)&(ranges>1)&(ranges<35)]+[.1,0,.78]
            xyz=xyz[~((xyz[:,0]>-.65)&(xyz[:,0]<1.5)&(np.abs(xyz[:,1])<.65)&(xyz[:,2]<1.))]
            if len(xyz):_,ids=np.unique(np.floor(xyz/.18).astype(np.int32),axis=0,return_index=True);xyz=xyz[np.sort(ids)]
            if len(xyz)>6000:xyz=xyz[np.linspace(0,len(xyz)-1,6000,dtype=int)]
            filtered,world,keep=temporal_filter(xyz,p,q,history,t)
            if len(history)<3 or np.count_nonzero(filtered[:,2]>.18)<100:
                log.write(json.dumps(dict(t=t,rejected='TEMPORAL_WARMUP_OR_INSUFFICIENT_STABLE_3D',kept=len(filtered)))+'\n');continue
            h=Header();h.stamp=m.header.stamp;h.frame_id='base_link';cp.publish(point_cloud2.create_cloud_xyz32(h,filtered.astype('float32')))
            packet=RGBDImage();packet.header=left.header;packet.rgb=left;packet.depth=right;packet.rgb_camera_info=infos['left'];packet.depth_camera_info=infos['right'];ip.publish(packet);op.publish(odom);st['packets']+=1
            record=dict(t=t,image_t=stamp(left),odom_t=stamp(odom),pair_error_s=abs(stamp(left)-stamp(right)),max_sync_error_s=delta,raw=len(xyz),kept=len(filtered),rejected=len(xyz)-len(filtered),processing_s=time.monotonic()-begin,xyz=p.tolist(),q=q.tolist())
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
