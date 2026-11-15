"""Near-ground fanout with full declared sensor mount rotation, including downward pitch."""
import os,sys,argparse,signal,copy,json
from pathlib import Path
from collections import deque
sys.path.insert(0,os.environ['R05_ADAPTER'])
from fanout import Fanout
from common import stamp,np
import rclpy
from sensor_msgs.msg import PointCloud2,PointField
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation,Slerp
from sensor_msgs_py.point_cloud2 import read_points_numpy
from rclpy.qos import qos_profile_sensor_data
class NearFanout(Fanout):
    def cloud(self,m):
        self.pl.publish(m);self.count+=1
        # Keep the full cloud for map/odometry. Terrain keeps the previously
        # qualified 64 x 512 far-field budget plus all independent near rays.
        rows=max(1,(m.height+63)//64);columns=max(2,(m.width+511)//512)
        out=copy.deepcopy(m);data=np.frombuffer(m.data,np.uint8).reshape(m.height,m.width,m.point_step);selected=np.ascontiguousarray(data[::rows,::columns,:]);out.data=selected.tobytes();out.height=selected.shape[0];out.width=selected.shape[1];out.row_step=out.width*out.point_step;self.pp.publish(out)

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path,required=True);a=ap.parse_args();a.out.mkdir(parents=True,exist_ok=True);rclpy.init(args=['--ros-args','-p','use_sim_time:=true']);f=NearFanout();n=f.n;n.destroy_publisher(f.pp);pub=n.create_publisher(PointCloud2,'/s04/perception/points',qos_profile_sensor_data);cfg=json.loads((a.out.parent/'mission.json').read_text());units=cfg['near_sensor']['units'];samples={u['name']:deque(maxlen=8) for u in units};near=samples['front'];queue=deque(maxlen=3);pending=deque(maxlen=3);stop=[False];rng=np.random.default_rng(41);log=(a.out/'near.jsonl').open('w');odoms=deque(maxlen=150)
    n.create_subscription(Odometry,'/s05/odometry',lambda m:odoms.append(m),50)
    def transform(t):
        if not odoms:return None
        times=np.array([stamp(m) for m in odoms]);j=int(np.searchsorted(times,t))
        if j==0 or j==len(times):
            m=odoms[0 if j==0 else -1]
            if abs(stamp(m)-t)>.03:return None
            p=m.pose.pose;xyz=np.array([p.position.x,p.position.y,p.position.z]);q=p.orientation;R=Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix()
        else:
            a0,b0=odoms[j-1],odoms[j];dt=stamp(b0)-stamp(a0)
            if dt<=0 or dt>.08:return None
            u=(t-stamp(a0))/dt;pa,pb=a0.pose.pose,b0.pose.pose;xyz=(1-u)*np.array([pa.position.x,pa.position.y,pa.position.z])+u*np.array([pb.position.x,pb.position.y,pb.position.z]);qs=[[q.x,q.y,q.z,q.w] for q in [pa.orientation,pb.orientation]];R=Slerp([0.,1.],Rotation.from_quat(qs))([u]).as_matrix()[0]
        return R,xyz
    class Queue:
        def publish(self,m):queue.append(m)
    f.pp=Queue()
    for unit in units:n.create_subscription(PointCloud2,'/r07/near_'+unit['name']+'/points',lambda m,k=unit['name']:samples[k].append(m),qos_profile_sensor_data)
    def tick():
        t=n.get_clock().now().nanoseconds*1e-9
        while near:
            other=near[0];capture=stamp(other)
            if t-capture<.04:break
            if t-capture>.3:near.popleft();continue
            tn=transform(capture)
            if tn is None:break
            near.popleft();parts=[];sources=[];target=tn[1]+tn[0]@np.array(f.c['lidar']['xyz'])
            for unit in units:
                candidates=[other] if unit['name']=='front' else samples[unit['name']]
                if not candidates:continue
                scan=min(candidates,key=lambda m:abs(stamp(m)-capture));delta=abs(stamp(scan)-capture);ts=transform(stamp(scan))
                if delta>.12 or ts is None:continue
                x=read_points_numpy(scan,field_names=['x','y','z'],skip_nans=True).reshape(-1,3);x=x[np.isfinite(x).all(axis=1)];x=x[rng.random(len(x))>=.02];rs=ts[0]@Rotation.from_euler('xyz',[0.,unit.get('pitch',0.),unit['yaw']]).as_matrix();origin=ts[1]+ts[0]@np.array(unit['xyz']);x=(x@rs.T+origin-target)@tn[0];parts.append(x);sources.append(dict(unit=unit['name'],capture_t=stamp(scan),points=len(x),pair_delta_s=delta))
            near_count=sum(len(x) for x in parts);dt=None
            if queue:
                m=min(queue,key=lambda m:abs(stamp(m)-capture));delta=abs(stamp(m)-capture);tm=transform(stamp(m))
                if delta<=.12 and tm is not None:
                    main=read_points_numpy(m,field_names=['x','y','z'],skip_nans=True).reshape(-1,3);main=main[np.isfinite(main).all(axis=1)];main=(main@tm[0].T+tm[1]+tm[0]@np.array(f.c['lidar']['xyz'])-target)@tn[0];parts.append(main);dt=delta
            if not parts:continue
            points=np.concatenate(parts).astype('<f4');out=PointCloud2();out.header=copy.deepcopy(other.header);out.header.frame_id='lidar_link';out.height=1;out.width=len(points);out.fields=[PointField(name=k,offset=i*4,datatype=PointField.FLOAT32,count=1) for i,k in enumerate(['x','y','z'])];out.point_step=12;out.row_step=12*len(points);out.is_dense=True;out.data=points.tobytes()
            pub.publish(out);log.write(json.dumps(dict(t=capture,near_points=near_count,total=out.width,delivery_t=t,pair_delta_s=dt,main_included=dt is not None,near_sources=sources,odom_motion_compensated=True,missing_far_area='UNOBSERVED'))+'\n');log.flush()
    n.create_timer(.02,tick)
    def end(s,f):stop[0]=True
    for s in [signal.SIGINT,signal.SIGTERM]:signal.signal(s,end)
    (a.out/'ready').write_text('real supplementary ray samples with noise/dropout/delay')
    try:
        while not stop[0]:rclpy.spin_once(n,timeout_sec=.005)
    finally:log.close();n.destroy_node();rclpy.try_shutdown()
if __name__=='__main__':main()
