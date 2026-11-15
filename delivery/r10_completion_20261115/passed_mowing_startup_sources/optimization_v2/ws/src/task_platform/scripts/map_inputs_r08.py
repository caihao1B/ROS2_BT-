from rig_r08 import runtime_rig
"""Lidar-map branch with actual cloud-time odometry; camera keeps its own branch."""
import argparse,signal,time,sys,copy,os,json,importlib.util
from pathlib import Path
from collections import deque
from accumulation_r08 import SurfaceWindow
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from rosgraph_msgs.msg import Clock
from common import BASE,read,ROOT
sys.path.insert(0,str(BASE/'ws/src/global_pose_manager/scripts'))
import localization_preprocessor as upstream

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path,required=True);a=ap.parse_args();a.out.mkdir(parents=True,exist_ok=True);binding=read(Path(os.environ['R05_BINDING']));rig=runtime_rig(a.out,read(ROOT/binding['rig_path']));rclpy.init();n=rclpy.create_node('r08_lidar_map_inputs',parameter_overrides=[rclpy.parameter.Parameter('use_sim_time',value=True)]);clouds=deque(maxlen=4);odoms=deque(maxlen=150);history=deque(maxlen=5);window=SurfaceWindow();st=dict(t=0.,last=-100.,saved=-100.,stop=False);log=(a.out/'frames.jsonl').open('w');cp=n.create_publisher(PointCloud2,'/s07/inputs/cloud',5);op=n.create_publisher(Odometry,'/s07/inputs/odom',10)
    n.create_subscription(Clock,'/clock',lambda m:st.update(t=m.clock.sec+m.clock.nanosec*1e-9),qos_profile_sensor_data)
    n.create_subscription(PointCloud2,'/s04/localization/points',lambda m:clouds.append(m),qos_profile_sensor_data)
    n.create_subscription(Odometry,'/s05/odometry',lambda m:odoms.append(m),50)
    def process():
        while clouds and odoms:
            m=clouds[0];t=upstream.stamp(m)
            if t<5 or t-st['last']<.2-1e-4 or st['t']-t>.4:clouds.popleft();continue
            if upstream.stamp(odoms[-1])<t:return
            clouds.popleft();o=upstream.interpolate_odom(odoms,t)
            if o is None:continue
            st['last']=t;begin=time.monotonic();p,q=upstream.pose(o);xyz=point_cloud2.read_points_numpy(m,field_names=['x','y','z'],skip_nans=False).reshape(-1,3);ranges=np.linalg.norm(xyz,axis=1);xyz=xyz[np.isfinite(xyz).all(axis=1)&(ranges>1)&(ranges<48)]+rig['lidar']['xyz'];mask=rig['self_filter'];xyz=xyz[~((xyz[:,0]>mask['x_min'])&(xyz[:,0]<mask['x_max'])&(xyz[:,1]>mask['y_min'])&(xyz[:,1]<mask['y_max'])&(xyz[:,2]<mask['z_max']))]
            if len(xyz):_,ids=np.unique(np.floor(xyz/.08).astype(np.int32),axis=0,return_index=True);xyz=xyz[np.sort(ids)]
            ground=np.abs(xyz[:,2])<.18;elevated=np.flatnonzero(~ground);gids=np.flatnonzero(ground)[::4]
            if len(elevated)>6000:elevated=elevated[np.linspace(0,len(elevated)-1,6000,dtype=int)]
            budget=6000-len(elevated)
            if len(gids)>budget:gids=gids[np.linspace(0,len(gids)-1,budget,dtype=int)] if budget else gids[:0]
            xyz=xyz[np.sort(np.r_[elevated,gids])];filtered,world,keep=upstream.temporal_filter(xyz,p,q,history,t)
            if len(history)<3:continue
            filtered,window_info=window.add(filtered,p,q,t);h=Header();h.stamp=m.header.stamp;h.frame_id='base_link';cp.publish(point_cloud2.create_cloud_xyz32(h,filtered.astype('float32')));o.header.stamp=m.header.stamp;op.publish(o)
            record=dict(input_width=m.width,input_height=m.height,rig_revision=rig['revision'],window=window_info,t=t,delivered_t=st['t'],alignment='interpolated odometry at original lidar capture time',points=len(filtered),surface_points=int(np.sum(filtered[:,2]>.2)),processing_s=time.monotonic()-begin)
            if t-st['saved']>=5:st['saved']=t;np.savez_compressed(a.out/f'cloud_{t:.3f}.npz',raw=xyz.astype('float32'),keep=keep,assembled=filtered.astype('float32'),xyz=p,q=q,t=t)
            log.write(json.dumps(record)+'\n');log.flush()
    n.create_timer(.01,process)
    def stop(s,f):st['stop']=True
    for s in [signal.SIGINT,signal.SIGTERM]:signal.signal(s,stop)
    (a.out/'ready').write_text('current lidar epoch; no camera blocking or truth')
    try:
        while not st['stop']:rclpy.spin_once(n,timeout_sec=.005)
    finally:log.close();n.destroy_node();rclpy.try_shutdown()
if __name__=='__main__':main()
