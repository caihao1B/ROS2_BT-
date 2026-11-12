"""Temporary sensor consumer. No simulator state, obstacle manifest or motion plans.
Only measured point clouds/RGB, clock, public workspace boundary and calibration.
"""
import argparse,json,os,signal,time
from pathlib import Path
import numpy as np
from PIL import Image
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image as ImageMsg, PointCloud2
from sensor_msgs_py import point_cloud2
ALLOWED=['/clock','/s02/probe/lidar/points','/s02/probe/image']
PUBLIC=['workspaces.geojson','connector.geojson','permanent.geojson','terrain_coarse.geojson','map_metadata.json']
def validate_config(cfg):
    if sorted(cfg['topics'])!=sorted(ALLOWED) or cfg['ros_domain_id']!=119:raise ValueError('sensor allowlist/domain violated')
    if Path(cfg['prior_path']).name!='prior':raise ValueError('public prior root required')
    return cfg
def main():
    p=argparse.ArgumentParser();p.add_argument('--config',type=Path,required=True);p.add_argument('--out',type=Path,required=True);p.add_argument('--seconds',type=float,required=True);p.add_argument('--wall',type=float,default=180);a=p.parse_args()
    cfg=validate_config(json.loads(a.config.read_text()))
    if os.environ.get('ROS_DOMAIN_ID')!='119':raise ValueError('wrong online domain')
    public={k:json.loads((Path(cfg['prior_path'])/k).read_text()) for k in PUBLIC}
    a.out.mkdir(parents=True,exist_ok=True);rclpy.init();n=rclpy.create_node('s02_online_probe')
    state=dict(clock=None,first=None,stopped=False,counts=dict(points=0,image=0),snapshots=[],graph={},error=None)
    started=time.monotonic();last_capture=-1;last_image=-1
    def stop(s,f):state['stopped']=True
    signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
    def clk(m):
        t=m.clock.sec+m.clock.nanosec*1e-9
        if state['clock'] is not None and t<state['clock']:state['error']='clock reversed';state['stopped']=True
        state['clock']=t
        if state['first'] is None:state['first']=t
    def points(m):
        nonlocal last_capture
        state['counts']['points']+=1;t=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
        if t<2 or t-last_capture<3:return
        last_capture=t
        xyz=point_cloud2.read_points_numpy(m,field_names=['x','y','z'],skip_nans=True).reshape(-1,3)
        xyz=xyz[np.isfinite(xyz).all(axis=1)]
        world=xyz+np.array(cfg['probe']['position'])
        name=f'cloud_{len(state["snapshots"]):03}.npz';np.savez_compressed(a.out/name,xyz=world,t=t)
        state['snapshots'].append(dict(t=t,file=name,point_count=len(world)))
        # Compact replay uses real surface returns above ground, without object labels.
        surface=world[world[:,2]>.035];surface=surface[::max(1,len(surface)//3500)]
        (a.out/'observed_points.json').write_text(json.dumps(np.round(surface,3).tolist()))
    def camera(m):
        nonlocal last_image
        state['counts']['image']+=1;t=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
        if t<2 or t-last_image<3:return
        last_image=t
        if m.encoding not in ('rgb8','bgr8'):state['error']='unsupported camera encoding '+m.encoding;return
        rgb=np.frombuffer(m.data,dtype=np.uint8).reshape(m.height,m.step)[:,:m.width*3].reshape(m.height,m.width,3)
        if m.encoding=='bgr8':rgb=rgb[:,:,::-1]
        Image.fromarray(rgb).save(a.out/'camera.png')
        state['image_t']=t;state['image_std']=float(rgb.std());state['image_shape']=list(rgb.shape)
    n.create_subscription(Clock,'/clock',clk,qos_profile_sensor_data)
    n.create_subscription(PointCloud2,'/s02/probe/lidar/points',points,qos_profile_sensor_data)
    n.create_subscription(ImageMsg,'/s02/probe/image',camera,qos_profile_sensor_data)
    (a.out/'ready').write_text('subscriptions created')
    try:
        while not state['stopped'] and time.monotonic()-started<a.wall:
            rclpy.spin_once(n,timeout_sec=.1)
            if state['clock'] is not None and state['clock']>=a.seconds:break
        state['graph']={name:types for name,types in n.get_topic_names_and_types()}
        state['subscriptions']={name:[x.node_name for x in n.get_subscriptions_info_by_topic(name)] for name in state['graph']}
        state['read_files']=[str(a.config)]+[str(Path(cfg['prior_path'])/k) for k in PUBLIC]
        state['wall_seconds']=time.monotonic()-started;state['ros_domain_id']=119
        state['completed']=state['clock'] is not None and state['clock']>=a.seconds and not state['stopped'] and state['error'] is None
    finally:
        (a.out/'result.json').write_text(json.dumps(state,indent=2)+'\n');n.destroy_node();rclpy.try_shutdown()
    return 0 if state.get('completed') and state['counts']['points']>0 and state['counts']['image']>0 else 1
if __name__=='__main__':raise SystemExit(main())
