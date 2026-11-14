"""Estimated context and noisy real simulated joint/flow feedback, never truth."""
import argparse,json,signal,math
from pathlib import Path
import numpy as np
import rclpy
from std_msgs.msg import String
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation
from rclpy.qos import qos_profile_sensor_data
def main():
    ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path,required=True);ap.add_argument('--config',type=Path,required=True);a=ap.parse_args();a.out.mkdir(parents=True,exist_ok=True);cfg=json.loads(a.config.read_text());rclpy.init();n=rclpy.create_node('r07_tool_observer',parameter_overrides=[rclpy.parameter.Parameter('use_sim_time',value=True)]);st=dict(t=0.,stop=False,loc=None,safety=None);rng=np.random.default_rng(41);fb=n.create_publisher(String,'/s10/tool/feedback',30);ctx=n.create_publisher(String,'/s10/context',30);log=(a.out/'observations.jsonl').open('w')
    n.create_subscription(Clock,'/clock',lambda m:st.update(t=m.clock.sec+m.clock.nanosec*1e-9),qos_profile_sensor_data)
    for topic,key in [('/r07/localization','loc'),('/r07/safety','safety')]:n.create_subscription(String,topic,lambda m,k=key:st.update({k:json.loads(m.data)}),20)
    def context():
        p=st['loc'] or {};s=st['safety'] or {};pose=p.get('pose',[0,0,0]);h=pose[2];offset=cfg['tool']['mount_xyz'][0];t=p.get('odom_t',-100.)
        d=dict(t=t,tool_xy=[pose[0]+offset*math.cos(h),pose[1]+offset*math.sin(h)],yaw=h,v=p.get('v',100),w=p.get('w',100),localization=bool(p.get('valid') and -.03<=st['t']-p.get('t',-100)<.15),perception=bool(s.get('valid') and -.03<=st['t']-s.get('t',-100)<.45 and s.get('max_speed',0)>0),target_ok=True,max_speed=s.get('max_speed',0),segment='POINT_SERVICE',segment_id='',target_condition_source='declared task grid center; arm / near-camera treatment is an external future project')
        ctx.publish(String(data=json.dumps(d)));log.write(json.dumps(dict(kind='context',**d))+'\n')
    # Publish an estimated context at each actual local-odometry epoch, avoiding
    # the extra supervisor -> reference -> timer delay. No new TF owner.
    def odometry(m):
        p=st['loc'] or {};s=st['safety'] or {};supervisor=p.get('native_supervisor',{});c=supervisor.get('correction')
        if not c or not supervisor.get('has_transform'):return
        pose=m.pose.pose;q=pose.orientation;R=Rotation.from_quat(c[3:]);B=Rotation.from_quat([q.x,q.y,q.z,q.w]);position=R.apply([pose.position.x,pose.position.y,pose.position.z])+c[:3];orientation=R*B;tool=position+orientation.apply(cfg['tool']['mount_xyz']);h=orientation.as_euler('xyz')[2];t=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
        d=dict(t=t,tool_xy=tool[:2].tolist(),yaw=float(h),v=m.twist.twist.linear.x,w=m.twist.twist.angular.z,localization=bool(p.get('valid') and -.03<=st['t']-p.get('t',-100)<.15),perception=bool(s.get('valid') and -.03<=st['t']-s.get('t',-100)<.45 and s.get('max_speed',0)>0),target_ok=True,max_speed=s.get('max_speed',0),segment='POINT_SERVICE',segment_id='',target_condition_source='declared task grid center; future arm project',pose_source='current measured local odometry transformed by qualified native S08 correction')
        ctx.publish(String(data=json.dumps(d)));log.write(json.dumps(dict(kind='context',**d))+'\n')
    n.create_subscription(Odometry,'/s05/odometry',odometry,30)
    def raw(m):
        d=json.loads(m.data);d['rpm_rad_s']=round(d['rpm_rad_s']+float(rng.normal(0,.005)),2);d['flow']=round(d['flow'],3);d['meter']=round(d['meter'],4);fb.publish(String(data=json.dumps(d)));log.write(json.dumps(dict(kind='feedback',**d))+'\n');log.flush()
    n.create_subscription(String,'/s10/tool/raw_feedback',raw,30)
    def stop(s,f):st['stop']=True
    for s in [signal.SIGINT,signal.SIGTERM]:signal.signal(s,stop)
    (a.out/'ready').write_text('no arm; measured simulated flow and estimated docking')
    try:
        while not st['stop']:rclpy.spin_once(n,timeout_sec=.01)
    finally:log.close();n.destroy_node();rclpy.try_shutdown()
if __name__=='__main__':main()
