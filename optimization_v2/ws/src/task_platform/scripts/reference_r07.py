"""Adapt the unchanged S08 supervisor output; no additional transform publisher."""
import argparse,json,math,time,signal
from pathlib import Path
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped

def stamp(m):return m.header.stamp.sec+m.header.stamp.nanosec*1e-9
def yaw(q):return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
def rot(h):return np.array([[math.cos(h),-math.sin(h)],[math.sin(h),math.cos(h)]])
def main():
    ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path,required=True);ap.add_argument('--config',type=Path,required=True);a=ap.parse_args();a.out.mkdir(parents=True,exist_ok=True);cfg=json.loads(a.config.read_text());rclpy.init();n=rclpy.create_node('r07_reference_adapter',parameter_overrides=[rclpy.parameter.Parameter('use_sim_time',value=True)]);pub=n.create_publisher(String,'/r07/localization',10);posepub=n.create_publisher(PoseWithCovarianceStamped,'/r07/pose',20);st=dict(t=0.,stop=False,status=None,pose=None,odom=None);log=(a.out/'reference.jsonl').open('w')
    n.create_subscription(Clock,'/clock',lambda m:st.update(t=m.clock.sec+m.clock.nanosec*1e-9),qos_profile_sensor_data)
    n.create_subscription(Odometry,'/s05/odometry',lambda m:st.update(odom=m),30)
    n.create_subscription(PoseWithCovarianceStamped,'/s08/pose',lambda m:st.update(pose=m),20)
    n.create_subscription(String,'/s08/status',lambda m:st.update(status=json.loads(m.data)),20)
    def publish():
        t=st['t'];s=st['status'] or {};m=st['pose'];o=st['odom'];valid=bool(s.get('navigation_allowed') and not s.get('plan_revalidation_required') and -.03<=t-s.get('t',-100)<.2 and m and -.03<=t-stamp(m)<.2)
        d=dict(t=t,valid=valid,reason=s.get('reason','INITIALIZING'),max_speed=s.get('max_speed',0),map_revision=cfg['map_revision'],rig_revision=cfg['rig']['revision'],native_supervisor=s,truth_used=False)
        if m and o:
            p=m.pose.pose;q=o.pose.pose;c=s['correction'];h=math.atan2(2*(c[6]*c[5]+c[3]*c[4]),1-2*(c[4]**2+c[5]**2));d.update(pose=[p.position.x,p.position.y,yaw(p.orientation)],odom=[q.position.x,q.position.y,yaw(q.orientation)],odom_t=stamp(m),v=o.twist.twist.linear.x,w=o.twist.twist.angular.z,transform=[c[0],c[1],h]);posepub.publish(m)
        pub.publish(String(data=json.dumps(d)));log.write(json.dumps(d)+'\n');log.flush();tmp=a.out/'live.tmp';tmp.write_text(json.dumps(d));tmp.replace(a.out/'live.json')
    n.create_timer(.03,publish)
    def stop(s,f):st['stop']=True
    for s in [signal.SIGINT,signal.SIGTERM]:signal.signal(s,stop)
    (a.out/'ready').write_text('S08 owns map->odom; adapter publishes no TF')
    try:
        while not st['stop']:rclpy.spin_once(n,timeout_sec=.01)
    finally:log.close();n.destroy_node();rclpy.try_shutdown()
if __name__=='__main__':main()
