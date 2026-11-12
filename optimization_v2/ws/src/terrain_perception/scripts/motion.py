"""Stationary S09 qualification. Keep the existing unique actuator stopped."""
import argparse,signal,time
import rclpy
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from rosgraph_msgs.msg import Clock
import common09
import common08
from s07_common import *
def main():
    ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path,required=True);ap.add_argument('--seconds',type=float,required=True);ap.add_argument('--case',required=True);a=ap.parse_args();a.out.mkdir(parents=True,exist_ok=True);rclpy.init();n=rclpy.create_node('s03_test_driver');pub=n.create_publisher(String,'/s03/request',1);st=dict(t=0.,epoch=0,status=None,health=None,drive=None,odom=None,range=0.,range_t=-100.,stop=False,distance=0.);seq=0;last=0.;last_log=-1.;previous=None;f=(a.out/'commands.jsonl').open('w')
    def clock(m):
        t=m.clock.sec+m.clock.nanosec*1e-9
        if t<st['t']:st['stop']=True
        st['t']=t
    def odom(m):
        nonlocal previous
        xyz,q=pose(m)
        if previous is not None:st['distance']+=float(np.linalg.norm(xyz-previous))
        previous=xyz;st['odom']=[*xyz.tolist(),*Rotation.from_quat(q).as_euler('xyz').tolist()]
    def update(m,key):st[key]=json.loads(m.data)
    def scan(m):
        r=np.asarray(m.ranges);theta=m.angle_min+np.arange(len(r))*m.angle_increment;x=r*np.cos(theta);y=r*np.sin(theta);mask=np.isfinite(r)&(x>0)&(abs(y)<.68);st['range']=float(x[mask].min()) if mask.any() else 8.;st['range_t']=stamp(m)
    n.create_subscription(Clock,'/clock',clock,qos_profile_sensor_data);n.create_subscription(Odometry,'/s05/odometry',odom,20);n.create_subscription(LaserScan,'/s03/front_scan',scan,qos_profile_sensor_data)
    for topic,key in [('/s08/status','status'),('/s05/safety_limit','health'),('/s03/drive/status','drive')]:n.create_subscription(String,topic,lambda m,k=key:update(m,k),10)
    def encoder(m):st['epoch']=json.loads(m.data)['epoch']
    n.create_subscription(String,'/s03/encoders',encoder,qos_profile_sensor_data)
    def end(s,f):st['stop']=True
    signal.signal(signal.SIGINT,end);signal.signal(signal.SIGTERM,end);(a.out/'ready').write_text('localization + local safety conjunctive admission')
    def send(mode,v,w):
        nonlocal seq
        seq+=1;m=String();m.data=json.dumps(dict(sequence=seq,epoch=st['epoch'],stamp=st['t'],mode=mode,v=v,w=w));pub.publish(m)
    try:
        while not st['stop'] and st['t']<a.seconds:
            rclpy.spin_once(n,timeout_sec=.005);wall=time.monotonic()
            if wall-last<.025:continue
            last=wall;t=st['t'];mode='STOP';v=w=0.;reason='S09_PARKED_PERCEPTION_QUALIFICATION_NO_NAVIGATION_TASK';status=st['status'];health=st['health']
            send(mode,v,w)
            if t-last_log>.1:f.write(json.dumps(dict(t=t,mode=mode,v=v,w=w,reason=reason,distance=st['distance'],status=status))+'\n');f.flush();last_log=t
    finally:
        for _ in range(8):send('STOP',0.,0.);rclpy.spin_once(n,timeout_sec=.02)
        f.close();dump(a.out/'result.json',dict(t=st['t'],distance=st['distance'],completed=not st['stop']));n.destroy_node();rclpy.try_shutdown()
if __name__=='__main__':main()
