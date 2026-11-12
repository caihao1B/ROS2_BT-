"""Test-only closed-loop circle/ramp client; obeys observed health and existing arbiter."""
import argparse,signal,time
import rclpy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
from support import *
def main():
    p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True);p.add_argument('--seconds',type=float,required=True);p.add_argument('--case',required=True);a=p.parse_args();a.out.mkdir(parents=True,exist_ok=True);f=(a.out/'commands.jsonl').open('w');rclpy.init();n=rclpy.create_node('s03_test_driver');pub=n.create_publisher(String,'/s03/request',1)
    st=dict(t=0.,epoch=0,odom=None,limit=None,drive=None,stop=False,angle=0.,prev_angle=0.,goal=False,range_t=-100.,front_range=0.);seq=0;last=0.;last_save=-1.
    def clock(m):
        t=m.clock.sec+m.clock.nanosec*1e-9
        if t<st['t']:
            st.update(odom=None,limit=None,angle=0.,prev_angle=0.,goal=False,range_t=-100.)
        st['t']=t
    def odom(m):xyz,q=pose(m);st['odom']=[*xyz,*euler(q)]
    def enc(m):st['epoch']=json.loads(m.data)['epoch']
    def limit(m):st['limit']=json.loads(m.data)
    def drive(m):st['drive']=json.loads(m.data)
    def scan(m):
        rr=np.array(m.ranges);aa=m.angle_min+np.arange(len(rr))*m.angle_increment;xx=rr*np.cos(aa);yy=rr*np.sin(aa);mask=np.isfinite(rr)&(xx>0)&(np.abs(yy)<.68);st['front_range']=float(np.min(xx[mask])) if mask.any() else 8.;st['range_t']=stamp(m)
    n.create_subscription(Clock,'/clock',clock,qos_profile_sensor_data);n.create_subscription(Odometry,'/s05/odometry',odom,10);n.create_subscription(String,'/s03/encoders',enc,qos_profile_sensor_data);n.create_subscription(String,'/s05/safety_limit',limit,10);n.create_subscription(String,'/s03/drive/status',drive,10)
    n.create_subscription(LaserScan,'/s03/front_scan',scan,qos_profile_sensor_data)
    def end(s,f):st['stop']=True
    signal.signal(signal.SIGINT,end);signal.signal(signal.SIGTERM,end);(a.out/'ready').write_text('fused odom + safety limit; no world pose')
    def send(mode,v,w):
        nonlocal seq
        seq+=1;m=String();m.data=json.dumps(dict(sequence=seq,epoch=st['epoch'],stamp=st['t'],mode=mode,v=v,w=w));pub.publish(m)
    try:
        while not st['stop'] and st['t']<a.seconds:
            rclpy.spin_once(n,timeout_sec=.005);now=time.monotonic()
            if now-last<.025:continue
            last=now;t=st['t']
            if t<last_save:last_save=-1.
            mode='STOP';v=w=0.;reason='NO_HEALTH_PERMISSION';health=st['limit']
            if health and st['odom'] and t>=9 and t<a.seconds-4 and 0<=t-health['t']<.3 and health['max_speed']>0 and not st['goal']:
                x,y,z,roll,pitch,yaw=st['odom'];cap=health['max_speed'];reason=health['reason']
                if not st['drive'] or not st['drive']['armed']:mode='ARM'
                else:
                    mode='DRIVE'
                    if a.case=='slope':
                        if x>11.5:st['goal']=True;mode='STOP'
                        else:v=min(.25,cap);w=max(-.15,min(.15,-.6*y-.7*yaw))
                    else:
                        phi=math.atan2(x,3-y);d=wrap(phi-st['prev_angle']);st['angle']+=d;st['prev_angle']=phi
                        if a.case=='loop' and st['angle']>=2*math.pi-.12:st['goal']=True;mode='STOP'
                        else:
                            target=np.array([3*math.sin(phi+.4),3-3*math.cos(phi+.4)]);delta=target-[x,y];ly=-math.sin(yaw)*delta[0]+math.cos(yaw)*delta[1];curvature=2*ly/max(.1,float(delta@delta));v=min(.45,cap);w=v*max(-.45,min(.45,curvature))
            if mode=='DRIVE' and (t-st['range_t']>.3 or st['front_range']<.35*abs(v)+v*v/(2*.8)+.65):mode='STOP';reason='FRONT_RANGE_PROTECTIVE_STOP'
            if mode!='DRIVE':v=w=0.
            send(mode,v,w)
            if t-last_save>=.05:f.write(json.dumps(dict(t=t,mode=mode,v=v,w=w,reason=reason,health=health,odom=st['odom'],angle=st['angle'],goal=st['goal']))+'\n');last_save=t
    finally:
        for _ in range(8):send('STOP',0.,0.);rclpy.spin_once(n,timeout_sec=.02)
        f.close();dump(a.out/'result.json',dict(completed=not st['stop'],t=st['t'],goal=st['goal'],loop_angle=st['angle']));n.destroy_node();rclpy.try_shutdown()
if __name__=='__main__':main()
