"""Unique online command arbiter and quantized encoder odometry. Never reads truth."""
import argparse,json,math,os,signal,time
from pathlib import Path
import rclpy
from rclpy.clock import Clock as RosClock,ClockType
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from kinematics import limit,encoder_delta,integrate,twist
class Drive:
    def __init__(self,c,out):
        self.c=c;self.out=out;self.n=rclpy.create_node('s03_drive_bridge');self.t=0.;self.clock_wall=-1e9;self.clock_prev=None;self.feedback_wall=-1e9;self.request_wall=-1e9
        self.feedback=None;self.request=None;self.pose=[0.,0.,0.];self.prev=None;self.epoch=0;self.resets=0;self.latched=True;self.reason='STARTUP';self.seq=0;self.input_seq=-1;self.odom=None
        self.pub=self.n.create_publisher(String,'/s03/actuator/request',1);self.status=self.n.create_publisher(String,'/s03/drive/status',10);self.opub=self.n.create_publisher(Odometry,'/s03/wheel/odometry',10);self.tf=TransformBroadcaster(self.n)
        self.n.create_subscription(Clock,'/clock',self.clock,qos_profile_sensor_data);self.n.create_subscription(String,'/s03/encoders',self.encoders,qos_profile_sensor_data);self.n.create_subscription(String,'/s03/request',self.command,1)
        self.timer=self.n.create_timer(.02,self.tick,clock=RosClock(clock_type=ClockType.STEADY_TIME))
    def clock(self,m):
        t=m.clock.sec+m.clock.nanosec*1e-9
        if self.clock_prev is not None and t<self.clock_prev:self.latched=True;self.reason='CLOCK_REWIND';self.prev=None;self.pose=[0.,0.,0.]
        if self.clock_prev is None or t!=self.clock_prev:self.clock_wall=time.monotonic()
        self.clock_prev=self.t=t
    def encoders(self,m):
        try:
            d=json.loads(m.data);q=d['q'];vel=d['velocity'];count=4 if self.c['id']=='ackermann' else 2
            if not d['valid'] or len(q)!=count or not all(math.isfinite(v) for v in q+vel):return
            if d['epoch']!=self.epoch:self.latched=True;self.reason='EPOCH_CHANGED_REQUIRES_ARM';self.epoch=d['epoch']
            if d['resets']!=self.resets:self.resets=d['resets'];self.pose=[0.,0.,0.];self.prev=None
            self.feedback_wall=time.monotonic();self.feedback=d
            resolution=2*math.pi/self.c['encoder']['ticks_per_revolution'];q=[round(x/(resolution if i<2 else .001))*(resolution if i<2 else .001) for i,x in enumerate(q)]
            t=d['t'];v,w=twist(vel,q,self.c)
            if self.prev:
                pt,pq=self.prev;dt=t-pt
                if 0<dt<=.5:
                    ds,dyaw=encoder_delta(pq,q,self.c)
                    if abs(ds)>2*self.c['limits']['max_speed']*dt+.01:self.latched=True;self.reason='ENCODER_JUMP'
                    else:self.pose=integrate(self.pose,ds,dyaw)
                elif dt<0 or dt>.5:self.latched=True;self.reason='FEEDBACK_TIME_GAP'
            self.prev=(t,q);x,y,yaw=self.pose
            o=Odometry();o.header.stamp.sec=int(t);o.header.stamp.nanosec=int((t-int(t))*1e9);o.header.frame_id='odom';o.child_frame_id='base_link';o.pose.pose.position.x=x;o.pose.pose.position.y=y;o.pose.pose.orientation.z=math.sin(yaw/2);o.pose.pose.orientation.w=math.cos(yaw/2);o.twist.twist.linear.x=v;o.twist.twist.angular.z=w
            for i,value in enumerate([.04,.04,1e6,1e6,1e6,.09]):o.pose.covariance[7*i]=value;o.twist.covariance[7*i]=value
            self.odom=o;self.opub.publish(o)
            tr=TransformStamped();tr.header=o.header;tr.child_frame_id=o.child_frame_id;tr.transform.translation.x=x;tr.transform.translation.y=y;tr.transform.rotation=o.pose.pose.orientation;self.tf.sendTransform(tr)
        except (ValueError,KeyError,TypeError):self.latched=True;self.reason='MALFORMED_FEEDBACK'
    def command(self,m):
        try:
            d=json.loads(m.data);required={'sequence','epoch','stamp','mode','v','w'}
            if set(d)!=required or not all(type(d[k]) in (int,float) and math.isfinite(d[k]) for k in ['sequence','epoch','stamp','v','w']):raise ValueError('FIELDS')
            if d['sequence']<=self.input_seq or d['epoch']!=self.epoch or not -.03<=self.t-d['stamp']<=self.c['safety']['command_sim_s']:raise ValueError('STALE_OR_REPLAY')
            self.input_seq=d['sequence'];self.request_wall=time.monotonic();self.request=d
            if d['mode']=='STOP':self.latched=True;self.reason='CANCELLED'
            elif d['mode']=='ARM' and self.feedback and not self.feedback['paused'] and self.odom and abs(self.odom.twist.twist.linear.x)<.01 and abs(self.odom.twist.twist.angular.z)<.02:
                self.latched=False;self.reason='ARMED'
            elif d['mode'] not in ['DRIVE','ARM']:raise ValueError('MODE')
        except (ValueError,KeyError,TypeError):self.latched=True;self.reason='INVALID_OR_STALE_COMMAND'
    def tick(self):
        now=time.monotonic();s=self.c['safety'];infos=self.n.get_publishers_info_by_topic('/s03/request');owners=[i.node_name for i in infos]
        if owners!=['s03_test_driver']:self.latched=True;self.reason='INPUT_OWNER_CONFLICT' if owners else 'NO_COMMAND_OWNER'
        elif now-self.clock_wall>s['clock_wall_s']:self.latched=True;self.reason='CLOCK_PAUSED'
        elif now-self.feedback_wall>s['feedback_wall_s']:self.latched=True;self.reason='FEEDBACK_TIMEOUT'
        elif now-self.request_wall>s['command_wall_s']:self.latched=True;self.reason='COMMAND_TIMEOUT'
        mode='STOP';v=w=0.
        if not self.latched and self.request:
            mode=self.request['mode'];v,w,_=limit(self.request['v'],self.request['w'],self.c)
            if mode!='DRIVE':v=w=0.
        self.seq+=1;request=dict(session=self.c['session'],revision=self.c['config_hash'],epoch=self.epoch,sequence=self.seq,stamp=self.t,mode=mode,v=v,w=w)
        msg=String();msg.data=json.dumps(request);self.pub.publish(msg)
        status=dict(t=self.t,epoch=self.epoch,reason=self.reason,armed=not self.latched,owners=owners,mode=mode,v=v,w=w,feedback_age=now-self.feedback_wall,command_age=now-self.request_wall)
        st=String();st.data=json.dumps(status);self.status.publish(st)
def main():
    p=argparse.ArgumentParser();p.add_argument('--config',type=Path,required=True);p.add_argument('--out',type=Path,required=True);a=p.parse_args();a.out.mkdir(parents=True,exist_ok=True);c=json.loads(a.config.read_text())
    if os.environ.get('ROS_DOMAIN_ID')!='119':raise ValueError('online domain required')
    rclpy.init();node=Drive(c,a.out);stop=[False]
    def end(s,f):stop[0]=True
    signal.signal(signal.SIGINT,end);signal.signal(signal.SIGTERM,end);(a.out/'bridge_ready').write_text('ready')
    try:
        while not stop[0]:rclpy.spin_once(node.n,timeout_sec=.05)
    finally:
        node.latched=True;node.reason='SHUTDOWN';node.tick();node.n.destroy_node();rclpy.try_shutdown()
if __name__=='__main__':main()
