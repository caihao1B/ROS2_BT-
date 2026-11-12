"""S04 instrumented-lane client; S03 remains the only actuator and odom TF owner."""
import argparse,json,signal,time
import rclpy
from std_msgs.msg import String
from rosgraph_msgs.msg import Clock
from rclpy.qos import qos_profile_sensor_data
from common import Path,dump
def main():
    p=argparse.ArgumentParser();p.add_argument('--seconds',type=float,required=True);p.add_argument('--out',type=Path,required=True);a=p.parse_args();a.out.mkdir(parents=True,exist_ok=True)
    rclpy.init();n=rclpy.create_node('s03_test_driver');pub=n.create_publisher(String,'/s03/request',1);state=dict(t=0.,epoch=0,seq=0,stopped=False,commands=0)
    def clock(m):state['t']=m.clock.sec+m.clock.nanosec*1e-9
    def enc(m):state['epoch']=json.loads(m.data)['epoch']
    n.create_subscription(Clock,'/clock',clock,qos_profile_sensor_data);n.create_subscription(String,'/s03/encoders',enc,qos_profile_sensor_data)
    def end(s,f):state['stopped']=True
    signal.signal(signal.SIGINT,end);signal.signal(signal.SIGTERM,end)
    def send(mode,v=0.):
        state['seq']+=1;msg=String();msg.data=json.dumps(dict(sequence=state['seq'],epoch=state['epoch'],stamp=state['t'],mode=mode,v=v,w=0.));pub.publish(msg);state['commands']+=1
    last=0.;(a.out/'ready').write_text('motion lane client')
    try:
        while not state['stopped'] and state['t']<a.seconds:
            rclpy.spin_once(n,timeout_sec=.01);now=time.monotonic()
            if now-last<.02:continue
            last=now;t=state['t'];u=(t-3)%16
            if t<3 or t>a.seconds-3:send('STOP')
            elif u<.6:send('ARM')
            elif u<5.6:send('DRIVE',.3)
            elif u<8:send('STOP')
            elif u<8.6:send('ARM')
            elif u<13.6:send('DRIVE',-.3)
            else:send('STOP')
    finally:
        for _ in range(8):send('STOP');rclpy.spin_once(n,timeout_sec=.02)
        dump(a.out/'result.json',state);n.destroy_node();rclpy.try_shutdown()
if __name__=='__main__':main()
