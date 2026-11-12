"""Reuse S03 Drive completely; extend only its zero-drive actuator serialization.
Explicit steering still requires its unique owner, epoch, ARM and watchdog gates.
"""
from common13 import *
import argparse,signal
sys.path.insert(0,str(BASE/'ws/src/chassis_bridge/scripts'))
from drive_bridge import Drive,rclpy,String
class SteeringPublisher:
 def __init__(self,drive,publisher):self.drive=drive;self.publisher=publisher;self.request={};self.wall=-1e9
 def receive(self,m):
  try:self.request=json.loads(m.data);self.wall=time.monotonic()
  except Exception:self.request={}
 def publish(self,m):
  d=json.loads(m.data);s=self.request;o=self.drive.odom;owners=self.drive.n.get_publishers_info_by_topic('/s13/steering_request')
  if d['mode']=='DRIVE' and d['v']==0 and d['w']==0 and not self.drive.latched and o and abs(o.twist.twist.linear.x)<.01 and abs(o.twist.twist.angular.z)<.02 and time.monotonic()-self.wall<.2 and -.03<=self.drive.t-s.get('t',-100)<.2 and len(owners)==1 and owners[0].node_name=='s03_test_driver':
   k=s.get('curvature',1e9)
   if math.isfinite(k) and abs(k)<=1/2.11:d.update(mode='STEER',steering_curvature=k)
  self.publisher.publish(String(data=json.dumps(d)))
def main():
 p=argparse.ArgumentParser();p.add_argument('--config',type=Path,required=True);p.add_argument('--out',type=Path,required=True);a=p.parse_args();a.out.mkdir(parents=True,exist_ok=True)
 rclpy.init(args=['--ros-args','-r','/tf:=/s05/internal/chassis_tf']);d=Drive(read(a.config),a.out);proxy=SteeringPublisher(d,d.pub);d.pub=proxy;d.n.create_subscription(String,'/s13/steering_request',proxy.receive,1);stop=[False]
 def end(s,f):stop[0]=True
 signal.signal(signal.SIGINT,end);signal.signal(signal.SIGTERM,end);(a.out/'bridge_ready').write_text('S03 safety preserved; stationary steering proxy')
 try:
  while not stop[0]:rclpy.spin_once(d.n,timeout_sec=.02)
 finally:d.latched=True;d.reason='SHUTDOWN';d.tick();d.n.destroy_node();rclpy.try_shutdown()
if __name__=='__main__':main()
