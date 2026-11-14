"""Open-loop qualification commands use clock + encoders only; no world pose."""
import argparse,json,signal,time
from pathlib import Path
import rclpy
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from rosgraph_msgs.msg import Clock

def main():
 p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True);a=p.parse_args();spec=json.loads((a.out/'spec.json').read_text());c=json.loads((a.out/'scene/vehicle.json').read_text())
 rclpy.init();n=rclpy.create_node('r02_qualification_driver');pub=n.create_publisher(String,'/s03/actuator/request',10);state=dict(t=0.,enc=None,stop=False);seq=0;begin=time.monotonic();last=0
 def clk(m):state['t']=m.clock.sec+m.clock.nanosec*1e-9
 def enc(m):state['enc']=json.loads(m.data)
 def stop(s,f):state['stop']=True
 signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
 n.create_subscription(Clock,'/clock',clk,qos_profile_sensor_data);n.create_subscription(String,'/s03/encoders',enc,qos_profile_sensor_data)
 (a.out/'driver_ready').write_text('ready');f=(a.out/'online.jsonl').open('w');total=sum(x['duration'] for x in spec['phases']);complete=False
 try:
  while time.monotonic()-begin<spec['wall_budget']-15:
   rclpy.spin_once(n,timeout_sec=.005);now=time.monotonic()
   if now-last<.015:continue
   last=now;t=state['t'];cursor=0.;phase=dict(name='finish',mode='STOP',v=0.,w=0.)
   for item in spec['phases']:
    if t<cursor+item['duration']:phase=item;break
    cursor+=item['duration']
   if state['stop']:phase=dict(name='interrupt',mode='STOP',v=0.,w=0.)
   e=state['enc']
   if e is None:continue
   seq+=1;request=dict(session=spec['session'],revision=c['config_hash'],epoch=e['epoch'],sequence=seq,stamp=t,mode=phase['mode'],v=phase['v'],w=phase['w'])
   if phase.get('send',True):m=String();m.data=json.dumps(request);pub.publish(m)
   f.write(json.dumps(dict(t=t,phase=phase['name'],request=request,encoders=e))+'\n')
   if t>total:complete=True;break
   if state['stop'] and abs(e['limited_v'])<.005:break
 finally:
  for _ in range(3):
   seq+=1;m=String();m.data=json.dumps(dict(session=spec['session'],revision=c['config_hash'],epoch=(state['enc']or{}).get('epoch',0),sequence=seq,stamp=state['t'],mode='STOP',v=0.,w=0.));pub.publish(m);rclpy.spin_once(n,timeout_sec=.02)
  f.close();(a.out/'driver_result.json').write_text(json.dumps(dict(complete=complete,t=state['t'],wall=time.monotonic()-begin,feedback=state['enc'])));n.destroy_node();rclpy.try_shutdown()
 return 0 if complete else 2
if __name__=='__main__':raise SystemExit(main())
