"""Domain 120: physical tool effect only, independently of online decisions or ledger."""
from common10 import *
from ledger_core import footprint
import argparse,signal
import rclpy
from std_msgs.msg import String
from shapely.geometry import Polygon,mapping

def main():
 p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True);p.add_argument('--config',type=Path,required=True);a=p.parse_args();a.out.mkdir(parents=True,exist_ok=True);c=read(a.config);rclpy.init();n=rclpy.create_node('s10_physical_effect_evaluator');st=dict(stop=False);log=(a.out/'physical.jsonl').open('w');jobs={};previous=None
 def observe(m):
  nonlocal previous
  d=json.loads(m.data);log.write(m.data+'\n');key=d['key'];j=jobs.setdefault(key,dict(dose=0.,area=Polygon(),active_samples=0));dt=d['t']-previous['t'] if previous else 0
  if abs(d['rpm_rad_s'])>=c['working_min_rad_s']:
   j['active_samples']+=1
   if c['vehicle']=='diff':
    if previous and previous['key']==key and 0<dt<.1:j['dose']+=max(0,min(d['flow'],previous['flow']))*dt
   else:
    # Fixed map datum is evaluator-only ground-truth reference, never supplied to online nodes.
    x,y=d['tool_xyz'][:2];j['area']=j['area'].union(footprint(x-28,y-14,d['tool_yaw'],.70,.90))
  previous=d
 n.create_subscription(String,'/s10/evaluation/effect',observe,100)
 def stop(s,f):st['stop']=True
 signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop);(a.out/'ready').write_text('truth domain 120 only')
 try:
  while not st['stop']:rclpy.spin_once(n,timeout_sec=.02)
 finally:
  dump(a.out/'result.json',dict(jobs={k:dict(dose=j['dose'],area_m2=j['area'].area,area=mapping(j['area']),active_samples=j['active_samples']) for k,j in jobs.items()}));log.close();n.destroy_node();rclpy.try_shutdown()
if __name__=='__main__':main()
