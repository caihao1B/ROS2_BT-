"""Independent domain120 recorder; only offline scorer reads these poses."""
import argparse,signal,os,json
from pathlib import Path
import rclpy
from tf2_msgs.msg import TFMessage
from rclpy.qos import qos_profile_sensor_data
p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True);a=p.parse_args();a.out.mkdir(parents=True,exist_ok=True)
if os.environ.get('ROS_DOMAIN_ID')!='120':raise ValueError('truth domain required')
rclpy.init();n=rclpy.create_node('s09_dynamic_evaluator');f=(a.out/'dynamic.jsonl').open('w');stop=[False]
def receive(m):
 for tr in m.transforms:
  t=tr.header.stamp.sec+tr.header.stamp.nanosec*1e-9;p=tr.transform.translation;f.write(json.dumps(dict(t=t,name=tr.child_frame_id,xyz=[p.x,p.y,p.z]))+'\n')
 f.flush()
n.create_subscription(TFMessage,'/s09/evaluation/dynamic_poses',receive,qos_profile_sensor_data)
def end(s,f):stop[0]=True
signal.signal(signal.SIGINT,end);signal.signal(signal.SIGTERM,end)
try:
 while not stop[0]:rclpy.spin_once(n,timeout_sec=.03)
finally:f.close();n.destroy_node();rclpy.try_shutdown()
