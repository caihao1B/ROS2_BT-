"""Independent domain120 recorder; only offline scorer reads these poses."""
import argparse,signal,os,json,math
from pathlib import Path
import rclpy
from tf2_msgs.msg import TFMessage
from ros_gz_interfaces.msg import Contacts
from rclpy.qos import qos_profile_sensor_data
p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True);a=p.parse_args();a.out.mkdir(parents=True,exist_ok=True)
if os.environ.get('ROS_DOMAIN_ID')!='120':raise ValueError('truth domain required')
rclpy.init();n=rclpy.create_node('s09_dynamic_evaluator');f=(a.out/'dynamic.jsonl').open('w');original=(a.out/'original_dynamic.jsonl').open('w');contacts=(a.out/'contact_events.jsonl').open('w');stop=[False]
def receive(m,output=f):
 for tr in m.transforms:
  t=tr.header.stamp.sec+tr.header.stamp.nanosec*1e-9;p=tr.transform.translation;output.write(json.dumps(dict(t=t,name=tr.child_frame_id,xyz=[p.x,p.y,p.z],yaw=math.atan2(2*(tr.transform.rotation.w*tr.transform.rotation.z+tr.transform.rotation.x*tr.transform.rotation.y),1-2*(tr.transform.rotation.y**2+tr.transform.rotation.z**2))))+'\n')
 output.flush()
n.create_subscription(TFMessage,'/s09/evaluation/dynamic_poses',receive,qos_profile_sensor_data)
n.create_subscription(TFMessage,'/s02/evaluation/dynamic_poses',lambda m:receive(m,original),qos_profile_sensor_data)
def contact(m):
 pairs=[c.collision1.name+'|'+c.collision2.name for c in m.contacts if 'vehicle::' in c.collision1.name+'|'+c.collision2.name and not ('wheel::' in c.collision1.name+'|'+c.collision2.name and 'ground::body::ground' in c.collision1.name+'|'+c.collision2.name)]
 if pairs:contacts.write(json.dumps(dict(t=m.header.stamp.sec+m.header.stamp.nanosec*1e-9,pairs=pairs))+'\n');contacts.flush()
n.create_subscription(Contacts,'/s03/evaluation/contacts',contact,qos_profile_sensor_data)
def end(s,f):stop[0]=True
signal.signal(signal.SIGINT,end);signal.signal(signal.SIGTERM,end)
try:
 while not stop[0]:rclpy.spin_once(n,timeout_sec=.03)
finally:f.close();original.close();contacts.close();n.destroy_node();rclpy.try_shutdown()
