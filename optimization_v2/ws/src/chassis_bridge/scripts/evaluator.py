"""Read-only separate-domain measurements. Never publishes actuator commands."""
import argparse,json,os,signal,time
from pathlib import Path
import rclpy
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from ros_gz_interfaces.msg import Contacts
def main():
    p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True);a=p.parse_args();a.out.mkdir(parents=True,exist_ok=True)
    if os.environ.get('ROS_DOMAIN_ID')!='120':raise ValueError('evaluation domain required')
    rclpy.init();n=rclpy.create_node('s03_evaluator');f=(a.out/'truth.jsonl').open('w');contacts={};stop=[False]
    def end(s,fr):stop[0]=True
    signal.signal(signal.SIGINT,end);signal.signal(signal.SIGTERM,end)
    def pose(m):
        d=json.loads(m.data);d['wall']=time.monotonic();f.write(json.dumps(d,separators=(',',':'))+'\n')
    def contact(m):
        for c in m.contacts:
            key=c.collision1.name+'|'+c.collision2.name;contacts[key]=contacts.get(key,0)+1
    n.create_subscription(String,'/s03/evaluation/pose',pose,qos_profile_sensor_data);n.create_subscription(Contacts,'/s03/evaluation/contacts',contact,qos_profile_sensor_data)
    (a.out/'ready').write_text('ready')
    try:
        while not stop[0]:rclpy.spin_once(n,timeout_sec=.05)
    finally:
        f.close();(a.out/'contacts.json').write_text(json.dumps(contacts,indent=2));n.destroy_node();rclpy.try_shutdown()
if __name__=='__main__':main()
