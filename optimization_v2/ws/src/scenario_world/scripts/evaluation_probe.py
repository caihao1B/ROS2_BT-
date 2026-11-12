"""Separate ROS domain: simulator truth and contact observations for OFFLINE scoring."""
import argparse,json,os,signal,time
from pathlib import Path
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from tf2_msgs.msg import TFMessage
from ros_gz_interfaces.msg import Contacts
def main():
    p=argparse.ArgumentParser();p.add_argument('--truth',type=Path,required=True);p.add_argument('--out',type=Path,required=True);p.add_argument('--seconds',type=float,required=True);p.add_argument('--wall',type=float,default=180);a=p.parse_args()
    if os.environ.get('ROS_DOMAIN_ID')!='120':raise ValueError('evaluation must use domain 120')
    manifest=json.loads(a.truth.read_text());a.out.mkdir(parents=True,exist_ok=True)
    state=dict(clock=None,trajectory=[],contact_pairs={},stopped=False,ros_domain_id=120,truth_file=str(a.truth))
    rclpy.init();n=rclpy.create_node('s02_evaluator');start=time.monotonic()
    def stop(s,f):state['stopped']=True
    signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
    def clk(m):state['clock']=m.clock.sec+m.clock.nanosec*1e-9
    def poses(m):
        if not m.transforms:return
        t=m.transforms[0].header.stamp.sec+m.transforms[0].header.stamp.nanosec*1e-9
        state['trajectory'].append(dict(t=t,poses={x.child_frame_id:[x.transform.translation.x,x.transform.translation.y,x.transform.translation.z] for x in m.transforms}))
    def contact(m):
        for c in m.contacts:
            key=c.collision1.name+'|'+c.collision2.name
            state['contact_pairs'][key]=state['contact_pairs'].get(key,0)+1
    n.create_subscription(Clock,'/clock',clk,qos_profile_sensor_data)
    n.create_subscription(TFMessage,'/s02/evaluation/dynamic_poses',poses,qos_profile_sensor_data)
    n.create_subscription(Contacts,'/s02/evaluation/contacts',contact,qos_profile_sensor_data)
    (a.out/'ready').write_text('subscriptions created')
    try:
        while not state['stopped'] and time.monotonic()-start<a.wall:
            rclpy.spin_once(n,timeout_sec=.1)
            if state['clock'] is not None and state['clock']>=a.seconds:break
        state['completed']=state['clock'] is not None and state['clock']>=a.seconds and not state['stopped']
        state['wall_seconds']=time.monotonic()-start
    finally:
        (a.out/'result.json').write_text(json.dumps(state,indent=2)+'\n');n.destroy_node();rclpy.try_shutdown()
    return 0 if state.get('completed') and len(state['trajectory'])>10 else 1
if __name__=='__main__':raise SystemExit(main())
