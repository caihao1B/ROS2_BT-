"""One bounded, read-only sample of per-cell terrain classification around the active point sprayer."""
import argparse
from collections import deque
import json
import math
import os
from pathlib import Path
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run',type=Path,required=True)
    parser.add_argument('--out',type=Path,required=True)
    args=parser.parse_args()
    spec=json.loads((args.run/'spec.json').read_text())
    if os.environ.get('ROS_DOMAIN_ID')!='119' or spec.get('task')!='spray' or spec.get('session')!=args.run.name:
        raise ValueError('This observation requires an explicitly bound point-spray session on domain 119')
    if (args.run/'runtime.json').exists():
        raise ValueError('Selected run already ended')
    rclpy.init(domain_id=119)
    n=rclpy.create_node('r10_terrain_evidence_reader')
    poses=deque(maxlen=120)
    state=dict(session=False,last=None,start=None)
    def odom(message):
        p=message.pose.pose.position
        t=message.header.stamp.sec+message.header.stamp.nanosec*1.e-9
        poses.append((t,p.x,p.y))
    def session(message):
        state['session']=json.loads(message.data).get('session')==spec['session']
    n.create_subscription(String,'/r06/directive',session,qos_profile_sensor_data)
    n.create_subscription(Odometry,'/s05/odometry',odom,qos_profile_sensor_data)
    begin=time.monotonic()
    args.out.parent.mkdir(parents=True,exist_ok=True)
    try:
        with args.out.open('x') as store:
            def cells(message):
                if not state['session'] or not poses:
                    return
                d=json.loads(message.data)
                t=d['t']
                if state['last'] is not None and t-state['last']<.9:
                    return
                _,x,y=min(poses,key=lambda pose:abs(pose[0]-t))
                if abs(_-t)>.15:
                    return
                if state['start'] is None:state['start']=t
                state['last']=t
                selected=[a for a in d['cells'] if abs(a[0]-x)<1.5 and abs(a[1]-y)<1.5]
                row=dict(t=t,odom_t=_,odom_xy=[x,y],frame=d.get('frame'),columns=d['columns'],cells=selected)
                store.write(json.dumps(row,separators=(',',':'))+'\n')
                store.flush()
            n.create_subscription(String,'/s09/terrain/cells',cells,qos_profile_sensor_data)
            while time.monotonic()-begin<420 and not (args.run/'runtime.json').exists():
                rclpy.spin_once(n,timeout_sec=.04)
                if state['start'] is not None and state['last']-state['start']>=120:
                    break
    finally:
        n.destroy_node()
        rclpy.try_shutdown()


if __name__=='__main__':
    main()
