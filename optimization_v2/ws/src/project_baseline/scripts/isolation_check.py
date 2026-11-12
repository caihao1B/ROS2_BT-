#!/usr/bin/env python3
"""Receive a C++ publisher on domain 119. No robot command topics are used."""
import argparse, json, os, time
import rclpy
from rclpy.node import Node

p=argparse.ArgumentParser();p.add_argument('--expect-empty',action='store_true');p.add_argument('--duration',type=float,default=7)
args=p.parse_args()
rclpy.init()
n = Node('s01_isolation_subscriber')
from std_msgs.msg import String
received=[]
sub=n.create_subscription(String,'/s01/isolation_probe',lambda m:received.append(m.data),10)
end=time.monotonic()+args.duration
while time.monotonic()<end and len(received)<5:
    rclpy.spin_once(n,timeout_sec=.1)
expected='S01_DOMAIN_119'
ok=not received if args.expect_empty else len(received)>=5 and all(s==expected for s in received)
print(json.dumps({'status':'PASS' if ok else 'FAIL', 'domain':os.getenv('ROS_DOMAIN_ID'),
                  'messages':received,'ament_prefix':os.getenv('AMENT_PREFIX_PATH')},ensure_ascii=False))
n.destroy_node();rclpy.shutdown()
raise SystemExit(0 if ok else 1)
