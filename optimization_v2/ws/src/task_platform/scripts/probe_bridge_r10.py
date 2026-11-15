"""ROS/Gazebo transport traffic during owned bridge shutdown; no simulation credit."""
import os,json,time,subprocess,signal
import rclpy
from std_msgs.msg import String
from gz.transport13 import Node
from gz.msgs10.stringmsg_pb2 import StringMsg
from common import APP,PKG,dump,sha

def run():
 out=APP/'results/R10/bridge_shutdown_probe';out.mkdir(parents=True,exist_ok=True);binary=APP/'build/r10_bridge/r10_tool_bridge'
 rclpy.init();ros=rclpy.create_node('r10_bridge_probe');gz=Node();received=[];command=[];fault=[]
 ros.create_subscription(String,'/s10/tool/raw_feedback',lambda m:received.append(m.data),20);cmd=ros.create_publisher(String,'/s10/tool/command',20);faultpub=ros.create_publisher(String,'/s10/tool/fault',20);raw=gz.advertise('/s10/tool/raw_feedback',StringMsg);gz.subscribe(StringMsg,'/s10/tool/command',lambda m:command.append(m.data));gz.subscribe(StringMsg,'/s10/tool/fault',lambda m:fault.append(m.data))
 def traffic(token):
  m=StringMsg();m.data=token;raw.publish(m);cmd.publish(String(data=token));faultpub.publish(String(data=token));rclpy.spin_once(ros,timeout_sec=.005);time.sleep(.005)
 cases=[]
 try:
  for i in range(6):
   token='probe_'+str(i);log=(out/f'{i}.log').open('w');p=subprocess.Popen([str(binary)],stdout=log,stderr=subprocess.STDOUT,start_new_session=True);begin=time.monotonic()
   try:
    while time.monotonic()-begin<8 and (received.count(token)<10 or command.count(token)<10 or fault.count(token)<10):traffic(token)
    transfer=all(v.count(token)>=10 for v in [received,command,fault]);p.send_signal(signal.SIGINT);ending=time.monotonic()
    while p.poll() is None and time.monotonic()-ending<5:traffic(token)
    forced=p.poll() is None
    if forced:p.kill()
    p.wait(timeout=3);cases.append(dict(cycle=i,both_directions_exact=transfer,exit=p.returncode,forced=forced,shutdown_wall_s=time.monotonic()-ending))
   finally:
    if p.poll() is None:p.kill();p.wait()
    log.close()
 finally:gz.unsubscribe('/s10/tool/command');gz.unsubscribe('/s10/tool/fault');ros.destroy_node();rclpy.shutdown()
 result=dict(passed=all(c['both_directions_exact'] and c['exit']==0 and not c['forced'] for c in cases),cases=cases,binary_sha256=sha(binary),source_sha256=sha(PKG/'r10_backend/tool_bridge.cpp'),probe_sha256=sha(__file__),scope='Actual ROS and Gazebo transport fixture; shutdown under active traffic, no physical work credit');dump(out/'result.json',result);print(json.dumps(result));return 0 if result['passed'] else 2
if __name__=='__main__':raise SystemExit(run())
