"""Replay real bounded CDR on an isolated domain and use normal ROS subscribers."""
import argparse,json,os,signal,subprocess,time
from pathlib import Path
import rclpy
from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy,qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2,Image,Imu,NavSatFix,CameraInfo
from geometry_msgs.msg import TwistWithCovarianceStamped
from tf2_msgs.msg import TFMessage
ROOT=Path(__file__).resolve().parents[5]
def main():
    p=argparse.ArgumentParser();p.add_argument('case',type=Path);a=p.parse_args();rclpy.init(domain_id=122);node=rclpy.create_node('s04_replay_check');counts={};sizes={}
    def received(key,m):
        counts[key]=counts.get(key,0)+1
        if key=='tf':sizes['tf']=len(m.transforms)
        if key=='points':sizes['cloud']=[m.height,m.width]
        if key=='left_info':sizes['baseline_left']=m.p[3]
    calq=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    for key,typ,topic in [('tf',TFMessage,'/tf_static'),('left_info',CameraInfo,'/s04/stereo/left/camera_info'),('right_info',CameraInfo,'/s04/stereo/right/camera_info'),('points',PointCloud2,'/s04/lidar/points'),('left',Image,'/s04/stereo/left/image_rect'),('right',Image,'/s04/stereo/right/image_rect'),('imu',Imu,'/s04/imu/data'),('wheel',TwistWithCovarianceStamped,'/s04/wheel/twist'),('gnss',NavSatFix,'/s04/gnss/fix')]:
        node.create_subscription(typ,topic,lambda m,k=key:received(k,m),calq if key=='tf' or key.endswith('info') else qos_profile_sensor_data)
    args=['ros2','bag','play',str(a.case/'online/sample_bag'),'--clock','--delay','2','--qos-profile-overrides-path',str(ROOT/'optimization_v2/config/sensor_replay_qos.yaml')]
    env=os.environ.copy();env['ROS_DOMAIN_ID']='122';log=(a.case/'replay_check.log').open('w');proc=subprocess.Popen(args,env=env,stdout=log,stderr=subprocess.STDOUT);start=time.monotonic()
    try:
        while time.monotonic()-start<20:
            rclpy.spin_once(node,timeout_sec=.05)
            if proc.poll() is not None:break
        if proc.poll() is None:proc.send_signal(signal.SIGINT);proc.wait(timeout=5)
    finally:
        if proc.poll() is None:proc.kill();proc.wait()
        log.close();node.destroy_node();rclpy.try_shutdown()
    result=dict(command=args,exit_code=proc.returncode,counts=counts,sizes=sizes,domain=122)
    result['passed']=proc.returncode==0 and len(counts)==9 and sizes.get('tf')==5 and sizes.get('cloud')==[32,1024]
    (a.case/'replay_check.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n');print(json.dumps(result,ensure_ascii=False));return 0 if result['passed'] else 1
if __name__=='__main__':raise SystemExit(main())
