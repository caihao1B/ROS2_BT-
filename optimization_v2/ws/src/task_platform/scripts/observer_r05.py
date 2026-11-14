"""Online-only sensor and localization probe, no simulator or evaluation subscription."""
import argparse,json,math,time,signal
from pathlib import Path
from collections import Counter
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2,Image,Imu,NavSatFix
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf2_msgs.msg import TFMessage
from rtabmap_msgs.msg import OdomInfo

def main():
 ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path,required=True);a=ap.parse_args();spec=json.loads((a.out/'spec.json').read_text());vehicle=json.loads((a.out/'scene/vehicle.json').read_text());rclpy.init();n=rclpy.create_node('r05_online_probe');state=dict(t=0.,stop=False,encoder=None,evidence=None,odom=None,move_start=None,moved=False,probe_phase='WAIT',probe_phase_t=0.,probe_origin=None,restarted=False,last_valid=None,hint_sent=False,restart_hint_sent=False);counts=Counter();times={};tfowners={};stream=(a.out/'observations.jsonl').open('w');seq=0;lastsend=0.;pub=n.create_publisher(String,'/s03/actuator/request',10);hintpub=n.create_publisher(PoseWithCovarianceStamped,'/s07/rtabmap/initialpose',10)
 def dump(p,x):
  tmp=p.with_suffix('.tmp');tmp.write_text(json.dumps(x));tmp.replace(p)
 def record(kind,data):stream.write(json.dumps(dict(data,kind=kind,observed_clock=state['t']))+'\n')
 def stamp(m):return m.header.stamp.sec+m.header.stamp.nanosec*1e-9
 def sensor(m,key):
  counts[key]+=1;times[key]=stamp(m)
  if key=='camera' and counts[key]%15==1:
   pixels=np.frombuffer(m.data,np.uint8);record('camera',dict(t=stamp(m),width=m.width,height=m.height,std=float(np.std(pixels))))
   if counts[key]==1:
    from PIL import Image as PILImage
    PILImage.fromarray(pixels.reshape(m.height,m.width,3)).save(a.out/'camera.png')
  if key=='lidar' and counts[key]%10==1:record('lidar',dict(t=stamp(m),height=m.height,width=m.width))
  if key=='gnss':record('gnss',dict(t=stamp(m),llh=[m.latitude,m.longitude,m.altitude],covariance=list(m.position_covariance)))
 def odom(m):
  p=m.pose.pose;state['odom']=[p.position.x,p.position.y,p.position.z,p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w];counts['odom']+=1;record('odom',dict(t=stamp(m),pose=state['odom'],v=m.twist.twist.linear.x,w=m.twist.twist.angular.z))
 def evidence(m):
  e=json.loads(m.data);state['evidence']=e;counts['quality']+=1;record('quality',e)
  if e.get('valid'):state['last_valid']=e
 def tf(m):
  for tr in m.transforms:
   edge=tr.header.frame_id+'->'+tr.child_frame_id;tfowners[edge]=tfowners.get(edge,0)+1
 def backend(m):counts['rtab_icp_info']+=1;record('rtab_icp_info',dict(t=stamp(m),lost=m.lost,inliers=m.icp_inliers_ratio))
 n.create_subscription(Clock,'/clock',lambda m:state.update(t=m.clock.sec+m.clock.nanosec*1e-9),qos_profile_sensor_data);n.create_subscription(String,'/s03/encoders',lambda m:state.update(encoder=json.loads(m.data)),qos_profile_sensor_data)
 for typ,topic,key in [(PointCloud2,'/s04/lidar/points','lidar'),(Image,'/s04/stereo/left/image_rect','camera'),(Imu,'/s04/localization/imu','imu'),(NavSatFix,'/s04/localization/gnss','gnss')]:n.create_subscription(typ,topic,lambda m,k=key:sensor(m,k),qos_profile_sensor_data)
 n.create_subscription(Odometry,'/s05/odometry',odom,50);n.create_subscription(String,'/s07/localization/evidence',evidence,20);n.create_subscription(OdomInfo,'/s05/icp/info',backend,qos_profile_sensor_data);n.create_subscription(TFMessage,'/tf',tf,100)
 def hint(pose):
  m=PoseWithCovarianceStamped();m.header.frame_id='map';m.pose.pose.position.x,m.pose.pose.position.y,m.pose.pose.position.z=map(float,pose[:3]);m.pose.pose.orientation.x,m.pose.pose.orientation.y,m.pose.pose.orientation.z,m.pose.pose.orientation.w=map(float,pose[3:]);m.pose.covariance=[1. if i in [0,7,14] else .25 if i in [21,28,35] else 0. for i in range(36)];hintpub.publish(m);record('hint',dict(t=state['t'],pose=pose,source='last observed map match' if state['hint_sent'] else 'declared coarse operator prior'))
 def end(sig,fr):state['stop']=True
 for sig in [signal.SIGINT,signal.SIGTERM]:signal.signal(sig,end)
 (a.out/'observer_ready').write_text('online observations only');begin=time.monotonic();lastlog=0.;restart_pose=None;valid_first=0;valid_after=0;mode='STOP';speed=0.
 try:
  while not state['stop'] and time.monotonic()-begin<spec['wall_budget']-25:
   rclpy.spin_once(n,timeout_sec=.003);now=time.monotonic();t=state['t'];e=state['evidence'];fresh=e is not None and e.get('valid') and 0<=t-e['t']<1.5
   if t>8 and not state['hint_sent']:hint([*spec['operator_hint'][:2],0.,0.,0.,math.sin(spec['operator_hint'][2]/2),math.cos(spec['operator_hint'][2]/2)]);state['hint_sent']=True
   if fresh and t<65:valid_first+=1
   if t>25 and t<40 and fresh and state['move_start'] is None and state['odom']:
    state['move_start']=t;state['probe_phase']='FORWARD';state['probe_phase_t']=t;state['probe_origin']=state['odom'][:2]
   mode='STOP';speed=0.
   if state['move_start'] is not None and state['odom']:
    distance=math.dist(state['probe_origin'],state['odom'][:2]);phase=state['probe_phase']
    if phase=='FORWARD' and distance>=2.2:state['probe_phase']='GEAR_STOP';state['probe_phase_t']=t
    elif phase=='GEAR_STOP' and t-state['probe_phase_t']>=1.5:state['probe_phase']='REVERSE';state['probe_phase_t']=t
    elif phase=='REVERSE' and distance<=.15:state['probe_phase']='DONE';state['moved']=True
    phase=state['probe_phase']
    if phase in ['FORWARD','REVERSE'] and fresh:
     mode='DRIVE' if state['encoder'] and state['encoder']['armed'] else 'ARM'
     speed=(.4 if phase=='FORWARD' else -.25) if mode=='DRIVE' else 0.
   if t>65 and not (a.out/'restart_requested.json').exists():
    if state['last_valid']:
     last=state['last_valid'];restart_pose=last['pose'];dump(a.out/'restart_requested.json',dict(t=t,pose=restart_pose,observation_t=last['t'],source='last accepted map pose used only as coarse restart prior; fresh matching remains mandatory'))
    else:dump(a.out/'restart_requested.json',dict(t=t,pose=None,reason='NO_VALID_POSE'))
   if (a.out/'restart_done.json').exists() and not state['restart_hint_sent']:
    done=json.loads((a.out/'restart_done.json').read_text())
    if t>done['t']+3 and restart_pose:hint(restart_pose);state['restart_hint_sent']=True;state['restarted']=True
   if state['restarted'] and fresh and e['t']>70:valid_after+=1
   if t>60:mode='STOP';speed=0.
   enc=state['encoder']
   if enc and now-lastsend>.015:
    seq+=1;lastsend=now;m=String();m.data=json.dumps(dict(session=spec['session'],revision=vehicle['config_hash'],epoch=enc['epoch'],sequence=seq,stamp=t,mode=mode,v=speed,w=0.));pub.publish(m)
   if now-lastlog>.4:
    lastlog=now;stream.flush();dump(a.out/'live.json',dict(t=t,counts=dict(counts),probe_phase=state['probe_phase'],move_start=state['move_start'],moved=state['moved'],restarted=state['restarted'],quality_fresh=bool(fresh),evidence=e,mode=mode,odom=state['odom']))
   if t>=spec['sim_seconds']:break
 finally:
  stream.close();dump(a.out/'observer_result.json',dict(t=state['t'],complete=state['t']>=spec['sim_seconds'],counts=dict(counts),tf_edges=tfowners,tf_publishers=[dict(node=x.node_name,namespace=x.node_namespace,gid=list(x.endpoint_gid)) for x in n.get_publishers_info_by_topic('/tf')],probe_phase=state['probe_phase'],move_start=state['move_start'],moved=state['moved'],restarted=state['restarted'],valid_first_callbacks=valid_first,valid_after_callbacks=valid_after,feedback=state['encoder']));n.destroy_node();rclpy.try_shutdown()
 return 0 if state['t']>=spec['sim_seconds'] else 2
if __name__=='__main__':raise SystemExit(main())
