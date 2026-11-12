from common13 import *
import argparse,signal
import rclpy
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry,Path as RosPath
from geometry_msgs.msg import PoseWithCovarianceStamped,PoseStamped
from tf2_msgs.msg import TFMessage

def main():
 p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True);p.add_argument('--seconds',type=float,required=True);a=p.parse_args();a.out.mkdir(parents=True,exist_ok=True);rclpy.init();n=rclpy.create_node('s13_observer');st=dict(t=0.,stop=False,status=None,pose=None,odom=None,tool=None,navigation_active=False,gate=None);f=(a.out/'observations.jsonl').open('w');last={};audits=[];edges=set();trace=RosPath();trace.header.frame_id='map';pub=n.create_publisher(RosPath,'/s13/trace',2);audit_pub=n.create_publisher(String,'/s13/ownership_audit',2);lease_t=-100.
 def record(k,d):f.write(json.dumps(dict(d,record_type=k),separators=(',',':'))+'\n')
 def string(m,k):
  nonlocal lease_t
  d=json.loads(m.data);record(k,d)
  if k in st:st[k]=d
  if k=='lease':st['navigation_active']=d.get('active',False);lease_t=d['t']
 def position(m,k):
  po=m.pose.pose;q=po.orientation;d=dict(t=stamp(m),pose=[po.position.x,po.position.y,po.position.z,q.x,q.y,q.z,q.w]);st[k]=d
  if stamp(m)-last.get(k,-100)<.09:return
  last[k]=stamp(m);record(k,d)
  if k=='pose':p=PoseStamped();p.header=m.header;p.pose=po;trace.header=m.header;trace.poses.append(p);pub.publish(trace)
 n.create_subscription(Clock,'/clock',lambda m:st.update(t=m.clock.sec+m.clock.nanosec*1e-9),qos_profile_sensor_data)
 n.create_subscription(PoseWithCovarianceStamped,'/s08/pose',lambda m:position(m,'pose'),20);n.create_subscription(Odometry,'/s05/odometry',lambda m:position(m,'odom'),30)
 n.create_subscription(RosPath,'/plan',lambda m:record('computed_plan',dict(t=st['t'],frame=m.header.frame_id,poses=[[p.pose.position.x,p.pose.position.y,yaw(p.pose.orientation)] for p in m.poses])),2)
 for t,k in [('/s08/status','status'),('/s05/safety_limit','health'),('/s09/safety','perception'),('/s10/action/state','tool'),('/s13/gate','gate'),('/s13/controller_metrics','metrics'),('/s13/lease','lease'),('/s03/encoders','encoders')]:n.create_subscription(String,t,lambda m,key=k:string(m,key),50)
 def tf(m):
  for x in m.transforms:edges.add((x.header.frame_id,x.child_frame_id))
 n.create_subscription(TFMessage,'/tf',tf,100)
 def stop(s,f):st['stop']=True
 signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop);(a.out/'ready').write_text('online estimated-only navigation observer')
 try:
  while not st['stop'] and st['t']<a.seconds:
   rclpy.spin_once(n,timeout_sec=.01)
   if st['t']-last.get('audit',-100)>.5:
    last['audit']=st['t'];audits.append(dict(t=st['t'],navigation_active=st['navigation_active'],tf=[x.node_name for x in n.get_publishers_info_by_topic('/tf')],request=[x.node_name for x in n.get_publishers_info_by_topic('/s03/request')],actuator=[x.node_name for x in n.get_publishers_info_by_topic('/s03/actuator/request')],nav=[x.node_name for x in n.get_publishers_info_by_topic('/s13/nav_cmd')],lease=[x.node_name for x in n.get_publishers_info_by_topic('/s13/lease')]));audit=audits[-1];ready=audit['request']==['s03_test_driver'] and audit['actuator']==['s03_drive_bridge'] and audit['nav']==['controller_server'] and audit['lease']==['s13_action_client'] and sorted(audit['tf'])==['s05_local_odometry','s08_global_pose_manager'] and 0<=st['t']-lease_t<.3;audit_pub.publish(String(data=json.dumps(dict(t=st['t'],ready=ready,lease_t=lease_t))));f.flush();dump(a.out/'live.json',{k:v for k,v in st.items() if k!='stop'})
 finally:
  subscriptions={t:[p.node_name for p in n.get_subscriptions_info_by_topic(t)] for t,typ in n.get_topic_names_and_types()};f.close();dump(a.out/'result.json',dict(t=st['t'],audits=audits,tf_edges=[list(x) for x in sorted(edges)],subscriptions=subscriptions));n.destroy_node();rclpy.try_shutdown()
if __name__=='__main__':main()
